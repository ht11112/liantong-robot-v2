"""Bounded real RK static integration. Only STATUS/STOP/FEEDBACK can be sent.

Publishes private measured Odometry, starts a private front180 sensor observer,
and exits after fresh final STOP evidence. No task/ARM/SETPOINT/TF entry exists.
"""
import argparse
from collections import deque
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from base_static_adapter import run_static_session

ROOT = Path(__file__).resolve().parent


def finish_sensor(sensor, timeout_s):
    """Reap only the read-only observer; never signal the physical owner."""
    result = dict(sensor_cleanup_errors=[], sensor_forced_kill=False)
    try:
        result['sensor_exit_code'] = sensor.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        result['sensor_timed_out'] = True
        try:
            sensor.terminate()
            try:
                result['sensor_exit_code'] = sensor.wait(timeout=5)
            except subprocess.TimeoutExpired:
                sensor.kill()
                result['sensor_forced_kill'] = True
                result['sensor_exit_code'] = sensor.wait(timeout=5)
        except Exception as exc:
            result['sensor_cleanup_errors'].append(type(exc).__name__ + ': ' + str(exc))
    except Exception as exc:
        result['sensor_cleanup_errors'].append(type(exc).__name__ + ': ' + str(exc))
    return result


def scan_evidence_in_window(sensors):
    """Transport evidence only; no unselected navigation age budget is assumed."""
    span = sensors.get('elapsed_s')
    if type(span) not in (int, float) or not math.isfinite(span) or span <= 0:
        return False
    for name in ('scan', 'front180'):
        channel = (sensors.get('channels') or {}).get(name) or {}
        latest = channel.get('latest') or {}
        age = latest.get('source_age_s')
        if (channel.get('missing', True) or channel.get('new_sources', 0) < 5
                or type(age) not in (int, float) or not math.isfinite(age)
                or not 0 <= age <= span or latest.get('future_source_stamp', False)):
            return False
    return True


def reviewed_module(name, relative, expected):
    path = ROOT / relative
    if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise RuntimeError('Static dependency bytes differ: ' + relative)
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-boot-id', required=True)
    parser.add_argument('--duration', type=float, required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 120:
        raise ValueError('Static duration must be 1..120 seconds')
    boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if boot_id != args.expected_boot_id:
        raise RuntimeError('RK boot changed; repeat the live inventory')
    manifest = json.loads((ROOT / 'static_package_manifest.json').read_text())
    for relative, expected in manifest['files'].items():
        path = ROOT / relative
        if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError('Package startup check failed: ' + relative)
    api = reviewed_module('p2p_static_wire', 'deps/control_client.py', manifest['files']['deps/control_client.py'])
    locator = reviewed_module('p2p_static_locator', 'deps/rk_link.py', manifest['files']['deps/rk_link.py'])
    transport = reviewed_module('p2p_static_transport', 'deps/transport.py', manifest['files']['deps/transport.py'])
    stop_api = reviewed_module('p2p_static_stop', 'deps/stop_observation.py', manifest['files']['deps/stop_observation.py'])
    core = reviewed_module('p2p_static_core', 'deps/core.py', manifest['files']['deps/core.py'])
    site = json.loads((ROOT / 'site_config.json').read_text())
    if site['user']['feedback_source_timeout_ms'] != 750 or site['sensing']['navigation_fov_deg'] != 180:
        raise RuntimeError('Static trial must use the user-selected 750ms and front180 design')
    import rospy
    from nav_msgs.msg import Odometry
    rospy.init_node('p2p_base_static_' + str(os.getpid()), anonymous=False, disable_signals=True)
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Real hardware integration must not use simulated ROS time')
    output = Path(args.output)
    sensor_output = output.with_name('sensor_result.json')
    cancelled = threading.Event()
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda _signum, _frame: cancelled.set())
    odom_topic = rospy.get_name() + '/wheel_odom'
    publisher = rospy.Publisher(odom_topic, Odometry, queue_size=1)
    mailbox = deque(maxlen=1)
    publishing_done = threading.Event()
    publication = dict(messages=0, errors=[], tf_published=False, topic=odom_topic,
                       frame_id='p2p_static_odom', child_frame_id='p2p_static_base_link',
                       source_time_preserved=True, geometry_calibrated=False,
                       output_header_seq_semantics='rospy_publisher_sequence',
                       original_MCU_sample_sequence_retained_in_base_evidence=True)

    def publish_loop():
        while not publishing_done.is_set():
            if not mailbox:
                publishing_done.wait(.01)
                continue
            observation = mailbox.popleft()
            try:
                at = time.monotonic()
                source_ros = rospy.Time.now().to_sec() - (at - observation['source_monotonic_s'])
                message = Odometry()
                message.header.seq = observation['sample_sequence']
                message.header.stamp = rospy.Time.from_sec(source_ros)
                message.header.frame_id = publication['frame_id']
                message.child_frame_id = publication['child_frame_id']
                message.pose.pose.position.x = observation['x']
                message.pose.pose.position.y = observation['y']
                message.pose.pose.orientation.z = math.sin(observation['yaw'] / 2.)
                message.pose.pose.orientation.w = math.cos(observation['yaw'] / 2.)
                message.twist.twist.linear.x = observation['linear_x']
                message.twist.twist.angular.z = observation['angular_z']
                # Geometry is uncalibrated; this diagnostic topic is not a pose
                # confidence claim and is not fed into a navigation controller.
                for index in (0, 7, 14, 21, 28, 35):
                    message.pose.covariance[index] = 1e3
                    message.twist.covariance[index] = 1e3
                publisher.publish(message)
                publication['messages'] += 1
            except Exception as exc:
                if len(publication['errors']) < 10:
                    publication['errors'].append(type(exc).__name__ + ': ' + str(exc))

    thread = threading.Thread(target=publish_loop, name='p2p-private-odom', daemon=True)
    thread.start()
    result = dict(schema='p2p-rk-static-integration-v1', boot_id=boot_id,
                  wall_started_s=time.time(), navigation_enabled=False, motion_requested=False,
                  actual_map_localization_valid=False, base_result=None,
                  firmware_modified=False, control_mode='permanent_query_STOP_only')
    sensor = None
    sensor_log = None
    try:
        sensor_log = output.with_name('sensor.log').open('w')
        sensor = subprocess.Popen([
            sys.executable, '-B', str(ROOT / 'ros_sensor_adapter.py'),
            '--scan-topic', '/lidar1/laser_fan', '--duration', str(args.duration + 10),
            '--odom-topic', odom_topic, '--map-topic', '/commissioning/room_map',
            '--output', str(sensor_output)], stdin=subprocess.DEVNULL,
            stdout=sensor_log, stderr=sensor_log)
        time.sleep(2.)  # Warm ROS imports before opening the sole physical port.
        if sensor.poll() is not None:
            raise RuntimeError('Sensor observer exited before static link startup')
        target = locator.locate()  # Fresh USB REV0264 identity + unoccupied fuser.
        result['target'] = target
        geometry = site['geometry']
        with transport.exclusive_port(target['port']) as physical:
            result['base_result'] = run_static_session(
                physical, api, stop_api,
                lambda: core.MeasuredOdometry(geometry['wheel_diameter_m'],
                                             geometry['wheel_separation_m'], calibrated=False),
                duration_s=args.duration, feedback_timeout_s=.750,
                cancelled=lambda: cancelled.is_set() or rospy.is_shutdown() or sensor.poll() is not None,
                odom_sink=mailbox.append)
        result['exclusive_port_closed_normally'] = True
    except BaseException as exc:
        result['runtime_error'] = dict(type=type(exc).__name__, message=str(exc))
    finally:
        publishing_done.set()
        thread.join(timeout=2.)
        result['publisher_thread_still_alive'] = thread.is_alive()
        try:
            publisher.unregister()
        except Exception as exc:
            publication['errors'].append('unregister: ' + type(exc).__name__ + ': ' + str(exc))
        if sensor is not None:
            result.update(finish_sensor(sensor, args.duration + 15))
        if sensor_log:
            sensor_log.close()
        result['odom_publication'] = publication
        if sensor_output.is_file():
            try:
                result['sensor_result'] = json.loads(sensor_output.read_text())
            except Exception as exc:
                result['sensor_result_error'] = type(exc).__name__ + ': ' + str(exc)
        rospy.signal_shutdown('Bounded p2p static integration ended')
    base = result.get('base_result') or {}
    sensors = result.get('sensor_result') or {}
    channels = sensors.get('channels') or {}
    odom_channel = channels.get('odom') or {}
    odom_latest = odom_channel.get('latest') or {}
    odom_payload = odom_latest.get('payload') or {}
    odom_header = odom_latest.get('header') or {}
    result['private_odom_received_and_frames_match'] = bool(
        not odom_channel.get('missing', True) and odom_channel.get('new_sources', 0) >= 5
        and odom_header.get('frame_id') == publication['frame_id']
        and odom_payload.get('child_frame_id') == publication['child_frame_id'])
    result['scan_evidence_from_current_observation_window'] = scan_evidence_in_window(sensors)
    result['navigation_scan_age_budget_configured'] = False
    result['static_integration_complete'] = bool(
        base.get('static_link_complete') and result.get('exclusive_port_closed_normally')
        and result.get('sensor_exit_code') == 0 and sensors.get('published_front180_messages', 0) > 0
        and publication['messages'] > 0 and not publication['errors']
        and result['private_odom_received_and_frames_match']
        and result['scan_evidence_from_current_observation_window']
        and not result.get('publisher_thread_still_alive') and not result.get('sensor_cleanup_errors')
        and not result.get('sensor_timed_out') and not result.get('sensor_result_error')
        and not sensors.get('callback_errors') and not sensors.get('runtime_error')
        and not result.get('runtime_error'))
    result['wall_ended_s'] = time.time()
    output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps(dict(output=str(output), static_integration_complete=result['static_integration_complete'],
                          first_failure=base.get('first_failure'), final_stop_error=base.get('final_stop_error'),
                          navigation_enabled=False)))
    return 0 if result['static_integration_complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
