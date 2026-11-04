"""One RK-local odometry goal: turn, drive straight, verify STOP. Import is inert."""
import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
import uuid

from simple_p2p import Config, SimplePointController
from simple_p2p_base import run_session


HERE = Path(__file__).resolve().parent
DEFAULT_CONFIG = (HERE / 'config' / 'simple-p2p.json' if (HERE / 'config' / 'simple-p2p.json').is_file()
                  else HERE.parent / 'config' / 'simple-p2p.json')


def durable_json(path, value):
    """Complete atomic snapshot. Call only off the serial/control thread."""
    path = Path(path)
    raw = json.dumps(value, indent=2, allow_nan=False)
    temporary = path.with_name(path.name+'.durable-tmp')
    with temporary.open('w', encoding='utf-8', newline='\n') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))
    if os.name != 'nt' and hasattr(os, 'O_DIRECTORY'):
        descriptor = os.open(str(path.parent), os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


class LiveStatusWriter:
    """One latest RAM slot; only its worker touches disk, at most twice/sec."""
    def __init__(self, path, boot_id, clock=time.monotonic):
        self.path, self.clock = Path(path), clock
        self.lock, self.done = threading.Lock(), threading.Event()
        self.latest = dict(schema='simple-p2p-live-v1', boot_id=boot_id, phase='starting',
            pose=None, raw_packet=None, source_monotonic_s=None, finished=False,
            pose_source_monotonic_s=None, pose_measurement_monotonic_s=None,
            position_verified=False)
        self.error = None
        self.thread = threading.Thread(target=self._run, name='live-status-writer', daemon=True)

    def submit(self, event):
        # Called by the serial thread: bounded copies of flat scalar dictionaries.
        with self.lock:
            self.latest['phase'] = event['phase']
            self.latest['event_monotonic_s'] = event['event_monotonic_s']
            if event['source_monotonic_s'] is not None:
                self.latest.update(source_monotonic_s=event['source_monotonic_s'],
                    raw_packet=event['raw_packet'])
                if event['pose'] is not None:
                    self.latest.update(pose=dict(event['pose']),
                        pose_source_monotonic_s=event['source_monotonic_s'],
                        pose_measurement_monotonic_s=event.get('measurement_monotonic_s'))
            for key in ('history_summary', 'control_summary'):
                if isinstance(event.get(key), dict):
                    self.latest[key] = dict(event[key])
            if event['event'] == 'origin':
                self.latest['origin'] = dict(pose=event['pose'], raw_packet=event['raw_packet'],
                    source_monotonic_s=event['source_monotonic_s'])

    def _write(self):
        with self.lock:
            snapshot = dict(self.latest)
        snapshot['written_monotonic_s'] = self.clock()  # Distinct from original source.
        durable_json(self.path, snapshot)

    def _run(self):
        try:
            while True:
                self._write()
                if self.done.wait(.5):
                    return
        except BaseException as exc:
            self.error = type(exc).__name__+': '+str(exc)

    def start(self):
        self.thread.start()

    def close(self, result, timeout=2.):
        self.done.set()
        self.thread.join(timeout)
        if self.thread.is_alive():
            self.error = 'Live status writer did not exit within bounded cleanup'
            return False
        # The port has already closed. No background write can race this final one.
        proof = result.get('final_stop_chain') or {}
        packet = (proof.get('zero_samples') or [None])[-1]
        pose = result.get('latest_odometry')
        with self.lock:
            self.latest.update(finished=True, phase='finished',
                position_verified=result.get('position_verified',False),
                first_failure=result.get('first_failure'),
                session_complete=result.get('session_complete',False))
            if packet is not None:
                self.latest['raw_packet'] = packet
            if result.get('position_verified') and pose is not None:
                self.latest['pose'] = {key:pose[key] for key in ('x','y','yaw')}
                self.latest['pose_source_monotonic_s'] = pose.get('source_monotonic_s')
                self.latest['pose_measurement_monotonic_s'] = pose.get('measurement_monotonic_s')
        try:
            self._write()
        except BaseException as exc:
            self.error = type(exc).__name__+': '+str(exc)
        return self.error is None


def finite(name, value, positive=False):
    if type(value) not in (int, float) or not math.isfinite(value) or (positive and value <= 0):
        raise ValueError(name + ' must be finite' + (' and positive' if positive else ''))
    return float(value)


def load_config(path):
    data = json.loads(Path(path).read_text(encoding='utf-8'))
    if set(data) != {'controller', 'obstacle'}:
        raise ValueError('Expected controller and obstacle sections only')
    controller = Config(**data['controller'])
    expected = {'stop_clearance_m', 'body_front_m', 'body_rear_m', 'body_half_width_m',
                'laser_x_m', 'laser_y_m', 'laser_yaw_rad', 'scan_timeout_s'}
    if set(data['obstacle']) != expected:
        raise ValueError('Obstacle configuration fields do not match')
    obstacle = {name: finite(name, value, positive=name not in ('laser_x_m', 'laser_y_m', 'laser_yaw_rad','stop_clearance_m'))
                for name, value in data['obstacle'].items()}
    if obstacle['stop_clearance_m'] < 0:
        raise ValueError('Extra stop clearance must be nonnegative; zero disables the extra margin')
    if obstacle['scan_timeout_s'] > .75:
        raise ValueError('Scan source timeout cannot exceed 750ms')
    return controller, obstacle


class ScanGate:
    """Measured front180 body intersection; an optional extra margin may be zero.

    Zero margin retains the body rectangle and uninflated turn sweep boundary.
    Never assert rear/whole-turn clearance from this scan alone.
    """
    def __init__(self, settings, *, turn_area_clear=False):
        self.settings = dict(settings)
        self.turn_area_clear = bool(turn_area_clear)
        self.lock = threading.Lock()
        self.latest = None
        self.error = 'waiting for first original scan'
        self.last_header_ns = None

    def accept(self, message, ros_now_ns, received_mono):
        try:
            stamp = message.header.stamp
            header_ns = int(stamp.secs) * 1000000000 + int(stamp.nsecs)
            if header_ns <= 0 or not 0 <= int(stamp.nsecs) < 1000000000:
                raise ValueError('invalid original scan timestamp')
            age = (ros_now_ns-header_ns)/1e9
            if not math.isfinite(age) or age < 0 or age > self.settings['scan_timeout_s']:
                raise ValueError('original scan timestamp is expired or in the future')
            amin = finite('angle_min', message.angle_min)
            amax = finite('angle_max', message.angle_max)
            inc = finite('angle_increment', message.angle_increment, positive=True)
            if inc > math.radians(1.)+1e-9:
                raise ValueError('front scan ray spacing exceeds one degree')
            lower = finite('range_min', message.range_min)
            upper = finite('range_max', message.range_max, positive=True)
            ranges = message.ranges
            if (not 3 <= len(ranges) <= 20000 or lower < 0 or upper <= lower
                    or abs(amax-(amin+(len(ranges)-1)*inc)) > inc*.02+1e-6):
                raise ValueError('invalid original scan layout')
            indices = [i for i in range(len(ranges)) if -math.pi/2-1e-6 <= amin+i*inc <= math.pi/2+1e-6]
            if (len(indices) < 3 or amin+indices[0]*inc > -math.pi/2+inc+1e-6
                    or amin+indices[-1]*inc < math.pi/2-inc-1e-6):
                raise ValueError('continuous measured front180 required')
            points, invalid = [], 0
            for i in indices:
                distance = float(ranges[i])
                if distance == math.inf:
                    continue
                if not math.isfinite(distance) or not lower <= distance <= upper:
                    invalid += 1
                    continue
                angle = amin+i*inc+self.settings['laser_yaw_rad']
                points.append((self.settings['laser_x_m']+distance*math.cos(angle),
                               self.settings['laser_y_m']+distance*math.sin(angle)))
            if len(points) < 10 or invalid:
                raise ValueError('insufficient or invalid measured obstacle rays')
            with self.lock:
                if self.last_header_ns is not None and header_ns < self.last_header_ns:
                    raise ValueError('original scan clock regressed')
                if self.last_header_ns == header_ns:
                    return  # A duplicate cannot renew its original source time.
                self.last_header_ns = header_ns
                self.latest = dict(source_monotonic_s=received_mono-age, points=tuple(points),
                                   header_ns=header_ns, finite_rays=len(points))
                self.error = None
        except (ValueError, TypeError, OverflowError, AttributeError) as exc:
            with self.lock:
                self.error = str(exc)

    def __call__(self, now, phase):
        with self.lock:
            latest, error = self.latest, self.error
        if error or latest is None:
            return dict(ok=False, reason=error or 'scan unavailable', source_monotonic_s=None)
        source = latest['source_monotonic_s']
        if not 0 <= now-source < self.settings['scan_timeout_s']:
            return dict(ok=False, reason='original scan source expired', source_monotonic_s=source)
        if phase == 'turn' and not self.turn_area_clear:
            return dict(ok=False, reason='front180 cannot prove rear turn clearance; operator turn-area check required',
                        source_monotonic_s=source)
        s = self.settings
        front, rear, half, margin = s['body_front_m'], s['body_rear_m'], s['body_half_width_m'], s['stop_clearance_m']
        for x, y in latest['points']:
            if phase == 'turn' and math.hypot(x, y) <= math.hypot(max(front, rear), half)+margin:
                return dict(ok=False, reason='observed obstacle within whole-body turn sweep clearance',
                            source_monotonic_s=source)
            # Rectangle distance measures clearance from the body, not laser origin.
            dx = max(-rear-x, 0., x-front)
            dy = max(abs(y)-half, 0.)
            if math.hypot(dx, dy) <= margin:
                return dict(ok=False, reason='obstacle within body clearance', source_monotonic_s=source)
        return dict(ok=True, reason='measured front sector clear; rear is operator-checked',
                    source_monotonic_s=source)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', type=Path, default=DEFAULT_CONFIG)
    parser.add_argument('--expected-boot-id', required=True)
    parser.add_argument('--output-dir', type=Path, required=True, help='New directory, never reused')
    parser.add_argument('--goal-x', type=float)
    parser.add_argument('--goal-y', type=float)
    parser.add_argument('--reset-odom', action='store_true', required=True,
                        help='Use the first healthy stopped feedback as local (0,0,0), not an MCU encoder reset')
    parser.add_argument('--execute', action='store_true', help='Explicitly authorize this one goal to ARM and move')
    parser.add_argument('--turn-area-clear', action='store_true',
                        help='Operator has checked the whole swept body area including the rear blind sector')
    parser.add_argument('--scan-topic', default='/lidar1/laser_fan')
    parser.add_argument('--expected-start-encoders', nargs=2, type=int, metavar=('LEFT_RAW','RIGHT_RAW'))
    parser.add_argument('--cancel-request-file', type=Path, help='Optional fixed broker-owned cancellation file')
    args = parser.parse_args(argv)
    if str(uuid.UUID(args.expected_boot_id)) != args.expected_boot_id:
        parser.error('Expected a canonical boot UUID')
    if args.execute and (args.goal_x is None or args.goal_y is None):
        parser.error('--execute requires --goal-x and --goal-y')
    if not args.execute and (args.goal_x is not None or args.goal_y is not None):
        parser.error('An origin-only observation does not accept a motion goal')
    if args.expected_start_encoders is not None and any(not 0 <= value <= 0xffffffff for value in args.expected_start_encoders):
        parser.error('--expected-start-encoders requires two original uint32 values')
    expected = None if args.expected_start_encoders is None else dict(zip(('left_raw','right_raw'),args.expected_start_encoders))
    goal_x = finite('goal_x', args.goal_x or 0.)
    goal_y = finite('goal_y', args.goal_y or 0.)
    config, obstacle = load_config(args.config)
    actual_boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if actual_boot != args.expected_boot_id:
        raise RuntimeError('RK boot changed; no old origin or position is reused')
    args.output_dir.mkdir(parents=True, exist_ok=False)
    cancelled = threading.Event()
    watcher_done = threading.Event()
    def cancel_handler(_sig, _frame):
        cancelled.set()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, cancel_handler)
    def watch_cancel_file():
        while not watcher_done.wait(.05):
            if (args.output_dir/'cancel.request').exists() or (args.cancel_request_file is not None and args.cancel_request_file.exists()):
                cancelled.set()
                return
    scan, subscriber, rospy = None, None, None
    if args.execute:
        import rospy
        from sensor_msgs.msg import LaserScan
        rospy.init_node('simple_p2p_' + uuid.uuid4().hex[:8], anonymous=False, disable_signals=True)
        if rospy.get_param('/use_sim_time', False):
            rospy.signal_shutdown('simulated time rejected')
            raise RuntimeError('Real robot control rejects simulated ROS time')
        scan = ScanGate(obstacle, turn_area_clear=args.turn_area_clear)
        def on_scan(message):
            scan.accept(message, rospy.Time.now().to_nsec(), time.monotonic())
        subscriber = rospy.Subscriber(args.scan_topic, LaserScan, on_scan, queue_size=1)
        deadline = time.monotonic()+5.
        while time.monotonic() < deadline and not cancelled.is_set():
            if scan(time.monotonic(), 'drive')['ok']:
                break
            time.sleep(.02)
        if not scan(time.monotonic(), 'drive')['ok']:
            raise RuntimeError('No fresh clear front scan before opening the control device')
    # Import exact bundled files at runtime. No module import opens any device.
    import importlib.util
    loader_spec = importlib.util.spec_from_file_location('_simple_transport', HERE/'deps/transport.py')
    transport = importlib.util.module_from_spec(loader_spec)
    loader_spec.loader.exec_module(transport)
    api = transport.load_dependency(HERE/'deps', 'control_client')
    core = transport.load_dependency(HERE/'deps', 'core')
    link = transport.load_dependency(HERE/'deps', 'rk_link')
    target = link.locate()
    result = None
    watcher = threading.Thread(target=watch_cancel_file, name='local-cancel-file', daemon=True)
    watcher.start()
    live = LiveStatusWriter(args.output_dir/'live_status.json',actual_boot)
    live.start()
    try:
        with transport.exclusive_port(target['port']) as physical:
            result = run_session(physical, api, core,
                SimplePointController(goal_x, goal_y, config=config, started_at=time.monotonic()),
                scan_provider=scan, boot_id=actual_boot, execute=args.execute,
                cancelled=cancelled.is_set, expected_start_encoders=expected, event_sink=live.submit)
        result['exclusive_port_closed_normally'] = True
    except BaseException as exc:
        if result is None:
            result = dict(session_complete=False, motion_complete=False, origin=None,
                          first_failure=None, final_stop_chain=None)
        result['exclusive_port_closed_normally'] = False
        error = dict(phase='serial_owner_or_cleanup', error_type=type(exc).__name__, message=str(exc))
        result.setdefault('cleanup_errors', []).append(error)
        if result.get('first_failure') is None:
            result['first_failure'] = error
        result['session_complete'] = False
    finally:
        watcher_done.set()
        watcher.join(timeout=1.)
        if subscriber is not None:
            subscriber.unregister()
        if rospy is not None:
            rospy.signal_shutdown('single local goal finished')
        live_ok = live.close(result)
        result['live_status_writer_complete'] = live_ok
        if not live_ok:
            result.setdefault('cleanup_errors',[]).append(dict(phase='live_status_writer',message=live.error))
            result['session_complete'] = False
    result.update(schema='simple-p2p-result-v1', boot_id=actual_boot, goal=dict(x=goal_x, y=goal_y),
                  motion_requested=args.execute, reset_odom_requested=True, encoders_reset=False,
                  config=asdict(config), obstacle=obstacle, geometry_accuracy_verified=False,
                  turn_area_operator_checked=args.turn_area_clear,
                  origin_valid_only_for_this_process=True, old_map_loaded=False, amcl_started=False)
    # All serial work has ended before filesystem serialization.
    durable_json(args.output_dir/'result.json', result)
    if result.get('origin') is not None:
        durable_json(args.output_dir/'origin.json', result['origin'])
    print(json.dumps({key: result.get(key) for key in ('schema', 'motion_requested', 'session_complete',
          'motion_complete', 'first_failure', 'origin', 'final_stop_chain', 'exclusive_port_closed_normally')},
          allow_nan=False))
    return 0 if result.get('session_complete') and result.get('first_failure') is None else 2


if __name__ == '__main__':
    raise SystemExit(main())
