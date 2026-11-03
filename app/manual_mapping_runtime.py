"""Empty-map hand-pushed GMAPPING recorder. Import opens no ROS/device.

The only serial writer is manual_mapping_query's permanent kind2/5 fence.
Ending recording never stops, arms, unlocks, or commands the wheelchair.
"""
import argparse
from collections import deque
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
from types import MappingProxyType

ROOT = Path(__file__).resolve().parent
GEOMETRY = dict(wheel_diameter_m=.20, wheel_separation_m=.30,
                laser_x_m=.13, laser_y_m=0., laser_z_m=.20, laser_yaw_rad=0.,
                calibrated=False, basis='existing nominal dimensions and mounting estimate')


def finite(value, name):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except (OverflowError, ValueError, TypeError):
        valid = False
    if not valid:
        raise ValueError(name + ' must be finite')
    return float(value)


def header_record(header):
    sec, ns, seq = header.stamp.secs, header.stamp.nsecs, header.seq
    if (type(sec) is not int or sec < 0 or type(ns) is not int or not 0 <= ns < 1000000000
            or type(seq) is not int or not 0 <= seq < 2**32 or type(header.frame_id) is not str):
        raise ValueError('Invalid original ROS header')
    return dict(seq=seq, stamp_secs=sec, stamp_nsecs=ns, frame_id=header.frame_id)


def stamp_ns(record):
    return record['stamp_secs'] * 1000000000 + record['stamp_nsecs']


def json_range(value):
    value = float(value)
    return value if math.isfinite(value) else ('inf' if value > 0 else '-inf' if value < 0 else None)


def crop_front_scan(raw, laser_frame):
    """Contiguous actual rays only; invalid rays become +inf, never fake free rays.

    Noetic GMAPPING skips +inf rays above its finite maxRange. The first-ray
    stamp is derived from the original first ray and time_increment, never now.
    """
    original = header_record(raw.header)
    if not stamp_ns(original):
        raise ValueError('Zero scan source stamp')
    amin = finite(raw.angle_min, 'angle_min')
    inc = finite(raw.angle_increment, 'angle_increment')
    amax = finite(raw.angle_max, 'angle_max')
    lower, upper = finite(raw.range_min, 'range_min'), finite(raw.range_max, 'range_max')
    dt = finite(raw.time_increment, 'time_increment')
    count = len(raw.ranges)
    if (not 3 <= count <= 20000 or inc <= 0 or inc > math.pi / 2 or lower < 0
            or upper <= lower or dt < 0 or abs(amax - (amin + (count-1)*inc)) > inc*.02 + 1e-6):
        raise ValueError('Invalid scan geometry/range/time layout')
    indices = [i for i in range(count) if -math.pi/2 - 1e-6 <= amin+i*inc <= math.pi/2 + 1e-6]
    if (not indices or len(indices) >= 2048 or indices != list(range(indices[0], indices[-1]+1))
            or amin+indices[0]*inc > -math.pi/2+inc+1e-6
            or amin+indices[-1]*inc < math.pi/2-inc-1e-6):
        raise ValueError('A continuous measured front180 sector is required')
    first, end = indices[0], indices[-1]+1
    cropped = deepcopy(raw)
    cropped.header.frame_id = laser_frame
    offset_ns = int(round(first*dt*1e9))
    derived = stamp_ns(original) + offset_ns
    cropped.header.stamp.secs, cropped.header.stamp.nsecs = divmod(derived, 1000000000)
    cropped.angle_min, cropped.angle_max = amin+first*inc, amin+(end-1)*inc
    selected, invalid, valid_finite, no_return = list(raw.ranges[first:end]), 0, 0, 0
    ranges = []
    for entry in selected:
        value = float(entry)
        if math.isfinite(value) and lower <= value <= upper:
            valid_finite += 1
            ranges.append(value)
        else:
            if value == math.inf:
                no_return += 1
            else:
                invalid += 1
            ranges.append(math.inf)
    cropped.ranges = ranges
    if getattr(raw, 'intensities', []):
        if len(raw.intensities) != count:
            raise ValueError('Intensity count does not match measured rays')
        cropped.intensities = list(raw.intensities[first:end])
    diagnostic = dict(raw_header=original, output_header=header_record(cropped.header),
        first_raw_index=first, end_raw_index_exclusive=end, raw_ray_count=count,
        selected_ray_count=len(ranges), derived_first_ray_offset_ns=offset_ns,
        angle_min=cropped.angle_min, angle_max=cropped.angle_max, angle_increment=inc,
        range_min=lower, range_max=upper, time_increment=dt,
        raw_selected_ranges=[json_range(x) for x in selected], invalid_rays_skipped=invalid,
        positive_infinite_rays=no_return, finite_valid_rays=valid_finite,
        rear_rays_invented=0, invalid_policy='positive_inf_skipped_by_finite_gmapping_maxRange',
        deskew_performed=False)
    return cropped, diagnostic


class SourceGate:
    """Original stamp monotonicity and age; duplicates never renew evidence."""
    def __init__(self, maximum_age_s=.75):
        self.maximum_age_s = finite(maximum_age_s, 'maximum_age_s')
        self.last_ns = None
        self.counts = dict(accepted=0, duplicate=0, reordered=0, future=0, stale=0)

    def accept(self, record, now_ros):
        source = stamp_ns(record)
        age = finite(now_ros, 'now_ros') - source/1e9
        reason = ('future' if age < 0 else 'stale' if age >= self.maximum_age_s else
                  'duplicate' if source == self.last_ns else
                  'reordered' if self.last_ns is not None and source < self.last_ns else None)
        if reason:
            self.counts[reason] += 1
            return False
        self.last_ns = source
        self.counts['accepted'] += 1
        return True


def source_ros_ns(source_mono, received_mono, received_ros_ns):
    """Bridge once at receipt. Queue/relay delay cannot make the source newer."""
    source, receipt = finite(source_mono, 'source_mono'), finite(received_mono, 'received_mono')
    if source < 0 or source > receipt or type(received_ros_ns) is not int or received_ros_ns <= 0:
        raise ValueError('Invalid source/receipt clock bridge')
    result = received_ros_ns - int(round((receipt-source)*1e9))
    if result <= 0:
        raise ValueError('Invalid mapped ROS source stamp')
    return result


def source_current(record, now_ros, maximum_age_s=.75):
    age = finite(now_ros, 'now_ros') - stamp_ns(record)/1e9
    return 0 <= age < maximum_age_s


def names_for(nonce):
    if type(nonce) is not str or not re.fullmatch('[A-Za-z0-9_]{4,48}', nonce):
        raise ValueError('nonce must be 4..48 ASCII letters/digits/underscores')
    namespace = '/manual_mapping_' + nonce
    frames = {key: 'manual_'+nonce+'_'+key for key in ('map', 'odom', 'base', 'laser')}
    return namespace, frames


def gmapping_parameters(frames):
    return dict(base_frame=frames['base'], odom_frame=frames['odom'], map_frame=frames['map'],
        delta=.05, xmin=-5., ymin=-5., xmax=5., ymax=5., map_update_interval=1.,
        linearUpdate=.05, angularUpdate=.05, temporalUpdate=.5, throttle_scans=1,
        particles=30, maxUrange=8., maxRange=50.)


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False), encoding='utf-8')
    os.replace(str(temporary), str(path))


def grid_snapshot(message, expected_frame):
    h = header_record(message.header)
    info = message.info
    if h['frame_id'] != expected_frame or not stamp_ns(h):
        raise ValueError('GMAPPING map frame/source differs')
    if (type(info.width) is not int or type(info.height) is not int
            or not 1 <= info.width <= 10000 or not 1 <= info.height <= 10000
            or info.width*info.height > 8000000):
        raise ValueError('Invalid new map dimensions')
    resolution = finite(info.resolution, 'resolution')
    origin = info.origin
    q = [finite(getattr(origin.orientation, k), 'origin quaternion') for k in ('x', 'y', 'z', 'w')]
    if resolution <= 0 or abs(sum(v*v for v in q)-1.) > 1e-3 or abs(q[0])+abs(q[1]) > 1e-5:
        raise ValueError('Invalid planar new map metadata')
    values = tuple(message.data)
    if len(values) != info.width*info.height or any(type(x) is not int or not -1 <= x <= 100 for x in values):
        raise ValueError('Invalid new occupancy cells')
    yaw = math.atan2(2*(q[3]*q[2]+q[0]*q[1]), 1-2*(q[1]**2+q[2]**2))
    return MappingProxyType(dict(header=MappingProxyType(h), width=info.width, height=info.height,
        resolution=resolution,
        origin=(finite(origin.position.x, 'origin.x'), finite(origin.position.y, 'origin.y'), yaw),
        data=values, known_cells=sum(x >= 0 for x in values)))


def save_new_map(directory, grid, stem='latest'):
    """Periodic PGM/small metadata; full raw grid only in final after query exit."""
    directory = Path(directory)
    if stem not in ('latest', 'final') or not directory.is_dir():
        raise ValueError('Existing new-map directory and fixed output stem required')
    width, height = grid['width'], grid['height']
    pixels = bytearray()
    for y in range(height-1, -1, -1):
        for value in grid['data'][y*width:(y+1)*width]:
            pixels.append(254 if 0 <= value <= 25 else 0 if value >= 65 else 205)
    pgm = ('P5\n%d %d\n255\n' % (width, height)).encode('ascii') + pixels
    temporary = directory/(stem+'.pgm.tmp')
    temporary.write_bytes(pgm)
    os.replace(str(temporary), str(directory/(stem+'.pgm')))
    x, y, yaw = grid['origin']
    yaml = ('image: %s.pgm\nresolution: %.17g\norigin: [%.17g, %.17g, %.17g]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n' %
            (stem, grid['resolution'], x, y, yaw))
    temporary = directory/(stem+'.yaml.tmp')
    temporary.write_text(yaml, encoding='utf-8')
    os.replace(str(temporary), str(directory/(stem+'.yaml')))
    digest = hashlib.sha256(pgm).hexdigest()
    metadata = dict(header=dict(grid['header']), width=width, height=height,
        resolution=grid['resolution'], origin=list(grid['origin']),
        known_cells=grid['known_cells'], pgm_sha256=digest)
    if stem == 'final':
        serializable = dict(grid, header=dict(grid['header']))
        atomic_json(directory/'final_grid.json', serializable)
    else:
        atomic_json(directory/'latest_metadata.json', metadata)
    return dict(pgm=str(directory/(stem+'.pgm')), yaml=str(directory/(stem+'.yaml')),
        grid=str(directory/'final_grid.json') if stem == 'final' else None,
        metadata=str(directory/'latest_metadata.json') if stem == 'latest' else None, **metadata)


class MainLoopTimings:
    """Bounded RAM timing evidence, independent of sensor/source clocks."""
    def __init__(self, limit=128, clock=time.monotonic):
        self.clock, self.rows = clock, deque(maxlen=limit)
        self.current, self.counts, self.longest = None, {}, {}

    def begin(self, stage, **details):
        self.current = dict(stage=stage, started_monotonic_s=self.clock(), **details)

    def finish(self, **details):
        if self.current is None:
            return
        row = dict(self.current, completed_monotonic_s=self.clock(), **details)
        row['duration_s'] = row['completed_monotonic_s']-row['started_monotonic_s']
        self.rows.append(row)
        stage = row['stage']
        self.counts[stage] = self.counts.get(stage, 0)+1
        if stage not in self.longest or row['duration_s'] > self.longest[stage]['duration_s']:
            self.longest[stage] = row
        self.current = None

    def report(self):
        return dict(recent=list(self.rows), counts=dict(self.counts), longest=dict(self.longest),
                    active=dict(self.current) if self.current else None, limit=self.rows.maxlen)


class LatestMapWriter:
    """One writer, at most one immutable in-flight grid plus one latest pending.

    The ROS loop never waits for disk IO. Failure is latched in RAM. close()
    drains the bounded pending slot and joins; final saving requires its exit.
    """
    def __init__(self, directory, write_map=save_new_map):
        self.directory, self.write_map = Path(directory), write_map
        self._condition = threading.Condition()
        self._pending = None
        self._closing = self._busy = False
        self._failure = self._latest_saved = None
        self._submitted = self._replaced = self._written = 0
        self._thread = threading.Thread(target=self._run, name='manual-map-writer', daemon=True)
        self._thread.start()

    def submit(self, grid):
        # grid_snapshot already detached/froze it. Transfer only a reference;
        # no large copy/serialization is allowed in this ROS-loop handoff.
        with self._condition:
            if self._closing or self._failure is not None:
                raise RuntimeError('Map writer closed or failed; no further submissions')
            self._submitted += 1
            if self._pending is not None:
                self._replaced += 1
            self._pending = grid
            self._condition.notify()

    def _run(self):
        while True:
            with self._condition:
                while self._pending is None and not self._closing:
                    self._condition.wait()
                if self._pending is None:
                    return
                grid, self._pending = self._pending, None
                self._busy = True
            try:
                report = self.write_map(self.directory, grid, 'latest')
            except BaseException as exc:
                with self._condition:
                    if self._failure is None:
                        self._failure = dict(error_type=type(exc).__name__, message=str(exc),
                                             monotonic_s=time.monotonic())
                    self._busy, self._closing, self._pending = False, True, None
                return
            with self._condition:
                self._busy = False
                self._written += 1
                self._latest_saved = report

    def diagnostics(self):
        with self._condition:
            return dict(submitted=self._submitted, pending_replacements=self._replaced,
                written=self._written, in_flight=self._busy, pending=self._pending is not None,
                failure=self._failure, latest_saved_map=self._latest_saved,
                closing=self._closing, thread_still_alive=self._thread.is_alive())

    def close(self, timeout_s=8.):
        with self._condition:
            self._closing = True
            self._condition.notify()
        self._thread.join(timeout=timeout_s)
        return self.diagnostics()


def mapping_outcome(query, observed_gap_count, runtime_clean):
    """A clean recording exit can still contain unintegrated odometry gaps."""
    gaps = max(observed_gap_count, (query.get('metrics') or {}).get('gap_count', 0))
    continuous = bool(gaps == 0 and query.get('odometry_continuity_validated', False))
    ended = bool(runtime_clean and query.get('recording_ended_cleanly', False))
    complete = bool(ended and continuous)
    return dict(gap_count=gaps, odometry_continuity_validated=continuous,
        odom_continuity_validated=continuous, recording_ended_cleanly=ended,
        normal_recording_end=ended, mapping_quality_complete=complete, recording_complete=complete)


def load_dependencies(root):
    manifest = json.loads((root/'manual_mapping_manifest.json').read_text(encoding='utf-8'))
    if manifest.get('schema') != 'manual-new-map-package-v1' or not isinstance(manifest.get('files'), dict):
        raise RuntimeError('Wrong manual mapping package manifest')
    for relative, expected in manifest['files'].items():
        path = (root/relative).resolve()
        if (not path.is_relative_to(root.resolve()) if hasattr(path, 'is_relative_to') else
                os.path.commonpath([str(path), str(root.resolve())]) != str(root.resolve())):
            raise RuntimeError('Manifest escapes new package')
        if (any(part.lower() in ('maps', 'amcl_base', 'seed') for part in Path(relative).parts)
                or (root/relative).is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != expected):
            raise RuntimeError('New mapping package source differs: '+relative)
    modules = {}
    for name, relative in (('api', 'deps/control_client.py'), ('locator', 'deps/rk_link.py'),
                           ('transport', 'deps/transport.py'), ('core', 'deps/core.py')):
        if relative not in manifest['files']:
            raise RuntimeError('Missing bound dependency: '+relative)
        spec = importlib.util.spec_from_file_location('manual_mapping_'+name, str(root/relative))
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        modules[name] = module
    return modules


def stop_own_child(child):
    running = child.poll() is None
    result = dict(was_running=running, sigint_sent=False, forced_kill=False)
    try:
        if running:
            child.send_signal(signal.SIGINT)
            result['sigint_sent'] = True
        result['exit_code'] = child.wait(timeout=6.)
    except subprocess.TimeoutExpired:
        child.kill()
        result['forced_kill'] = True
        result['exit_code'] = child.wait(timeout=2.)
    result['clean'] = not result['forced_kill'] and (result['exit_code'] == 0 or
                                                   running and result['exit_code'] == -signal.SIGINT)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-boot-id', required=True)
    parser.add_argument('--nonce', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=1200.)
    parser.add_argument('--scan-topic', default='/lidar1/laser_fan')
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or not 1 <= args.duration <= 1200:
        raise ValueError('Recording duration must be 1..1200 seconds')
    boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
    if boot != args.expected_boot_id:
        raise RuntimeError('RK boot changed; repeat actual inventory')
    namespace, frames = names_for(args.nonce)
    if not args.scan_topic.startswith('/') or args.scan_topic.startswith(namespace+'/'):
        raise ValueError('Explicit original scan input must differ from private outputs')
    deps = load_dependencies(ROOT)
    from manual_mapping_query import run_manual_mapping_session, make_raw_rpm_odometry
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    if any((output.parent/name).exists() for name in (output.name, 'origin_record.json', 'mapping_status.json', 'new_map')):
        raise RuntimeError('A new unused recording directory is required; no prior outputs overwritten')
    claim = output.parent/'mapping_launch_claim.json'
    with claim.open('x', encoding='utf-8') as stream:
        json.dump(dict(boot_id=boot, nonce=args.nonce, pid=os.getpid(), wall_s=time.time()), stream)
    new_maps = output.parent/'new_map'
    new_maps.mkdir()
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda _sig, _frame: stopped.set())
    rospy.init_node('manual_mapping_'+args.nonce, anonymous=False, disable_signals=True)
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('Real hand-pushed mapping requires actual ROS time')
    publications = dict(rospy.get_published_topics())
    if any(topic.startswith(namespace+'/') for topic in publications):
        raise RuntimeError('Private mapping namespace already has publishers')
    params = gmapping_parameters(frames)
    for key, value in params.items():
        rospy.set_param(namespace+'/slam_gmapping/'+key, value)
    result = dict(schema='manual-new-map-recording-v1', boot_id=boot, nonce=args.nonce,
        namespace=namespace, frames=frames, geometry=GEOMETRY, gmapping_parameters=params,
        wall_started_s=time.time(), recording_complete=False, first_failure=None,
        navigation_enabled=False, motion_permission=False, allowed_serial_request_kinds=[2, 5],
        STOP_ARM_SETPOINT_allowed=False, driver_state_modified=False, old_map_loaded=False,
        pc_disconnect_action='continue_local_recording', geometry_calibrated=False,
        origin_semantics='first_real_sample_at_user_mark; axle_center_estimate; current_heading_plus_x',
        source_time_semantics='original_CAN_source_bridged_once_at_receipt; cropped_first_ray_derived',
        odom_header_seq_semantics='rospy_publisher_sequence; MCU_sample_sequence_in_query_evidence',
        mapping_engineering_source_age_s=.75, original_api_gross_feedback_range_RPM=15,
        manual_push_speed_calibration_claim=False)
    odom_mailbox, scan_mailbox, map_mailbox = deque(), deque(maxlen=1), deque(maxlen=1)
    query_box, origin_box = {}, {}
    mailbox_lock = threading.Lock()
    scan_gate = SourceGate(.75)
    scan_history = deque(maxlen=20)
    timings = MainLoopTimings()
    counters = dict(odom_published=0, scan_published=0, scan_before_origin=0, map_received=0,
                    odom_gap_count=0)
    latest = dict(odom=None, scan=None, map=None)
    query_thread, child, log, map_writer = None, None, None, None
    subscribers, publishers = [], []
    final_grid, final_saved = None, None

    def fail(exc, phase):
        if result['first_failure'] is None:
            result['first_failure'] = dict(phase=phase, error_type=type(exc).__name__,
                message=str(exc), monotonic_s=time.monotonic(), wall_s=time.time())
        stopped.set()

    def stop_requested():
        # The physical query loop calls this frequently: RAM only, no FS/ROS IO.
        return stopped.is_set()

    def odom_sink(observation, packet, start, completed):
        # Sole physical owner: no file writes, ROS publications or services here.
        received = time.monotonic()
        ros_ns = rospy.Time.now().to_nsec()
        entry = dict(observation=dict(observation), packet=dict(packet), request_start=start,
            completed=completed, received_mono=received, received_ros_ns=ros_ns,
            received_wall_s=time.time())
        entry['source_ros_ns'] = source_ros_ns(observation['source_monotonic_s'], received, ros_ns)
        with mailbox_lock:
            if len(odom_mailbox) >= 4096:
                raise RuntimeError('Odometry RAM mailbox overflow; recording ended without control')
            odom_mailbox.append(entry)

    def query_worker():
        try:
            target = deps['locator'].locate()
            query_box['target'] = target
            # Context creation, use and close all belong to this single thread.
            with deps['transport'].exclusive_port(target['port']) as physical:
                query_box['result'] = run_manual_mapping_session(physical, deps['api'],
                    lambda: make_raw_rpm_odometry(deps['core'].MeasuredOdometry,
                        GEOMETRY['wheel_diameter_m'], GEOMETRY['wheel_separation_m'], calibrated=False),
                    boot_id=boot, duration_s=args.duration, feedback_timeout_s=.75,
                    stop_recording=stop_requested, odom_sink=odom_sink)
            query_box['port_closed'] = True
        except BaseException as exc:
            query_box['error'] = dict(type=type(exc).__name__, message=str(exc))
        finally:
            query_box['finished'] = True

    def map_callback(message):
        map_mailbox.append(message)

    def make_tf(parent, child_frame, stamp, x, y, z, yaw):
        message = TransformStamped()
        message.header.frame_id, message.child_frame_id, message.header.stamp = parent, child_frame, stamp
        message.transform.translation.x, message.transform.translation.y, message.transform.translation.z = x, y, z
        message.transform.rotation.z, message.transform.rotation.w = math.sin(yaw/2.), math.cos(yaw/2.)
        return message

    try:
        scan_pub = rospy.Publisher(namespace+'/scan_front180', LaserScan, queue_size=5)
        odom_pub = rospy.Publisher(namespace+'/wheel_odom', Odometry, queue_size=20)
        publishers.extend([scan_pub, odom_pub])
        broadcaster, static = tf2_ros.TransformBroadcaster(), tf2_ros.StaticTransformBroadcaster()
        static.sendTransform(make_tf(frames['base'], frames['laser'], rospy.Time.now(),
            GEOMETRY['laser_x_m'], GEOMETRY['laser_y_m'], GEOMETRY['laser_z_m'], GEOMETRY['laser_yaw_rad']))
        subscribers.append(rospy.Subscriber(args.scan_topic, LaserScan, scan_mailbox.append, queue_size=1))
        subscribers.append(rospy.Subscriber(namespace+'/map', OccupancyGrid, map_callback, queue_size=1))
        log = (output.parent/'gmapping.log').open('x', encoding='utf-8')
        child = subprocess.Popen(['/opt/ros/noetic/lib/gmapping/slam_gmapping',
            '__name:=slam_gmapping', '__ns:='+namespace, 'scan:='+namespace+'/scan_front180',
            'map:='+namespace+'/map', 'map_metadata:='+namespace+'/map_metadata'],
            stdin=subprocess.DEVNULL, stdout=log, stderr=log)
        result['gmapping_pid'] = child.pid
        map_writer = LatestMapWriter(new_maps)
        query_thread = threading.Thread(target=query_worker, name='manual-mapping-query', daemon=True)
        query_thread.start()
        started, status_at, save_at = time.monotonic(), 0., 0.
        while not query_box.get('finished'):
            writer_state = map_writer.diagnostics()
            if writer_state['failure'] is not None:
                raise RuntimeError('Background map writer failed: '+str(writer_state['failure']))
            result['latest_saved_map'] = writer_state['latest_saved_map']
            if (rospy.is_shutdown() or (output.parent/'stop_recording.request').exists()
                    or (output.parent/'cancel.request').exists()):
                stopped.set()
            if child.poll() is not None:
                raise RuntimeError('Own GMAPPING exited during recording: '+str(child.returncode))
            timings.begin('odom_batch_publish')
            with mailbox_lock:
                pending = list(odom_mailbox)
                odom_mailbox.clear()
            timings.current['batch_count'] = len(pending)
            for entry in pending:
                obs, source_ns = entry['observation'], entry['source_ros_ns']
                age = time.monotonic()-obs['source_monotonic_s']
                timings.current.update(checking_source_monotonic_s=obs['source_monotonic_s'],
                                       checking_source_age_s=age)
                if not 0 <= age < .75:
                    raise RuntimeError('Original wheel source expired before ROS publication')
                if obs.get('discontinuity'):
                    counters['odom_gap_count'] += 1
                sec, ns = divmod(source_ns, 1000000000)
                stamp = rospy.Time(sec, ns)
                message = Odometry()
                message.header.frame_id, message.header.stamp = frames['odom'], stamp
                message.child_frame_id = frames['base']
                message.pose.pose.position.x, message.pose.pose.position.y = obs['x'], obs['y']
                message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(obs['yaw']/2.), math.cos(obs['yaw']/2.)
                message.twist.twist.linear.x, message.twist.twist.angular.z = obs['linear_x'], obs['angular_z']
                for index in (0, 7, 14, 21, 28, 35):
                    message.pose.covariance[index] = message.twist.covariance[index] = 1e3
                odom_pub.publish(message)
                broadcaster.sendTransform(make_tf(frames['odom'], frames['base'], stamp,
                    obs['x'], obs['y'], 0., obs['yaw']))
                latest['odom'] = dict(observation=obs, source_ros_ns=source_ns,
                    received_monotonic_s=entry['received_mono'], header=header_record(message.header))
                counters['odom_published'] += 1
                if not origin_box:
                    if any(obs[key] != 0. for key in ('x', 'y', 'yaw')):
                        raise RuntimeError('First observed true sample is not fresh origin0')
                    origin_box.update(schema='manual-wheel-origin-v1', boot_id=boot, nonce=args.nonce,
                        frames=frames, geometry=GEOMETRY, pose=dict(x=0., y=0., yaw=0.),
                        source_ros_ns=source_ns, source_monotonic_s=obs['source_monotonic_s'],
                        received_monotonic_s=entry['received_mono'], received_wall_s=entry['received_wall_s'],
                        sample_sequence=obs['sample_sequence'], raw_packet=entry['packet'],
                        physical_mark_is_user_supplied=True, encoders_reset=False,
                        previous_unrecorded_route_reconstructed=False)
                    atomic_json(output.parent/'origin_record.json', origin_box)
            timings.finish()
            if scan_mailbox:
                raw = scan_mailbox.popleft()
                raw_header = header_record(raw.header)
                if not origin_box:
                    counters['scan_before_origin'] += 1
                elif scan_gate.accept(raw_header, rospy.Time.now().to_sec()):
                    cropped, diag = crop_front_scan(raw, frames['laser'])
                    if stamp_ns(diag['output_header']) < origin_box['source_ros_ns']:
                        counters['scan_before_origin'] += 1
                    elif stamp_ns(diag['output_header'])/1e9 > rospy.Time.now().to_sec():
                        raise RuntimeError('Derived scan source is in the future; no restamping')
                    else:
                        scan_pub.publish(cropped)
                        diag['received_monotonic_s'] = time.monotonic()
                        latest['scan'] = diag
                        scan_history.append(diag)
                        counters['scan_published'] += 1
            if (latest['scan'] is not None and not stopped.is_set()
                    and not source_current(latest['scan']['output_header'], rospy.Time.now().to_sec())):
                raise RuntimeError('Original laser source expired; saving partial map and ending recording without control')
            if map_mailbox:
                timings.begin('grid_snapshot')
                final_grid = grid_snapshot(map_mailbox.popleft(), frames['map'])
                timings.finish(cells=final_grid['width']*final_grid['height'])
                counters['map_received'] += 1
                latest['map'] = {k: final_grid[k] for k in ('header', 'width', 'height', 'resolution', 'known_cells')}
                latest['map']['header'] = dict(final_grid['header'])
            at = time.monotonic()
            if final_grid is not None and at >= save_at:
                timings.begin('submit_periodic_map')
                map_writer.submit(final_grid)
                timings.finish()
                save_at = at+5.
            if at >= status_at:
                timings.begin('status_write')
                odom_age = at-latest['odom']['observation']['source_monotonic_s'] if latest['odom'] else None
                scan_age = rospy.Time.now().to_sec()-stamp_ns(latest['scan']['output_header'])/1e9 if latest['scan'] else None
                ready = bool(origin_box and final_grid and final_grid['known_cells'] > 0 and
                    odom_age is not None and 0 <= odom_age < .75 and scan_age is not None and 0 <= scan_age < .75
                    and not stopped.is_set())
                atomic_json(output.parent/'mapping_status.json', dict(schema='manual-mapping-status-v1',
                    boot_id=boot, nonce=args.nonce, phase='recording' if ready else 'warming_or_ending',
                    ready_to_push=ready, recording_active=not stopped.is_set(),
                    monotonic_s=at, elapsed_s=at-started, origin_recorded=bool(origin_box),
                    latest=latest, odom_source_age_s=odom_age, scan_source_age_s=scan_age,
                    counters=counters, gmapping_pid=child.pid, motion_permission=False,
                    gap_count=counters['odom_gap_count'],
                    odom_continuity_validated=counters['odom_gap_count'] == 0,
                    odometry_continuity_validated=counters['odom_gap_count'] == 0,
                    continuity_scope='observed_samples_so_far; no_gap_displacement_invented',
                    stop_recording_only=True, first_failure=result['first_failure']))
                timings.finish()
                status_at = at+.5
            if at-started > args.duration+3.:
                raise RuntimeError('Query thread exceeded bounded recording/drain duration')
            time.sleep(.01)
    except BaseException as exc:
        timings.finish(error_type=type(exc).__name__, message=str(exc))
        result['main_loop_timings_at_failure'] = timings.report()
        fail(exc, 'recording_runtime')
    finally:
        stopped.set()
        if query_thread is not None:
            query_thread.join(timeout=2.)
            result['query_thread_still_alive'] = query_thread.is_alive()
        else:
            result['query_thread_still_alive'] = False
        result['query'] = query_box.get('result')
        result['target'] = query_box.get('target')
        result['exclusive_port_closed_normally'] = query_box.get('port_closed', False)
        query_cleanup_verified = bool(result['exclusive_port_closed_normally'] and not result['query_thread_still_alive'])
        if query_thread is not None and not query_cleanup_verified:
            fail(RuntimeError('Query thread/port cleanup not verified; final map save deferred'), 'query_cleanup')
        if query_box.get('error'):
            fail(RuntimeError(str(query_box['error'])), 'query_worker')
        query_failure = (result.get('query') or {}).get('first_failure')
        if result['first_failure'] is None and query_failure:
            result['first_failure'] = dict(query_failure, component='manual_mapping_query')
        writer_closed = True
        if map_writer is not None:
            result['map_writer'] = map_writer.close(timeout_s=8.)
            writer_closed = not result['map_writer']['thread_still_alive']
            result['latest_saved_map'] = result['map_writer']['latest_saved_map']
            if not writer_closed or result['map_writer']['failure'] is not None:
                fail(RuntimeError(str(result['map_writer'])), 'map_writer_cleanup')
        if final_grid is not None and writer_closed and query_cleanup_verified:
            try:
                final_saved = save_new_map(new_maps, final_grid, 'final')
            except Exception as exc:
                fail(exc, 'save_final_map')
        result['final_saved_map'] = final_saved
        if child is not None:
            try:
                result['gmapping_cleanup'] = stop_own_child(child)
            except Exception as exc:
                fail(exc, 'gmapping_cleanup')
        for handle in subscribers+publishers:
            try:
                handle.unregister()
            except Exception as exc:
                fail(exc, 'ROS_unregister')
        try:
            rospy.delete_param(namespace+'/slam_gmapping')
        except KeyError:
            pass
        except Exception as exc:
            fail(exc, 'own_parameter_cleanup')
        if log:
            log.close()
        rospy.signal_shutdown('Hand-pushed recording ended; no base control issued')
    result.update(origin=origin_box or None, latest=latest, counters=counters,
        main_loop_timings=timings.report(),
        scan_source_counts=scan_gate.counts, recent_original_front_scans=list(scan_history),
        wall_ended_s=time.time(), map_saved=bool(final_saved),
        map_geometry_accuracy_verified=False, previous_unrecorded_route_reconstructed=False)
    runtime_clean = bool(final_saved and final_saved['known_cells'] > 0 and result['exclusive_port_closed_normally']
        and not result['query_thread_still_alive'] and (result.get('gmapping_cleanup') or {}).get('clean')
        and not (result.get('map_writer') or {}).get('thread_still_alive', True)
        and not (result.get('map_writer') or {}).get('failure')
        and result['first_failure'] is None)
    result.update(mapping_outcome(result.get('query') or {}, counters['odom_gap_count'], runtime_clean))
    atomic_json(output, result)
    atomic_json(output.parent/'mapping_status.json', dict(schema='manual-mapping-status-v1',
        boot_id=boot, nonce=args.nonce, phase='finished', recording_active=False, ready_to_push=False,
        recording_complete=result['recording_complete'], map_saved=result['map_saved'],
        recording_ended_cleanly=result['recording_ended_cleanly'],
        mapping_quality_complete=result['mapping_quality_complete'], gap_count=result['gap_count'],
        odometry_continuity_validated=result['odometry_continuity_validated'],
        odom_continuity_validated=result['odom_continuity_validated'],
        first_failure=result['first_failure'], origin_recorded=bool(origin_box), latest=latest,
        counters=counters, stop_recording_only=True, motion_permission=False))
    print(json.dumps(dict(output=str(output), recording_complete=result['recording_complete'],
        map_saved=result['map_saved'], first_failure=result['first_failure'], motion_permission=False)))
    return 0 if result['recording_complete'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
