"""ROS1 bridge for measured odom, AMCL and the obstacle test."""
from collections import deque
from copy import copy, deepcopy
import math
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

from manual_mapping_runtime import crop_front_scan
from simple_p2p_history import PoseHistory, PoseHistoryError


NS = 1_000_000_000
SOURCE_MAX_AGE_S = .25
OBSTACLE_ODOM_MAX_AGE_S = .5
OBSTACLE_SCAN_MAX_AGE_S = .5
LASER_X_M = .13
MIN_RANGE_M = 1.
AMCL_MAX_AGE_S = .5
AMCL_SCAN_PERIOD_S = .2


# 将 MCU 测量时间转换到 ROS 时间，仅保存用于配对的真实位姿。
class OdomHistory:
    """Align ROS stamps to bounded, contiguous real MCU source samples."""
    def __init__(self, ros_offset_ns, *, max_age_s=SOURCE_MAX_AGE_S):
        if not math.isfinite(max_age_s) or max_age_s <= 0:
            raise ValueError('Invalid odometry history source age')
        self.ros_offset_ns = ros_offset_ns
        self.max_age_s = max_age_s
        self.revision = 0
        self.history = PoseHistory(capacity=128)
        self.last = None
        self.observation_sequence = 0
        self.source_sequences = {}
        self.lock = threading.Lock()

    def invalidate(self):
        with self.lock:
            self.history.invalidate()
            self.source_sequences.clear()
            self.revision += 1

    # 会话、序号和测量间隔决定连续区间；缺口不能靠插值跨越。
    def add(self, exchange):
        status = exchange.status
        with self.lock:
            previous = self.last
            if not status.source_valid:
                self.history.invalidate()
                self.revision += 1
                return None
            identity = status.boot, status.epoch
            same_epoch = previous is not None and identity == (previous.boot, previous.epoch)
            if same_epoch and status.sample_sequence == previous.sample_sequence:
                return None
            sequence_step = ((status.sample_sequence - previous.sample_sequence) & 0xffffffff
                             if same_epoch else 1)
            tick_step = ((status.measurement_tick_ms - previous.measurement_tick_ms) & 0xffffffff
                         if same_epoch else 0)
            if same_epoch and (not 0 < tick_step < 0x80000000 or sequence_step >= 0x80000000):
                self.history.invalidate()
                self.revision += 1
                return None
            # The actual MCU publication occurs during the exchange. Its oldest
            # plausible measurement time subtracts the MCU source age and its
            # 1 ms tick quantization from send time.
            # An ACK for the same sample cannot move that bound forward.
            source = exchange.sent_at - (status.source_age_ms + 1) / 1000
            if not 0 <= exchange.received_at - source < self.max_age_s or source < 0:
                self.history.invalidate()
                self.revision += 1
                return None
            mono_ns = round(source * NS)
            pose = dict(x=status.x, y=status.y, yaw=status.yaw, linear_x=status.v,
                        angular_z=status.w, boot=status.boot, epoch=status.epoch,
                        source_monotonic_s=source, measurement_monotonic_s=mono_ns / NS,
                        sample_sequence=status.sample_sequence,
                        mcu_speed_tick_ms=status.measurement_tick_ms,
                        stamp_ns=self.ros_offset_ns + mono_ns)
            try:
                # V12 publishes accumulated poses. A stationary recovery can
                # first hide a discontinuity at a source sequence step of four.
                # Smaller skips can still bracket a scan with real samples.
                # The history index is local bookkeeping, never a fake source.
                observation = (self.observation_sequence + 1) & 0xffffffff
                gap = tick_step > 250 or sequence_step >= 4
                self.history.push(dict(pose, sample_sequence=observation),
                                  epoch='{}:{}'.format(*identity),
                                  gap=gap)
                if gap or previous is not None and not same_epoch:
                    self.revision += 1
            except PoseHistoryError:
                self.history.invalidate()
                self.revision += 1
                return None
            self.observation_sequence = observation
            self.source_sequences[observation] = status.sample_sequence
            if len(self.source_sequences) > 128:
                del self.source_sequences[next(iter(self.source_sequences))]
            self.last = status
            return pose

    def at(self, stamp_ns, now_mono, *, max_age_s=None):
        if max_age_s is None:
            max_age_s = self.max_age_s
        with self.lock:
            latest = self.history.latest(now_mono, max_age_s)
            pose = self.history.sample_at((stamp_ns - self.ros_offset_ns) / NS)
            if not 0 <= now_mono - pose['source_monotonic_s'] < max_age_s:
                raise PoseHistoryError('Scan stamp has no fresh odometry bracket')
            for key in ('sample_sequence', 'lower_sample_sequence', 'upper_sample_sequence'):
                if pose.get(key) is not None:
                    pose[key] = self.source_sequences[pose[key]]
            return dict(pose, stamp_ns=stamp_ns, boot=latest['boot'], epoch=latest['epoch'])

    def diagnostics(self, now_mono):
        with self.lock:
            result = self.history.summary()
            sequence = result['newest_sample_sequence']
            result['newest_sample_sequence'] = self.source_sequences.get(sequence)
            source = result['newest_source_monotonic_s']
            result['source_age_ms'] = None if source is None else (now_mono-source)*1000
            for edge in ('oldest', 'newest'):
                value = result[edge + '_measurement_monotonic_s']
                result[edge + '_stamp_ns'] = (None if value is None else
                                            self.ros_offset_ns + round(value*NS))
            return result


# 按雷达安装姿态和有效距离过滤前方扫描，保留原始时间戳。
def front_scan(raw, laser_frame, laser_yaw, min_range_m):
    # The shared crop preserves the actual time of the first retained ray.
    if laser_yaw:
        raw = copy(raw)
        raw.angle_min += laser_yaw
        raw.angle_max += laser_yaw
    scan, _ = crop_front_scan(raw, laser_frame)
    scan.angle_min -= laser_yaw
    scan.angle_max -= laser_yaw
    scan.ranges = [r if r >= min_range_m else math.inf for r in scan.ranges]
    if sum(math.isfinite(r) and scan.range_min < r < scan.range_max for r in scan.ranges) < 10:
        raise ValueError('Not enough measured front-sector ranges for AMCL')
    return scan


# 扫描只与时间两侧的真实里程计样本配对；过期或无配对时不可用于避障。
class ObstacleScans:
    def __init__(self, odom, *, laser_x=LASER_X_M, laser_y=0., laser_yaw=0., min_range_m=MIN_RANGE_M):
        if (not all(math.isfinite(value) for value in (laser_x, laser_y, laser_yaw, min_range_m))
                or min_range_m < 0.):
            raise ValueError('Invalid obstacle laser mounting or minimum range')
        self.odom = odom
        self.laser_x, self.laser_y, self.laser_yaw = laser_x, laser_y, laser_yaw
        self.min_range_m = min_range_m
        # A newer scan can arrive before its upper odometry bracket.
        self.frames = deque(maxlen=4)
        self.lock = threading.Lock()
        self.received = None
        self.pairing = {}
        self.last_usable_stamp_ns = None

    def add(self, raw, now_mono):
        original = raw
        if self.laser_yaw:
            raw = copy(raw)
            raw.angle_min += self.laser_yaw
            raw.angle_max += self.laser_yaw
        scan, diagnostic = crop_front_scan(raw, raw.header.frame_id)
        stamp = scan.header.stamp.to_nsec()
        source = (stamp - self.odom.ros_offset_ns) / NS
        age = now_mono - source
        with self.lock:
            self.received = dict(stamp_ns=stamp, age_at_receive_ms=age*1000,
                                 rejection=('scan timestamp is in the future' if age < 0 else
                                            'scan expired on receipt' if age >= OBSTACLE_SCAN_MAX_AGE_S else None))
        if not 0 <= age < OBSTACLE_SCAN_MAX_AGE_S:
            return
        points, clear_rays = [], []
        minimum = max(self.min_range_m, scan.range_min)
        # Cropping maps bad rays to +inf for SLAM. Inspect original values so
        # NaN/negative rays and real no-return rays remain distinct here.
        selected = original.ranges[diagnostic['first_raw_index']:diagnostic['end_raw_index_exclusive']]
        for index, distance in enumerate(selected):
            if minimum <= distance < scan.range_max:
                angle = scan.angle_min + index * scan.angle_increment
                c, s = math.cos(angle), math.sin(angle)
                end = self.laser_x + distance*c, self.laser_y + distance*s
                points.append(end)
                clear_rays.append((self.laser_x + minimum*c, self.laser_y + minimum*s, *end))
        observation = dict(points=tuple(points), clear_rays=tuple(clear_rays), stamp_ns=stamp,
                           source_mono=source, expires_mono=source+OBSTACLE_SCAN_MAX_AGE_S,
                           usable=bool(points), finite_count=diagnostic['finite_valid_rays'],
                           invalid_count=diagnostic['invalid_rays_skipped'],
                           no_return_count=diagnostic['positive_infinite_rays'])
        with self.lock:
            if not self.frames or stamp > self.frames[-1][0]:
                self.frames.append((stamp, observation))

    def latest(self, now_mono):
        observation = self.latest_observation(now_mono)
        return ((observation['pose'], observation['points']) if observation is not None else None)

    def latest_observation(self, now_mono):
        with self.lock:
            frames = tuple(self.frames)
            received = self.received
        failure = None
        for stamp, observation in reversed(frames):
            if now_mono >= observation['expires_mono']:
                if failure is None:
                    failure = dict(stamp_ns=stamp, reason='scan expired before pairing')
                continue
            try:
                pose = self.odom.at(stamp, now_mono, max_age_s=OBSTACLE_ODOM_MAX_AGE_S)
            except PoseHistoryError as exc:
                if failure is None:
                    failure = dict(stamp_ns=stamp, reason=str(exc))
                continue
            reason = (None if observation['usable'] else
                      'filtered_ranges' if observation['finite_count'] else
                      'invalid_ranges' if observation['invalid_count'] else
                      'no_return' if observation['no_return_count'] else 'no_usable_ranges')
            with self.lock:
                self.pairing = dict(checked_monotonic_s=now_mono, paired=True,
                                    usable=observation['usable'], failure_reason=reason,
                                    scan_stamp_ns=stamp,
                                    lower_sample_sequence=pose.get('lower_sample_sequence', pose['sample_sequence']),
                                    upper_sample_sequence=pose.get('upper_sample_sequence', pose['sample_sequence']))
                if observation['usable']:
                    self.last_usable_stamp_ns = stamp
            return dict(observation, pose=pose)
        with self.lock:
            self.pairing = dict(checked_monotonic_s=now_mono, paired=False, usable=False,
                                failure_reason=(failure['reason'] if failure else
                                                received['rejection'] or 'no accepted scan' if received else
                                                'no scan received'),
                                scan_stamp_ns=failure['stamp_ns'] if failure else
                                              received['stamp_ns'] if received else None,
                                lower_sample_sequence=None, upper_sample_sequence=None)
        return None

    def diagnostics(self, now_mono):
        self.latest_observation(now_mono)
        with self.lock:
            result = dict(self.pairing, latest_received=None if self.received is None else dict(self.received),
                          last_usable_stamp_ns=self.last_usable_stamp_ns)
        stamp = result['scan_stamp_ns']
        result['scan_age_ms'] = None if stamp is None else (now_mono - (stamp-self.odom.ros_offset_ns)/NS)*1000
        result['odom'] = self.odom.diagnostics(now_mono)
        return result


# 建立 ROS 时间和单调时钟的偏移；此桥要求真实时间。
def ros_odom_history(rospy, *, max_age_s=SOURCE_MAX_AGE_S):
    if not rospy.core.is_initialized():
        rospy.init_node('v11_rk_bridge', anonymous=True, disable_signals=True)
    if rospy.get_param('/use_sim_time', False):
        raise RuntimeError('V11 ROS bridge requires real ROS time')
    before = time.monotonic_ns()
    ros_now = rospy.Time.now()
    offset_ns = ros_now.to_nsec() - (before + time.monotonic_ns()) // 2
    return ros_now, OdomHistory(offset_ns, max_age_s=max_age_s)


# 雷达订阅缓存观测，串口反馈另行写入同一里程计历史。
class RosObstacleScanner:
    """Continuously pair real front-sector returns with measured odometry."""
    def __init__(self, scan_topic='/lidar1/laser_fan', *, laser_x=LASER_X_M, laser_y=0.,
                 laser_yaw=0., min_range_m=MIN_RANGE_M):
        import rospy
        from sensor_msgs.msg import LaserScan

        _, odom = ros_odom_history(rospy)
        self.scans = ObstacleScans(odom, laser_x=laser_x, laser_y=laser_y,
                                   laser_yaw=laser_yaw, min_range_m=min_range_m)
        self.subscriber = rospy.Subscriber(scan_topic, LaserScan, self._received,
                                           queue_size=1, tcp_nodelay=True)

    def _received(self, scan):
        try:
            self.scans.add(scan, time.monotonic())
        except ValueError as exc:
            with self.scans.lock:
                self.scans.received = dict(stamp_ns=None, age_at_receive_ms=None, rejection=str(exc))

    def publish(self, exchange):
        self.scans.odom.add(exchange)

    def invalidate_history(self):
        self.scans.odom.invalidate()

    def latest(self):
        return self.scans.latest(time.monotonic())

    def latest_observation(self):
        return self.scans.latest_observation(time.monotonic())

    def diagnostics(self):
        return self.scans.diagnostics(time.monotonic())

    def close(self):
        self.subscriber.unregister()


def amcl_pose(message):
    pose = message.pose.pose
    q = pose.orientation
    coordinates = (pose.position.x, pose.position.y, q.x, q.y, q.z, q.w)
    covariance = list(message.pose.covariance)
    if (not all(math.isfinite(value) for value in coordinates + tuple(covariance))
            or abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.) > .001
            or any(covariance[index] < 0 for index in (0, 7, 35))):
        raise ValueError('Invalid AMCL pose or covariance')
    yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    return dict(x=pose.position.x, y=pose.position.y, yaw=yaw,
                stamp_ns=message.header.stamp.to_nsec(), covariance=covariance)


def stop_child(child):
    if child.poll() is None:
        child.send_signal(signal.SIGINT)
        try:
            child.wait(timeout=2.)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait(timeout=2.)


def _map_odom_alignment(map_pose, odom_pose):
    yaw = math.atan2(math.sin(map_pose['yaw']-odom_pose['yaw']),
                     math.cos(map_pose['yaw']-odom_pose['yaw']))
    c, s = math.cos(yaw), math.sin(yaw)
    return dict(x=map_pose['x']-c*odom_pose['x']+s*odom_pose['y'],
                y=map_pose['y']-s*odom_pose['x']-c*odom_pose['y'], yaw=yaw)


class _AmclCandidates:
    """Convergence concerns map→odom, so real vehicle motion is not pose jitter."""
    def __init__(self):
        self.rows = deque(maxlen=3)
        self.last_stamp = None
        self.reason = 'Waiting for three distinct AMCL observations'

    def add(self, map_pose, odom_pose, source, now):
        stamp = map_pose['stamp_ns']
        if self.last_stamp is not None and stamp <= self.last_stamp:
            return None
        self.last_stamp = stamp
        covariance = map_pose['covariance']
        if (covariance[0] > .10**2 or covariance[7] > .10**2
                or covariance[35] > math.radians(5.)**2):
            self.rows.clear()
            self.reason = 'AMCL uncertainty exceeds 0.10 m / 5 degrees'
            return None
        # History continuity uses source times; transport latency is checked separately.
        if self.rows and source-self.rows[-1][0] >= AMCL_MAX_AGE_S:
            self.rows.clear()
        alignment = _map_odom_alignment(map_pose, odom_pose)
        self.rows.append((source, alignment))
        if len(self.rows) < 3:
            self.reason = 'Waiting for three distinct AMCL observations'
            return None
        for _, a in self.rows:
            for _, b in self.rows:
                angle = math.atan2(math.sin(a['yaw']-b['yaw']), math.cos(a['yaw']-b['yaw']))
                if math.hypot(a['x']-b['x'], a['y']-b['y']) > .05 or abs(angle) > math.radians(3.):
                    self.reason = 'AMCL alignment has not stabilized within 0.05 m / 3 degrees'
                    return None
        self.reason = None
        return alignment


# AMCL 是可选定位流程；手动对齐模式不依赖它。
class RosLocalizer:
    """Create on RK's main thread; poll odom and localize from separate workers.

    AMCL does not own map→odom. The returned poses share one exact scan stamp;
    the application can calculate and retain its own fixed alignment.
    """
    def __init__(self, map_yaml, scan_topic='/lidar1/laser_fan', *, timeout_s=25., continuous=False,
                 laser_x=LASER_X_M, laser_y=0., laser_z=.2, laser_yaw=0., min_range_m=MIN_RANGE_M):
        self.map_yaml = Path(map_yaml).resolve(strict=True)
        if not self.map_yaml.is_file():
            raise ValueError('AMCL requires an existing map YAML file')
        if (not all(math.isfinite(value) for value in
                    (timeout_s, laser_x, laser_y, laser_z, laser_yaw, min_range_m))
                or timeout_s <= 0 or min_range_m < 0.):
            raise ValueError('Invalid localization timing, mounting or minimum range')
        import rospy
        import tf2_ros
        from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan

        self.continuous = continuous
        self.max_age_s = AMCL_MAX_AGE_S if continuous else SOURCE_MAX_AGE_S
        ros_now, self.odom = ros_odom_history(rospy, max_age_s=self.max_age_s)

        self.rospy = rospy
        self.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self.TransformStamped, self.Odometry, self.LaserScan = TransformStamped, Odometry, LaserScan
        self.scan_topic, self.timeout_s = scan_topic, timeout_s
        self.laser_yaw, self.min_range_m = laser_yaw, min_range_m
        self.session_lock = threading.Lock()
        self.tracking_lock = threading.Lock()
        self._session = None
        self._sessions = []
        self._tracking = dict(enabled=False, status='inactive', fresh=False, reason=None, session_id=None,
            map_pose=None, odom_pose=None, alignment=None, covariance=None, sigma_x_m=None, sigma_y_m=None,
            sigma_yaw_rad=None, age_ms=None, source_monotonic_s=None, received_monotonic_s=None)
        self.closed = threading.Event()
        name = 'v11_' + uuid.uuid4().hex[:10]
        self.namespace = '/' + name
        self.frames = {frame: name + '_' + frame for frame in ('map', 'odom', 'base', 'laser')}
        self.odom_pub = rospy.Publisher(self.namespace+'/wheel_odom', Odometry, queue_size=1)
        self.tf = tf2_ros.TransformBroadcaster()
        self.static_tf = tf2_ros.StaticTransformBroadcaster()
        self.static_tf.sendTransform(self._transform(ros_now, 'base', 'laser',
                                                     laser_x, laser_y, laser_z, laser_yaw))

    def _transform(self, stamp, parent, child, x, y, z, yaw):
        transform = self.TransformStamped()
        transform.header.stamp, transform.header.frame_id = stamp, self.frames[parent]
        transform.child_frame_id = self.frames[child]
        t, q = transform.transform.translation, transform.transform.rotation
        t.x, t.y, t.z = x, y, z
        q.z, q.w = math.sin(yaw/2), math.cos(yaw/2)
        return transform

    def publish(self, exchange):
        if self.closed.is_set():
            return False
        revision = self.odom.revision
        pose = self.odom.add(exchange)
        if self.continuous and revision != self.odom.revision:
            self._cancel_tracking('MCU odometry session or continuous history changed', 'odom_fallback')
        if pose is None:
            return False
        stamp = self.rospy.Time(*divmod(pose['stamp_ns'], NS))
        tf = self._transform(stamp, 'odom', 'base', pose['x'], pose['y'], 0., pose['yaw'])
        msg = self.Odometry()
        msg.header.stamp, msg.header.frame_id, msg.child_frame_id = stamp, tf.header.frame_id, tf.child_frame_id
        msg.pose.pose.position.x, msg.pose.pose.position.y = pose['x'], pose['y']
        msg.pose.pose.orientation = tf.transform.rotation
        msg.twist.twist.linear.x, msg.twist.twist.angular.z = pose['linear_x'], pose['angular_z']
        self.tf.sendTransform(tf)
        self.odom_pub.publish(msg)
        return True

    def invalidate_history(self):
        self.odom.invalidate()
        if self.continuous:
            self._cancel_tracking('MCU odometry history invalidated', 'odom_fallback')

    def localize(self, seed, *, cancelled=None):
        x, y, yaw = (seed[key] for key in ('x', 'y', 'yaw'))
        if not all(type(value) in (int, float) and math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError('AMCL seed requires finite x, y and yaw')
        if not self.session_lock.acquire(blocking=False):
            raise RuntimeError('AMCL localization already running')
        try:
            if self.closed.is_set():
                raise RuntimeError('V11 ROS adapter is closed')
            self.stop_tracking()
            session = dict(id=uuid.uuid4().hex, stop=threading.Event(), ready=threading.Event(),
                           result=None, error=None, odom_revision=self.odom.revision)
            worker = threading.Thread(target=self._localize, args=(session, x, y, yaw, cancelled),
                                      name='v12-amcl-tracking')
            session['thread'] = worker
            with self.tracking_lock:
                self._session = session
                self._sessions = [s for s in self._sessions if s['thread'].is_alive()]
                self._sessions.append(session)
                self._tracking = dict.fromkeys(self._tracking)
                self._tracking.update(enabled=self.continuous, status='initializing', fresh=False,
                                      session_id=session['id'])
            worker.start()
            while not session['ready'].wait(.02):
                if cancelled is not None and cancelled():
                    self._cancel_tracking('AMCL initialization cancelled', 'inactive')
            if session['error'] is not None:
                raise session['error']
            if not self.continuous:
                worker.join()
            return session['result']
        finally:
            self.session_lock.release()

    def _active_session(self, session):
        with self.tracking_lock:
            return self._session is session and not session['stop'].is_set() and not self.closed.is_set()

    def _cancel_tracking(self, reason, status):
        with self.tracking_lock:
            session, self._session = self._session, None
            self._tracking.update(enabled=False, status=status, fresh=False, reason=reason)
            if session is not None:
                session['stop'].set()
                if session['result'] is None:
                    session['error'] = RuntimeError(reason)
                session['ready'].set()

    def stop_tracking(self):
        self._cancel_tracking('AMCL tracking stopped', 'inactive')

    def tracking_snapshot(self):
        with self.tracking_lock:
            result = deepcopy(self._tracking)
        source = result['source_monotonic_s']
        if source is not None:
            result['age_ms'] = (time.monotonic()-source)*1000.
            fresh = (0 <= result['age_ms'] < self.max_age_s*1000
                     and time.monotonic() < result['valid_until_monotonic_s'])
            if result['fresh'] and not fresh:
                result.update(fresh=False, status='odom_fallback', reason='AMCL source expired')
        return result

    def _localize(self, session, x, y, yaw, cancelled):
        ros = self.rospy
        namespace = self.namespace+'/loc_'+session['id'][:8]
        after_ns = ros.Time.now().to_nsec()
        deadline = time.monotonic()+self.timeout_s
        scans, poses, relayed = deque(maxlen=1), deque(maxlen=1), deque(maxlen=32)
        children, handles = [], []
        candidates = _AmclCandidates()
        last_sent = last_pose_stamp = None

        def scan_received(raw):
            try:
                scan = front_scan(raw, self.frames['laser'], self.laser_yaw, self.min_range_m)
            except ValueError:
                return
            stamp_ns = scan.header.stamp.to_nsec()
            if (self._active_session(session) and stamp_ns >= after_ns
                    and 0 <= ros.Time.now().to_nsec()-stamp_ns < self.max_age_s*NS):
                scans.append(scan)

        def pose_received(message):
            if self._active_session(session):
                poses.append((message, time.monotonic()))

        try:
            ros.set_param(namespace+'/map_server/frame_id', self.frames['map'])
            ros.set_param(namespace+'/amcl', dict(global_frame_id=self.frames['map'],
                odom_frame_id=self.frames['odom'], base_frame_id=self.frames['base'],
                use_map_topic=True, first_map_only=True, tf_broadcast=False,
                # The first accepted pose must use statistics updated by the scan.
                resample_interval=1,
                laser_model_type='likelihood_field', save_pose_rate=-1.,
                initial_pose_x=float(x), initial_pose_y=float(y), initial_pose_a=float(yaw)))
            nomotion = None
            if self.continuous:
                from std_srvs.srv import Empty
                ros.set_param(namespace+'/amcl/laser_max_beams', 60)
                ros.set_param(namespace+'/amcl/min_particles', 200)
                ros.set_param(namespace+'/amcl/max_particles', 1000)
                nomotion = ros.ServiceProxy(namespace+'/request_nomotion_update', Empty)
            scan_pub = ros.Publisher(namespace+'/scan', self.LaserScan, queue_size=1)
            handles.append(scan_pub)
            handles.append(ros.Subscriber(namespace+'/amcl_pose', self.PoseWithCovarianceStamped,
                                          pose_received, queue_size=1, tcp_nodelay=True))
            handles.append(ros.Subscriber(self.scan_topic, self.LaserScan, scan_received,
                                          queue_size=1, tcp_nodelay=True))
            for name, args in (('map_server', [str(self.map_yaml)]), ('amcl', [])):
                children.append(subprocess.Popen(['/opt/ros/noetic/lib/{0}/{0}'.format(name),
                    *args, '__name:='+name, '__ns:='+namespace], stdin=subprocess.DEVNULL,
                    start_new_session=True))
            while self._active_session(session) and not ros.is_shutdown():
                if session['result'] is None:
                    if cancelled is not None and cancelled():
                        raise RuntimeError('AMCL initialization cancelled')
                    if time.monotonic() >= deadline:
                        raise TimeoutError('No trusted AMCL pose aligned with real odometry in this session')
                if self.continuous and self.odom.revision != session['odom_revision']:
                    raise RuntimeError('MCU odometry session or continuous history changed')
                if any(child.poll() is not None for child in children):
                    raise RuntimeError('AMCL or map_server exited before localization')
                now_ros = ros.Time.now().to_nsec()
                if scans:
                    scan = scans[-1]
                    stamp = scan.header.stamp.to_nsec()
                    now = time.monotonic()
                    if ((not relayed or stamp > relayed[-1]) and 0 <= now_ros-stamp < self.max_age_s*NS
                            and (not self.continuous or last_sent is None or now-last_sent >= AMCL_SCAN_PERIOD_S)):
                        try:
                            self.odom.at(stamp, now)
                        except PoseHistoryError:
                            pass
                        else:
                            if nomotion is not None:
                                try:
                                    ros.wait_for_service(namespace+'/request_nomotion_update', timeout=.02)
                                    nomotion()
                                except Exception:
                                    session['stop'].wait(.02)
                                    continue
                            relayed.append(stamp)
                            scan_pub.publish(scan)
                            last_sent = time.monotonic()
                if poses:
                    message, received = poses[-1]
                    stamp = message.header.stamp.to_nsec()
                    if (message.header.frame_id == self.frames['map'] and stamp in relayed
                            and (last_pose_stamp is None or stamp > last_pose_stamp)
                            and 0 <= now_ros-stamp < self.max_age_s*NS):
                        try:
                            map_pose, odom_pose = amcl_pose(message), self.odom.at(stamp, time.monotonic())
                        except (PoseHistoryError, ValueError):
                            pass
                        else:
                            last_pose_stamp = stamp
                            source = (stamp-self.odom.ros_offset_ns)/NS
                            oldest = min(source, odom_pose['source_monotonic_s'])
                            alignment = (candidates.add(map_pose, odom_pose, oldest, time.monotonic())
                                         if self.continuous else _map_odom_alignment(map_pose, odom_pose))
                            with self.tracking_lock:
                                if (self._session is session and not session['stop'].is_set()
                                        and (not self.continuous or self.odom.revision == session['odom_revision'])):
                                    if alignment is None:
                                        self._tracking.update(status=('initializing' if session['result'] is None
                                                                      else 'odom_fallback'), fresh=False,
                                                              reason=candidates.reason)
                                    else:
                                        covariance = map_pose['covariance']
                                        self._tracking.update(enabled=self.continuous, status='tracking', fresh=True,
                                            reason=None, map_pose=map_pose, odom_pose=odom_pose, alignment=alignment,
                                            covariance=covariance, sigma_x_m=math.sqrt(covariance[0]),
                                            sigma_y_m=math.sqrt(covariance[7]), sigma_yaw_rad=math.sqrt(covariance[35]),
                                            source_monotonic_s=source, received_monotonic_s=received,
                                            valid_until_monotonic_s=oldest+self.max_age_s)
                                        if session['result'] is None:
                                            session['result'] = map_pose, odom_pose
                                            session['ready'].set()
                            if session['result'] is not None and not self.continuous:
                                break
                session['stop'].wait(.02)
            if session['result'] is None and session['error'] is None:
                raise RuntimeError('AMCL localization cancelled')
        except Exception as exc:
            with self.tracking_lock:
                if self._session is session:
                    self._tracking.update(status='odom_fallback', fresh=False, reason=str(exc))
                if session['result'] is None:
                    session['error'] = exc
                session['ready'].set()
        finally:
            for child in reversed(children):
                stop_child(child)
            for handle in reversed(handles):
                handle.unregister()
            try:
                ros.delete_param(namespace)
            except KeyError:
                pass
            with self.tracking_lock:
                if self._session is session and not self.continuous:
                    self._tracking.update(enabled=False, status='inactive', fresh=False)
                    self._session = None

    def close(self):
        self.closed.set()
        self.stop_tracking()
        with self.session_lock:
            for session in self._sessions:
                session['thread'].join()
            self.odom_pub.unregister()
            self.tf.pub_tf.unregister()
            self.static_tf.pub_tf.unregister()
