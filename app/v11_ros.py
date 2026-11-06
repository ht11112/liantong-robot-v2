"""ROS1 bridge for V11 measured odom and a temporary AMCL session."""
from collections import deque
from copy import copy
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


class OdomHistory:
    """Align ROS stamps to bounded, contiguous real MCU source samples."""
    def __init__(self, ros_offset_ns):
        self.ros_offset_ns = ros_offset_ns
        self.history = PoseHistory(capacity=128)
        self.last = None
        self.lock = threading.Lock()

    def add(self, exchange):
        status = exchange.status
        with self.lock:
            previous = self.last
            if not status.source_valid:
                self.history.invalidate()
                return None
            identity = status.boot, status.epoch
            same_epoch = previous is not None and identity == (previous.boot, previous.epoch)
            if same_epoch and status.sample_sequence == previous.sample_sequence:
                return None
            tick_step = ((status.measurement_tick_ms - previous.measurement_tick_ms) & 0xffffffff
                         if same_epoch else 0)
            if same_epoch and not 0 < tick_step < 0x80000000:
                self.history.invalidate()
                return None
            # The actual MCU publication occurs during the exchange. Its oldest
            # plausible measurement time subtracts the MCU source age and its
            # 1 ms tick quantization from send time.
            # An ACK for the same sample cannot move that bound forward.
            source = exchange.sent_at - (status.source_age_ms + 1) / 1000
            if not 0 <= exchange.received_at - source < SOURCE_MAX_AGE_S or source < 0:
                self.history.invalidate()
                return None
            mono_ns = round(source * NS)
            pose = dict(x=status.x, y=status.y, yaw=status.yaw, linear_x=status.v,
                        angular_z=status.w, boot=status.boot, epoch=status.epoch,
                        source_monotonic_s=source, measurement_monotonic_s=mono_ns / NS,
                        sample_sequence=status.sample_sequence,
                        mcu_speed_tick_ms=status.measurement_tick_ms,
                        stamp_ns=self.ros_offset_ns + mono_ns)
            try:
                self.history.push(pose, epoch='{}:{}'.format(*identity), gap=tick_step > 250)
            except PoseHistoryError:
                self.history.invalidate()
                return None
            self.last = status
            return pose

    def at(self, stamp_ns, now_mono):
        with self.lock:
            latest = self.history.latest(now_mono, SOURCE_MAX_AGE_S)
            pose = self.history.sample_at((stamp_ns - self.ros_offset_ns) / NS)
            if not 0 <= now_mono - pose['source_monotonic_s'] < SOURCE_MAX_AGE_S:
                raise PoseHistoryError('AMCL stamp has no fresh odometry bracket')
            return dict(pose, stamp_ns=stamp_ns, boot=latest['boot'], epoch=latest['epoch'])


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


class RosLocalizer:
    """Create on RK's main thread; poll odom and localize from separate workers.

    AMCL does not own map→odom. The returned poses share one exact scan stamp;
    the application can calculate and retain its own fixed alignment.
    """
    def __init__(self, map_yaml, scan_topic='/lidar1/laser_fan', *, timeout_s=25.,
                 laser_x=.13, laser_y=0., laser_z=.2, laser_yaw=0., min_range_m=1.):
        self.map_yaml = Path(map_yaml).resolve(strict=True)
        if not self.map_yaml.is_file():
            raise ValueError('AMCL requires an existing map YAML file')
        if (not all(math.isfinite(value) for value in
                    (timeout_s, laser_x, laser_y, laser_z, laser_yaw, min_range_m))
                or timeout_s <= 0 or min_range_m < 1.):
            raise ValueError('Invalid localization timing, mounting or minimum range')
        import rospy
        import tf2_ros
        from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import LaserScan

        if not rospy.core.is_initialized():
            rospy.init_node('v11_rk_bridge', anonymous=True, disable_signals=True)
        if rospy.get_param('/use_sim_time', False):
            raise RuntimeError('V11 localization requires real ROS time')

        self.rospy = rospy
        self.PoseWithCovarianceStamped = PoseWithCovarianceStamped
        self.TransformStamped, self.Odometry, self.LaserScan = TransformStamped, Odometry, LaserScan
        self.scan_topic, self.timeout_s = scan_topic, timeout_s
        self.laser_yaw, self.min_range_m = laser_yaw, min_range_m
        self.session_lock = threading.Lock()
        self.closed = threading.Event()
        name = 'v11_' + uuid.uuid4().hex[:10]
        self.namespace = '/' + name
        self.frames = {frame: name + '_' + frame for frame in ('map', 'odom', 'base', 'laser')}
        before = time.monotonic_ns()
        ros_now = rospy.Time.now()
        offset_ns = ros_now.to_nsec() - (before + time.monotonic_ns()) // 2
        self.odom = OdomHistory(offset_ns)
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
        pose = self.odom.add(exchange)
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

    def localize(self, seed):
        x, y, yaw = (seed[key] for key in ('x', 'y', 'yaw'))
        if not all(type(value) in (int, float) and math.isfinite(value) for value in (x, y, yaw)):
            raise ValueError('AMCL seed requires finite x, y and yaw')
        if not self.session_lock.acquire(blocking=False):
            raise RuntimeError('AMCL localization already running')
        try:
            if self.closed.is_set():
                raise RuntimeError('V11 ROS adapter is closed')
            return self._localize(x, y, yaw)
        finally:
            self.session_lock.release()

    def _localize(self, x, y, yaw):
        ros = self.rospy
        namespace = self.namespace+'/loc_'+uuid.uuid4().hex[:8]
        after_ns = ros.Time.now().to_nsec()
        deadline = time.monotonic()+self.timeout_s
        scans, poses, relayed = deque(maxlen=1), deque(maxlen=1), deque(maxlen=32)
        children, handles = [], []

        def scan_received(raw):
            stamp_ns = raw.header.stamp.to_nsec()
            if stamp_ns < after_ns or not 0 <= ros.Time.now().to_nsec()-stamp_ns < SOURCE_MAX_AGE_S*NS:
                return
            try:
                scans.append(front_scan(raw, self.frames['laser'], self.laser_yaw, self.min_range_m))
            except ValueError:
                return

        try:
            ros.set_param(namespace+'/map_server/frame_id', self.frames['map'])
            ros.set_param(namespace+'/amcl', dict(global_frame_id=self.frames['map'],
                odom_frame_id=self.frames['odom'], base_frame_id=self.frames['base'],
                use_map_topic=True, first_map_only=True, tf_broadcast=False,
                # The first accepted pose must use statistics updated by the scan.
                resample_interval=1,
                laser_model_type='likelihood_field', save_pose_rate=-1.,
                initial_pose_x=float(x), initial_pose_y=float(y), initial_pose_a=float(yaw)))
            scan_pub = ros.Publisher(namespace+'/scan', self.LaserScan, queue_size=1)
            handles.append(scan_pub)
            handles.append(ros.Subscriber(namespace+'/amcl_pose', self.PoseWithCovarianceStamped,
                                          poses.append, queue_size=1, tcp_nodelay=True))
            handles.append(ros.Subscriber(self.scan_topic, self.LaserScan, scan_received,
                                          queue_size=1, tcp_nodelay=True))
            for name, args in (('map_server', [str(self.map_yaml)]), ('amcl', [])):
                children.append(subprocess.Popen(['/opt/ros/noetic/lib/{0}/{0}'.format(name),
                    *args, '__name:='+name, '__ns:='+namespace], stdin=subprocess.DEVNULL,
                    start_new_session=True))
            while time.monotonic() < deadline and not self.closed.is_set() and not ros.is_shutdown():
                if any(child.poll() is not None for child in children):
                    raise RuntimeError('AMCL or map_server exited before localization')
                now_ros = ros.Time.now().to_nsec()
                if scans:
                    scan = scans[-1]
                    stamp = scan.header.stamp.to_nsec()
                    if (not relayed or stamp > relayed[-1]) and 0 <= now_ros-stamp < SOURCE_MAX_AGE_S*NS:
                        try:
                            self.odom.at(stamp, time.monotonic())
                        except PoseHistoryError:
                            pass
                        else:
                            scan_pub.publish(scan)
                            relayed.append(stamp)
                if poses:
                    message = poses[-1]
                    stamp = message.header.stamp.to_nsec()
                    if (message.header.frame_id == self.frames['map'] and stamp in relayed
                            and 0 <= now_ros-stamp < SOURCE_MAX_AGE_S*NS):
                        try:
                            result = amcl_pose(message), self.odom.at(stamp, time.monotonic())
                        except (PoseHistoryError, ValueError):
                            pass
                        else:
                            if time.monotonic() < deadline and not self.closed.is_set():
                                return result
                self.closed.wait(.02)
            raise TimeoutError('No fresh AMCL pose aligned with real odometry in this session')
        finally:
            for child in reversed(children):
                stop_child(child)
            for handle in reversed(handles):
                handle.unregister()
            try:
                ros.delete_param(namespace)
            except KeyError:
                pass

    def close(self):
        self.closed.set()
        with self.session_lock:
            self.odom_pub.unregister()
            self.tf.pub_tf.unregister()
            self.static_tf.pub_tf.unregister()
