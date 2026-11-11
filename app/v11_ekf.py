"""Optional ROS1 wheel/gyro EKF output, independent of the navigation pose."""

import math
from pathlib import Path
import subprocess
import threading
import time

from v11_ros import NS, SOURCE_MAX_AGE_S, ros_odom_history, stop_child


NAMESPACE = '/v12_ekf'
NODE = NAMESPACE + '/filter'
WHEEL_TOPIC = NAMESPACE + '/wheel_odom'
IMU_TOPIC = NAMESPACE + '/imu'
ESTIMATE_TOPIC = NAMESPACE + '/odometry/estimate'
FILTERED_TOPIC = NAMESPACE + '/odometry/filtered'
ODOM_FRAME = 'v12_ekf_odom'
BODY_FRAME = 'base_link'
# Provisional wheel twist variances for vx, nonholonomic vy, and yaw rate.
WHEEL_TWIST_VARIANCE = {0: .02**2, 7: .01**2, 35: .07**2}
# Navigation allows this much time between real wheel samples. The IMU and
# filter estimate must still meet the shorter SOURCE_MAX_AGE_S bound.
WHEEL_MAX_AGE_S = .5


def _parameters():
    return dict(frequency=30., sensor_timeout=.10, two_d_mode=True, publish_tf=False,
                print_diagnostics=True, use_control=False,
                odom_frame=ODOM_FRAME, world_frame=ODOM_FRAME, base_link_frame=BODY_FRAME,
                odom0=WHEEL_TOPIC, odom0_config=[i in (6, 7, 11) for i in range(15)],
                odom0_queue_size=2, odom0_nodelay=True,
                imu0=IMU_TOPIC, imu0_config=[i == 11 for i in range(15)],
                imu0_queue_size=2, imu0_nodelay=True)


def _fresh(now, stamp, max_age_s=SOURCE_MAX_AGE_S):
    return 0 <= now - stamp <= max_age_s * NS


def _valid_estimate(message):
    if message.header.frame_id != ODOM_FRAME or message.child_frame_id != BODY_FRAME:
        return False
    position = message.pose.pose.position
    q = message.pose.pose.orientation
    twist = message.twist.twist
    quaternion = (q.x, q.y, q.z, q.w)
    return (all(math.isfinite(value) for value in (
        position.x, position.y, position.z, *quaternion,
        twist.linear.x, twist.linear.y, twist.angular.z,
        *message.pose.covariance, *message.twist.covariance))
        and abs(math.hypot(*quaternion) - 1.) < .005)


class RosEkfOdometry:
    """Publish fresh wheel and gyro measurements, and gate the private EKF estimate."""

    def __init__(self, imu_topic, executable=None):
        import rospy
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu

        _, self.odom = ros_odom_history(rospy)
        self.rospy, self.Odometry, self.Imu = rospy, Odometry, Imu
        if not imu_topic or rospy.resolve_name(imu_topic) == IMU_TOPIC:
            raise ValueError('Select the upstream body-frame IMU topic')
        command = (['rosrun', 'robot_localization', 'ekf_localization_node'] if executable is None
                   else [str(Path(executable).resolve(strict=True))])
        self.lock = threading.RLock()
        self.closed = False
        self.segment = 0
        self.wheel = None
        self.imu = None
        self.last_output = None
        self.output_floor_ns = 0
        self.handles = []
        self.child = None
        self.params_loaded = False
        try:
            rospy.set_param(NODE, _parameters())
            self.params_loaded = True
            self.wheel_pub = rospy.Publisher(WHEEL_TOPIC, Odometry, queue_size=2)
            self.handles.append(self.wheel_pub)
            self.imu_pub = rospy.Publisher(IMU_TOPIC, Imu, queue_size=2)
            self.handles.append(self.imu_pub)
            self.filtered_pub = rospy.Publisher(FILTERED_TOPIC, Odometry, queue_size=2)
            self.handles.append(self.filtered_pub)
            self.handles.append(rospy.Subscriber(imu_topic, Imu, self._imu_received,
                                                 queue_size=1, tcp_nodelay=True))
            self.handles.append(rospy.Subscriber(ESTIMATE_TOPIC, Odometry, self._estimate_received,
                                                 queue_size=1, tcp_nodelay=True))
            self.child = subprocess.Popen(command + [
                '__ns:=' + NAMESPACE, '__name:=filter', 'odometry/filtered:=' + ESTIMATE_TOPIC])
        except BaseException:
            self.close()
            raise

    def _clear_wheel(self):
        self.wheel = None
        self.output_floor_ns = self.rospy.Time.now().to_nsec()

    def invalidate_history(self):
        with self.lock:
            self.odom.invalidate()
            self.segment = self.odom.history.summary()['segment']
            self._clear_wheel()

    def publish(self, exchange):
        with self.lock:
            if self.closed:
                return False
            previous = self.odom.last
            pose = self.odom.add(exchange)
            segment = self.odom.history.summary()['segment']
            cut = segment != self.segment
            self.segment = segment
            identity_changed = previous is not None and (
                previous.boot, previous.epoch) != (exchange.status.boot, exchange.status.epoch)
            # A missing interpolation bracket does not invalidate a newly
            # measured twist. Invalid history and MCU identity changes still do.
            if cut and previous is not None and (pose is None or identity_changed):
                self._clear_wheel()
                return False
            if pose is None:
                return False
            stamp_ns = pose['stamp_ns']
            source_ns = round(pose['source_monotonic_s'] * NS)
            now, mono = self.rospy.Time.now().to_nsec(), time.monotonic_ns()
            if stamp_ns > now or source_ns > mono:
                self._clear_wheel()
                return False
            # A delayed cache consumer must not refresh or erase the last
            # accepted source; its original timestamps keep aging normally.
            if not (_fresh(now, stamp_ns) and _fresh(mono, source_ns)):
                return False
            message = self.Odometry()
            message.header.stamp = self.rospy.Time(*divmod(stamp_ns, NS))
            message.header.frame_id, message.child_frame_id = ODOM_FRAME, BODY_FRAME
            message.pose.pose.position.x, message.pose.pose.position.y = pose['x'], pose['y']
            q = message.pose.pose.orientation
            q.z, q.w = math.sin(pose['yaw']/2.), math.cos(pose['yaw']/2.)
            message.twist.twist.linear.x = pose['linear_x']
            message.twist.twist.angular.z = pose['angular_z']
            for index, variance in WHEEL_TWIST_VARIANCE.items():
                message.twist.covariance[index] = variance
            self.wheel_pub.publish(message)
            self.wheel = stamp_ns, source_ns
            return True

    def _imu_received(self, message):
        stamp_ns = message.header.stamp.to_nsec()
        angular = message.angular_velocity
        rates = (angular.x, angular.y, angular.z)
        variance = message.angular_velocity_covariance[8]
        if (message.header.frame_id != BODY_FRAME or stamp_ns <= 0 or not all(map(math.isfinite, rates))
                or not math.isfinite(variance) or variance <= 0.):
            return
        with self.lock:
            if (self.closed or not _fresh(self.rospy.Time.now().to_nsec(), stamp_ns)
                    or self.imu is not None and stamp_ns <= self.imu[0]):
                return
            gyro = self.Imu()
            gyro.header = message.header
            gyro.angular_velocity.z = rates[2]
            gyro.angular_velocity_covariance[8] = variance
            gyro.orientation_covariance[0] = gyro.linear_acceleration_covariance[0] = -1.
            self.imu_pub.publish(gyro)
            self.imu = stamp_ns, time.monotonic_ns(), rates

    def imu_snapshot(self):
        with self.lock:
            if self.imu is None or self.closed:
                return None
            stamp_ns, receipt_ns, rates = self.imu
        source_age = self.rospy.Time.now().to_nsec() - stamp_ns
        receipt_age = time.monotonic_ns() - receipt_ns
        age = max(source_age, receipt_age)
        return dict(fresh=0 <= min(source_age, receipt_age) and age <= SOURCE_MAX_AGE_S * NS,
                    age_ms=age / 1_000_000 if age >= 0 else None,
                    angular_velocity_rad_s=dict(zip('xyz', rates)))

    def _inputs_fresh(self, now, mono):
        return (self.wheel is not None
                and _fresh(now, self.wheel[0], WHEEL_MAX_AGE_S)
                and _fresh(mono, self.wheel[1], WHEEL_MAX_AGE_S)
                and self.imu is not None
                and _fresh(now, self.imu[0])
                and _fresh(mono, self.imu[1]))

    def ekf_snapshot(self):
        with self.lock:
            if self.last_output is None or self.closed:
                return None
            stamp_ns, receipt_ns, yaw_rad = self.last_output
            now, mono = self.rospy.Time.now().to_nsec(), time.monotonic_ns()
            age = max(now - stamp_ns, mono - receipt_ns)
            fresh = (stamp_ns >= self.output_floor_ns and _fresh(now, stamp_ns)
                     and _fresh(mono, receipt_ns) and self._inputs_fresh(now, mono))
            return dict(fresh=fresh, age_ms=age / 1_000_000 if age >= 0 else None,
                        yaw_rad=yaw_rad if fresh else None)

    def worker_snapshot(self):
        # Copy the data and every input deadline together. Parent reads must not
        # extend wheel/IMU validity just because the filter estimate is newer.
        with self.lock:
            now, mono = self.rospy.Time.now().to_nsec(), time.monotonic_ns()

            def deadline(source, limit):
                if source is None:
                    return mono / NS
                return min(mono + limit * NS - (now-source[0]), source[1] + limit * NS) / NS

            imu_deadline = deadline(self.imu, SOURCE_MAX_AGE_S)
            return dict(written_mono=mono / NS, imu=self.imu_snapshot(), ekf=self.ekf_snapshot(),
                        imu_valid_until_mono=imu_deadline,
                        ekf_valid_until_mono=min(imu_deadline, deadline(self.wheel, WHEEL_MAX_AGE_S),
                                                 deadline(self.last_output, SOURCE_MAX_AGE_S)))

    def _estimate_received(self, message):
        if not _valid_estimate(message):
            return
        stamp_ns = message.header.stamp.to_nsec()
        with self.lock:
            if self.closed:
                return
            now, mono = self.rospy.Time.now().to_nsec(), time.monotonic_ns()
            if ((self.last_output is not None and stamp_ns <= self.last_output[0])
                    or stamp_ns < self.output_floor_ns
                    or not _fresh(now, stamp_ns)
                    or not self._inputs_fresh(now, mono)):
                return
            q = message.pose.pose.orientation
            yaw_rad = math.atan2(2. * (q.w*q.z + q.x*q.y), 1. - 2. * (q.y*q.y + q.z*q.z))
            self.filtered_pub.publish(message)
            self.last_output = stamp_ns, mono, yaw_rad

    def close(self):
        with self.lock:
            if self.closed:
                return
            self.closed = True
        try:
            if self.child is not None:
                stop_child(self.child)
        finally:
            for handle in reversed(self.handles):
                handle.unregister()
            if self.params_loaded:
                try:
                    self.rospy.delete_param(NODE)
                except KeyError:
                    pass
