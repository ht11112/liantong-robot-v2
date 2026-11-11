"""Exercise the EKF ROS boundary with real V12 sample history and no devices."""

from types import SimpleNamespace as Record
import sys
import unittest
from unittest.mock import Mock, patch

from test_v11_ros import OFFSET, Stamp, exchange
from v11_ekf import (BODY_FRAME, ESTIMATE_TOPIC, FILTERED_TOPIC, IMU_TOPIC, NODE, NS, ODOM_FRAME,
                     WHEEL_TOPIC, RosEkfOdometry)
from v11_ros import OdomHistory


def vector():
    return Record(x=0., y=0., z=0.)


def odometry():
    return Record(header=Record(), child_frame_id=BODY_FRAME,
                  pose=Record(pose=Record(position=vector(), orientation=Record(x=0., y=0., z=0., w=1.)),
                              covariance=[0.] * 36),
                  twist=Record(twist=Record(linear=vector(), angular=vector()), covariance=[0.] * 36))


def imu():
    return Record(header=Record(), angular_velocity=vector(), angular_velocity_covariance=[0.] * 9,
                  orientation_covariance=[0.] * 9, linear_acceleration_covariance=[0.] * 9)


class V11EkfTests(unittest.TestCase):
    def setUp(self):
        self.now = 10.1
        self.publishers = {}
        self.subscribers = {}

        def publisher(topic, *args, **kwargs):
            self.publishers[topic] = Mock()
            return self.publishers[topic]

        def subscriber(topic, kind, callback, **kwargs):
            self.subscribers[topic] = callback
            return Mock()

        ros_time = Mock(side_effect=lambda secs, nsecs: Stamp(secs*NS+nsecs))
        ros_time.now.side_effect = lambda: self.stamp(self.now)
        self.ros = Record(Time=ros_time, resolve_name=lambda name: name, Publisher=publisher,
                          Subscriber=subscriber, set_param=Mock(), delete_param=Mock())
        self.child = Mock()
        self.child.poll.return_value = 0
        modules = {'rospy': self.ros, 'nav_msgs.msg': Record(Odometry=odometry),
                   'sensor_msgs.msg': Record(Imu=imu)}
        self.start = self.enter_patch('v11_ekf.subprocess.Popen', return_value=self.child)
        self.enter_patch('v11_ekf.ros_odom_history', return_value=(self.stamp(self.now), OdomHistory(OFFSET)))
        self.enter_patch('v11_ekf.time.monotonic_ns', side_effect=lambda: round(self.now*NS))
        with patch.dict(sys.modules, modules):
            self.ekf = RosEkfOdometry('/imu/raw')
        self.addCleanup(self.ekf.close)

    def enter_patch(self, target, **kwargs):
        patcher = patch(target, **kwargs)
        self.addCleanup(patcher.stop)
        return patcher.start()

    def stamp(self, seconds):
        return Stamp(OFFSET + round(seconds*NS))

    def send_gyro(self, seconds, *, frame=BODY_FRAME, variance=.002, rates=(0., 0., .1)):
        message = imu()
        message.header.frame_id, message.header.stamp = frame, self.stamp(seconds)
        message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z = rates
        message.angular_velocity_covariance[8] = variance
        self.subscribers['/imu/raw'](message)
        return message

    def send_estimate(self, seconds, *, orientation=None):
        message = odometry()
        message.header.frame_id, message.header.stamp = ODOM_FRAME, self.stamp(seconds)
        if orientation is not None:
            message.pose.pose.orientation = orientation
        self.subscribers[ESTIMATE_TOPIC](message)
        return message

    def test_node_fuses_only_wheel_twist_and_body_gyro(self):
        name, config = self.ros.set_param.call_args.args
        self.assertEqual(name, NODE)
        self.assertFalse(config['publish_tf'])
        self.assertEqual([i for i, selected in enumerate(config['odom0_config']) if selected], [6, 7, 11])
        self.assertEqual([i for i, selected in enumerate(config['imu0_config']) if selected], [11])
        self.assertIn('odometry/filtered:='+ESTIMATE_TOPIC, self.start.call_args.args[0])
        self.assertTrue(self.ekf.publish(exchange(1, 100, 10.021)))
        wheel = self.publishers[WHEEL_TOPIC].publish.call_args.args[0]
        self.assertEqual(wheel.header.stamp.to_nsec(), self.stamp(10.).to_nsec())
        self.assertTrue(all(wheel.twist.covariance[i] > 0. for i in (0, 7, 35)))
        self.assertAlmostEqual(wheel.twist.covariance[35], .0049)
        self.send_gyro(10.08, frame='different_frame')
        self.send_gyro(10.08, variance=0.)
        self.publishers[IMU_TOPIC].publish.assert_not_called()
        self.send_gyro(10.08)
        gyro = self.publishers[IMU_TOPIC].publish.call_args.args[0]
        self.assertEqual(gyro.header.stamp.to_nsec(), self.stamp(10.08).to_nsec())
        self.assertEqual((gyro.orientation_covariance[0], gyro.linear_acceleration_covariance[0]), (-1., -1.))

    def test_imu_snapshot_uses_measurement_age_and_rejects_duplicates(self):
        self.assertIsNone(self.ekf.imu_snapshot())
        self.send_gyro(10.08, rates=(.2, -.3, 0.))
        self.assertEqual(self.ekf.imu_snapshot(), dict(
            fresh=True, age_ms=20., angular_velocity_rad_s=dict(x=.2, y=-.3, z=0.)))
        self.now = 10.34
        self.send_gyro(10.08)
        self.send_gyro(10.33, rates=(float('nan'), 0., .1))
        self.assertEqual(self.ekf.imu_snapshot(), dict(
            fresh=False, age_ms=260., angular_velocity_rad_s=dict(x=.2, y=-.3, z=0.)))

    def test_estimate_preserves_stamp_and_duplicate_wheel_does_not_keep_it_fresh(self):
        self.ekf.publish(exchange(1, 100, 10.021))
        self.send_gyro(10.08)
        first = self.send_estimate(10.09)
        filtered = self.publishers[FILTERED_TOPIC].publish
        filtered.assert_called_once_with(first)
        self.now = 10.251
        self.send_gyro(10.24)
        bridged = self.send_estimate(10.25)
        filtered.assert_called_with(bridged)
        self.assertTrue(self.ekf.ekf_snapshot()['fresh'])
        self.now = 10.501
        self.assertFalse(self.ekf.publish(exchange(1, 100, 10.481)))
        self.send_gyro(10.49)
        self.send_estimate(10.50)
        self.assertEqual(filtered.call_count, 2)
        self.assertFalse(self.ekf.ekf_snapshot()['fresh'])

    def test_ekf_snapshot_reports_latest_fused_yaw_and_expires_with_inputs(self):
        self.assertIsNone(self.ekf.ekf_snapshot())
        self.ekf.publish(exchange(1, 100, 10.021))
        self.send_gyro(10.08)
        self.send_estimate(10.09, orientation=Record(x=0., y=0., z=-.6, w=.8))
        estimate = self.ekf.ekf_snapshot()
        self.assertTrue(estimate['fresh'])
        self.assertEqual(estimate['age_ms'], 10.)
        self.assertAlmostEqual(estimate['yaw_rad'], -1.2870022176)
        self.now = 10.351
        self.send_gyro(10.34)
        self.assertEqual(self.ekf.ekf_snapshot(), dict(fresh=False, age_ms=261., yaw_rad=None))
        self.assertTrue(self.ekf.imu_snapshot()['fresh'])

    def test_fresh_wheel_and_new_estimate_cannot_mask_stale_imu(self):
        self.ekf.publish(exchange(1, 100, 10.021))
        self.send_gyro(10.08)
        self.send_estimate(10.09)
        filtered = self.publishers[FILTERED_TOPIC].publish
        self.now = 10.341
        self.assertTrue(self.ekf.publish(exchange(2, 250, 10.171)))
        self.send_estimate(10.34)
        self.assertEqual(filtered.call_count, 1)
        self.assertFalse(self.ekf.ekf_snapshot()['fresh'])

    def test_epoch_change_invalidates_until_next_distinct_wheel_and_fresh_estimate(self):
        self.ekf.publish(exchange(1, 100, 10.021))
        self.send_gyro(10.08)
        self.send_estimate(10.09)
        filtered = self.publishers[FILTERED_TOPIC].publish
        self.now = 10.15
        self.assertFalse(self.ekf.publish(exchange(1, 100, 10.121, epoch=2)))
        self.assertFalse(self.ekf.ekf_snapshot()['fresh'])
        self.send_estimate(10.149)
        self.assertEqual(filtered.call_count, 1)
        self.now = 10.23
        self.assertTrue(self.ekf.publish(exchange(2, 150, 10.171, epoch=2)))
        self.send_estimate(10.149)
        self.assertEqual(filtered.call_count, 1)
        self.send_estimate(10.22)
        self.assertEqual(filtered.call_count, 2)
        self.ekf.invalidate_history()
        self.send_estimate(10.229)
        self.assertEqual(filtered.call_count, 2)

    def test_fresh_same_epoch_twist_survives_interpolation_gap(self):
        self.ekf.publish(exchange(1, 100, 10.021))
        self.send_gyro(10.08)
        self.send_estimate(10.09)
        floor = self.ekf.output_floor_ns
        first_segment = self.ekf.segment
        self.now = 10.25
        self.send_gyro(10.24)
        self.assertTrue(self.ekf.publish(exchange(5, 300, 10.221)))
        self.assertGreater(self.ekf.segment, first_segment)
        self.assertEqual(self.ekf.output_floor_ns, floor)
        wheel = self.publishers[WHEEL_TOPIC].publish.call_args.args[0]
        self.assertEqual(wheel.header.stamp.to_nsec(), self.stamp(10.2).to_nsec())
        self.assertEqual(wheel.twist.twist.angular.z, .01)
        self.send_estimate(10.24)
        self.assertTrue(self.ekf.ekf_snapshot()['fresh'])

        self.now = 10.60
        self.send_gyro(10.59)
        self.assertTrue(self.ekf.publish(exchange(6, 650, 10.571)))
        self.assertEqual(self.ekf.output_floor_ns, floor)
        self.send_estimate(10.59)
        self.assertTrue(self.ekf.ekf_snapshot()['fresh'])

    def test_delayed_backlog_neither_erases_nor_refreshes_accepted_wheel(self):
        self.ekf.publish(exchange(1, 100, 10.021))
        self.send_gyro(10.08)
        self.send_estimate(10.09)
        accepted, floor = self.ekf.wheel, self.ekf.output_floor_ns
        self.now = 10.40
        self.send_gyro(10.39)
        self.assertFalse(self.ekf.publish(exchange(2, 150, 10.071)))
        self.assertEqual(self.ekf.wheel, accepted)
        self.assertEqual(self.ekf.output_floor_ns, floor)
        self.send_estimate(10.39)
        self.assertTrue(self.ekf.ekf_snapshot()['fresh'])
        self.now = 10.501
        self.send_gyro(10.49)
        self.assertFalse(self.ekf.publish(exchange(3, 200, 10.121)))
        self.assertEqual(self.ekf.wheel, accepted)
        self.assertEqual(self.ekf.output_floor_ns, floor)
        self.send_estimate(10.50)
        self.assertFalse(self.ekf.ekf_snapshot()['fresh'])
        self.assertEqual(self.publishers[WHEEL_TOPIC].publish.call_count, 1)
        self.assertEqual(self.publishers[FILTERED_TOPIC].publish.call_count, 2)

    def test_true_invalid_backward_and_malformed_sources_clear_wheel(self):
        cases = (exchange(2, 150, 10.071, valid=False),
                 exchange(2, 150, 10.171),
                 exchange(6, 350, 10.271, x=float('nan')),
                 exchange(8, 350, 10.371))
        for index, sample in enumerate(cases):
            with self.subTest(sample=sample):
                self.now = 10.1 + index*.1
                self.assertTrue(self.ekf.publish(exchange(1+index*2, 100+index*100, 10.021+index*.1)))
                self.send_gyro(self.now-.02)
                self.send_estimate(self.now-.01)
                self.assertFalse(self.ekf.publish(sample))
                self.assertIsNone(self.ekf.wheel)
                self.assertFalse(self.ekf.ekf_snapshot()['fresh'])

    def test_boot_change_clears_wheel_despite_a_fresh_measurement(self):
        self.ekf.publish(exchange(1, 100, 10.021))
        self.send_gyro(10.08)
        self.send_estimate(10.09)
        rebooted = exchange(2, 150, 10.071)
        rebooted.status.boot += 1
        self.assertFalse(self.ekf.publish(rebooted))
        self.assertIsNone(self.ekf.wheel)
        self.assertFalse(self.ekf.ekf_snapshot()['fresh'])

    def test_future_source_or_backwards_host_clock_clears_wheel(self):
        self.ekf.publish(exchange(1, 100, 10.021))
        self.send_gyro(10.08)
        self.send_estimate(10.09)
        self.assertFalse(self.ekf.publish(exchange(2, 150, 10.171)))
        self.assertIsNone(self.ekf.wheel)
        self.assertFalse(self.ekf.ekf_snapshot()['fresh'])
        self.now = 10.30
        self.assertTrue(self.ekf.publish(exchange(3, 200, 10.221)))
        with patch('v11_ekf.time.monotonic_ns', return_value=round(10.1*NS)):
            self.assertFalse(self.ekf.publish(exchange(4, 250, 10.271)))
        self.assertIsNone(self.ekf.wheel)

    def test_worker_snapshot_carries_remaining_wheel_budget(self):
        self.ekf.publish(exchange(1, 100, 10.021))
        self.now = 10.49
        self.send_gyro(10.48)
        self.send_estimate(10.48)
        exported = self.ekf.worker_snapshot()
        self.assertTrue(exported['ekf']['fresh'])
        self.assertAlmostEqual(exported['written_mono'], 10.49)
        self.assertAlmostEqual(exported['imu_valid_until_mono'], 10.73)
        self.assertAlmostEqual(exported['ekf_valid_until_mono'], 10.50)
        self.now = 10.501
        self.send_gyro(10.50)
        self.send_estimate(10.50)
        expired = self.ekf.worker_snapshot()
        self.assertFalse(expired['ekf']['fresh'])
        self.assertAlmostEqual(expired['ekf_valid_until_mono'], 10.50)
        self.assertFalse(self.ekf.ekf_snapshot()['fresh'])


if __name__ == '__main__':
    unittest.main()
