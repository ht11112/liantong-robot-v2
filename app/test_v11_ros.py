"""Pure ROS bridge checks: no ROS master or hardware needed."""
import math
from types import SimpleNamespace as NSObj
import unittest

from v11_ros import NS, OdomHistory, amcl_pose, front_scan
from simple_p2p_history import PoseHistoryError


OFFSET = 1_700_000_000 * NS


def exchange(sequence, tick, sent, *, x=0., yaw=0., epoch=1, valid=True):
    status = NSObj(boot=12, epoch=epoch, sample_sequence=sequence,
                   measurement_tick_ms=tick, source_valid=valid, source_age_ms=20,
                   x=x, y=0., yaw=yaw, v=.03, w=.01)
    return NSObj(status=status, sent_at=sent, received_at=sent+.01)


class Stamp:
    def __init__(self, stamp_ns):
        self.secs, self.nsecs = divmod(stamp_ns, NS)

    def to_nsec(self):
        return self.secs*NS + self.nsecs


class V11RosTests(unittest.TestCase):
    def test_same_stamp_uses_real_odom_bracket_and_short_arc(self):
        odom = OdomHistory(OFFSET)
        first = odom.add(exchange(1, 100, 10.021, x=1., yaw=math.radians(179)))
        second = odom.add(exchange(2, 150, 10.071, x=3., yaw=math.radians(-179)))
        self.assertEqual(first['stamp_ns'], OFFSET + 10*NS)
        stamp = OFFSET + 10*NS + 25_000_000
        matched = odom.at(stamp, 10.1)
        self.assertEqual(matched['stamp_ns'], stamp)
        self.assertAlmostEqual(matched['x'], 2.)
        self.assertAlmostEqual(abs(matched['yaw']), math.pi)
        self.assertEqual(matched['source_monotonic_s'], 10.)
        self.assertEqual((matched['lower_sample_sequence'], matched['upper_sample_sequence']), (1, 2))
        self.assertEqual(odom.at(second['stamp_ns'], 10.1), second)
        with self.assertRaises(PoseHistoryError):
            odom.at(second['stamp_ns']+1, 10.1)

    def test_new_reply_for_same_sample_does_not_renew_source(self):
        odom = OdomHistory(OFFSET)
        first = odom.add(exchange(1, 100, 10.021))
        self.assertIsNone(odom.add(exchange(1, 100, 10.24)))
        self.assertEqual(odom.at(first['stamp_ns'], 10.249), first)
        with self.assertRaises(PoseHistoryError):
            odom.at(first['stamp_ns'], 10.25)

    def test_invalid_source_missing_sequence_or_new_epoch_cuts_bracket(self):
        for cause in ('invalid', 'sequence', 'tick', 'epoch'):
            with self.subTest(cause=cause):
                odom = OdomHistory(OFFSET)
                first = odom.add(exchange(1, 100, 10.02))
                if cause == 'invalid':
                    self.assertIsNone(odom.add(exchange(1, 100, 10.03, valid=False)))
                sample = odom.add(exchange(3 if cause == 'sequence' else 2,
                    400 if cause == 'tick' else 150, 10.07, epoch=2 if cause == 'epoch' else 1))
                self.assertEqual(odom.at(sample['stamp_ns'], 10.1)['epoch'], sample['epoch'])
                with self.assertRaises(PoseHistoryError):
                    odom.at(first['stamp_ns']+1, 10.1)

    def test_backwards_mcu_tick_and_source_time_are_not_restamped(self):
        for tick, sent in ((99, 10.07), (150, 10.01)):
            with self.subTest(tick=tick, sent=sent):
                odom = OdomHistory(OFFSET)
                first = odom.add(exchange(1, 100, 10.02))
                self.assertIsNone(odom.add(exchange(2, tick, sent)))
                with self.assertRaises(PoseHistoryError):
                    odom.at(first['stamp_ns'], 10.1)

    def test_front_scan_masks_below_one_metre_and_keeps_actual_first_ray_stamp(self):
        raw = NSObj(header=NSObj(frame_id='original', seq=5, stamp=Stamp(OFFSET)),
                    angle_min=-math.pi, angle_max=math.pi, angle_increment=math.pi/360,
                    range_min=.1, range_max=50., time_increment=.0001,
                    intensities=[], ranges=[2.]*721)
        raw.ranges[360:363] = [.999, 1., float('nan')]
        scan = front_scan(raw, 'v11_laser', 0., 1.)
        self.assertEqual(scan.header.frame_id, 'v11_laser')
        self.assertEqual(scan.header.stamp.to_nsec(), OFFSET + 18_000_000)
        self.assertAlmostEqual(scan.angle_min, -math.pi/2)
        self.assertAlmostEqual(scan.angle_max, math.pi/2)
        self.assertEqual(len(scan.ranges), 361)
        self.assertEqual(scan.ranges[180:183], [math.inf, 1., math.inf])
        self.assertEqual(raw.ranges[360], .999)
        self.assertEqual(raw.header.stamp.to_nsec(), OFFSET)
        raw.ranges = [.9]*721
        with self.assertRaises(ValueError):
            front_scan(raw, 'v11_laser', 0., 1.)

    def test_amcl_yaw_and_exact_observation_stamp(self):
        message = NSObj(header=NSObj(stamp=Stamp(OFFSET+23)),
            pose=NSObj(pose=NSObj(position=NSObj(x=1., y=2.), orientation=NSObj(
                x=0., y=0., z=math.sin(-.8/2), w=math.cos(-.8/2))), covariance=[0.]*36))
        pose = amcl_pose(message)
        self.assertAlmostEqual(pose['yaw'], -.8)
        self.assertEqual(pose['stamp_ns'], OFFSET+23)
        message.pose.covariance[35] = -1.
        with self.assertRaises(ValueError):
            amcl_pose(message)


if __name__ == '__main__':
    unittest.main()
