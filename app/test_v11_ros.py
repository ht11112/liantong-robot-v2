"""Pure ROS bridge checks: no ROS master or hardware needed."""
import json
import math
from types import SimpleNamespace as NSObj
import unittest

from v11_ros import NS, ObstacleScans, OdomHistory, amcl_pose, front_scan
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


def obstacle_scan(first_ray_s, ranges):
    return NSObj(header=NSObj(frame_id='original', seq=1, stamp=Stamp(OFFSET + round(first_ray_s*NS))),
                 angle_min=-math.pi, angle_max=math.pi, angle_increment=math.pi/4,
                 range_min=.1, range_max=50., time_increment=.005, intensities=[], ranges=ranges)


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

    def test_consumer_overrun_invalidates_interpolation_until_new_real_bracket(self):
        odom = OdomHistory(OFFSET)
        first = odom.add(exchange(1, 100, 10.021))
        odom.invalidate()
        self.assertIsNone(odom.add(exchange(1, 100, 10.041)))
        with self.assertRaises(PoseHistoryError):
            odom.at(first['stamp_ns'], 10.05)
        second = odom.add(exchange(2, 150, 10.071))
        with self.assertRaises(PoseHistoryError):
            odom.at(first['stamp_ns']+25_000_000, 10.1)
        third = odom.add(exchange(3, 200, 10.121))
        paired = odom.at(second['stamp_ns']+25_000_000, 10.15)
        self.assertEqual((paired['lower_sample_sequence'], paired['upper_sample_sequence']), (2, 3))
        self.assertEqual(odom.at(third['stamp_ns'], 10.15), third)

    def test_invalid_source_large_sequence_gap_or_new_epoch_cuts_bracket(self):
        for cause in ('invalid', 'sequence', 'hidden_recovery', 'tick', 'epoch'):
            with self.subTest(cause=cause):
                odom = OdomHistory(OFFSET)
                first = odom.add(exchange(1, 100, 10.02))
                if cause == 'invalid':
                    self.assertIsNone(odom.add(exchange(1, 100, 10.03, valid=False)))
                sequence = 6 if cause == 'sequence' else 5 if cause == 'hidden_recovery' else 2
                sample = odom.add(exchange(sequence,
                    400 if cause == 'tick' else 150, 10.07, epoch=2 if cause == 'epoch' else 1))
                self.assertEqual(odom.at(sample['stamp_ns'], 10.1)['epoch'], sample['epoch'])
                with self.assertRaises(PoseHistoryError):
                    odom.at(first['stamp_ns']+1, 10.1)

    def test_skipped_queries_pair_accumulated_poses_and_preserve_source_ids(self):
        for step in (2, 3):
            with self.subTest(step=step):
                odom = OdomHistory(OFFSET)
                odom.add(exchange(10, 100, 10.021, x=1.))
                tick = 100 + 50 * step
                sent = 10.021 + .05 * step
                second = odom.add(exchange(10 + step, tick, sent, x=3.))
                matched = odom.at(OFFSET + 10*NS + 25_000_000*step, sent + .03)
                self.assertAlmostEqual(matched['x'], 2.)
                self.assertIsNone(matched['sample_sequence'])
                self.assertEqual((matched['lower_sample_sequence'], matched['upper_sample_sequence']),
                                 (10, 10 + step))
                self.assertEqual(odom.at(second['stamp_ns'], sent + .03), second)
                self.assertIsNone(odom.add(exchange(10 + step, tick, sent + .04)))

    def test_query_skip_does_not_bridge_invalid_or_long_time_gap(self):
        for invalid in (False, True):
            with self.subTest(invalid=invalid):
                odom = OdomHistory(OFFSET)
                first = odom.add(exchange(1, 100, 10.021))
                if invalid:
                    odom.add(exchange(2, 150, 10.071, valid=False))
                sample = odom.add(exchange(3, 200 if invalid else 351, 10.121))
                with self.assertRaises(PoseHistoryError):
                    odom.at(first['stamp_ns']+1, 10.15)
                self.assertEqual(odom.at(sample['stamp_ns'], 10.15), sample)

    def test_source_sequence_wrap_and_bounded_lookup(self):
        odom = OdomHistory(OFFSET)
        odom.add(exchange(0xfffffffe, 100, 10.021))
        sample = odom.add(exchange(0, 200, 10.121))
        matched = odom.at(OFFSET + 10*NS + 50_000_000, 10.15)
        self.assertEqual((matched['lower_sample_sequence'], matched['upper_sample_sequence']), (0xfffffffe, 0))
        for index in range(1, 150):
            sample = odom.add(exchange(index, 200 + index*50, 10.121 + index*.05))
        self.assertLessEqual(len(odom.source_sequences), 128)
        self.assertEqual(odom.at(sample['stamp_ns'], 17.6), sample)

    def test_backwards_mcu_tick_and_source_time_are_not_restamped(self):
        for tick, sent in ((99, 10.07), (150, 10.01)):
            with self.subTest(tick=tick, sent=sent):
                odom = OdomHistory(OFFSET)
                first = odom.add(exchange(1, 100, 10.02))
                self.assertIsNone(odom.add(exchange(2, tick, sent)))
                with self.assertRaises(PoseHistoryError):
                    odom.at(first['stamp_ns'], 10.1)

    def test_backwards_source_sequence_is_not_hidden_by_local_history_index(self):
        odom = OdomHistory(OFFSET)
        first = odom.add(exchange(10, 100, 10.021))
        self.assertIsNone(odom.add(exchange(9, 150, 10.071)))
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


class ObstacleScanTests(unittest.TestCase):
    def test_pair_diagnostics_distinguish_missing_bracket_recovery_and_expiry(self):
        odom = OdomHistory(OFFSET)
        scans = ObstacleScans(odom)
        self.assertEqual(scans.diagnostics(10.)['failure_reason'], 'no scan received')
        odom.add(exchange(1, 100, 10.021))
        scans.add(obstacle_scan(10.01, [2.]*9), 10.04)
        missing = scans.diagnostics(10.04)
        self.assertFalse(missing['paired'])
        self.assertEqual(missing['failure_reason'], 'query is outside the observed segment')
        self.assertEqual(missing['odom']['count'], 1)
        self.assertEqual(missing['odom']['newest_sample_sequence'], 1)
        self.assertGreater(missing['scan_stamp_ns'], missing['odom']['newest_stamp_ns'])

        odom.add(exchange(2, 150, 10.071))
        paired = scans.diagnostics(10.09)
        self.assertTrue(paired['paired'])
        self.assertTrue(paired['usable'])
        self.assertIsNone(paired['failure_reason'])
        self.assertEqual((paired['lower_sample_sequence'], paired['upper_sample_sequence']), (1, 2))
        self.assertAlmostEqual(paired['scan_age_ms'], 70.)

        expired = scans.diagnostics(10.52)
        self.assertFalse(expired['paired'])
        self.assertEqual(expired['failure_reason'], 'scan expired before pairing')
        self.assertEqual(expired['last_usable_stamp_ns'], paired['scan_stamp_ns'])
        json.dumps(expired, allow_nan=False)

    def test_obstacles_pair_delayed_scan_with_older_real_odom_bracket(self):
        odom = OdomHistory(OFFSET)
        scans = ObstacleScans(odom)
        odom.add(exchange(1, 100, 10.021))
        odom.add(exchange(4, 250, 10.171))
        scans.add(obstacle_scan(10.09, [2.]*9), 10.38)

        frame = scans.latest_observation(10.39)
        self.assertTrue(frame['usable'])
        self.assertAlmostEqual(frame['expires_mono'], 10.60)
        pose = frame['pose']
        self.assertEqual((pose['lower_sample_sequence'], pose['upper_sample_sequence']), (1, 4))
        self.assertAlmostEqual(10.39 - pose['source_monotonic_s'], .39)
        with self.assertRaisesRegex(PoseHistoryError, 'no fresh odometry bracket'):
            odom.at(frame['stamp_ns'], 10.39)

    def test_gap_diagnostics_show_fresh_sample_but_lost_scan_bracket(self):
        odom = OdomHistory(OFFSET)
        scans = ObstacleScans(odom)
        odom.add(exchange(1, 100, 10.021))
        odom.add(exchange(2, 150, 10.071))
        odom.add(exchange(8, 450, 10.371))
        scans.add(obstacle_scan(10.31, [2.]*9), 10.38)
        diagnostic = scans.diagnostics(10.39)
        self.assertEqual(diagnostic['odom']['count'], 1)
        self.assertEqual(diagnostic['odom']['gap_count'], 1)
        self.assertEqual(diagnostic['odom']['newest_sample_sequence'], 8)
        self.assertAlmostEqual(diagnostic['odom']['source_age_ms'], 40.)
        self.assertLess(diagnostic['scan_stamp_ns'], diagnostic['odom']['oldest_stamp_ns'])
        self.assertEqual(diagnostic['failure_reason'], 'query is outside the observed segment')

    def test_rejected_raw_scan_diagnostics_do_not_create_a_frame(self):
        for now, reason in ((10.52, 'scan expired on receipt'), (10., 'scan timestamp is in the future')):
            with self.subTest(now=now):
                scans = ObstacleScans(OdomHistory(OFFSET))
                scans.add(obstacle_scan(10.01, [2.]*9), now)
                diagnostic = scans.diagnostics(now)
                self.assertEqual(len(scans.frames), 0)
                self.assertEqual(diagnostic['failure_reason'], reason)
                self.assertEqual(diagnostic['latest_received']['rejection'], reason)
                json.dumps(diagnostic, allow_nan=False)

    def test_range_diagnostics_remain_separate_from_pairing_failure(self):
        for distance, reason in ((.9, 'filtered_ranges'), (math.nan, 'invalid_ranges'), (math.inf, 'no_return')):
            with self.subTest(distance=distance):
                odom = OdomHistory(OFFSET)
                odom.add(exchange(1, 100, 10.021))
                odom.add(exchange(2, 150, 10.071))
                scans = ObstacleScans(odom)
                scans.add(obstacle_scan(10.01, [distance]*9), 10.08)
                diagnostic = scans.diagnostics(10.09)
                self.assertTrue(diagnostic['paired'])
                self.assertFalse(diagnostic['usable'])
                self.assertEqual(diagnostic['failure_reason'], reason)
                self.assertIsNone(diagnostic['last_usable_stamp_ns'])
                json.dumps(diagnostic, allow_nan=False)

    def test_front_returns_at_measured_scan_pose_use_vehicle_offset(self):
        odom = OdomHistory(OFFSET)
        scans = ObstacleScans(odom)
        odom.add(exchange(1, 100, 10.021, x=1.))
        odom.add(exchange(2, 150, 10.071, x=3.))
        raw = obstacle_scan(10.01, [4., 4., 2., math.nan, 1., .999, 3., 4., 4.])
        scans.add(raw, 10.08)

        pose, points = scans.latest(10.09)
        self.assertAlmostEqual(pose['x'], 1.8)
        self.assertEqual((pose['boot'], pose['epoch']), (12, 1))
        self.assertEqual(pose['stamp_ns'], OFFSET + 10*NS + 20_000_000)
        self.assertEqual(len(points), 3)
        for actual, expected in zip(points, ((.13, -2.), (1.13, 0.), (.13, 3.))):
            self.assertAlmostEqual(actual[0], expected[0])
            self.assertAlmostEqual(actual[1], expected[1])
        self.assertEqual(raw.header.stamp.to_nsec(), OFFSET + 10*NS + 10_000_000)

    def test_new_scan_waits_for_bracket_and_valid_empty_scan_replaces_returns(self):
        odom = OdomHistory(OFFSET)
        scans = ObstacleScans(odom)
        odom.add(exchange(1, 100, 10.021))
        odom.add(exchange(2, 150, 10.071))
        scans.add(obstacle_scan(10.01, [2.]*9), 10.08)
        previous = scans.latest(10.09)
        scans.add(obstacle_scan(10.07, [math.inf]*9), 10.09)
        self.assertEqual(scans.latest(10.1), previous)

        odom.add(exchange(3, 200, 10.121))
        pose, points = scans.latest(10.14)
        self.assertEqual(pose['stamp_ns'], OFFSET + 10*NS + 80_000_000)
        self.assertEqual(points, ())
        self.assertEqual(scans.latest(10.15), (pose, points))

    def test_duplicate_frames_do_not_renew_and_odom_epoch_cuts_bracket(self):
        odom = OdomHistory(OFFSET)
        scans = ObstacleScans(odom)
        odom.add(exchange(1, 100, 10.021))
        odom.add(exchange(2, 150, 10.071))
        scans.add(obstacle_scan(10.01, [2.]*9), 10.08)
        scans.add(obstacle_scan(10.01, [math.inf]*9), 10.09)
        frame = scans.latest_observation(10.1)
        self.assertEqual(len(frame['points']), 5)
        self.assertAlmostEqual(frame['expires_mono'], 10.52)

        odom.add(exchange(1, 100, 10.121, epoch=2))
        self.assertIsNone(scans.latest(10.14))

    def test_invalid_and_no_return_frames_never_provide_clearing_evidence(self):
        for distance, expected_invalid, expected_no_return in ((math.nan, 5, 0), (-1., 5, 0), (math.inf, 0, 5)):
            with self.subTest(distance=distance):
                odom = OdomHistory(OFFSET)
                odom.add(exchange(1, 100, 10.021))
                odom.add(exchange(2, 150, 10.071))
                scans = ObstacleScans(odom)
                scans.add(obstacle_scan(10.01, [distance]*9), 10.08)
                frame = scans.latest_observation(10.09)
                self.assertFalse(frame['usable'])
                self.assertEqual(frame['clear_rays'], ())
                self.assertEqual(frame['invalid_count'], expected_invalid)
                self.assertEqual(frame['no_return_count'], expected_no_return)

    def test_finite_rays_start_outside_blind_range_and_end_at_measured_hit(self):
        odom = OdomHistory(OFFSET)
        odom.add(exchange(1, 100, 10.021))
        odom.add(exchange(2, 150, 10.071))
        scans = ObstacleScans(odom)
        scans.add(obstacle_scan(10.01, [math.inf, math.inf, .9, math.nan, 2., -1., math.inf, math.inf, math.inf]), 10.08)
        frame = scans.latest_observation(10.09)
        self.assertTrue(frame['usable'])
        self.assertEqual(frame['points'], ((2.13, 0.),))
        self.assertEqual(frame['clear_rays'], ((1.13, 0., 2.13, 0.),))
        self.assertEqual(frame['invalid_count'], 2)
        self.assertEqual(frame['finite_count'], 2)

    def test_configured_mount_and_lower_range_apply_to_hits_and_clearing(self):
        odom = OdomHistory(OFFSET)
        odom.add(exchange(1, 100, 10.021))
        odom.add(exchange(2, 150, 10.071))
        scans = ObstacleScans(odom, laser_x=.2, laser_y=.3, laser_yaw=math.pi/2, min_range_m=.2)
        scans.add(obstacle_scan(10.01, [math.inf, math.inf, .8, math.inf, math.inf, math.inf, math.inf, math.inf, math.inf]), 10.08)
        frame = scans.latest_observation(10.09)
        self.assertEqual(frame['points'], ((1., .3),))
        self.assertEqual(frame['clear_rays'], ((.4, .3, 1., .3),))
        self.assertEqual(frame['stamp_ns'], OFFSET + 10*NS + 10_000_000)

    def test_configured_minimum_respects_sensor_minimum(self):
        odom = OdomHistory(OFFSET)
        odom.add(exchange(1, 100, 10.021))
        odom.add(exchange(2, 150, 10.071))
        scans = ObstacleScans(odom, min_range_m=0.)
        scan = obstacle_scan(10.01, [math.inf]*4 + [.8] + [math.inf]*4)
        scan.range_min = .25
        scans.add(scan, 10.08)
        self.assertEqual(scans.latest_observation(10.09)['clear_rays'], ((.38, 0., .93, 0.),))
        with self.assertRaises(ValueError):
            ObstacleScans(odom, min_range_m=-.1)


if __name__ == '__main__':
    unittest.main()
