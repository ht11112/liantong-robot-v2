"""Offline pose-history tests: no device, clock, map, ROS or raw integration."""
import math
import unittest

from simple_p2p_history import PoseHistory, PoseHistoryError


def pose(sequence, source, x=0., y=0., yaw=0., **fields):
    return dict(source_monotonic_s=source, sample_sequence=sequence,
                x=x, y=y, yaw=yaw, **fields)


class PoseHistoryTests(unittest.TestCase):
    def test_latest_retains_accumulated_pose_and_original_source_and_owns_copy(self):
        history = PoseHistory()
        original = pose(10, 1., x=9.5, y=-2., yaw=4., linear_x=.01,
                        mcu_speed_tick_ms=123, integrated=True)
        expected = dict(original)
        self.assertTrue(history.push(original))
        original['x'] = 100.
        latest = history.latest(1.2)
        self.assertEqual(latest, expected)
        latest['x'] = -100.
        self.assertEqual(history.latest(1.3), expected)

    def test_freshness_boundary_future_and_empty_never_return_fallback(self):
        history = PoseHistory()
        with self.assertRaises(PoseHistoryError):
            history.latest(1.)
        history.push(pose(1, 1.))
        self.assertEqual(history.latest(1.749)['source_monotonic_s'], 1.)
        for now in (1.75, 3., .999):
            with self.subTest(now=now), self.assertRaises(PoseHistoryError):
                history.latest(now)
        for maximum in (0, -1, float('nan'), True):
            with self.assertRaises(PoseHistoryError):
                history.latest(1.1, maximum)

    def test_duplicate_does_not_add_or_renew_and_rewritten_identity_is_rejected(self):
        history = PoseHistory()
        first = pose(1, 1.)
        history.push(first)
        self.assertFalse(history.push(dict(first)))
        self.assertEqual(history.summary()['count'], 1)
        with self.assertRaises(PoseHistoryError):
            history.push(dict(first, source_monotonic_s=1.1))
        with self.assertRaises(PoseHistoryError):
            history.push(dict(first, x=.1))
        with self.assertRaises(PoseHistoryError):
            history.latest(1.75)

    def test_bad_measurement_order_or_sequence_reject_without_destroying_history(self):
        history = PoseHistory()
        history.push(pose(10, 1.))
        for invalid in (pose(11, .9), pose(11, 1.), pose(9, 1.1)):
            with self.subTest(invalid=invalid), self.assertRaises(PoseHistoryError):
                history.push(invalid)
        self.assertEqual(history.latest(1.1), pose(10, 1.))

    def test_separate_measurement_axis_accepts_conservative_source_regression(self):
        history = PoseHistory()
        history.push(pose(10, 1., x=2., measurement_monotonic_s=1.2))
        history.push(pose(11, .98, x=4., measurement_monotonic_s=1.3))
        self.assertEqual(history.latest(1.4)['source_monotonic_s'], .98)
        self.assertAlmostEqual(history.at(1.25)['x'], 3.)
        self.assertEqual(history.at(1.25)['source_monotonic_s'], .98)
        # A new measurement axis never renews original source freshness.
        with self.assertRaises(PoseHistoryError):
            history.latest(1.73)
        with self.assertRaises(PoseHistoryError):
            history.push(pose(12, 1.4, measurement_monotonic_s=1.29))

    def test_optional_measurement_keyword_and_conflicting_values(self):
        history = PoseHistory()
        history.push(pose(1, 1.), measurement_monotonic_s=1.1)
        self.assertEqual(history.latest(1.2)['measurement_monotonic_s'], 1.1)
        with self.assertRaises(PoseHistoryError):
            history.push(pose(2, 1.2, measurement_monotonic_s=1.3), measurement_monotonic_s=1.4)
        self.assertEqual(history.summary()['count'], 1)

    def test_interpolation_has_both_real_brackets_and_cannot_be_reinjected(self):
        history = PoseHistory()
        first, second = pose(1, 1., x=3., y=-5.), pose(2, 2., x=5., y=1.)
        history.push(first)
        history.push(second)
        self.assertEqual(history.at(1.), first)
        self.assertEqual(history.at(2.), second)
        middle = history.sample_at(1.5)
        self.assertEqual((middle['x'], middle['y']), (4., -2.))
        self.assertEqual(middle['source_monotonic_s'], 1.)
        self.assertEqual(middle['measurement_monotonic_s'], 1.5)
        self.assertEqual((middle['lower_sample_sequence'], middle['upper_sample_sequence']), (1, 2))
        self.assertTrue(middle['interpolated'])
        self.assertIsNone(middle['sample_sequence'])
        with self.assertRaises(PoseHistoryError):
            history.push(middle)
        for outside in (.999, 2.001, 200.):
            with self.assertRaises(PoseHistoryError):
                history.at(outside)

    def test_yaw_crosses_plus_minus_pi_by_short_arc_and_preserves_endpoints(self):
        for sign in (1, -1):
            with self.subTest(sign=sign):
                history = PoseHistory()
                first = pose(1, 1., yaw=sign*math.radians(179))
                second = pose(2, 2., yaw=-sign*math.radians(179))
                history.push(first)
                history.push(second)
                self.assertAlmostEqual(abs(history.at(1.5)['yaw']), math.pi)
                self.assertAlmostEqual(history.at(1.25)['yaw'], sign*math.radians(179.5))
                self.assertEqual(history.at(1.), first)
                self.assertEqual(history.at(2.), second)

    def test_capacity_evicts_oldest_without_extrapolation(self):
        history = PoseHistory(capacity=3)
        for sequence in range(1, 8):
            history.push(pose(sequence, float(sequence), x=float(sequence)))
        self.assertEqual(history.summary()['count'], 3)
        self.assertEqual(history.summary()['oldest_measurement_monotonic_s'], 5.)
        self.assertEqual(history.at(5.5)['x'], 5.5)
        with self.assertRaises(PoseHistoryError):
            history.at(4.5)
        with self.assertRaises(PoseHistoryError):
            history.at(7.01)

    def test_explicit_core_and_inferred_gaps_cut_without_invented_motion(self):
        for kind in ('explicit', 'core', 'sequence'):
            with self.subTest(kind=kind):
                history = PoseHistory()
                history.push(pose(1, 1., x=10.))
                next_pose = pose(3 if kind == 'sequence' else 2, 2., x=10.1)
                if kind == 'core':
                    next_pose['discontinuity'] = True
                history.push(next_pose, gap=(kind == 'explicit'))
                self.assertEqual(history.summary()['count'], 1)
                self.assertEqual(history.summary()['gap_count'], 1)
                self.assertEqual(history.latest(2.1)['x'], 10.1)
                with self.assertRaises(PoseHistoryError):
                    history.at(1.5)

    def test_reset_and_invalidate_preserve_replay_fence_same_epoch(self):
        for method in ('reset', 'invalidate'):
            with self.subTest(method=method):
                history = PoseHistory(epoch='boot/session')
                first = pose(10, 3.)
                history.push(first)
                getattr(history, method)()
                self.assertFalse(history.push(first))
                with self.assertRaises(PoseHistoryError):
                    history.latest(3.1)
                with self.assertRaises(PoseHistoryError):
                    history.push(pose(9, 3.1))
                history.push(pose(11, 3.2))
                self.assertEqual(history.summary()['count'], 1)
                with self.assertRaises(PoseHistoryError):
                    history.at(3.1)

    def test_explicit_new_epoch_allows_new_clock_and_sequence_but_cuts_history(self):
        for via_reset in (False, True):
            with self.subTest(via_reset=via_reset):
                history = PoseHistory(epoch='old')
                history.push(pose(900, 100.))
                if via_reset:
                    history.reset(epoch='new')
                history.push(pose(1, 1.), epoch='new')
                self.assertEqual(history.summary()['epoch'], 'new')
                self.assertEqual(history.latest(1.1), pose(1, 1.))
                with self.assertRaises(PoseHistoryError):
                    history.at(50.)

    def test_uint32_sequence_wrap_is_continuous(self):
        history = PoseHistory()
        history.push(pose(0xffffffff, 1., x=1.))
        history.push(pose(0, 2., x=2.))
        self.assertEqual(history.summary()['gap_count'], 0)
        self.assertEqual(history.at(1.5)['x'], 1.5)

    def test_invalid_nonfinite_mutable_or_boolean_fields_reject_without_mutation(self):
        history = PoseHistory()
        history.push(pose(1, 1.))
        for field in ('x', 'y', 'yaw', 'source_monotonic_s', 'measurement_monotonic_s'):
            for invalid in (True, float('nan'), float('inf'), 10**10000):
                with self.subTest(field=field, kind=type(invalid).__name__), self.assertRaises(PoseHistoryError):
                    history.push(dict(pose(2, 2.), **{field: invalid}))
        for invalid in (pose(True, 2.), pose(-1, 2.), pose(2**32, 2.),
                        pose(2, -1.), pose(2, 2., raw={}), pose(2, 2., discontinuity=1)):
            with self.assertRaises(PoseHistoryError):
                history.push(invalid)
        self.assertEqual(history.summary()['count'], 1)
        for capacity in (1, 4097, True, 2.5):
            with self.assertRaises(PoseHistoryError):
                PoseHistory(capacity=capacity)
        for query in (True, float('nan'), float('inf'), -1):
            with self.assertRaises(PoseHistoryError):
                history.at(query)


if __name__ == '__main__':
    unittest.main()
