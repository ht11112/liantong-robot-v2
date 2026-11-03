import copy
import math
import unittest

from trial_obstacle_gate import evaluate_forward_scan


def clear_scan():
    return dict(header=dict(frame_id="trial_laser", stamp_secs=100, stamp_nsecs=123,
                            source_stamp=100.000000123),
                source_monotonic_s=10., angle_min=-math.pi,
                angle_increment=math.pi / 180., range_min=.1, range_max=50.,
                ranges=[float("inf")] * 360)


def point(scan, x, y):
    angle = math.atan2(y, x - .13)
    index = round((angle - scan["angle_min"]) / scan["angle_increment"])
    scan["ranges"][index] = math.hypot(x - .13, y)


class ForwardObstacleGateTests(unittest.TestCase):
    def test_positive_inf_is_valid_no_return_not_unknown(self):
        result = evaluate_forward_scan(clear_scan(), 10.1)
        self.assertTrue(result["ready"])
        self.assertGreaterEqual(result["coverage"]["positive_infinite_returns"], 180)
        self.assertIsNone(result["minimum_front_clearance_m"])
        self.assertFalse(result["rear_coverage_verified"])
        self.assertFalse(result["motion_permission"])

    def test_rear_nan_is_not_invented_free_or_a_required_front_return(self):
        scan = clear_scan()
        for i in range(len(scan["ranges"])):
            angle = scan["angle_min"] + i * scan["angle_increment"]
            if abs(angle) > math.pi / 2. + 1e-12:
                scan["ranges"][i] = float("nan")
        self.assertTrue(evaluate_forward_scan(scan, 10.1)["ready"])

    def test_every_front_unknown_type_rejects(self):
        for raw in (None, float("nan"), float("-inf"), "-inf", "nan", True, .01, 51., 0.):
            with self.subTest(raw=raw):
                scan = clear_scan()
                scan["ranges"][180] = raw
                self.assertFalse(evaluate_forward_scan(scan, 10.1)["ready"])

    def test_json_positive_inf_strings_are_valid(self):
        for raw in ("inf", "+inf"):
            scan = clear_scan()
            scan["ranges"] = [raw] * 360
            self.assertTrue(evaluate_forward_scan(scan, 10.1)["ready"])

    def test_original_source_not_mutated_and_ages_with_same_scan(self):
        scan = clear_scan()
        before = copy.deepcopy(scan)
        a = evaluate_forward_scan(scan, 10.1)
        b = evaluate_forward_scan(scan, 10.75)
        self.assertEqual(scan, before)
        self.assertEqual(a["source_evidence"]["source_monotonic_s"], 10.)
        self.assertFalse(b["ready"])
        self.assertAlmostEqual(b["source_evidence"]["source_age_s"], .75)

    def test_future_or_nonfinite_source_rejects(self):
        for source in (10.2, -1., float("nan"), True):
            scan = clear_scan()
            scan["source_monotonic_s"] = source
            self.assertFalse(evaluate_forward_scan(scan, 10.1)["ready"])

    def test_front_threshold_is_body_edge_not_laser_range(self):
        for x, ready in ((.399, False), (.4, False), (.401, True)):
            scan = clear_scan()
            point(scan, x, 0.)
            result = evaluate_forward_scan(scan, 10.1)
            self.assertEqual(result["ready"], ready)
            self.assertAlmostEqual(result["minimum_front_clearance_m"], x - .2)

    def test_obstacle_below_localization_point_floor_still_stops(self):
        scan = clear_scan()
        scan["ranges"][180] = .15  # Valid lidar range < localization's .3 m floor.
        self.assertFalse(evaluate_forward_scan(scan, 10.1)["ready"])

    def test_observed_front_side_proximity_stops(self):
        scan = clear_scan()
        # A +90deg ray sees the front half of the right/left side, not the rear.
        scan["ranges"][270] = .30
        result = evaluate_forward_scan(scan, 10.1)
        self.assertFalse(result["ready"])
        self.assertAlmostEqual(result["observed_side_front_clearance_m"], .05)

    def test_far_lateral_obstacle_does_not_block_forward_corridor(self):
        scan = clear_scan()
        scan["ranges"][270] = .5
        self.assertTrue(evaluate_forward_scan(scan, 10.1)["ready"])

    def test_finite_far_returns_are_coverage(self):
        scan = clear_scan()
        scan["ranges"] = [2.] * 360
        result = evaluate_forward_scan(scan, 10.1)
        self.assertTrue(result["ready"])
        self.assertGreaterEqual(result["coverage"]["finite_returns"], 180)

    def test_missing_front_edge_or_sparse_rays_rejects(self):
        scan = clear_scan()
        scan.update(angle_min=-math.pi / 4, angle_increment=math.pi / 180,
                    ranges=[float("inf")] * 91)
        self.assertFalse(evaluate_forward_scan(scan, 10.1)["ready"])
        scan.update(angle_min=-math.pi / 2, angle_increment=math.pi / 2,
                    ranges=[float("inf")] * 3)
        self.assertFalse(evaluate_forward_scan(scan, 10.1)["ready"])

    def test_negative_increment_is_equivalent(self):
        scan = clear_scan()
        scan["angle_min"] = math.pi
        scan["angle_increment"] = -scan["angle_increment"]
        self.assertTrue(evaluate_forward_scan(scan, 10.1)["ready"])

    def test_insufficient_no_return_range_rejects(self):
        scan = clear_scan()
        scan["range_max"] = .3
        self.assertFalse(evaluate_forward_scan(scan, 10.1)["ready"])

    def test_exact_frame_and_header_stamp_checked(self):
        scan = clear_scan()
        self.assertTrue(evaluate_forward_scan(scan, 10.1, expected_frame_id="trial_laser")["ready"])
        self.assertFalse(evaluate_forward_scan(scan, 10.1, expected_frame_id="other")["ready"])
        scan["header"]["source_stamp"] = 101.
        self.assertFalse(evaluate_forward_scan(scan, 10.1)["ready"])

    def test_malformed_observations_fail_closed(self):
        for field, value in (("ranges", []), ("ranges", True), ("angle_increment", 0.),
                             ("angle_min", float("inf")), ("range_min", -.1),
                             ("range_max", float("inf")), ("header", {})):
            scan = clear_scan()
            scan[field] = value
            self.assertFalse(evaluate_forward_scan(scan, 10.1)["ready"])

    def test_invalid_configuration_rejected(self):
        for params in (dict(front_clearance_m=True), dict(side_margin_m=-1),
                       dict(body_width_m=float("nan")), dict(max_source_age_s=0),
                       dict(max_angular_gap_deg=6), dict(expected_frame_id="")):
            with self.assertRaises(ValueError):
                evaluate_forward_scan(clear_scan(), 10.1, **params)

    def test_dense_1200beam_cropped_geometry_with_real_float_precision(self):
        scan = clear_scan()
        scan.update(angle_min=-3.1415927410125732, angle_increment=.005235987715423107,
                    range_min=.10000000149011612, ranges=[float("nan")] * 1200)
        for i in range(1200):
            angle = math.atan2(math.sin(scan["angle_min"] + i * scan["angle_increment"]),
                               math.cos(scan["angle_min"] + i * scan["angle_increment"]))
            if abs(angle) <= math.pi / 2.:
                scan["ranges"][i] = float("inf") if i % 7 == 0 else 2.
        result = evaluate_forward_scan(scan, 10.1)
        self.assertTrue(result["ready"])
        self.assertEqual(result["coverage"]["front_beams"], 600)


if __name__ == "__main__":
    unittest.main()
