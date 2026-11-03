"""Offline stamp/frame/pose conversions only; no ROS or physical resources."""
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

from localization_ros_adapter import (SourceGate, amcl_diagnostics, corrected_scan,
    mounting_assumption, odom_transform, scan_snapshot, validate_frames)


def header(seq=1, secs=10, nsecs=0, frame="map"):
    return NS(seq=seq, stamp=NS(secs=secs, nsecs=nsecs), frame_id=frame)


def pose(x=1., y=2., yaw=.3):
    return NS(position=NS(x=x, y=y, z=0.),
              orientation=NS(x=0., y=0., z=math.sin(yaw / 2.), w=math.cos(yaw / 2.)))


def scan():
    return NS(header=header(), ranges=[1., 2., 3., 4., 5.],
              angle_min=-math.pi, angle_max=math.pi, angle_increment=math.pi / 2.,
              range_min=.1, range_max=12., time_increment=.001, scan_time=.1)


class LocalizationAdapterTests(unittest.TestCase):
    def test_import_is_ros_free(self):
        before = set(sys.modules)
        path = Path(__file__).with_name("localization_ros_adapter.py")
        spec = importlib.util.spec_from_file_location("isolated_loc_adapter_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertFalse({"rospy", "tf2_ros"} & (set(sys.modules) - before))

    def test_explicit_distinct_namespace_frames(self):
        result = validate_frames("p2p_abc_laser", "p2p_abc_base", "p2p_abc_odom", "p2p_abc_map")
        self.assertEqual(result["map"], "p2p_abc_map")
        for frames in (("map", "base_link", "odom", "map"),
                       ("p2p_a_laser", "p2p_a_base", "p2p_a_base", "p2p_a_map"),
                       ("/p2p_a_laser", "p2p_a_base", "p2p_a_odom", "p2p_a_map")):
            with self.assertRaises(ValueError):
                validate_frames(*frames)

    def test_raw_wrong_map_frame_is_explicitly_corrected_without_changing_time_or_coordinates(self):
        raw = scan()
        raw.header.stamp.nsecs = 123456789
        corrected = corrected_scan(raw, "p2p_abc_laser")
        self.assertEqual(raw.header.frame_id, "map")
        self.assertEqual(corrected.header.frame_id, "p2p_abc_laser")
        self.assertEqual(corrected.header.stamp.nsecs, 123456789)
        self.assertEqual(corrected.header.seq, raw.header.seq)
        self.assertEqual(corrected.ranges[1:4], [2., 3., 4.])
        self.assertTrue(math.isnan(corrected.ranges[0]))
        self.assertEqual(raw.ranges[0], 1.)

    def test_mounting_is_assumed_and_used_only_to_choose_visible_beams(self):
        assumed = mounting_assumption()
        self.assertEqual((assumed["x_m"], assumed["y_m"], assumed["z_m"], assumed["yaw_rad"]), (.13, 0., .2, 0.))
        self.assertTrue(assumed["assumed"])
        self.assertFalse(assumed["measured"])
        result = corrected_scan(scan(), "p2p_a_laser", math.pi)
        self.assertEqual(result.ranges[0], 1.)
        self.assertTrue(math.isnan(result.ranges[2]))

    def test_odom_tf_preserves_measured_pose_and_original_source_stamp(self):
        message = NS(header=header(frame="p2p_a_odom", nsecs=123),
                     child_frame_id="p2p_a_base", pose=NS(pose=pose()))
        result = odom_transform(message, "p2p_a_odom", "p2p_a_base")
        self.assertEqual(result["header"]["stamp_nsecs"], 123)
        self.assertEqual(result["translation"], dict(x=1., y=2., z=0.))
        self.assertAlmostEqual(result["rotation"]["z"], math.sin(.15))
        self.assertEqual(message.header.frame_id, "p2p_a_odom")
        for frame, child in (("odom", "p2p_a_base"), ("p2p_a_odom", "base_link")):
            bad = deepcopy(message)
            bad.header.frame_id, bad.child_frame_id = frame, child
            with self.assertRaises(ValueError):
                odom_transform(bad, "p2p_a_odom", "p2p_a_base")

    def test_source_gate_does_not_renew_duplicate_stamp_even_if_sequence_changes(self):
        gate = SourceGate(.75)
        self.assertTrue(gate.accept(header(frame="p2p_a_odom"), 10.1))
        self.assertFalse(gate.accept(header(seq=2, frame="p2p_a_odom"), 10.7))
        report = gate.report(10.75)
        self.assertEqual(report["counts"]["accepted"], 1)
        self.assertEqual(report["counts"]["duplicate"], 1)
        self.assertEqual(report["latest_header"]["seq"], 1)
        self.assertEqual(report["current_distinct_sources"], 0)

    def test_source_gate_rejects_future_stale_reordered_zero_and_presession(self):
        gate = SourceGate(.75, earliest_source=10.)
        self.assertFalse(gate.accept(header(secs=0), 10.))
        self.assertEqual(gate.last_rejection, "zero_stamp")
        self.assertFalse(gate.accept(header(secs=11), 10.))
        self.assertEqual(gate.last_rejection, "future")
        self.assertFalse(gate.accept(header(secs=9), 10.))
        self.assertEqual(gate.last_rejection, "before_session")
        self.assertFalse(gate.accept(header(), 10.75))
        self.assertEqual(gate.last_rejection, "stale")
        self.assertTrue(gate.accept(header(nsecs=100000000), 10.2))
        self.assertFalse(gate.accept(header(nsecs=50000000), 10.3))
        self.assertEqual(gate.last_rejection, "reordered")

    def test_source_stamp_advances_with_rospy_generated_or_zero_sequence(self):
        gate = SourceGate(.75)
        self.assertTrue(gate.accept(header(seq=0), 10.1))
        self.assertTrue(gate.accept(header(seq=0, nsecs=100), 10.2))
        self.assertEqual(gate.report(10.3)["counts"]["accepted"], 2)

    def test_five_distinct_sources_must_all_be_current_at_report_time(self):
        gate = SourceGate(.5)
        for i in range(5):
            self.assertTrue(gate.accept(header(seq=i, nsecs=i * 50000000), 10.21))
        self.assertTrue(gate.report(10.3)["five_current_distinct_sources"])
        self.assertFalse(gate.report(10.55)["five_current_distinct_sources"])

    def test_amcl_covariance_and_frame_diagnostics_do_not_claim_localization(self):
        covariance = [0.] * 36
        covariance[0], covariance[7], covariance[35] = .01, .04, math.radians(3.) ** 2
        message = NS(header=header(frame="p2p_a_map"), pose=NS(pose=pose(), covariance=covariance))
        result = amcl_diagnostics(message, "p2p_a_map")
        self.assertEqual(result["position_std_m"], [.1, .2])
        self.assertAlmostEqual(result["yaw_std_deg"], 3.)
        self.assertFalse(result["localization_validated"])
        self.assertFalse(result["covariance_positive_semidefinite_assessed"])
        bad = deepcopy(message)
        bad.pose.covariance[0] = -1.
        with self.assertRaises(ValueError):
            amcl_diagnostics(bad, "p2p_a_map")
        with self.assertRaises(ValueError):
            amcl_diagnostics(message, "map")

    def test_invalid_quaternion_and_nonfinite_covariance_are_rejected(self):
        message = NS(header=header(frame="p2p_a_map"), pose=NS(pose=pose(), covariance=[0.] * 36))
        message.pose.pose.orientation.w = 0.
        with self.assertRaises(ValueError):
            amcl_diagnostics(message, "p2p_a_map")
        message.pose.pose = pose()
        message.pose.covariance[7] = float("nan")
        with self.assertRaises(ValueError):
            amcl_diagnostics(message, "p2p_a_map")

    def test_snapshot_has_complete_ranges_exact_headers_and_strict_json_nonfinite_encoding(self):
        raw = scan()
        raw.ranges = [float("inf"), float("nan"), float("inf"), -float("inf"), 2.]
        raw.header.stamp.nsecs = 456
        corrected = corrected_scan(raw, "p2p_a_laser")
        result = scan_snapshot(raw, corrected)
        self.assertEqual(result["ranges"], [None, None, "inf", "-inf", None])
        self.assertEqual(result["raw_header"]["frame_id"], "map")
        self.assertEqual(result["header"]["frame_id"], "p2p_a_laser")
        self.assertEqual(result["header"]["stamp_nsecs"], 456)
        self.assertEqual(len(result["ranges"]), 5)
        json.dumps(result, allow_nan=False)

    def test_age_budgets_must_be_explicit_finite_positive(self):
        for value in (None, 0, -1, True, float("nan"), float("inf"), 10 ** 1000):
            with self.subTest(value_type=type(value)), self.assertRaises(ValueError):
                SourceGate(value)


if __name__ == "__main__":
    unittest.main()
