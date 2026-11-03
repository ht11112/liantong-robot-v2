"""Pure conversion/provenance tests; importing the adapter never imports rospy."""
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest

from ros_sensor_adapter import (SourceRegistry, amcl_summary, crop_scan_message,
    map_summary, odom_summary, scan_summary, source_header, tf_summary,
    validate_runtime_options)


def header(sequence=1, secs=10, nsecs=0, frame="laser_original"):
    return NS(seq=sequence, stamp=NS(secs=secs, nsecs=nsecs), frame_id=frame)


def xyz(x=0., y=0., z=0.):
    return NS(x=x, y=y, z=z)


def pose():
    return NS(position=xyz(1., 2., 0.), orientation=NS(x=0., y=0., z=0., w=1.))


def scan():
    return NS(header=header(), angle_min=-math.pi, angle_max=math.pi,
              angle_increment=math.pi / 2, time_increment=.001, scan_time=.1,
              range_min=.1, range_max=10., ranges=[1., 2., 3., 4., 5.],
              intensities=[10., 20., 30., 40., 50.])


class SensorAdapterTests(unittest.TestCase):
    def test_import_has_no_ros_dependency(self):
        before = set(sys.modules)
        path = Path(__file__).with_name("ros_sensor_adapter.py")
        spec = importlib.util.spec_from_file_location("isolated_sensor_adapter_test", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertNotIn("rospy", set(sys.modules) - before)

    def test_crop_preserves_original_header_frame_layout_and_input(self):
        original = scan()
        cropped = crop_scan_message(original)
        self.assertEqual(source_header(cropped.header), source_header(original.header))
        self.assertIsNot(cropped.header, original.header)
        self.assertEqual(original.ranges, [1., 2., 3., 4., 5.])
        self.assertTrue(math.isnan(cropped.ranges[0]))
        self.assertTrue(math.isnan(cropped.ranges[-1]))
        self.assertEqual(cropped.ranges[1:4], [2., 3., 4.])
        for field in ("angle_min", "angle_max", "angle_increment", "scan_time",
                      "time_increment", "range_min", "range_max", "intensities"):
            self.assertEqual(getattr(cropped, field), getattr(original, field))

    def test_crop_honors_mounting_yaw_without_publishing_new_tf(self):
        original = scan()
        cropped = crop_scan_message(original, math.pi)
        self.assertEqual(cropped.ranges[0], 1.)
        self.assertTrue(math.isnan(cropped.ranges[2]))
        self.assertEqual(cropped.header.frame_id, "laser_original")

    def test_publisher_generated_sequence_cannot_modify_original_source_registry(self):
        message = scan()
        cropped = crop_scan_message(message)
        registry = SourceRegistry()
        registry.observe("scan", message.header, scan_summary(message), 100.)
        registry.observe("front180", cropped.header, scan_summary(cropped), 100.)
        # Model Noetic rospy serialize_message's output sequence assignment.
        cropped.header.seq = 777
        result = registry.report(11., 101.)
        for key in ("scan", "front180"):
            saved = result[key]["latest"]["header"]
            self.assertEqual(saved["seq"], 1)
            self.assertEqual(saved["source_stamp"], 10.)
            self.assertEqual(saved["frame_id"], "laser_original")
        self.assertEqual(message.header.seq, 1)
        self.assertEqual(cropped.header.stamp.secs, 10)
        self.assertEqual(cropped.header.frame_id, "laser_original")

    def test_scan_nonfinite_ranges_are_counts_not_json_nan(self):
        message = scan()
        message.ranges = [float("nan"), float("inf"), -float("inf"), .01, 1.]
        summary = scan_summary(message)
        self.assertEqual(summary["counts"], dict(nan=1, positive_inf=1, negative_inf=1,
                                                finite_out_of_range=1, finite_in_range=1))
        json.dumps(summary, allow_nan=False)

    def test_duplicate_seq_or_new_relay_receipt_cannot_refresh_old_stamp(self):
        registry = SourceRegistry()
        self.assertTrue(registry.observe("scan", header(), {"tag": "original"}, 100.))
        self.assertFalse(registry.observe("scan", header(sequence=2), {"tag": "replayed"}, 200.))
        report = registry.report(11., 201.)["scan"]
        self.assertEqual(report["latest"]["header"]["seq"], 1)
        self.assertEqual(report["latest"]["received_monotonic"], 100.)
        self.assertEqual(report["latest"]["source_age_s"], 1.)
        self.assertEqual(report["duplicates"], 1)

    def test_source_stamp_advances_even_for_ros_publisher_with_zero_sequence(self):
        registry = SourceRegistry()
        registry.observe("tf:odom->base", header(0), {}, 100.)
        self.assertTrue(registry.observe("tf:odom->base", header(0, nsecs=100), {}, 101.))
        self.assertFalse(registry.observe("tf:odom->base", header(1, secs=9), {}, 102.))
        latest = registry.report(11., 103.)["tf:odom->base"]
        self.assertEqual(latest["reordered"], 1)
        self.assertEqual(latest["latest"]["header"]["stamp_nsecs"], 100)

    def test_receipt_age_is_diagnostic_and_does_not_replace_source_time(self):
        registry = SourceRegistry()
        registry.observe("scan", header(), {}, 100., received_ros=10.1)
        registry.observe("scan", header(), {}, 101., received_ros=10.8)
        result = registry.report(11., 102.)["scan"]
        self.assertAlmostEqual(result["max_source_age_at_receive_s"], .8)
        self.assertAlmostEqual(result["latest"]["source_age_at_receive_s"], .1)
        self.assertEqual(result["latest"]["received_ros"], 10.1)
        self.assertEqual(result["latest"]["header"]["source_stamp"], 10.)

    def test_static_zero_stamp_remains_unknown_age_and_missing_is_explicit(self):
        registry = SourceRegistry()
        registry.observe("map", header(secs=0), {"static": True}, 100., static=True)
        result = registry.report(12., 101., expected=("map", "odom", "amcl"))
        self.assertTrue(result["map"]["static"])
        self.assertIsNone(result["map"]["latest"]["source_age_s"])
        self.assertFalse(result["map"]["latest"]["source_time_known"])
        self.assertTrue(result["odom"]["missing"])
        self.assertTrue(result["amcl"]["missing"])

    def test_future_tf_stamp_is_reported_without_restamping(self):
        registry = SourceRegistry()
        registry.observe("tf:map->odom", header(secs=12), {}, 100.)
        latest = registry.report(11.75, 100.1)["tf:map->odom"]["latest"]
        self.assertEqual(latest["source_age_s"], -.25)
        self.assertTrue(latest["future_source_stamp"])
        self.assertEqual(latest["header"]["source_stamp"], 12.)

    def test_odom_amcl_and_tf_summary_keep_original_coordinates(self):
        odom = NS(child_frame_id="base_original", pose=NS(pose=pose()),
                  twist=NS(twist=NS(linear=xyz(.001), angular=xyz(z=.002))))
        amcl = NS(pose=NS(pose=pose(), covariance=[0.] * 36))
        transform = NS(child_frame_id="base_original",
                       transform=NS(translation=xyz(1, 2, 3), rotation=pose().orientation))
        self.assertEqual(odom_summary(odom)["child_frame_id"], "base_original")
        self.assertEqual(odom_summary(odom)["twist"]["angular"]["z"], .002)
        self.assertFalse(amcl_summary(amcl)["localization_quality_assessed"])
        self.assertEqual(tf_summary(transform)["translation"]["z"], 3.)

    def test_map_summary_preserves_unknown_and_does_not_claim_identity(self):
        message = NS(info=NS(width=2, height=2, resolution=.05, origin=pose()),
                     data=[-1, 0, 65, 100])
        summary = map_summary(message)
        self.assertEqual(summary["unknown_cells"], 1)
        self.assertEqual(summary["zero_cost_cells"], 1)
        self.assertEqual(summary["nonzero_known_cells"], 2)
        self.assertFalse(summary["map_identity_verified"])
        message.data = [0]
        with self.assertRaises(ValueError):
            map_summary(message)

    def test_duration_and_output_must_be_bounded_and_private(self):
        for duration in (0, -1, True, float("nan"), float("inf"), 10 ** 1000):
            with self.subTest(duration_type=type(duration)), self.assertRaises(ValueError):
                validate_runtime_options(duration, "~front180_scan")
        for topic in ("/scan", "/cmd_vel", "scan", "~", None):
            with self.subTest(topic=topic), self.assertRaises(ValueError):
                validate_runtime_options(1., topic)
        self.assertEqual(validate_runtime_options(.1, "~front180_scan"), .1)

    def test_invalid_source_header_is_not_replaced_by_receipt_time(self):
        for bad in (header(sequence=True), header(secs=-1), header(nsecs=1000000000)):
            with self.assertRaises(ValueError):
                SourceRegistry().observe("scan", bad, {}, 100.)


if __name__ == "__main__":
    unittest.main()
