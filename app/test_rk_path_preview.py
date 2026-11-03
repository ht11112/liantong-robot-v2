"""Pure current-pose/path validation; no ROS or physical imports."""
from copy import deepcopy
import importlib.util
import math
from pathlib import Path
import sys
import unittest

from rk_path_preview import (SAVED_MAP_ID, forward_goal, grid_cell, names_for_preview,
    planner_parameters, validate_joint, validate_live_tf, validate_live_amcl,
    validate_live_scan, validate_path)


def joint():
    rows = []
    for i in range(5):
        stamp = 100.1 + i * .1
        rows.append(dict(source_key=[100, 100000000 + i * 100000000],
            original_source_stamp=stamp, pose=[1., 2., .3],
            tf_translation_error_m=.01, tf_yaw_error_rad=.01,
            scan_scores=[dict(used=80, near_wall_ratio=.9)] * 3,
            scan_source_keys=[[100, i * 100000000 + n] for n in (1, 2, 3)],
            anchor=dict(boot_id="current-boot", owner_id="abcdef12", valid=True,
                anchor_established=True, failure=None,
                latest=dict(source_monotonic_s=50.5))))
    return dict(map_id=SAVED_MAP_ID, nonce="abcdef12", stationary_diagnostic_only=True,
        hardware_execution_authorized=False, validated_at_monotonic_s=50.6,
        validated_at_ros_s=100.6, seed_ros_s=100., post_seed_five_observations=rows)


def grid():
    return dict(width=30, height=30, resolution=.05, origin=[0., 0., 0.], data=[0] * 900)


def path():
    return dict(frame_id="p2p_abcdef12_map", poses=[dict(frame_id="p2p_abcdef12_map",
        x=.2 + i * .05, y=.5, z=0., orientation=dict(x=0., y=0., z=0., w=1.)) for i in range(11)])


class PathPreviewTests(unittest.TestCase):
    def test_import_does_not_load_ros_or_serial(self):
        before = set(sys.modules)
        spec = importlib.util.spec_from_file_location("isolated_preview_test", Path(__file__).with_name("rk_path_preview.py"))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertFalse({"rospy", "tf2_ros", "serial"} & (set(sys.modules) - before))

    def test_names_use_new_preview_namespace_and_actual_localization_frames(self):
        names = names_for_preview("abcdef12", "12345")
        self.assertEqual(names["frames"]["map"], "p2p_abcdef12_map")
        self.assertEqual(names["map_topic"], "/p2p_loc_abcdef12/map")
        self.assertNotEqual(names["shadow_topic"], "/cmd_vel")
        self.assertIn("p2p_preview_abcdef12_12345", names["service"])
        for nonce in ("../escape", "a;b", True):
            with self.assertRaises(ValueError):
                names_for_preview(nonce, "123")

    def test_current_five_postseed_joint_and_latest_anchor_required(self):
        result = validate_joint(joint(), "current-boot", "abcdef12", 50.7, 100.7)
        self.assertEqual(result["pose"], [1., 2., .3])
        self.assertEqual(result["source_key"], [100, 500000000])
        for edit in ("boot", "nonce", "map", "five", "postseed", "repeat", "scan", "anchor"):
            bad = joint()
            if edit == "boot": bad["post_seed_five_observations"][2]["anchor"]["boot_id"] = "old"
            if edit == "nonce": bad["nonce"] = "other"
            if edit == "map": bad["map_id"] = "0" * 64
            if edit == "five": bad["post_seed_five_observations"].pop()
            if edit == "postseed": bad["seed_ros_s"] = 100.2
            if edit == "repeat": bad["post_seed_five_observations"][2]["source_key"] = [100, 200000000]
            if edit == "scan": bad["post_seed_five_observations"][0]["scan_scores"][0] = dict(used=59, near_wall_ratio=.9)
            if edit == "anchor": bad["post_seed_five_observations"][-1]["anchor"]["latest"]["source_monotonic_s"] = 49.
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                validate_joint(bad, "current-boot", "abcdef12", 50.7, 100.7)

    def test_stale_future_and_nonfinite_joint_never_become_a_start_pose(self):
        for mono, ros in ((53., 100.7), (50.7, 103.), (50.5, 100.7), (50.7, 100.5)):
            with self.subTest(mono=mono, ros=ros), self.assertRaises(ValueError):
                validate_joint(joint(), "current-boot", "abcdef12", mono, ros)
        bad = joint()
        bad["post_seed_five_observations"][-1]["pose"][0] = float("nan")
        with self.assertRaises(ValueError):
            validate_joint(bad, "current-boot", "abcdef12", 50.7, 100.7)

    def test_goal_is_half_meter_in_current_heading_not_map_positive_x(self):
        result = forward_goal([1., 2., math.pi / 2.], .5)
        self.assertAlmostEqual(result[0], 1.)
        self.assertAlmostEqual(result[1], 2.5)
        for distance in (0, -1, float("nan"), True, 2):
            with self.assertRaises(ValueError):
                forward_goal([0., 0., 0.], distance)

    def test_live_tf_frame_age_and_agreement_are_independent(self):
        frames = names_for_preview("abcdef12", "x")["frames"]
        value = dict(parent=frames["map"], child=frames["base"], source_stamp=100.65,
                     translation=dict(x=1., y=2.), rotation=dict(x=0., y=0., z=math.sin(.15), w=math.cos(.15)))
        validate_live_tf(value, [1., 2., .3], frames, 100.7)
        for key, replacement in (("parent", "map"), ("source_stamp", 99.), ("translation", dict(x=1.04, y=2.))):
            bad = deepcopy(value)
            bad[key] = replacement
            with self.assertRaises(ValueError):
                validate_live_tf(bad, [1., 2., .3], frames, 100.7)

    def test_plan_parameters_do_not_clear_or_add_unreachable_and_keep_unknown(self):
        names = names_for_preview("abcdef12", "123")
        values = planner_parameters(names, dict(geometry=dict(planning_width_m=.5, front_from_axle_m=.2, rear_from_axle_m=.2)))
        self.assertFalse(values["make_plan_clear_costmap"])
        self.assertFalse(values["make_plan_add_unreachable_goal"])
        self.assertFalse(values["GlobalPlanner"]["allow_unknown"])
        self.assertEqual(values["GlobalPlanner"]["default_tolerance"], 0.)
        self.assertEqual(values["DWAPlannerROS"]["max_vel_x"], 0.)
        self.assertNotIn("xy_goal_tolerance", values["DWAPlannerROS"])
        for name in ("global_costmap", "local_costmap"):
            self.assertTrue(values[name]["track_unknown_space"])
            self.assertEqual([layer["name"] for layer in values[name]["plugins"]], ["static_layer", "obstacle_layer", "inflation_layer"])
            self.assertEqual(values[name]["static_layer"]["map_topic"], names["map_topic"])

    def test_live_amcl_counterevidence_invalidates_a_still_fresh_success_file(self):
        frames = names_for_preview("abcdef12", "x")["frames"]
        value = dict(header=dict(frame_id=frames["map"], stamp_secs=100, stamp_nsecs=600000000),
            position=dict(x=1., y=2.), orientation=dict(x=0., y=0., z=math.sin(.15), w=math.cos(.15)),
            covariance=[0.] * 36)
        validate_live_amcl(value, joint(), frames, 100.7)
        for edit in ("covariance", "old", "pose", "yaw"):
            bad = deepcopy(value)
            if edit == "covariance": bad["covariance"][0] = .11 ** 2
            if edit == "old": bad["header"]["stamp_secs"] = 99
            if edit == "pose": bad["position"]["x"] += .04
            if edit == "yaw": bad["orientation"] = dict(x=0., y=0., z=math.sin(.3), w=math.cos(.3))
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                validate_live_amcl(bad, joint(), frames, 100.7)

    def test_scan_stale_or_wrong_frame_rejected_before_matching(self):
        frames = names_for_preview("abcdef12", "x")["frames"]
        value = dict(header=dict(frame_id=frames["laser"], stamp_secs=100, stamp_nsecs=0),
                     ranges=[], angle_min=0., angle_increment=.1, range_min=.1, range_max=8.)
        for now in (100.75, 99.):
            with self.assertRaises(ValueError):
                validate_live_scan(value, [0., 0., 0.], frames, now, {}, [])
        value["header"]["frame_id"] = "map"
        with self.assertRaises(ValueError):
            validate_live_scan(value, [0., 0., 0.], frames, 100.1, {}, [])

    def test_valid_path_is_known_and_does_not_claim_drivability_or_arrival(self):
        result = validate_path(path(), [.2, .5, 0.], [.7, .5, 0.], grid(), grid(), "p2p_abcdef12_map")
        self.assertAlmostEqual(result["path_length_m"], .5)
        self.assertFalse(result["motion_feasibility_validated"])
        self.assertFalse(result["physical_clearance_validated"])
        self.assertTrue(result["tolerances_are_path_representation_checks_not_arrival_policy"])

    def test_path_frame_nonfinite_start_end_and_discontinuity_rejected(self):
        for edit in ("frame", "poseframe", "nan", "start", "end", "jump"):
            bad = path()
            if edit == "frame": bad["frame_id"] = "map"
            if edit == "poseframe": bad["poses"][3]["frame_id"] = "odom"
            if edit == "nan": bad["poses"][3]["x"] = float("nan")
            if edit == "start": bad["poses"][0]["x"] -= .2
            if edit == "end": bad["poses"][-1]["x"] += .2
            if edit == "jump": bad["poses"][5]["x"] += .25
            with self.subTest(edit=edit), self.assertRaises(ValueError):
                validate_path(bad, [.2, .5, 0.], [.7, .5, 0.], grid(), grid(), "p2p_abcdef12_map")

    def test_unknown_saved_cell_and_live_lethal_or_unknown_reject(self):
        for value in (-1, 65, 100):
            saved = grid()
            saved["data"][10 * 30 + 9] = value
            with self.assertRaises(ValueError):
                validate_path(path(), [.2, .5, 0.], [.7, .5, 0.], saved, grid(), "p2p_abcdef12_map")
        for value in (-1, 99, 100):
            live = grid()
            live["data"][10 * 30 + 9] = value
            with self.assertRaises(ValueError):
                validate_path(path(), [.2, .5, 0.], [.7, .5, 0.], grid(), live, "p2p_abcdef12_map")

    def test_segment_sampling_catches_wall_between_returned_vertices(self):
        values = path()
        values["poses"] = values["poses"][::2]
        saved = grid()
        saved["data"][10 * 30 + 9] = 100
        with self.assertRaises(ValueError):
            validate_path(values, [.2, .5, 0.], [.7, .5, 0.], saved, grid(), "p2p_abcdef12_map")

    def test_blocked_requested_goal_cannot_pass_via_endpoint_grid_tolerance(self):
        values = path()
        for index, item in enumerate(values["poses"]):
            item["x"], item["y"] = .225 + index * .05, .525
        values["poses"].pop()
        saved = grid()
        saved["data"][10 * 30 + 14] = 100
        with self.assertRaisesRegex(ValueError, "Requested goal"):
            validate_path(values, [.225, .525, 0.], [.725, .525, 0.], saved, grid(), "p2p_abcdef12_map")

    def test_grid_origin_yaw_conversion_and_outside(self):
        value = grid()
        value["origin"] = [1., 2., math.pi / 2.]
        value["data"][2 * 30 + 1] = 42
        self.assertEqual(grid_cell(value, 1. - .125, 2. + .075), 42)
        with self.assertRaises(ValueError):
            grid_cell(value, 100., 100.)


if __name__ == "__main__":
    unittest.main()
