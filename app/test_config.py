import copy
import json
import math
from pathlib import Path
import unittest

from config import ConfigFormatError, USER_FIELDS, load_config, loads_config, validate_config


ROOT = Path(__file__).resolve().parent


def complete_config():
    value = load_config(ROOT / "site_config.template.json")
    value["user"].update(dict(
        max_linear_m_s=.03, max_angular_deg_s=6.9,
        front_clearance_cm=20., side_clearance_cm=10., rear_clearance_cm=None,
        position_tolerance_cm=8., heading_required=False, heading_tolerance_deg=None,
        max_duration_s=90., max_path_m=2., feedback_source_timeout_ms=800.,
        scan_source_timeout_ms=700., localization_timeout_ms=1500.,
        command_timeout_ms=600., supervisor_timeout_ms=700.,
        stop_confirmation_timeout_s=2., allow_reverse=False,
        obstacle_mode="stop", connection_loss_action="continue_local_task"))
    return value


def codes(report, kind="errors"):
    return {item["code"] for item in report[kind]}


class ConfigTests(unittest.TestCase):
    def test_template_has_no_confirmed_user_values(self):
        value = load_config(ROOT / "site_config.template.json")
        self.assertEqual(set(value["user"]), set(USER_FIELDS))
        self.assertTrue(all(item is None for item in value["user"].values()))
        report = validate_config(value)
        self.assertEqual(report["errors"], [])
        self.assertEqual(len(report["pending"]), len(USER_FIELDS))
        self.assertFalse(report["effective"]["hardware_execution_authorized"])

    def test_site_only_records_nine_user_confirmations(self):
        value = load_config(ROOT / "site_config.json")
        actual = {key: item for key, item in value["user"].items() if item is not None}
        self.assertEqual(actual, dict(max_linear_m_s=.03, max_angular_deg_s=6.9,
                                     front_clearance_cm=20,
                                     max_duration_s=200,
                                     position_tolerance_cm=10, heading_required=False,
                                     allow_reverse=False,
                                     feedback_source_timeout_ms=750,
                                     connection_loss_action="continue_local_task"))
        report = validate_config(value)
        self.assertEqual(report["errors"], [])
        self.assertEqual({item["path"] for item in report["pending"]}, {
            "user." + key for key, item in value["user"].items()
            if item is None and key not in ("heading_tolerance_deg", "rear_clearance_cm")})
        self.assertIsNone(value["user"]["obstacle_mode"])
        semantics = report["effective"]["feedback_timeout_semantics"]
        self.assertEqual(semantics["source_age_origin"], "original_sample_first_observation")
        self.assertFalse(semantics["duplicate_sample_refreshes_age"])
        self.assertFalse(semantics["fault_or_cancel_waits_for_timeout"])
        self.assertIn("GEOMETRY_UNCONFIRMED", codes(report, "warnings"))
        self.assertIn("FIRMWARE_NOT_CURRENTLY_VERIFIED", codes(report, "warnings"))

    def test_complete_requires_all_conditional_fields(self):
        report = validate_config(load_config(ROOT / "site_config.json"), require_complete=True)
        self.assertIn("REQUIRED_VALUE_MISSING", codes(report))
        good = validate_config(complete_config(), require_complete=True)
        self.assertEqual(good["errors"], [])
        self.assertEqual(good["pending"], [])

    def test_heading_and_reverse_conditions(self):
        value = complete_config()
        value["user"].update(heading_required=True, allow_reverse=True)
        report = validate_config(value, True)
        paths = {item["path"] for item in report["pending"]}
        self.assertEqual(paths, {"user.heading_tolerance_deg", "user.rear_clearance_cm"})
        self.assertIn("REVERSE_WITH_UNOBSERVED_REAR", codes(report, "warnings"))
        value["user"].update(heading_tolerance_deg=10., rear_clearance_cm=20.)
        self.assertEqual(validate_config(value, True)["errors"], [])

    def test_no_inherited_250ms_host_cap(self):
        report = validate_config(complete_config(), True)
        self.assertEqual(report["errors"], [])
        self.assertEqual(report["effective"]["user"]["feedback_source_timeout_ms"], 800.)
        self.assertIn("HOST_SOURCE_BUDGET_EXCEEDS_CAN_BASELINE", codes(report, "warnings"))
        self.assertIn("HOST_BUDGET_EXCEEDS_MCU_LEASE", codes(report, "warnings"))

    def test_firmware_entries_are_read_only_and_unverified(self):
        for key, replacement in (("baseline_lease_ms", 1000),
                                 ("baseline_max_wheel_rpm", 6),
                                 ("verified_current", True),
                                 ("baseline_lease_ms", True)):
            value = complete_config()
            value["firmware"][key] = replacement
            self.assertIn("READ_ONLY_BASELINE", codes(validate_config(value)))

    def test_rear_coverage_cannot_be_asserted_in_draft(self):
        value = complete_config()
        value["sensing"]["rear_coverage_confirmed"] = True
        self.assertIn("READ_ONLY_DESIGN_SETTING", codes(validate_config(value)))

    def test_new_180_degree_design_preserves_raw_and_unknown(self):
        for filename in ("site_config.json", "site_config.template.json"):
            value = load_config(ROOT / filename)
            report = validate_config(value)
            self.assertEqual(report["errors"], [])
            scan = report["effective"]["sensing_design"]
            self.assertEqual(scan["navigation_fov_deg"], 180)
            self.assertEqual(scan["rear_unobserved_deg"], 180)
            self.assertFalse(scan["rear_coverage_confirmed"])
            self.assertTrue(scan["raw_scan_preserved"])
            self.assertFalse(scan["unobserved_is_free"])
        value["sensing"]["unobserved_is_free"] = True
        self.assertIn("READ_ONLY_DESIGN_SETTING", codes(validate_config(value)))

    def test_observe_only_keeps_planner_and_scan_health(self):
        value = complete_config()
        value["user"]["obstacle_mode"] = "observe_only"
        report = validate_config(value, True)
        self.assertEqual(report["errors"], [])
        self.assertFalse(report["effective"]["independent_obstacle_stop_enabled"])
        self.assertTrue(report["effective"]["planner_collision_check_required"])
        self.assertTrue(report["effective"]["scan_health_required"])
        self.assertIn("OBSTACLES_OBSERVE_ONLY", codes(report, "warnings"))

    def test_unknown_keys_at_every_level_rejected(self):
        for group in (None, "user", "geometry", "firmware", "sensing"):
            value = complete_config()
            (value if group is None else value[group])["surprise"] = 123
            self.assertIn("UNKNOWN_KEY", codes(validate_config(value)))

    def test_duplicate_json_keys_rejected_at_all_levels(self):
        for raw in ('{"user":1,"user":2}', '{"user":{"a":1,"a":2}}'):
            with self.assertRaises(ConfigFormatError):
                loads_config(raw)

    def test_nonfinite_json_extensions_rejected(self):
        for constant in ("NaN", "Infinity", "-Infinity", "1e999", "-1e999"):
            with self.assertRaises(ConfigFormatError):
                loads_config('{"x":' + constant + '}')

    def test_nonfinite_and_bool_numeric_values_rejected(self):
        for item in (float("nan"), float("inf"), -float("inf"), True, False, "800", 10 ** 1000):
            value = complete_config()
            value["user"]["feedback_source_timeout_ms"] = item
            self.assertIn("FINITE_NUMBER_REQUIRED", codes(validate_config(value)))

    def test_negative_zero_and_invalid_enums(self):
        for key, item in (("front_clearance_cm", -1), ("max_duration_s", 0),
                          ("heading_tolerance_deg", 181), ("allow_reverse", 1),
                          ("obstacle_mode", "disabled"), ("connection_loss_action", "retry")):
            value = complete_config()
            value["user"][key] = item
            self.assertTrue(validate_config(value)["errors"], (key, item))

    def test_unknown_nested_shapes_and_bad_root_rejected(self):
        for value in (None, [], 1, "text"):
            self.assertIn("OBJECT_REQUIRED", codes(validate_config(value)))
        value = complete_config()
        value["geometry"] = []
        self.assertIn("OBJECT_REQUIRED", codes(validate_config(value)))

    def test_large_upper_limits_warn_and_do_not_enlarge_firmware(self):
        value = complete_config()
        value["user"]["max_linear_m_s"] = .1
        report = validate_config(value)
        self.assertEqual(report["errors"], [])
        self.assertIn("REQUESTED_ENVELOPE_EXCEEDS_FIRMWARE_CAPABILITY", codes(report, "warnings"))
        self.assertFalse(report["effective"]["wheel_envelope"]["automatic_scaling"])
        self.assertFalse(report["effective"]["wheel_envelope"]["full_continuous_rectangle_supported"])
        self.assertEqual(report["effective"]["wheel_envelope"]["integer_command_rpm_range"], 5)
        self.assertEqual(report["effective"]["quantization"]["straight_max_integer_rpm_within_speed_limit"], 5)

    def test_partial_speed_envelope_still_checks_firmware_boundary(self):
        value = load_config(ROOT / "site_config.template.json")
        value["user"]["max_linear_m_s"] = .1
        report = validate_config(value)
        self.assertIn("REQUESTED_ENVELOPE_EXCEEDS_FIRMWARE_CAPABILITY", codes(report, "warnings"))
        self.assertFalse(report["effective"]["wheel_envelope"]["complete"])

    def test_connection_policy_matches_task_model(self):
        for action in ("continue_local_task", "pause", "cancel"):
            value = complete_config()
            value["user"]["connection_loss_action"] = action
            self.assertEqual(validate_config(value, True)["errors"], [])
        value["user"]["connection_loss_action"] = "stop"
        self.assertIn("ENUM_REQUIRED", codes(validate_config(value)))

    def test_complete_design_does_not_certify_deployment(self):
        report = validate_config(complete_config(), True)
        self.assertEqual(report["errors"], [])
        self.assertEqual(report["effective"]["validation_scope"], "offline_design_inputs_only")
        self.assertEqual(report["effective"]["deployment_readiness"], "not_assessed")
        self.assertFalse(report["effective"]["hardware_execution_authorized"])

    def test_invalid_report_remains_strict_json_serializable(self):
        value = complete_config()
        value["geometry"]["wheel_diameter_m"] = float("nan")
        value["user"]["scan_source_timeout_ms"] = float("inf")
        report = validate_config(value)
        self.assertTrue(report["errors"])
        json.dumps(report, allow_nan=False)

    def test_user_limits_are_not_rounded_up_to_overspeed(self):
        report = validate_config(complete_config())
        quant = report["effective"]["quantization"]
        self.assertFalse(quant["nearest_rounding_permitted"])
        self.assertEqual(quant["straight_max_integer_rpm_within_speed_limit"], 2)
        self.assertEqual(quant["in_place_max_integer_rpm_within_turn_limit"], 1)
        self.assertLessEqual(quant["straight_max_representable_m_s"], .03)
        self.assertLessEqual(quant["in_place_max_representable_deg_s"], 6.9)
        self.assertAlmostEqual(quant["in_place_turn_step_deg_s"], 4.)

    def test_straight_integer_rpm_just_below_at_and_above_each_boundary(self):
        for rpm in range(1, 6):
            boundary = (2 * rpm) * ((.2 * (math.pi / 60.)) / 2.)
            for factor, expected in ((1. - 1e-13, rpm - 1), (1., rpm),
                                     (1. + 1e-13, rpm)):
                value = complete_config()
                ceiling = boundary * factor
                value["user"]["max_linear_m_s"] = ceiling
                report = validate_config(value)
                self.assertEqual(report["errors"], [])
                quant = report["effective"]["quantization"]
                self.assertEqual(quant["straight_max_integer_rpm_within_speed_limit"], expected)
                self.assertLessEqual(quant["straight_max_representable_m_s"], ceiling)

    def test_turn_integer_rpm_just_below_at_and_above_each_boundary(self):
        for rpm in range(1, 6):
            boundary = (2 * rpm) * ((.2 / .3) * 3.)
            for factor, expected in ((1. - 1e-13, rpm - 1), (1., rpm),
                                     (1. + 1e-13, rpm)):
                value = complete_config()
                ceiling = boundary * factor
                value["user"]["max_angular_deg_s"] = ceiling
                report = validate_config(value)
                self.assertEqual(report["errors"], [])
                quant = report["effective"]["quantization"]
                self.assertEqual(quant["in_place_max_integer_rpm_within_turn_limit"], expected)
                self.assertLessEqual(quant["in_place_max_representable_deg_s"], ceiling)

    def test_timeout_distance_is_estimate_excluding_braking(self):
        report = validate_config(complete_config())
        self.assertAlmostEqual(report["effective"]["distance_during_timeout_m"]["feedback_source_timeout_ms"], .024)
        self.assertTrue(report["effective"]["timeout_distance_excludes_braking_and_scheduling"])

    def test_geometry_and_arithmetic_extremes_fail_without_exception(self):
        for diameter in (True, -1., 0., float("inf"), 5e-324):
            value = complete_config()
            value["geometry"]["wheel_diameter_m"] = diameter
            self.assertTrue(validate_config(value)["errors"])
        value = complete_config()
        value["user"]["max_linear_m_s"] = 1e308
        value["user"]["feedback_source_timeout_ms"] = 1e308
        self.assertTrue(validate_config(value)["errors"])

    def test_input_is_not_mutated(self):
        value = complete_config()
        before = copy.deepcopy(value)
        validate_config(value)
        self.assertEqual(value, before)


if __name__ == "__main__":
    unittest.main()
