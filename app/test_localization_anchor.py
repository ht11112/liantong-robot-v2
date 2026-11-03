"""Pure stationary-window tests, with no device or ROS imports."""
import copy
import math
import unittest

from localization_anchor import AnchorViolation, StaticLocalizationAnchor


def anchor(**changes):
    values = dict(boot_id="current-boot", owner_id="new-owner",
                  reference_after_monotonic_s=100., source_timeout_s=.75,
                  max_encoder_change_counts=0, max_encoder_travel_counts=0,
                  max_position_change_m=.002, max_yaw_change_rad=.003,
                  max_linear_speed_m_s=.003, max_angular_speed_rad_s=.005,
                  max_wheel_speed_rpm=.2)
    values.update(changes)
    return StaticLocalizationAnchor(**values)


def sample(sequence=1, left=0, right=0, x=0., y=0., yaw=0., rpm=0.):
    packet = dict(sample_sequence=sequence, speed_tick_ms=sequence * 100,
                  position_left_raw=left, position_right_raw=right,
                  speed_left_rpm=rpm, speed_right_rpm=0.,
                  controller_state=1, controller_error=0, fault_raw=0, valid_mask=31)
    odom = dict(sample_sequence=sequence, mcu_speed_tick_ms=sequence * 100,
                x=x, y=y, yaw=yaw, linear_x=0., angular_z=0., discontinuity=False)
    return packet, odom


def observe(checker, pair, source=100.1, now=100.11, **identity):
    identity.setdefault("boot_id", "current-boot")
    identity.setdefault("owner_id", "new-owner")
    return checker.observe(*pair, source_monotonic_s=source, now_monotonic_s=now, **identity)


class LocalizationAnchorTests(unittest.TestCase):
    def test_only_new_current_source_establishes_anchor(self):
        checker = anchor()
        old = observe(checker, sample(), source=100., now=100.01)
        self.assertFalse(old["anchor_established"])
        current = observe(checker, sample(2), source=100.1, now=100.11)
        self.assertTrue(current["valid"])
        self.assertEqual(current["anchor"]["sample_sequence"], 2)
        self.assertFalse(current["map_localization_valid"])
        self.assertFalse(current["hardware_execution_authorized"])

    def test_raw_encoder_motion_detected_even_when_rpm_and_odom_zero(self):
        checker = anchor()
        observe(checker, sample(left=1234, right=5678))
        with self.assertRaisesRegex(AnchorViolation, "RAW_ENCODER_CHANGED"):
            observe(checker, sample(2, left=1235, right=5678), 100.2, 100.21)
        state = checker.snapshot(100.21)
        self.assertEqual(state["failure"]["details"]["delta_counts"], [1, 0])
        self.assertFalse(state["encoder_distance_conversion_available"])

    def test_move_out_and_back_cannot_hide_absolute_encoder_travel(self):
        checker = anchor(max_encoder_change_counts=1, max_encoder_travel_counts=1)
        observe(checker, sample())
        observe(checker, sample(2, left=1), 100.2, 100.21)
        with self.assertRaisesRegex(AnchorViolation, "RAW_ENCODER_TRAVEL"):
            observe(checker, sample(3, left=0), 100.3, 100.31)

    def test_uint32_encoder_wrap_is_one_count_and_not_billions(self):
        checker = anchor(max_encoder_change_counts=1, max_encoder_travel_counts=1)
        observe(checker, sample(left=0xffffffff, right=0))
        state = observe(checker, sample(2, left=0, right=0xffffffff), 100.2, 100.21)
        self.assertEqual(state["max_encoder_change_counts"], [1, 1])
        self.assertEqual(state["total_absolute_encoder_travel_counts"], [1, 1])

    def test_half_range_encoder_delta_is_ambiguous(self):
        checker = anchor(max_encoder_change_counts=0x7fffffff, max_encoder_travel_counts=0x7fffffff)
        observe(checker, sample())
        with self.assertRaisesRegex(AnchorViolation, "AMBIGUOUS_ENCODER_DELTA"):
            observe(checker, sample(2, left=0x80000000), 100.2, 100.21)

    def test_duplicate_never_refreshes_time_and_deadline_equality_fails(self):
        checker = anchor()
        observe(checker, sample())
        duplicate = observe(checker, sample(), source=100.6, now=100.61)
        self.assertEqual(duplicate["latest"]["source_monotonic_s"], 100.1)
        self.assertEqual(duplicate["duplicate_samples"], 1)
        with self.assertRaisesRegex(AnchorViolation, "SOURCE_EXPIRED"):
            checker.check(now_monotonic_s=100.85, boot_id="current-boot", owner_id="new-owner")

    def test_late_new_source_cannot_erase_expired_window(self):
        checker = anchor()
        observe(checker, sample())
        with self.assertRaisesRegex(AnchorViolation, "SOURCE_EXPIRED"):
            observe(checker, sample(2), source=101., now=101.01)
        self.assertEqual(checker.snapshot(101.01)["accepted_samples"], 1)

    def test_boot_owner_and_clock_changes_latch_failure(self):
        for changes in (dict(owner_id="other-owner"), dict(boot_id="other-boot")):
            checker = anchor()
            observe(checker, sample())
            with self.assertRaisesRegex(AnchorViolation, "OWNER_OR_BOOT_CHANGED"):
                observe(checker, sample(2), 100.2, 100.21, **changes)
        checker = anchor()
        observe(checker, sample())
        with self.assertRaisesRegex(AnchorViolation, "CLOCK_MOVED_BACKWARDS"):
            observe(checker, sample(2), 100.01, 100.02)

    def test_first_failure_is_permanent_and_does_not_reanchor(self):
        checker = anchor()
        observe(checker, sample())
        with self.assertRaises(AnchorViolation):
            observe(checker, sample(2, left=1), 100.2, 100.21)
        initial_failure = checker.snapshot(100.21)["failure"]
        with self.assertRaises(AnchorViolation):
            observe(checker, sample(3), 100.3, 100.31)
        state = checker.snapshot(100.31)
        self.assertEqual(state["failure"], initial_failure)
        self.assertEqual(state["anchor"]["sample_sequence"], 1)

    def test_pose_twist_and_raw_speed_limits_are_independent(self):
        for kind in ("position", "yaw", "twist", "rpm"):
            checker = anchor()
            observe(checker, sample())
            pair = sample(2)
            if kind == "position":
                pair[1]["x"] = .003
            elif kind == "yaw":
                pair[1]["yaw"] = .004
            elif kind == "twist":
                pair[1]["linear_x"] = .004
            else:
                pair[0]["speed_left_rpm"] = .3
            with self.assertRaises(AnchorViolation):
                observe(checker, pair, 100.2, 100.21)

    def test_yaw_wrap_uses_shortest_change(self):
        checker = anchor(max_yaw_change_rad=.003)
        observe(checker, sample(yaw=math.pi - .001))
        state = observe(checker, sample(2, yaw=-math.pi + .001), 100.2, 100.21)
        self.assertTrue(state["valid"])
        self.assertAlmostEqual(state["max_yaw_change_rad"], .002)

    def test_wrong_sample_binding_gap_and_duplicate_mutation_rejected(self):
        for mode in ("binding", "gap", "duplicate"):
            checker = anchor()
            observe(checker, sample())
            pair = sample(2)
            if mode == "binding":
                pair[1]["sample_sequence"] = 3
            elif mode == "gap":
                pair = sample(3)
            else:
                pair = sample(1, left=1)
            with self.assertRaises(AnchorViolation):
                observe(checker, pair, 100.2, 100.21)

    def test_tolerances_must_be_explicit_finite_and_not_booleans(self):
        for changes in (dict(max_encoder_change_counts=True), dict(max_encoder_change_counts=-1),
                        dict(max_position_change_m=float("nan")), dict(max_yaw_change_rad=-1),
                        dict(source_timeout_s=0), dict(max_encoder_travel_counts=0x80000000)):
            with self.assertRaises(ValueError):
                anchor(**changes)
        with self.assertRaises(TypeError):
            StaticLocalizationAnchor(boot_id="b", owner_id="o")

    def test_does_not_mutate_or_retain_caller_mutable_objects(self):
        checker = anchor()
        pair = sample()
        before = copy.deepcopy(pair)
        state = observe(checker, pair)
        self.assertEqual(pair, before)
        state["anchor"]["encoder_raw"][0] = 999
        pair[0]["position_left_raw"] = 888
        self.assertEqual(checker.snapshot(100.11)["anchor"]["encoder_raw"][0], 0)

    def test_nonfinite_future_and_nonadvancing_sources_cannot_enter_window(self):
        for source in (float("nan"), float("inf"), 101.):
            checker = anchor()
            with self.assertRaisesRegex(AnchorViolation, "INVALID_SAMPLE"):
                observe(checker, sample(), source=source, now=100.11)
        checker = anchor()
        observe(checker, sample())
        with self.assertRaisesRegex(AnchorViolation, "SOURCE_DID_NOT_ADVANCE"):
            observe(checker, sample(2), source=100.1, now=100.21)

    def test_overflowing_pose_difference_latches_instead_of_escaping(self):
        checker = anchor(max_position_change_m=1e308, max_yaw_change_rad=1e308)
        observe(checker, sample(x=-1e308, yaw=-1e308))
        with self.assertRaisesRegex(AnchorViolation, "POSE_DELTA_INVALID"):
            observe(checker, sample(2, x=1e308, yaw=1e308), 100.2, 100.21)
        self.assertEqual(checker.snapshot(100.21)["failure"]["code"], "POSE_DELTA_INVALID")


if __name__ == "__main__":
    unittest.main()
