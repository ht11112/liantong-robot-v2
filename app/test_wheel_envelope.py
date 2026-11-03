"""Offline behavior and boundary checks; no robot or filesystem access."""

import math
import unittest

from wheel_envelope import bounded_wheels_from_twist


class WheelEnvelopeTests(unittest.TestCase):
    def convert(self, v=0.0, w=0.0, **changes):
        values = dict(v=v, w=w, max_linear=0.03, max_angular_deg=6.9,
                      diameter=0.2, separation=0.3, max_rpm=5)
        values.update(changes)
        return bounded_wheels_from_twist(**values)

    def test_straight_limit_uses_two_rpm_not_rounded_three(self):
        result = self.convert(v=0.03)
        self.assertEqual(result["wire_target"], {"left_rpm": 2, "right_rpm": -2})
        self.assertAlmostEqual(result["estimated"]["linear_mps"], 0.020943951023931956)
        self.assertEqual(result["estimated"]["angular_rad_s"], 0.0)
        self.assertLessEqual(result["estimated"]["linear_mps"], 0.03)
        self.assertAlmostEqual(result["quantization_error"]["linear_mps"],
                               0.020943951023931956 - 0.03)
        self.assertEqual(result["candidates_enumerated"], 121)

    def test_pure_turn_keeps_symmetry_and_uses_one_rpm(self):
        result = self.convert(w=math.radians(6.9))
        self.assertEqual(result["physical_wheels"], {"left_rpm": -1, "right_rpm": 1})
        self.assertEqual(result["wire_target"], {"left_rpm": -1, "right_rpm": -1})
        self.assertEqual(result["estimated"]["linear_mps"], 0.0)
        self.assertAlmostEqual(result["estimated"]["angular_deg_s"], 4.0)
        self.assertLessEqual(result["estimated"]["angular_deg_s"], 6.9)

    def test_combined_axes_are_bounded_after_quantization(self):
        result = self.convert(v=0.03, w=math.radians(6.9))
        self.assertEqual(result["physical_wheels"], {"left_rpm": 1, "right_rpm": 4})
        self.assertAlmostEqual(result["estimated"]["linear_mps"], math.pi / 120.0)
        self.assertAlmostEqual(result["estimated"]["angular_deg_s"], 6.0)
        self.assertFalse(result["zero_quantized"])

    def test_request_clipping_uses_maxima(self):
        clipped = self.convert(v=10.0, w=10.0)
        normal = self.convert(v=0.03, w=math.radians(6.9))
        self.assertTrue(clipped["request_clipped"])
        self.assertEqual(clipped["wire_target"], normal["wire_target"])
        self.assertEqual(clipped["bounded_request"], normal["bounded_request"])
        self.assertEqual(clipped["quantization_error"], normal["quantization_error"])

    def test_zero_request_is_zero_without_quantization_warning(self):
        result = self.convert()
        self.assertTrue(result["zero_output"])
        self.assertFalse(result["zero_quantized"])
        self.assertEqual(result["quantization_error"]["normalized_squared"], 0.0)

    def test_small_request_may_quantize_to_zero(self):
        for v, w in ((0.001, 0.0), (-0.001, 0.0),
                     (0.0, math.radians(0.1)), (0.0, -math.radians(0.1)),
                     (0.001, math.radians(0.1))):
            with self.subTest(v=v, w=w):
                result = self.convert(v=v, w=w)
                self.assertTrue(result["zero_quantized"])
                self.assertEqual(result["wire_target"], {"left_rpm": 0, "right_rpm": 0})

    def test_one_axis_can_quantize_to_zero_and_is_reported(self):
        result = self.convert(v=0.03, w=math.radians(0.01))
        self.assertEqual(result["physical_wheels"], {"left_rpm": 2, "right_rpm": 2})
        self.assertEqual(result["axis_quantized_to_zero"], {"linear": False, "angular": True})

    def test_linear_ceiling_threshold_is_strict(self):
        one_rpm = 0.2 * (math.pi / 60.0)
        below = self.convert(v=one_rpm, max_linear=one_rpm - 1e-12)
        exact = self.convert(v=one_rpm, max_linear=one_rpm)
        self.assertTrue(below["zero_quantized"])
        self.assertEqual(exact["physical_wheels"], {"left_rpm": 1, "right_rpm": 1})

    def test_angular_ceiling_threshold_is_strict(self):
        below = self.convert(w=math.radians(4.0), max_angular_deg=4.0 - 1e-10)
        exact = self.convert(w=math.radians(4.0), max_angular_deg=4.0)
        above = self.convert(w=math.radians(4.0), max_angular_deg=4.0 + 1e-10)
        self.assertTrue(below["zero_quantized"])
        self.assertEqual(exact["physical_wheels"], {"left_rpm": -1, "right_rpm": 1})
        self.assertEqual(above["physical_wheels"], {"left_rpm": -1, "right_rpm": 1})

    def test_reverse_and_mirror_equivariance(self):
        for v in (0.0, 0.001, 0.012, 0.03, -0.021):
            for w_deg in (0.0, 0.1, 2.5, 6.9, -4.1):
                w = math.radians(w_deg)
                forward = self.convert(v=v, w=w)
                reverse = self.convert(v=-v, w=-w)
                mirror = self.convert(v=v, w=-w)
                left = forward["physical_wheels"]["left_rpm"]
                right = forward["physical_wheels"]["right_rpm"]
                with self.subTest(v=v, w=w):
                    self.assertEqual(reverse["physical_wheels"],
                                     {"left_rpm": -left, "right_rpm": -right})
                    self.assertEqual(mirror["physical_wheels"],
                                     {"left_rpm": right, "right_rpm": left})

    def test_all_integer_pairs_are_reachable_with_sufficient_axis_caps(self):
        # Independent inverse formula: all 121 legal targets must be fixed points.
        for left in range(-5, 6):
            for right in range(-5, 6):
                v = ((left + right) * math.pi * 0.2) / 120.0
                w = ((right - left) * math.pi * 0.2) / (60.0 * 0.3)
                result = self.convert(v=v, w=w, max_linear=0.2, max_angular_deg=50.0)
                with self.subTest(left=left, right=right):
                    self.assertEqual(result["physical_wheels"],
                                     {"left_rpm": left, "right_rpm": right})

    def test_signs_caps_and_nominal_interface_cap_over_request_grid(self):
        for v_index in range(-12, 13):
            for w_index in range(-12, 13):
                v = v_index * 0.004
                w = math.radians(w_index)
                result = self.convert(v=v, w=w)
                actual = result["estimated"]
                with self.subTest(v=v, w=w):
                    self.assertLessEqual(abs(actual["linear_mps"]), 0.03)
                    self.assertLessEqual(abs(actual["angular_rad_s"]), math.radians(6.9))
                    self.assertLessEqual(abs(actual["angular_deg_s"]), 6.9)
                    self.assertGreaterEqual(actual["linear_mps"] * v, 0.0)
                    self.assertGreaterEqual(actual["angular_rad_s"] * w, 0.0)
                    self.assertTrue(all(type(rpm) is int and abs(rpm) <= 5
                                        for rpm in result["wire_target"].values()))

    def test_lower_or_zero_nominal_rpm_cap(self):
        one = self.convert(v=0.03, max_rpm=1)
        zero = self.convert(v=0.03, max_rpm=0)
        self.assertEqual(one["wire_target"], {"left_rpm": 1, "right_rpm": -1})
        self.assertEqual(one["candidates_enumerated"], 9)
        self.assertTrue(zero["zero_quantized"])
        self.assertEqual(zero["candidates_enumerated"], 1)

    def test_geometry_is_explicitly_an_unverified_estimate(self):
        result = self.convert(v=0.03)
        self.assertEqual(result["geometry_estimate"],
                         {"diameter_m": 0.2, "separation_m": 0.3, "verified": False})
        self.assertFalse(result["hardware_verified"])
        self.assertIn("not_verified_hardware", result["limits"]["rpm_cap_basis"])

    def test_reject_invalid_numeric_inputs(self):
        class FakeInt(int):
            pass

        class FakeFloat(float):
            pass

        for name in ("v", "w", "max_linear", "max_angular_deg", "diameter", "separation"):
            for invalid in (True, False, float("nan"), float("inf"), -float("inf"),
                            "0.03", None, FakeInt(1), FakeFloat(1.0), object(), 10 ** 1000):
                with self.subTest(name=name, invalid_type=type(invalid)):
                    with self.assertRaises(ValueError):
                        self.convert(**{name: invalid})
        for name in ("max_linear", "max_angular_deg", "diameter", "separation"):
            for invalid in (0, -1):
                with self.subTest(name=name, invalid=invalid):
                    with self.assertRaises(ValueError):
                        self.convert(**{name: invalid})
        for invalid in (-1, 6, True, 5.0, FakeInt(5), "5", float("nan")):
            with self.subTest(max_rpm=invalid):
                with self.assertRaises(ValueError):
                    self.convert(max_rpm=invalid)

    def test_reject_nonfinite_derived_geometry(self):
        with self.assertRaises(ValueError):
            self.convert(diameter=1e308, separation=1e-308)


if __name__ == "__main__":
    unittest.main()
