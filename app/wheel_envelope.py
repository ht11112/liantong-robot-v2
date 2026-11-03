"""Pure offline integer-RPM envelope for an estimated differential drive.

``w`` uses radians/second, as in a Twist.  Wire 9 represents each wheel as an
integer RPM, with a negative sign for the physical right wheel's forward
direction.  Nothing here opens a port, issues a command, or verifies hardware.
The caller supplies geometry; diameter=0.2 m and separation=0.3 m are estimates.
"""

import math


def _number(name, value, positive=False):
    # Exact built-in types avoid bool and numeric subclasses with custom hooks.
    if type(value) not in (int, float):
        raise ValueError(name + " must be a finite built-in int or float")
    try:
        result = float(value)
    except (OverflowError, ValueError):
        raise ValueError(name + " must be finite") from None
    if not math.isfinite(result) or (positive and result <= 0.0):
        raise ValueError(name + " must be finite" + (" and positive" if positive else ""))
    return result


def _clip(value, limit):
    return max(-limit, min(limit, value))


def _same_axis_direction(actual, requested):
    if requested == 0.0:
        return actual == 0.0
    return actual == 0.0 or (actual > 0.0) == (requested > 0.0)


def bounded_wheels_from_twist(v, w, max_linear, max_angular_deg, diameter,
                              separation, max_rpm=5):
    """Return a bounded integer wheel target and its estimated inverse Twist.

    Requests are clipped per axis to the supplied speed ceilings first.  Every
    integer physical wheel pair in [-max_rpm, max_rpm] is considered (121 pairs
    at the nominal cap of 5).  A candidate must obey both estimated chassis
    speed ceilings without a floating-point tolerance and must not reverse
    either requested chassis axis.  A pure straight request requires equal
    physical wheel RPM; a pure turn requires opposite physical wheel RPM.

    Selection minimizes squared error normalized by the two chassis ceilings.
    Ties prefer less wheel effort, then less linear error.  No minimum speed is
    forced: a nonzero request can quantize to zero.  The requested ceilings are
    maxima, not promises that the estimated vehicle reaches those exact speeds.
    max_rpm is an exact built-in integer from 0 through 5; 0 disables output.

    The returned wire target is data only.  Geometry, inverse velocity and the
    RPM cap are explicitly unverified estimates, not measured motion or proof
    that a chassis controller, protocol exchange, or navigation task works.
    """
    v = _number("v", v)
    w = _number("w", w)
    max_linear = _number("max_linear", max_linear, positive=True)
    max_angular_deg = _number("max_angular_deg", max_angular_deg, positive=True)
    diameter = _number("diameter", diameter, positive=True)
    separation = _number("separation", separation, positive=True)
    if type(max_rpm) is not int or not 0 <= max_rpm <= 5:
        raise ValueError("max_rpm must be an exact built-in int from 0 through 5")

    max_angular = math.radians(max_angular_deg)
    wheel_mps_per_rpm = diameter * (math.pi / 60.0)
    # pi cancels when expressing the inverse yaw rate in degrees/second.
    angular_deg_per_rpm_difference = (diameter / separation) * 3.0
    if (not math.isfinite(max_angular) or max_angular <= 0.0
            or not math.isfinite(wheel_mps_per_rpm) or wheel_mps_per_rpm <= 0.0
            or not math.isfinite(angular_deg_per_rpm_difference)
            or angular_deg_per_rpm_difference <= 0.0
            or not math.isfinite(wheel_mps_per_rpm * max_rpm)
            or not math.isfinite(angular_deg_per_rpm_difference * (2 * max_rpm))):
        raise ValueError("geometry or angular ceiling is outside finite model range")

    target_v = _clip(v, max_linear)
    target_w = _clip(w, max_angular)
    feasible = 0
    best = None
    for left in range(-max_rpm, max_rpm + 1):
        for right in range(-max_rpm, max_rpm + 1):
            if target_w == 0.0 and left != right:
                continue
            if target_v == 0.0 and left != -right:
                continue
            # Integer sums/differences preserve exact symmetry at zero.
            actual_v = (left + right) * (wheel_mps_per_rpm / 2.0)
            actual_w_deg = (right - left) * angular_deg_per_rpm_difference
            actual_w = math.radians(actual_w_deg)
            if (abs(actual_v) > max_linear or abs(actual_w) > max_angular
                    or abs(actual_w_deg) > max_angular_deg):
                continue
            if (not _same_axis_direction(actual_v, target_v)
                    or not _same_axis_direction(actual_w, target_w)):
                continue
            feasible += 1
            error_v = actual_v - target_v
            error_w = actual_w - target_w
            normalized_v = abs(error_v) / max_linear
            normalized_w = abs(error_w) / max_angular
            score = normalized_v * normalized_v + normalized_w * normalized_w
            # All tie terms are invariant under reverse and left/right mirror.
            rank = (score, abs(left) + abs(right), normalized_v, normalized_w,
                    abs(actual_v), abs(actual_w))
            candidate = (rank, left, right, actual_v, actual_w, actual_w_deg, error_v, error_w)
            if best is None or rank < best[0]:
                best = candidate

    # The all-zero pair always passes finite positive limits and direction rules.
    rank, left, right, actual_v, actual_w, actual_w_deg, error_v, error_w = best
    requested_nonzero = v != 0.0 or w != 0.0
    zero_output = left == 0 and right == 0
    return {
        "wire_target": {"left_rpm": left, "right_rpm": -right},
        "physical_wheels": {"left_rpm": left, "right_rpm": right},
        "requested": {"linear_mps": v, "angular_rad_s": w},
        "bounded_request": {"linear_mps": target_v, "angular_rad_s": target_w,
                            "angular_deg_s": math.degrees(target_w)},
        "request_clipped": target_v != v or target_w != w,
        "estimated": {"linear_mps": actual_v, "angular_rad_s": actual_w,
                      "angular_deg_s": actual_w_deg},
        "quantization_error": {"linear_mps": error_v, "angular_rad_s": error_w,
                               "angular_deg_s": math.degrees(error_w),
                               "normalized_squared": rank[0]},
        "zero_output": zero_output,
        "zero_quantized": requested_nonzero and zero_output,
        "axis_quantized_to_zero": {
            "linear": target_v != 0.0 and actual_v == 0.0,
            "angular": target_w != 0.0 and actual_w == 0.0,
        },
        "limits": {"linear_mps": max_linear, "angular_deg_s": max_angular_deg,
                   "integer_rpm_cap": max_rpm,
                   "rpm_cap_basis": "nominal_interface_cap_not_verified_hardware_limit"},
        "geometry_estimate": {"diameter_m": diameter, "separation_m": separation,
                              "verified": False},
        "model": "differential_drive_no_slip_estimate",
        "hardware_verified": False,
        "candidates_enumerated": (2 * max_rpm + 1) ** 2,
        "candidates_feasible": feasible,
    }
