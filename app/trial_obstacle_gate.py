"""Pure observed-front obstacle diagnostic for one short forward trial.

This module performs no I/O and issues no motion permission. Geometry and the
0.10 m side margin are explicit trial estimates, not measured body dimensions
or user navigation defaults. Rear space and the lidar's minimum-range blind
region remain unobserved. The caller must separately enforce the trial's
no-reverse/no-in-place/limited-heading rules and current localization.
"""
import math


def _finite(value, name, positive=False):
    if type(value) not in (int, float):
        raise ValueError(name + " must be a finite built-in number")
    try:
        result = float(value)
    except (ValueError, OverflowError):
        raise ValueError(name + " must be finite")
    if not math.isfinite(result) or (positive and result <= 0.):
        raise ValueError(name + " must be finite" + (" and positive" if positive else ""))
    return result


def _source_header(header):
    if type(header) is not dict or type(header.get("frame_id")) is not str or not header["frame_id"]:
        raise ValueError("Original nonempty scan frame required")
    result = dict(header)
    if "stamp_secs" in header or "stamp_nsecs" in header:
        secs, nsecs = header.get("stamp_secs"), header.get("stamp_nsecs")
        if (type(secs) is not int or secs < 0 or type(nsecs) is not int
                or not 0 <= nsecs < 1000000000 or (secs, nsecs) == (0, 0)):
            raise ValueError("Original exact nonzero scan stamp required")
        stamp = secs + nsecs / 1e9
        if "source_stamp" in header and abs(_finite(header["source_stamp"], "source stamp") - stamp) > 1e-6:
            raise ValueError("Scan source stamp representations disagree")
    elif _finite(header.get("source_stamp"), "source stamp") <= 0.:
        raise ValueError("Original nonzero scan source stamp required")
    return result


def evaluate_forward_scan(scan, now_monotonic, *, max_source_age_s=.75,
                          body_width_m=.5, front_m=.2, rear_m=.2,
                          laser_x_m=.13, laser_y_m=0., laser_yaw_rad=0.,
                          front_clearance_m=.20, side_margin_m=.10,
                          max_angular_gap_deg=1., expected_frame_id=None):
    """Return an observed-front diagnostic, preserving the original source.

    Scan data use ROS radians/metres and original ``header`` plus conservative
    ``source_monotonic_s`` supplied by the caller. In the front 180 degrees,
    finite returns inside [range_min,range_max] and positive infinity are valid;
    infinity means no return up to range_max. NaN/null/-inf/out-of-range data
    are unknown and cannot clear this gate. Cropped rear NaNs are ignored.

    All beams are checked, with no decimation. Front angular coverage must have
    no gap or missing edge over the explicit angular gap limit. This is a lidar
    ray coverage check, not proof that small objects between rays are absent.
    Obstacle thresholds use <= for front clearance and < for the side margin.
    Invalid observations return ready=False; invalid configuration raises.
    """
    now = _finite(now_monotonic, "current monotonic time")
    timeout = _finite(max_source_age_s, "source timeout", True)
    width = _finite(body_width_m, "body width", True)
    front = _finite(front_m, "body front", True)
    rear = _finite(rear_m, "body rear", True)
    lx, ly, yaw = (_finite(laser_x_m, "laser x"), _finite(laser_y_m, "laser y"),
                   _finite(laser_yaw_rad, "laser yaw"))
    clearance = _finite(front_clearance_m, "front clearance", True)
    margin = _finite(side_margin_m, "trial side margin", True)
    gap_limit = _finite(max_angular_gap_deg, "angular coverage gap", True)
    if now < 0 or gap_limit > 5.:
        raise ValueError("Nonnegative current time and coverage gap no greater than 5 degrees required")
    if expected_frame_id is not None and (type(expected_frame_id) is not str or not expected_frame_id):
        raise ValueError("Expected frame must be a nonempty string")
    half = width / 2.
    corridor = half + margin
    result = dict(ready=False, failure=None, minimum_front_clearance_m=None,
        observed_side_front_clearance_m=None, coverage={}, source_evidence={},
        scope="observed_front180_short_forward_trial_only", motion_permission=False,
        rear_coverage_verified=False, unobserved_is_free=False,
        sensor_minimum_range_blind_region_assessed=False, physical_clearance_verified=False,
        geometry_estimated=True, side_margin_is_trial_candidate_not_user_default=True,
        geometry=dict(body_width_m=width, front_m=front, rear_m=rear,
                      laser_x_m=lx, laser_y_m=ly, laser_yaw_rad=yaw),
        limits=dict(max_source_age_s=timeout, front_clearance_m=clearance,
                    side_margin_m=margin, forward_corridor_half_width_m=corridor,
                    max_angular_gap_deg=gap_limit))
    try:
        if type(scan) is not dict:
            raise ValueError("Scan must be a dictionary")
        header = _source_header(scan.get("header"))
        if expected_frame_id is not None and header["frame_id"] != expected_frame_id:
            raise ValueError("Scan frame differs from this trial laser")
        source = _finite(scan.get("source_monotonic_s"), "original monotonic source")
        age = now - source
        result["source_evidence"] = dict(header=header, source_monotonic_s=source,
                                         checked_monotonic_s=now, source_age_s=age,
                                         source_time_rewritten=False)
        if source < 0 or not 0 <= age < timeout:
            raise ValueError("Scan original source is stale or future")
        amin = _finite(scan.get("angle_min"), "angle minimum")
        increment = _finite(scan.get("angle_increment"), "angle increment")
        lower = _finite(scan.get("range_min"), "range minimum")
        upper = _finite(scan.get("range_max"), "range maximum", True)
        values = scan.get("ranges")
        if (type(values) not in (list, tuple) or not 2 <= len(values) <= 20000
                or increment == 0. or lower < 0. or lower >= upper
                or abs(increment) * (len(values) - 1) > 2. * math.pi + 2. * abs(increment)):
            raise ValueError("Invalid bounded scan geometry/ranges")
        # No-return rays can only clear space up to the advertised range_max.
        required_range = max(math.hypot(front + clearance - lx, y - ly)
                             for y in (-corridor, corridor))
        if upper < required_range:
            raise ValueError("Scan maximum range cannot observe this forward stop envelope")
        angles, unknown, finite_count, infinite_count = [], 0, 0, 0
        min_front, min_side = None, None
        for index, raw in enumerate(values):
            bearing = amin + index * increment + yaw
            if not math.isfinite(bearing):
                raise ValueError("Scan angle arithmetic overflow")
            angle = math.atan2(math.sin(bearing), math.cos(bearing))
            if abs(angle) > math.pi / 2. + 1e-12:
                continue
            angles.append(angle)
            if type(raw) is str and raw in ("inf", "+inf"):
                distance = float("inf")
            elif type(raw) in (int, float):
                try:
                    distance = float(raw)
                except (ValueError, OverflowError):
                    distance = float("nan")
            else:
                distance = float("nan")
            if distance == float("inf"):
                infinite_count += 1
                continue
            if not math.isfinite(distance) or not lower <= distance <= upper:
                unknown += 1
                continue
            finite_count += 1
            x, y = lx + distance * math.cos(angle), ly + distance * math.sin(angle)
            if abs(y) <= corridor:
                value = x - front
                min_front = value if min_front is None else min(min_front, value)
            # Distance to the whole estimated body rectangle, for observable
            # front points only. This does not invent rear-side observations.
            dx, dy = max(-rear - x, 0., x - front), max(abs(y) - half, 0.)
            value = math.hypot(dx, dy)
            min_side = value if min_side is None else min(min_side, value)
        angles.sort()
        max_gap = max((b - a for a, b in zip(angles, angles[1:])), default=math.pi)
        edge = max((angles[0] + math.pi / 2., math.pi / 2. - angles[-1])) if angles else math.pi
        result["minimum_front_clearance_m"] = min_front
        result["observed_side_front_clearance_m"] = min_side
        result["coverage"] = dict(front_beams=len(angles), finite_returns=finite_count,
            positive_infinite_returns=infinite_count, unknown_front_beams=unknown,
            observed_front_beams=finite_count + infinite_count,
            max_beam_gap_deg=math.degrees(max_gap), max_missing_edge_deg=math.degrees(edge),
            range_min_m=lower, range_max_m=upper, required_envelope_range_m=required_range,
            rear_beams_ignored=len(values) - len(angles), front_ray_coverage_complete=False)
        if min_front is not None and min_front <= clearance:
            raise ValueError("Obstacle at or inside front body clearance")
        if min_side is not None and min_side < margin:
            raise ValueError("Observed side/front obstacle too close to body rectangle")
        if unknown:
            raise ValueError("Front scan contains unknown ranges")
        if len(angles) < 3 or math.degrees(max(max_gap, edge)) > gap_limit + 1e-7:
            raise ValueError("Front 180 degree angular coverage is incomplete")
        result["coverage"]["front_ray_coverage_complete"] = True
        result["ready"] = True
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        result["failure"] = str(exc)
    return result
