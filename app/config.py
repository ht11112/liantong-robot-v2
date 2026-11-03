"""Draft configuration for the independent point-to-point design.

Pure Python 3.8+ and no device, ROS, network or motion authority.  A complete
configuration is still only a reviewed design input, never a hardware permit.
"""
import json
import math
from pathlib import Path


SCHEMA = "p2p-navigation-config-v1"
USER_FIELDS = (
    "max_linear_m_s", "max_angular_deg_s", "front_clearance_cm",
    "side_clearance_cm", "rear_clearance_cm", "position_tolerance_cm",
    "heading_required", "heading_tolerance_deg", "max_duration_s", "max_path_m",
    "feedback_source_timeout_ms", "scan_source_timeout_ms",
    "localization_timeout_ms", "command_timeout_ms", "supervisor_timeout_ms",
    "stop_confirmation_timeout_s", "allow_reverse", "obstacle_mode",
    "connection_loss_action",
)
GEOMETRY_FIELDS = (
    "wheel_diameter_m", "wheel_separation_m", "planning_width_m",
    "front_from_axle_m", "rear_from_axle_m", "confirmed",
)
FIRMWARE_BASELINE = {
    "baseline_lease_ms": 500,
    "baseline_max_wheel_rpm": 5,
    "baseline_can_source_timeout_ms": 250,
    "baseline_feedback_stop_rpm": 15,
    "verified_current": False,
}
SENSING_DESIGN = {
    "navigation_fov_deg": 180,
    "rear_unobserved_deg": 180,
    "rear_coverage_confirmed": False,
    "raw_scan_preserved": True,
    "unobserved_is_free": False,
}
_TOP_FIELDS = {"schema", "user", "geometry", "firmware", "sensing"}
_BOOL_FIELDS = {"heading_required", "allow_reverse"}
_ENUM_FIELDS = {
    "obstacle_mode": ("stop", "observe_only"),
    "connection_loss_action": ("continue_local_task", "pause", "cancel"),
}
_CLEARANCES = {"front_clearance_cm", "side_clearance_cm", "rear_clearance_cm"}


class ConfigFormatError(ValueError):
    """The JSON text cannot be interpreted without ambiguity."""


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ConfigFormatError("Duplicate JSON key: " + key)
        result[key] = value
    return result


def _reject_constant(value):
    raise ConfigFormatError("Non-finite JSON number: " + value)


def _finite_float(text):
    value = float(text)
    if not math.isfinite(value):
        raise ConfigFormatError("JSON number overflows a finite float: " + text)
    return value


def loads_config(text):
    """Parse strict JSON; duplicate keys and NaN/Infinity are always rejected."""
    try:
        return json.loads(text, object_pairs_hook=_unique_object,
                          parse_constant=_reject_constant, parse_float=_finite_float)
    except ConfigFormatError:
        raise
    except (ValueError, TypeError, OverflowError) as exc:
        raise ConfigFormatError(str(exc)) from exc


def load_config(path):
    return loads_config(Path(path).read_text(encoding="utf-8-sig"))


def _finite_number(value):
    if type(value) not in (int, float):
        return False
    try:
        return math.isfinite(value)
    except (ValueError, OverflowError, TypeError):
        return False


def validate_config(config, require_complete=False):
    """Return errors/pending/warnings/effective; never issue execution authority.

    Issues have stable ``code``, ``path`` and ``message`` fields.  Null draft
    values remain null.  Firmware entries describe a read-only, unverified
    historical baseline, so changing a host budget cannot change that baseline.
    ``require_complete`` checks design inputs only; it never certifies deployment.
    """
    if type(require_complete) is not bool:
        raise TypeError("require_complete must be bool")
    report = {"errors": [], "pending": [], "warnings": [], "effective": {}}

    def issue(kind, code, path, message):
        report[kind].append(dict(code=code, path=path, message=message))

    def error(code, path, message):
        issue("errors", code, path, message)

    def warning(code, path, message):
        issue("warnings", code, path, message)

    def pending(path, message):
        issue("pending", "VALUE_PENDING", path, message)
        if require_complete:
            error("REQUIRED_VALUE_MISSING", path, message)

    def object_at(parent, key, allowed):
        value = parent.get(key)
        if type(value) is not dict:
            error("OBJECT_REQUIRED", key, "Expected a configuration object")
            return {}
        for field in value:
            if type(field) is not str or field not in allowed:
                error("UNKNOWN_KEY", key + "." + str(field), "Unknown configuration key")
        return value

    if type(config) is not dict:
        error("OBJECT_REQUIRED", "$", "Configuration root must be an object")
        return report
    for key in config:
        if type(key) is not str or key not in _TOP_FIELDS:
            error("UNKNOWN_KEY", str(key), "Unknown configuration key")
    if config.get("schema") != SCHEMA or type(config.get("schema")) is not str:
        error("SCHEMA_MISMATCH", "schema", "Expected " + SCHEMA)

    user = object_at(config, "user", set(USER_FIELDS))
    geometry = object_at(config, "geometry", set(GEOMETRY_FIELDS))
    firmware = object_at(config, "firmware", set(FIRMWARE_BASELINE))
    sensing = object_at(config, "sensing", set(SENSING_DESIGN))
    valid = {}
    for name in USER_FIELDS:
        value = user.get(name)
        path = "user." + name
        optional = ((name == "heading_tolerance_deg" and user.get("heading_required") is False)
                    or (name == "rear_clearance_cm" and user.get("allow_reverse") is False))
        if value is None:
            if not optional:
                pending(path, "User choice is required")
            valid[name] = None
            continue
        if name in _BOOL_FIELDS:
            if type(value) is not bool:
                error("BOOLEAN_REQUIRED", path, "Expected true or false")
            else:
                valid[name] = value
        elif name in _ENUM_FIELDS:
            if type(value) is not str or value not in _ENUM_FIELDS[name]:
                error("ENUM_REQUIRED", path, "Expected one of: " + ", ".join(_ENUM_FIELDS[name]))
            else:
                valid[name] = value
        elif not _finite_number(value):
            error("FINITE_NUMBER_REQUIRED", path, "Expected a finite number; booleans are not numbers")
        elif value < 0 or (value == 0 and name not in _CLEARANCES):
            error("OUT_OF_RANGE", path, "Value must be " + ("nonnegative" if name in _CLEARANCES else "positive"))
        elif name == "heading_tolerance_deg" and value > 180:
            error("OUT_OF_RANGE", path, "Heading tolerance cannot exceed 180 degrees")
        else:
            valid[name] = value
            if name in _CLEARANCES and value == 0:
                warning("ZERO_CLEARANCE", path, "Zero adds no independent clearance beyond the body envelope")

    geometry_valid = {}
    for name in GEOMETRY_FIELDS:
        value = geometry.get(name)
        path = "geometry." + name
        if name == "confirmed":
            if type(value) is not bool:
                error("BOOLEAN_REQUIRED", path, "Geometry confirmation must be a boolean")
            elif not value:
                warning("GEOMETRY_UNCONFIRMED", path, "Geometry is estimated; RPM calculations are design estimates")
        elif not _finite_number(value):
            error("FINITE_NUMBER_REQUIRED", path, "Expected a finite geometry number")
        elif value <= 0:
            error("OUT_OF_RANGE", path, "Geometry dimensions must be positive")
        else:
            geometry_valid[name] = value

    for name, expected in FIRMWARE_BASELINE.items():
        actual = firmware.get(name)
        if type(actual) is not type(expected) or actual != expected:
            error("READ_ONLY_BASELINE", "firmware." + name,
                  "Read-only historical baseline must equal " + repr(expected))
    for name, expected in SENSING_DESIGN.items():
        actual = sensing.get(name)
        if type(actual) is not type(expected) or actual != expected:
            error("READ_ONLY_DESIGN_SETTING", "sensing." + name,
                  "This task's explicit navigation scan design must equal " + repr(expected))
    warning("FIRMWARE_NOT_CURRENTLY_VERIFIED", "firmware.verified_current",
            "500 ms lease, 5 RPM command, 250 ms CAN source age and 15 RPM feedback limit are historical assumptions, not current hardware evidence")

    effective = report["effective"]
    effective["user"] = {name: valid.get(name) for name in USER_FIELDS}
    effective["geometry"] = {name: geometry_valid.get(name) for name in GEOMETRY_FIELDS}
    effective["geometry"]["confirmed"] = (
        geometry.get("confirmed") if type(geometry.get("confirmed")) is bool else None)
    effective["firmware_baseline"] = dict(FIRMWARE_BASELINE)
    effective["sensing_design"] = dict(SENSING_DESIGN)
    effective["independent_obstacle_stop_enabled"] = (
        None if valid.get("obstacle_mode") is None else valid["obstacle_mode"] == "stop")
    effective["planner_collision_check_required"] = True
    effective["scan_health_required"] = True
    effective["hardware_execution_authorized"] = False
    effective["validation_scope"] = "offline_design_inputs_only"
    effective["deployment_readiness"] = "not_assessed"
    effective["feedback_timeout_semantics"] = dict(
        source_age_origin="original_sample_first_observation",
        duplicate_sample_refreshes_age=False,
        fault_or_cancel_waits_for_timeout=False)

    if valid.get("obstacle_mode") == "observe_only":
        warning("OBSTACLES_OBSERVE_ONLY", "user.obstacle_mode",
                "Independent obstacle stopping is disabled; planner collision checking and scan health remain required")
    if valid.get("allow_reverse") is True:
        warning("REVERSE_WITH_UNOBSERVED_REAR", "user.allow_reverse",
                "The new navigation sector is front +/-90 degrees and leaves the rear 180 degrees unobserved; a rear clearance value does not create rear sensing")
    if valid.get("connection_loss_action") == "continue_local_task":
        warning("REMOTE_STOP_UNAVAILABLE_WHEN_DISCONNECTED", "user.connection_loss_action",
                "The local task may continue without PC or page connectivity; a remote stop request cannot be assumed deliverable during disconnection")

    for name in ("command_timeout_ms", "supervisor_timeout_ms", "feedback_source_timeout_ms"):
        value = valid.get(name)
        if value is not None and value > FIRMWARE_BASELINE["baseline_lease_ms"]:
            warning("HOST_BUDGET_EXCEEDS_MCU_LEASE", "user." + name,
                    "This host budget exceeds the assumed 500 ms MCU SETPOINT lease and does not modify it; continued fresh SETPOINTs can still renew that lease")
    feedback = valid.get("feedback_source_timeout_ms")
    if feedback is not None and feedback > FIRMWARE_BASELINE["baseline_can_source_timeout_ms"]:
        warning("HOST_SOURCE_BUDGET_EXCEEDS_CAN_BASELINE", "user.feedback_source_timeout_ms",
                "Host acceptance is configured above the assumed 250 ms CAN source limit; existing firmware/client source checks may remain stricter")
    if valid.get("stop_confirmation_timeout_s") is not None and valid["stop_confirmation_timeout_s"] < .6:
        warning("STOP_BUDGET_SHORTER_THAN_BASELINE_WINDOW", "user.stop_confirmation_timeout_s",
                "The historical MCU speed-confirmation loop alone allows 600 ms, excluding command exchange and host evidence collection")

    diameter = geometry_valid.get("wheel_diameter_m")
    separation = geometry_valid.get("wheel_separation_m")
    linear = valid.get("max_linear_m_s")
    angular = valid.get("max_angular_deg_s")
    if diameter is not None and separation is not None:
        try:
            factor = 60. / (math.pi * diameter)
            # Match wheel_envelope's inverse arithmetic, including operation
            # order.  A numerical epsilon must never enlarge a user ceiling.
            linear_step = diameter * (math.pi / 60.)
            angular_deg_per_rpm_difference = (diameter / separation) * 3.
            turn_step = 2. * angular_deg_per_rpm_difference
            if not all(math.isfinite(x) and x > 0 for x in (factor, linear_step, turn_step)):
                raise ValueError("Unrepresentable geometry conversion")
            quant = dict(straight_speed_step_m_s=linear_step,
                         in_place_turn_step_deg_s=turn_step,
                         command_generation="enumerate_integer_wheels_then_enforce_all_limits",
                         nearest_rounding_permitted=False)
            effective["quantization"] = quant
            if linear is not None:
                straight = [(rpm, (2 * rpm) * (linear_step / 2.)) for rpm in range(6)]
                rpm, speed = max((rpm, speed) for rpm, speed in straight if speed <= linear)
                quant["straight_max_integer_rpm_within_speed_limit"] = rpm
                quant["straight_max_representable_m_s"] = speed
            if angular is not None:
                turns = [(rpm, (2 * rpm) * angular_deg_per_rpm_difference) for rpm in range(6)]
                rpm, speed = max((rpm, speed) for rpm, speed in turns
                                 if speed <= angular and math.radians(speed) <= math.radians(angular))
                quant["in_place_max_integer_rpm_within_turn_limit"] = rpm
                quant["in_place_max_representable_deg_s"] = speed
            if linear is not None or angular is not None:
                peak = ((linear or 0.) + math.radians(angular or 0.) * separation / 2.) * factor
                if not math.isfinite(peak):
                    raise ValueError("Unrepresentable speed envelope")
                effective["wheel_envelope"] = dict(continuous_peak_rpm=peak,
                                                   firmware_max_rpm=5,
                                                   complete=linear is not None and angular is not None,
                                                   full_continuous_rectangle_supported=peak <= 5.,
                                                   integer_command_rpm_range=5,
                                                   automatic_scaling=False)
                if peak > 5.:
                    warning("REQUESTED_ENVELOPE_EXCEEDS_FIRMWARE_CAPABILITY", "user.max_linear_m_s",
                            "The continuous speed rectangle reaches {:.6f} wheel RPM, beyond the assumed +/-5 RPM firmware. User values are upper limits: legal integer pairs can achieve lower speeds; the full requested rectangle is not promised and firmware limits must still be enforced".format(peak))
            if linear is not None or angular is not None:
                warning("INTEGER_RPM_QUANTIZATION", "user.max_linear_m_s",
                        "Estimated straight speed steps are {:.9f} m/s and pure-turn steps {:.6f} deg/s; nearest rounding can exceed user limits, so integer wheel pairs must be enumerated and checked before output".format(linear_step, turn_step))
        except (ValueError, OverflowError, ZeroDivisionError):
            error("NUMERIC_ENVELOPE_UNREPRESENTABLE", "geometry",
                  "Geometry and requested speeds cannot be represented safely as finite calculations")
            effective.pop("wheel_envelope", None)
            effective.pop("quantization", None)

    if linear is not None:
        travel = {}
        for name in ("feedback_source_timeout_ms", "command_timeout_ms", "supervisor_timeout_ms"):
            timeout = valid.get(name)
            if timeout is not None:
                distance = linear * (timeout / 1000.)
                if math.isfinite(distance):
                    travel[name] = distance
                else:
                    error("NUMERIC_ENVELOPE_UNREPRESENTABLE", "user." + name,
                          "Speed times timeout is not finite")
        effective["distance_during_timeout_m"] = travel
        effective["timeout_distance_excludes_braking_and_scheduling"] = True
    return report
