"""Pure checks for one current owner's stationary localization window.

No device, ROS, filesystem, clock or motion interface is used.  The caller
supplies already decoded/fresh raw feedback and measured wheel odometry from
the SAME owner callback, before any lossy publication mailbox.  Tolerances and
identity are explicit inputs, not proof that hardware or a map pose is valid.
Encoder counts are never converted to distance: counts/revolution are unknown.
"""
from copy import deepcopy
import math


class AnchorViolation(RuntimeError):
    """The stationary window is invalid; the caller must stop its static task."""


def _number(value, name, minimum=None, positive=False):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except (ValueError, OverflowError, TypeError):
        valid = False
    if not valid or positive and value <= 0 or minimum is not None and value < minimum:
        raise ValueError(name + " must be a finite number within its explicit bounds")
    return float(value)


def _identity(value, name):
    if type(value) is not str or not value.strip():
        raise ValueError(name + " must be a nonempty identity string")
    return value


def _uint32(value, name):
    if type(value) is not int or not 0 <= value <= 0xffffffff:
        raise ValueError(name + " must be a uint32 integer")
    return value


def _counts(value, name):
    if type(value) is not int or not 0 <= value < 0x80000000:
        raise ValueError(name + " must be an explicit integer count bound below half the uint32 range")
    return value


def _delta32(current, previous):
    delta = (current - previous) & 0xffffffff
    if delta == 0x80000000:
        raise ValueError("Half-range encoder delta is direction-ambiguous")
    return delta if delta < 0x80000000 else delta - 0x100000000


class StaticLocalizationAnchor:
    """Latched stationary checks; no reset, old-pose seed or execution permit.

    New data cannot erase an already expired source deadline.  A first source
    at/before reference_after_monotonic_s remains pending, never the anchor.
    check() must also be called when no samples arrive.  Callers serialize all
    mutation/check/read access in the owner thread or one external lock.  The retained reference
    is measured wheel odometry, and does not certify AMCL or map localization.
    """
    def __init__(self, *, boot_id, owner_id, reference_after_monotonic_s,
                 source_timeout_s, max_encoder_change_counts,
                 max_encoder_travel_counts, max_position_change_m,
                 max_yaw_change_rad, max_linear_speed_m_s,
                 max_angular_speed_rad_s, max_wheel_speed_rpm):
        self.boot_id = _identity(boot_id, "boot_id")
        self.owner_id = _identity(owner_id, "owner_id")
        self.reference_after = _number(reference_after_monotonic_s, "reference_after_monotonic_s", minimum=0)
        self.source_timeout = _number(source_timeout_s, "source_timeout_s", positive=True)
        self.limits = dict(
            encoder_change_counts=_counts(max_encoder_change_counts, "max_encoder_change_counts"),
            encoder_travel_counts=_counts(max_encoder_travel_counts, "max_encoder_travel_counts"),
            position_change_m=_number(max_position_change_m, "max_position_change_m", minimum=0),
            yaw_change_rad=_number(max_yaw_change_rad, "max_yaw_change_rad", minimum=0),
            linear_speed_m_s=_number(max_linear_speed_m_s, "max_linear_speed_m_s", minimum=0),
            angular_speed_rad_s=_number(max_angular_speed_rad_s, "max_angular_speed_rad_s", minimum=0),
            wheel_speed_rpm=_number(max_wheel_speed_rpm, "max_wheel_speed_rpm", minimum=0))
        self.anchor = self.last = None
        self.last_now = None
        self.failure = None
        self.accepted_samples = self.duplicates = self.waiting_sources = 0
        self.encoder_travel = [0, 0]
        self.max_encoder_change = [0, 0]
        self.max_position_change = self.max_yaw_change = 0.

    def _fail(self, code, now, **details):
        if self.failure is None:
            self.failure = dict(code=code, at_monotonic_s=now, details=details)
        raise AnchorViolation(self.failure["code"])

    def _enter(self, now, boot_id, owner_id):
        if self.failure is not None:
            raise AnchorViolation(self.failure["code"])
        try:
            now = _number(now, "now_monotonic_s", minimum=0)
        except ValueError as exc:
            self._fail("INVALID_CLOCK", self.last_now, error=str(exc))
        if self.last_now is not None and now < self.last_now:
            self._fail("CLOCK_MOVED_BACKWARDS", now)
        self.last_now = now
        if (type(boot_id) is not str or type(owner_id) is not str
                or boot_id != self.boot_id or owner_id != self.owner_id):
            self._fail("OWNER_OR_BOOT_CHANGED", now)
        if self.last is not None and not 0 <= now - self.last["source_monotonic_s"] < self.source_timeout:
            self._fail("SOURCE_EXPIRED", now, source_monotonic_s=self.last["source_monotonic_s"])
        return now

    def _sample(self, packet, odom, source, now):
        try:
            if type(packet) is not dict or type(odom) is not dict:
                raise ValueError("Raw packet and measured odometry must be dictionaries")
            source = _number(source, "source_monotonic_s", minimum=0)
            if source > now:
                raise ValueError("Future source time is invalid")
            sequence = _uint32(packet["sample_sequence"], "sample_sequence")
            if sequence == 0:
                raise ValueError("Zero sample sequence is rejected by the underlying Client")
            speed_tick = _uint32(packet["speed_tick_ms"], "speed_tick_ms")
            raw = [_uint32(packet[key], key) for key in ("position_left_raw", "position_right_raw")]
            wheel_speed = [_number(packet[key], key) for key in ("speed_left_rpm", "speed_right_rpm")]
            pose = [_number(odom[key], key) for key in ("x", "y", "yaw")]
            twist = [_number(odom[key], key) for key in ("linear_x", "angular_z")]
            if (type(odom.get("sample_sequence")) is not int or odom["sample_sequence"] != sequence
                    or type(odom.get("mcu_speed_tick_ms")) is not int or odom["mcu_speed_tick_ms"] != speed_tick):
                raise ValueError("Odometry does not belong to the same raw feedback sample")
            if odom.get("discontinuity") is not False:
                raise ValueError("Measured odometry has a gap or missing discontinuity check")
            if (packet["controller_state"] != 1 or packet["controller_error"] != 0
                    or packet["fault_raw"] != 0 or packet["valid_mask"] != 31):
                raise ValueError("Feedback does not describe a healthy disarmed controller")
        except (KeyError, ValueError, TypeError) as exc:
            self._fail("INVALID_SAMPLE", now, error=str(exc))
        if max(abs(value) for value in wheel_speed) > self.limits["wheel_speed_rpm"]:
            self._fail("RAW_WHEEL_SPEED", now, wheel_speed_rpm=wheel_speed)
        if abs(twist[0]) > self.limits["linear_speed_m_s"] or abs(twist[1]) > self.limits["angular_speed_rad_s"]:
            self._fail("MEASURED_TWIST_CHANGED", now, linear_m_s=twist[0], angular_rad_s=twist[1])
        return dict(sample_sequence=sequence, speed_tick_ms=speed_tick,
                    encoder_raw=raw, wheel_speed_rpm=wheel_speed, pose=pose, twist=twist,
                    source_monotonic_s=source, observed_monotonic_s=now)

    def observe(self, packet, odom, *, source_monotonic_s, now_monotonic_s, boot_id, owner_id):
        now = self._enter(now_monotonic_s, boot_id, owner_id)
        sample = self._sample(packet, odom, source_monotonic_s, now)
        if self.last is None:
            if sample["source_monotonic_s"] <= self.reference_after:
                self.waiting_sources += 1
                return self.snapshot(now)
            if now - sample["source_monotonic_s"] >= self.source_timeout:
                self._fail("SOURCE_EXPIRED", now)
            self.anchor = deepcopy(sample)
        else:
            step = (sample["sample_sequence"] - self.last["sample_sequence"]) & 0xffffffff
            if step == 0:
                keys = ("speed_tick_ms", "encoder_raw", "wheel_speed_rpm", "pose", "twist")
                if any(sample[key] != self.last[key] for key in keys):
                    self._fail("DUPLICATE_SAMPLE_CHANGED", now)
                self.duplicates += 1
                return self.snapshot(now)  # Keep the FIRST source/observation.
            if step != 1:
                self._fail("SAMPLE_SEQUENCE_GAP_OR_REORDER", now, unsigned_step=step)
            if sample["source_monotonic_s"] <= self.last["source_monotonic_s"]:
                self._fail("SOURCE_DID_NOT_ADVANCE", now)
            if now - sample["source_monotonic_s"] >= self.source_timeout:
                self._fail("SOURCE_EXPIRED", now)
            try:
                changes = [_delta32(current, initial) for current, initial in zip(sample["encoder_raw"], self.anchor["encoder_raw"])]
                steps = [_delta32(current, previous) for current, previous in zip(sample["encoder_raw"], self.last["encoder_raw"])]
            except ValueError as exc:
                self._fail("AMBIGUOUS_ENCODER_DELTA", now, error=str(exc))
            travel = [total + abs(delta) for total, delta in zip(self.encoder_travel, steps)]
            if max(abs(delta) for delta in changes) > self.limits["encoder_change_counts"]:
                self._fail("RAW_ENCODER_CHANGED", now, delta_counts=changes, rpm=sample["wheel_speed_rpm"])
            if max(travel) > self.limits["encoder_travel_counts"]:
                self._fail("RAW_ENCODER_TRAVEL", now, total_absolute_counts=travel)
            dx, dy = (sample["pose"][i] - self.anchor["pose"][i] for i in (0, 1))
            position = math.hypot(dx, dy)
            yaw_delta = sample["pose"][2] - self.anchor["pose"][2]
            if not all(math.isfinite(value) for value in (dx, dy, position, yaw_delta)):
                self._fail("POSE_DELTA_INVALID", now)
            yaw = abs(math.atan2(math.sin(yaw_delta), math.cos(yaw_delta)))
            if not math.isfinite(position) or not math.isfinite(yaw):
                self._fail("POSE_DELTA_INVALID", now)
            if position > self.limits["position_change_m"] or yaw > self.limits["yaw_change_rad"]:
                self._fail("MEASURED_POSE_CHANGED", now, displacement_m=position, yaw_change_rad=yaw)
            self.encoder_travel = travel
            self.max_encoder_change = [max(old, abs(delta)) for old, delta in zip(self.max_encoder_change, changes)]
            self.max_position_change = max(self.max_position_change, position)
            self.max_yaw_change = max(self.max_yaw_change, yaw)
        self.last = deepcopy(sample)
        self.accepted_samples += 1
        return self.snapshot(now)

    def check(self, *, now_monotonic_s, boot_id, owner_id):
        now = self._enter(now_monotonic_s, boot_id, owner_id)
        return self.snapshot(now)

    def snapshot(self, now_monotonic_s):
        """Audit view only; reading does not renew a source or clear a failure."""
        now = _number(now_monotonic_s, "now_monotonic_s", minimum=0)
        age = None if self.last is None else now - self.last["source_monotonic_s"]
        fresh = age is not None and 0 <= age < self.source_timeout
        return dict(schema="p2p-static-localization-anchor-v1", boot_id=self.boot_id,
                    owner_id=self.owner_id, anchor_established=self.anchor is not None,
                    valid=self.anchor is not None and self.failure is None and fresh,
                    failure=deepcopy(self.failure), source_age_s=age,
                    reference_after_monotonic_s=self.reference_after, source_timeout_s=self.source_timeout,
                    anchor=deepcopy(self.anchor), latest=deepcopy(self.last), limits=dict(self.limits),
                    accepted_samples=self.accepted_samples, duplicate_samples=self.duplicates,
                    waiting_old_sources=self.waiting_sources,
                    max_encoder_change_counts=list(self.max_encoder_change),
                    total_absolute_encoder_travel_counts=list(self.encoder_travel),
                    max_position_change_m=self.max_position_change, max_yaw_change_rad=self.max_yaw_change,
                    encoder_distance_conversion_available=False,
                    pose_basis="current_owner_measured_wheel_odometry_only",
                    map_localization_valid=False, hardware_execution_authorized=False)
