"""Read-only hand-pushed mapping odometry. Import opens no device.

The caller locates the exact USB device and owns exclusive_port. Every physical
write is fenced to STATUS/FEEDBACK: no STOP, ARM, SETPOINT, or driver changes.
Evidence stays in bounded RAM; the caller serializes it after closing the port.
"""
import math
import time

from base_static_adapter import QueryOnlyPort, FeedbackSourceTracker, MCU_IDENTITY, crc16


class ManualQueryFault(RuntimeError):
    pass


def positive(name, value):
    if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
        raise ValueError(name + " must be finite and positive")
    return float(value)


def make_raw_rpm_odometry(core_class, diameter_m=.2, separation_m=.3, calibrated=False):
    """Keep the reviewed MCU-time integration; retain even +/-0.1 measured RPM."""
    class RawRpmOdometry(core_class):
        DEADBAND_RPM = 0.0
    return RawRpmOdometry(positive("diameter_m", diameter_m),
                          positive("separation_m", separation_m), calibrated=calibrated)


class ManualQueryPort(QueryOnlyPort):
    """Innermost complete-frame fence, one original PID/thread, finite RAM."""
    def __init__(self, physical, *, clock=time.monotonic, evidence_limit=262144):
        super().__init__(physical)
        if type(evidence_limit) is not int or not 256 <= evidence_limit <= 1048576:
            raise ValueError("evidence_limit must be an integer 256..1048576")
        self.clock, self.limit, self.events = clock, evidence_limit, []
        self.finishing = self.overflow = False
        self.read_attempts = self.empty_reads = 0

    def reserve(self):
        self._check_owner()
        if len(self.events) >= self.limit - (0 if self.finishing else 128):
            self.overflow = True
            raise ManualQueryFault("Raw evidence RAM budget exhausted; no unrecorded IO permitted")

    def write(self, raw):
        self._check_owner()
        if (type(raw) is not bytes or len(raw) != 20 or raw[:3] != b"\xaa\x55\x09"
                or raw[3] not in (2, 5) or not int.from_bytes(raw[4:6], "little")
                or raw[6:18] != bytes(12)
                or int.from_bytes(raw[18:20], "little") != crc16(raw[:18])):
            raise ManualQueryFault("Manual mapping wire permits only zero-field kind2/5; every STOP/ARM/SETPOINT is forbidden")
        self.reserve()
        event = dict(direction="tx", kind=raw[3], sequence=int.from_bytes(raw[4:6], "little"),
                     raw_hex=raw.hex(), started_monotonic_s=self.clock(), bytes_accepted=None)
        self.events.append(event)
        try:
            kind = str(raw[3])
            self.attempts[kind] += 1
            written = self._physical.write(raw)
            event["bytes_accepted"] = written if type(written) is int else None
            if type(written) is not int or written != 20:
                raise OSError("Short/unknown query write; never retry or finish the frame")
            self.complete_writes[kind] += 1
            return written
        except BaseException as exc:
            event["error"] = type(exc).__name__ + ": " + str(exc)
            raise
        finally:
            event["completed_monotonic_s"] = self.clock()

    def read(self, count):
        self.reserve()
        began = self.clock()
        self.read_attempts += 1
        try:
            raw = super().read(count)
        except BaseException as exc:
            self.events.append(dict(direction="rx", started_monotonic_s=began,
                completed_monotonic_s=self.clock(), error=type(exc).__name__ + ": " + str(exc)))
            raise
        if raw:
            if type(raw) is not bytes or len(raw) > count:
                raise ManualQueryFault("Physical read did not return bounded bytes")
            self.events.append(dict(direction="rx", raw_hex=raw.hex(),
                started_monotonic_s=began, completed_monotonic_s=self.clock()))
        else:
            self.empty_reads += 1
        return raw


def run_manual_mapping_session(physical_port, api, odom_factory, *, boot_id,
        duration_s=1200., feedback_timeout_s=.75, stop_recording=lambda: False,
        odom_sink=None, clock=time.monotonic, sleep=time.sleep, wall_clock=time.time,
        evidence_limit=262144, row_limit=100000):
    """Read a disabled base while it is hand pushed; never issue any control.

    odom_sink(odom, original_packet, request_start, completed) must only enqueue
    RAM data. stop_recording ends collection, without stopping/locking wheels.
    Normal and failed exits drain only the original in-flight query deadline.
    """
    duration_s = positive("duration_s", duration_s)
    feedback_timeout_s = positive("feedback_timeout_s", feedback_timeout_s)
    if duration_s > 1200 or feedback_timeout_s != .75:
        raise ValueError("Manual candidate duration <=1200s and original host source timeout 750ms required")
    if type(boot_id) is not str or not boot_id or type(row_limit) is not int or not 16 <= row_limit <= 150000:
        raise ValueError("Explicit boot identity and row_limit 16..150000 required")
    if (api.VERSION, api.IDENTITY, api.LEASE_MS, api.MAX_RPM) != (9, MCU_IDENTITY, 500, 5):
        raise ValueError("Reviewed v9 Client constants required")
    fence = ManualQueryPort(physical_port, clock=clock, evidence_limit=evidence_limit)
    client, source, odom = api.Client(fence), FeedbackSourceTracker(feedback_timeout_s), odom_factory()
    if getattr(odom, "DEADBAND_RPM", None) != 0.:
        raise ValueError("Hand-pushed mapping requires true raw-RPM odometry with no deadband")
    geometry = dict(wheel_diameter_m=positive("diameter", odom.diameter),
                    wheel_separation_m=positive("separation", odom.separation), calibrated=False)
    result = dict(schema="p2p-manual-mapping-query-v1", boot_id=boot_id,
        scope="query_only_hand_pushed_measured_odometry", recording_complete=False,
        allowed_request_kinds=[2, 5], STOP_ARM_SETPOINT_allowed=False,
        driver_state_modified=False, motion_permission=False, origin=None, first_failure=None,
        host_feedback_source_timeout_s=.75, CAN_source_limit_s=.25, geometry=geometry)
    rows, last_clock, previous_sample = [], None, None
    odometry_discontinuities = []  # At most one entry per retained response row.
    metrics = dict(distinct_feedback_samples=0, odometry_samples=0, source_age_max_s=0.,
                   feedback_query_gap_max_s=0., status_query_gap_max_s=0., gap_count=0)
    last_starts = {}

    def now():
        nonlocal last_clock
        at = clock()
        if type(at) not in (int, float) or not math.isfinite(at) or at < 0 or last_clock is not None and at < last_clock:
            raise ManualQueryFault("Monotonic clock invalid or moved backwards")
        last_clock = at
        return at

    def failure(exc, phase):
        if result["first_failure"] is None:
            result["first_failure"] = dict(error_type=type(exc).__name__, message=str(exc),
                phase=phase, monotonic_s=clock(), client_failure=client.last_exchange_failure)

    def capture(kind, packet):
        nonlocal previous_sample
        done = client.last_exchange_completion
        if done is None or done["kind"] != kind or done["bytes_accepted"] != 20:
            raise ManualQueryFault("Matching complete physical query missing")
        row = dict(kind=kind, packet=dict(packet), request_hex=done["request_hex"],
            reply_hex=client.trace[-1]["reply_hex"], request_start_monotonic_s=done["started_monotonic_s"],
            completed_monotonic_s=done["completed_monotonic_s"])
        if len(rows) >= row_limit:
            raise ManualQueryFault("Response RAM budget exhausted")
        rows.append(row)
        client.trace[:] = client.trace[-16:]
        start, end = row["request_start_monotonic_s"], row["completed_monotonic_s"]
        if kind == 2:
            if (packet["identity"] != MCU_IDENTITY or packet["state"] != 1 or packet["session"]
                    or packet["target_pack"] or packet["error"] or packet["uart_critical_total"]
                    or packet["guard_flags"] & 3 != 1):
                raise ManualQueryFault("Base is not healthy DISARMED/session0/target0; exiting without control")
            return
        api.fresh(packet)  # Includes original driver state, gross speed, fault and CAN250ms checks.
        if packet["controller_state"] != 1:
            raise ManualQueryFault("Hand-pushed mapping requires DISARMED feedback")
        measured = source.accept(packet, start, end)
        row.update(measured)
        metrics["source_age_max_s"] = max(metrics["source_age_max_s"], measured["source_age_s"])
        if not measured["distinct"]:
            return
        metrics["distinct_feedback_samples"] += 1
        observation = odom.update(packet)
        if observation is None:
            raise ManualQueryFault("Distinct real feedback produced no odometry observation")
        observation = dict(observation, source_monotonic_s=measured["source_monotonic_s"])
        row["wheel_odometry"] = observation
        if observation.get("discontinuity"):
            # The reviewed core already skipped integration across this gap.
            # Hand-pushed collection may continue, but never calls it a complete
            # trajectory and never invents the missing interval's displacement.
            previous = previous_sample
            odometry_discontinuities.append(dict(packet=dict(packet), previous_sample=previous,
                source_monotonic_s=measured["source_monotonic_s"],
                request_start_monotonic_s=start, completed_monotonic_s=end,
                sequence_step=None if previous is None else
                    (packet["sample_sequence"]-previous["packet"]["sample_sequence"]) & 0xffffffff,
                dt_s=None if previous is None else
                    ((packet["speed_tick_ms"]-previous["packet"]["speed_tick_ms"]) & 0xffffffff)/1000.,
                source_gap_s=None if previous is None else
                    measured["source_monotonic_s"]-previous["source_monotonic_s"],
                observation=dict(observation), missing_motion_integrated=False))
            metrics["gap_count"] += 1
        previous_sample = dict(packet=dict(packet), source_monotonic_s=measured["source_monotonic_s"],
                               completed_monotonic_s=end)
        metrics["odometry_samples"] += 1
        if result["origin"] is None:
            if any(observation[key] != 0. for key in ("x", "y", "yaw")):
                raise ManualQueryFault("First true sample must define fresh odometry origin")
            wall = wall_clock()
            result["origin"] = dict(boot_id=boot_id, pose=dict(x=0., y=0., yaw=0.),
                source_monotonic_s=measured["source_monotonic_s"], received_monotonic_s=end,
                received_wall_s=wall, source_wall_s=wall - (end - measured["source_monotonic_s"]),
                request_start_monotonic_s=start, sample_sequence=packet["sample_sequence"],
                raw_packet=dict(packet), geometry=geometry, physical_mark_is_user_supplied=True)
        if odom_sink is not None:
            odom_sink(dict(observation), dict(packet), start, end)

    def begin(kind):
        at = now()
        if kind in last_starts:
            key = "feedback_query_gap_max_s" if kind == 5 else "status_query_gap_max_s"
            metrics[key] = max(metrics[key], at - last_starts[kind])
        last_starts[kind] = at
        return client.begin_exchange(kind)

    began, pending = now(), None
    try:
        result["nonblocking_configuration"] = client.enable_nonblocking()
        deadline = began + duration_s
        pending = begin(2)  # First prove disabled state; no STOP/preparation side effect.
        while pending is not None:
            packet = client.poll_exchange(pending["id"])
            if packet is not None:
                capture(2, packet); pending = None
            else:
                sleep(.001)
        next_feedback = next_status = now()
        while now() < deadline:
            requested = stop_recording()
            if type(requested) is not bool:
                raise ManualQueryFault("stop_recording must return bool")
            if requested:
                result["termination_reason"] = "operator_stopped_recording"
                break
            if source.source is not None:
                source.check_age(now())
            elif now() - began >= feedback_timeout_s:
                raise ManualQueryFault("First real feedback source did not arrive within 750ms")
            if len(rows) >= row_limit - 2:
                raise ManualQueryFault("Response RAM budget reserve reached")
            if pending is not None:
                packet = client.poll_exchange(pending["id"])
                if packet is not None:
                    capture(pending["kind"], packet); pending = None
            if pending is None:
                at = now()
                if at >= next_feedback:
                    pending = begin(5); next_feedback = at + .025
                elif at >= next_status:
                    pending = begin(2); next_status = at + .100
            sleep(.001)
        else:
            result["termination_reason"] = "fixed_duration_complete"
    except BaseException as exc:
        failure(exc, "recording")
        result["termination_reason"] = "fatal_query_failure"
    finally:
        fence.finishing = True
        original = client.pending_exchange
        quiesce = dict(pending=original, complete=False, queries_started=0,
            started_monotonic_s=clock(), deadline_basis="original_Client_query_deadline_only")
        result["normal_quiesce"] = quiesce
        try:
            while original is not None:
                packet = client.poll_exchange(original["id"])
                if packet is not None:
                    capture(original["kind"], packet); break
                sleep(.001)
            if source.source is not None:
                source.check_age(now())
            quiesce["complete"] = True
        except BaseException as exc:
            quiesce["error"] = type(exc).__name__ + ": " + str(exc)
            failure(exc, "query_drain")
        quiesce["duration_s"] = clock() - quiesce["started_monotonic_s"]
    result.update(rows=rows, raw_events=fence.events, raw_evidence_overflow=fence.overflow,
        query_counts=dict(fence.attempts), complete_write_counts=dict(fence.complete_writes),
        metrics=metrics, elapsed_s=clock()-began, physical_read_attempts=fence.read_attempts,
        empty_nonblocking_reads=fence.empty_reads, evidence_limit=evidence_limit, row_limit=row_limit)
    result["odometry_discontinuities"] = odometry_discontinuities
    result["odometry_continuity_validated"] = metrics["gap_count"] == 0
    result["odom_continuity_validated"] = result["odometry_continuity_validated"]
    result["normal_recording_end"] = bool(result["origin"] and not result["first_failure"]
        and quiesce["complete"] and not fence.overflow)
    result["recording_ended_cleanly"] = result["normal_recording_end"]
    result["recording_complete"] = bool(result["normal_recording_end"]
        and result["odometry_continuity_validated"])
    return result
