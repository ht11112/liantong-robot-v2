"""Query/STOP-only adapter for a caller-owned, exclusively opened serial port.

Importing this module opens nothing.  The caller must freshly run reviewed
rk_link.locate(), acquire transport.exclusive_port(target['port']), and supply
reviewed nonblocking Client, stop_observation and MeasuredOdometry dependencies.
No path fallback, second FD, ARM, SETPOINT, target or TF publisher exists here.
All records remain bounded in RAM; the caller writes them after port cleanup.
"""
from collections import deque
import math
import os
import struct
import threading
import time
from types import SimpleNamespace


ALLOWED_KINDS = (2, 3, 5)
MCU_IDENTITY = 0x47344339
CAN_SOURCE_LIMIT_S = .250  # Existing api.fresh() boundary, not host timeout.
_SAMPLE_TICKS = ("sample_tick_ms", "left_tick_ms", "right_tick_ms", "speed_tick_ms")
_SAMPLE_CONTENT = _SAMPLE_TICKS + (
    "position_left_raw", "position_right_raw", "speed_raw", "status_raw", "fault_raw", "flags", "valid_mask")


class StaticAdapterFault(RuntimeError):
    pass


def crc16(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xa001 if value & 1 else 0)
    return value


def _positive(name, value):
    try:
        valid = type(value) in (int, float) and math.isfinite(value) and value > 0
    except (OverflowError, ValueError, TypeError):
        valid = False
    if not valid:
        raise ValueError(name + " must be a finite positive number")
    return float(value)


class QueryOnlyPort:
    """Innermost 20-byte wire fence, even if Client public methods are bypassed."""
    def __init__(self, physical):
        self._physical = physical
        self._owner = (os.getpid(), threading.get_ident())
        self.attempts = {str(kind): 0 for kind in ALLOWED_KINDS}
        self.complete_writes = {str(kind): 0 for kind in ALLOWED_KINDS}

    def _check_owner(self):
        if self._owner != (os.getpid(), threading.get_ident()):
            raise StaticAdapterFault("Original serial owner PID/thread required")

    @property
    def timeout(self):
        return self._physical.timeout

    @timeout.setter
    def timeout(self, value):
        self._check_owner()
        self._physical.timeout = value

    @property
    def write_timeout(self):
        return self._physical.write_timeout

    @write_timeout.setter
    def write_timeout(self, value):
        self._check_owner()
        self._physical.write_timeout = value

    def read(self, count):
        self._check_owner()
        if type(count) is not int or not 0 <= count <= 64:
            raise ValueError("Only bounded response reads of at most 64 bytes are allowed")
        return self._physical.read(count)

    def write(self, raw):
        self._check_owner()
        if (type(raw) is not bytes or len(raw) != 20 or raw[:3] != b"\xaa\x55\x09"
                or raw[3] not in ALLOWED_KINDS or not struct.unpack_from("<H", raw, 4)[0]
                or raw[6:18] != bytes(12)
                or struct.unpack_from("<H", raw, 18)[0] != crc16(raw[:18])):
            raise ValueError("Static wire rejects every frame except zero-field STATUS/STOP/FEEDBACK")
        kind = str(raw[3])
        self.attempts[kind] += 1
        written = self._physical.write(raw)
        if type(written) is not int or written != 20:
            raise OSError("Short/unknown physical write; never retry or finish the old frame")
        self.complete_writes[kind] += 1
        return written


class FeedbackSourceTracker:
    """Preserve the first conservative source origin for each advancing sample."""
    def __init__(self, timeout_s=.750):
        self.timeout_s = _positive("feedback_timeout_s", timeout_s)
        self.sequence = self.packet = self.source = None
        self.distinct_samples = 0

    def check_age(self, now):
        if self.source is None or not 0 <= now - self.source < self.timeout_s:
            raise StaticAdapterFault("Host feedback source age exceeded configured timeout")
        return now - self.source

    def accept(self, packet, request_start, completed):
        if not 0 <= request_start <= completed:
            raise StaticAdapterFault("Invalid feedback request/completion clock")
        if self.source is not None:
            # A late new sample cannot erase an already reached host deadline.
            self.check_age(completed)
        sequence = packet["sample_sequence"]
        if type(sequence) is not int or not 0 < sequence <= 0xffffffff:
            raise StaticAdapterFault("Invalid feedback sample sequence")
        distinct = self.sequence is None or sequence != self.sequence
        if self.sequence is not None:
            step = (sequence - self.sequence) & 0xffffffff
            if step == 0:
                if any(packet[key] != self.packet[key] for key in _SAMPLE_CONTENT):
                    raise StaticAdapterFault("Duplicate CAN sample changed its content")
            elif not 0 < step < 0x80000000:
                raise StaticAdapterFault("CAN sample sequence moved backwards")
        if distinct:
            can_age = max(((packet["tick_ms"] - packet[key]) & 0xffffffff) / 1000.
                          for key in _SAMPLE_TICKS)
            if can_age > CAN_SOURCE_LIMIT_S:
                raise StaticAdapterFault("CAN source age exceeds existing 250 ms boundary")
            self.source = request_start - can_age
            self.sequence, self.packet = sequence, dict(packet)
            self.distinct_samples += 1
        return dict(distinct=distinct, source_monotonic_s=self.source,
                    source_age_s=self.check_age(completed))


def run_static_session(physical_port, api, stop_api, odom_factory, *, duration_s,
                       feedback_timeout_s=.750, clock=time.monotonic, sleep=time.sleep,
                       cancelled=lambda: False, odom_sink=None, row_limit=4096,
                       zero_proof_timeout_s=2.0, zero_speed_rpm=.2):
    """Run one static link window with new STOP/STATUS/distinct-zero bookends.

    The supplied port remains owned/closed by the caller's exclusive_port
    context.  odom_sink, if used, must only enqueue measured wheel odometry to a
    private namespace without a global TF; there is no ROS dependency here.
    A complete result covers only this static link and measured-wheel chain.
    """
    duration_s = _positive("duration_s", duration_s)
    feedback_timeout_s = _positive("feedback_timeout_s", feedback_timeout_s)
    zero_proof_timeout_s = _positive("zero_proof_timeout_s", zero_proof_timeout_s)
    if (type(zero_speed_rpm) not in (int, float) or not math.isfinite(zero_speed_rpm)
            or not 0 <= zero_speed_rpm <= .2):
        raise ValueError("zero_speed_rpm must be finite within 0..0.2 RPM")
    if duration_s > 3600 or type(row_limit) is not int or not 16 <= row_limit <= 65536:
        raise ValueError("Duration must be <=3600 seconds and row_limit 16..65536")
    if (api.VERSION, api.IDENTITY, api.LEASE_MS, api.MAX_RPM) != (9, MCU_IDENTITY, 500, 5):
        raise ValueError("Unexpected reviewed Client protocol constants")
    if not all(hasattr(api.Client, method) for method in ("enable_nonblocking", "begin_exchange", "poll_exchange", "cancel_for_stop")):
        raise ValueError("Reviewed nonblocking Client required")
    rows = deque(maxlen=row_limit)
    row_count = 0
    fence = QueryOnlyPort(physical_port)
    client = api.Client(fence)
    odom = odom_factory()
    source = FeedbackSourceTracker(feedback_timeout_s)
    result = dict(schema="p2p-static-base-link-v1", scope="query_STOP_measured_wheel_odometry_only",
                  navigation_validated=False, localization_validated=False,
                  motion_requested=False, ARM_SETPOINT_allowed=False,
                  allowed_request_kinds=list(ALLOWED_KINDS),
                  host_feedback_source_timeout_s=feedback_timeout_s,
                  existing_CAN_source_limit_s=CAN_SOURCE_LIMIT_S,
                  static_zero_speed_rpm=zero_speed_rpm,
                  first_failure=None, final_stop_chain=None, initial_stop_chain=None,
                  active_window_complete=False, static_link_complete=False)
    phase = "initial_stop"
    last_clock = None
    last_starts = {}
    metrics = dict(feedback_responses=0, distinct_feedback_samples=0,
                   odometry_samples=0, source_age_max_s=0., exchange_max_s=0.,
                   feedback_query_gap_max_s=0., status_query_gap_max_s=0.)

    def now():
        nonlocal last_clock
        value = clock()
        if (type(value) not in (int, float) or not math.isfinite(value) or value < 0
                or last_clock is not None and value < last_clock):
            raise StaticAdapterFault("Monotonic clock invalid or moved backwards")
        last_clock = value
        return value

    def check_cancel():
        value = cancelled()
        if type(value) is not bool or value:
            raise StaticAdapterFault("Static check cancelled; immediate STOP required")

    def capture(kind, packet, *, healthy=False):
        nonlocal row_count
        done = client.last_exchange_completion
        if not done or done["kind"] != kind or done["bytes_accepted"] != 20:
            raise StaticAdapterFault("Physical full-write completion ticket missing")
        start, end = done["started_monotonic_s"], done["completed_monotonic_s"]
        if not 0 <= start <= end <= now():
            raise StaticAdapterFault("Invalid physical completion timing")
        row = dict(phase=phase, kind=kind, request_start_monotonic_s=start,
                   completed_monotonic_s=end, packet=dict(packet),
                   request_hex=done["request_hex"])
        rows.append(row)
        row_count += 1
        client.trace[:] = client.trace[-40:]
        if phase == "active":
            metrics["exchange_max_s"] = max(metrics["exchange_max_s"], end - start)
            if kind in last_starts:
                name = "feedback_query_gap_max_s" if kind == 5 else "status_query_gap_max_s"
                metrics[name] = max(metrics[name], start - last_starts[kind])
            last_starts[kind] = start
        if kind in (2, 3) and packet.get("identity") != MCU_IDENTITY:
            raise StaticAdapterFault("Unexpected MCU identity")
        if healthy and kind == 2 and not stop_api.stopped(packet):
            raise StaticAdapterFault("Static STATUS indicates armed/moving/faulted MCU")
        if kind == 5:
            api.fresh(packet)  # Existing CAN <=250 ms, independent of host 750 ms.
            if (packet["controller_state"] != 1 or packet["controller_error"]
                    or max(abs(packet["speed_left_rpm"]), abs(packet["speed_right_rpm"])) > zero_speed_rpm):
                raise StaticAdapterFault("Static raw feedback exceeds zero-speed tolerance or is faulted; STOP immediately")
            measured = source.accept(packet, start, end)
            row.update(measured)
            observation = odom.update(packet) if measured["distinct"] else None
            if observation is not None:
                row["wheel_odometry"] = observation
                if observation.get("discontinuity"):
                    raise StaticAdapterFault("Measured wheel sample gap; no extrapolation")
                if odom_sink is not None:
                    odom_sink(dict(observation, source_monotonic_s=measured["source_monotonic_s"]))
            if phase == "active":
                metrics["feedback_responses"] += 1
                metrics["distinct_feedback_samples"] += int(measured["distinct"])
                metrics["odometry_samples"] += int(observation is not None)
                metrics["source_age_max_s"] = max(metrics["source_age_max_s"], measured["source_age_s"])
            return row
        return row

    def poll_query(kind, cancelable):
        if cancelable:
            check_cancel()
        ticket = client.begin_exchange(kind)
        while True:
            if cancelable:
                check_cancel()
            packet = client.poll_exchange(ticket["id"])
            if packet is not None:
                return capture(kind, packet, healthy=True)
            if source.source is not None:
                source.check_age(now())
            sleep(.001)

    def stop_chain(cancelable):
        nonlocal source
        ack_row = None

        def stop():
            nonlocal ack_row
            packet = client.stop()  # Cancels an in-flight query without waiting.
            ack_row = capture(3, packet)
            return packet

        status_rows = []

        def observe(packet):
            status_rows.append(capture(2, packet))

        status = stop_api.verify_stop(SimpleNamespace(stop=stop, stopping_status=client.stopping_status),
                                      observe, clock=clock, sleep=sleep)
        status_row = status_rows[-1]
        if status_row["request_start_monotonic_s"] < ack_row["completed_monotonic_s"]:
            raise StaticAdapterFault("Post-STOP STATUS did not follow the new ACK")
        # Only a new confirmed STOP chain starts a new collection stage.  This
        # never clears first_failure or resumes the completed/failed active leg.
        source = FeedbackSourceTracker(feedback_timeout_s)
        # The STOP changes the reference; do not integrate across cleanup.
        odom.invalidate()
        proof = dict(stop_ack=ack_row, stopped_status=status_row,
                     distinct_zero_feedback_samples=0, first_zero_sample=None,
                     last_zero_sample=None, complete=False)
        deadline = now() + zero_proof_timeout_s
        last_zero_sequence = None
        while now() < deadline:
            row = poll_query(5, cancelable)
            packet = row["packet"]
            if (row["distinct"] and packet["sample_sequence"] != last_zero_sequence
                    and all(0 < ((packet[key] - status["tick_ms"]) & 0xffffffff) < 0x80000000
                            for key in _SAMPLE_TICKS)):
                proof["distinct_zero_feedback_samples"] += 1
                last_zero_sequence = packet["sample_sequence"]
                stamp = dict(sample_sequence=last_zero_sequence, source_monotonic_s=row["source_monotonic_s"],
                             completed_monotonic_s=row["completed_monotonic_s"],
                             speed_left_rpm=packet["speed_left_rpm"], speed_right_rpm=packet["speed_right_rpm"])
                if proof["first_zero_sample"] is None:
                    proof["first_zero_sample"] = stamp
                proof["last_zero_sample"] = stamp
                if proof["distinct_zero_feedback_samples"] >= 5:
                    proof["complete"] = True
                    return proof
            sleep(.010)
        raise StaticAdapterFault("Five different fresh zero samples after the new STOP/STATUS were not observed")

    began = now()
    try:
        result["nonblocking_configuration"] = client.enable_nonblocking()
        result["initial_stop_chain"] = stop_chain(True)
        phase = "active"
        active_start = now()
        result["active_start_monotonic_s"] = active_start
        deadline = active_start + duration_s
        next_feedback = next_status = active_start
        pending = None
        while now() < deadline:
            check_cancel()  # Fault/cancel never waits for the source timeout.
            source.check_age(now())
            if pending is not None:
                packet = client.poll_exchange(pending["id"])
                if packet is not None:
                    capture(pending["kind"], packet, healthy=True)
                    pending = None
            if pending is None:
                at = now()
                if at >= next_feedback:  # Feedback gets the first free transaction.
                    pending = client.begin_exchange(5)
                    next_feedback = at + .025
                elif at >= next_status:
                    pending = client.begin_exchange(2)
                    next_status = at + .100
            sleep(.001)
        result["active_window_complete"] = True
        result["active_duration_s"] = now() - active_start
        # This owner permanently permits only queries/STOP.  On an ordinary
        # fixed-duration finish, let the one already-written query finish on
        # its ORIGINAL Client deadline before placing STOP on half-duplex wire.
        # Faults/cancellation leave through the exception path immediately.
        phase = "normal_quiesce"
        quiesce_start = now()
        quiesce = dict(started_monotonic_s=quiesce_start,
                       pending_at_window_end=None if pending is None else dict(pending),
                       queries_started=0, complete=False,
                       deadline_basis="original_inflight_Client_deadline_only")
        result["normal_quiesce"] = quiesce
        try:
            check_cancel()
            while pending is not None:
                check_cancel()
                source.check_age(now())
                packet = client.poll_exchange(pending["id"])
                if packet is not None:
                    check_cancel()
                    capture(pending["kind"], packet, healthy=True)
                    pending = None
                else:
                    sleep(.001)
            quiesce["complete"] = True
        finally:
            quiesce["duration_s"] = now() - quiesce_start
    except Exception as exc:
        result["first_failure"] = dict(phase=phase, error_type=type(exc).__name__, message=str(exc))
        result["failure_monotonic_s"] = clock()
    finally:
        phase = "final_stop"
        try:
            result["final_stop_chain"] = stop_chain(False)
        except Exception as exc:
            result["final_stop_error"] = dict(error_type=type(exc).__name__, message=str(exc))
        client.trace[:] = client.trace[-40:]
    ended = clock()
    active_duration = result.get("active_duration_s")
    metrics["feedback_response_hz"] = (metrics["feedback_responses"] / active_duration if active_duration else None)
    metrics["distinct_feedback_hz"] = (metrics["distinct_feedback_samples"] / active_duration if active_duration else None)
    result.update(elapsed_s=ended - began, metrics=metrics, counts=fence.attempts,
                  complete_write_counts=fence.complete_writes, rows=list(rows),
                  total_rows=row_count, retained_rows=len(rows), dropped_old_rows=max(0, row_count - len(rows)))
    result["static_link_complete"] = bool(result["active_window_complete"] and result["first_failure"] is None
                                         and result["initial_stop_chain"] and result["initial_stop_chain"]["complete"]
                                         and result["final_stop_chain"] and result["final_stop_chain"]["complete"])
    return result
