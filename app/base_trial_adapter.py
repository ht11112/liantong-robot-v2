"""One bounded motion trial on a caller-owned exclusive nonblocking v9 port.

Import opens nothing. The caller supplies the reviewed Client, STOP observer,
measured odometry, and current ROS health/command callbacks. This separate trial
module does not alter the historical query-only adapters or their wire fence.
All IO evidence stays in bounded RAM until the caller has closed the port.
"""
from collections import deque
from copy import deepcopy
import math
import os
import secrets
import struct
import threading
import time
from types import SimpleNamespace

from base_static_adapter import crc16, FeedbackSourceTracker, _SAMPLE_CONTENT
from wheel_envelope import bounded_wheels_from_twist


MCU_IDENTITY = 0x47344339
ALLOWED_KINDS = (2, 3, 5, 6, 7)


class TrialFault(RuntimeError):
    pass


def number(name, value, positive=False):
    if type(value) not in (int, float):
        raise ValueError(name + " must be a finite built-in number")
    try:
        value = float(value)
    except (ValueError, OverflowError):
        raise ValueError(name + " must be finite") from None
    if not math.isfinite(value) or positive and value <= 0:
        raise ValueError(name + " must be finite" + (" and positive" if positive else ""))
    return value


class MotionSourceTracker(FeedbackSourceTracker):
    """The controller state is live metadata, not frozen CAN sample content.

    serial_transport.c guard_flags() writes current state into bits 8..15 on
    each reply. READY->RUNNING may change those bits for the same CAN sample.
    All other CAN content and guard/error bits must still remain identical.
    """
    def accept(self, packet, request_start, completed):
        checked = dict(packet)
        checked["flags"] &= ~0xff00
        # Parent preserves the first request-start minus maximum CAN age and
        # checks the OLD source deadline before accepting a later new sample.
        return super().accept(checked, request_start, completed)


def quantize_forward(command, *, max_linear_m_s=.03, max_angular_deg_s=6.9,
                     diameter_m=.2, separation_m=.3):
    """Use the reviewed envelope, then reject unsupported wheel reversal.

    A nonzero angular request at zero linear speed is not part of this trial.
    A mixed request whose optimal integer pair reverses a wheel is rejected;
    it is never silently sent as a backwards or in-place motion.
    """
    v = number("linear_mps", command["linear_mps"])
    w = number("angular_rad_s", command["angular_rad_s"])
    if v < 0 or v == 0 and w != 0:
        raise TrialFault("Trial forbids reverse chassis motion and in-place rotation")
    result = bounded_wheels_from_twist(v, w, max_linear_m_s, max_angular_deg_s,
                                      diameter_m, separation_m)
    wheels = result["physical_wheels"]
    if wheels["left_rpm"] < 0 or wheels["right_rpm"] < 0:
        raise TrialFault("Quantized output reverses a physical wheel; unsupported trial command")
    return result


class CommandTracker:
    def __init__(self, task_id, armed_at, timeout_s):
        self.task_id, self.armed_at, self.timeout_s = task_id, armed_at, timeout_s
        self.latest = None

    def accept(self, command, at):
        if type(command) is not dict or command.get("task_id") != self.task_id:
            raise TrialFault("Command task identity differs")
        seq = command.get("sequence")
        if type(seq) is not int or seq < 1:
            raise TrialFault("Command requires an advancing integer sequence")
        stamp = number("received_monotonic_s", command.get("received_monotonic_s"))
        normalized = dict(task_id=self.task_id, sequence=seq, received_monotonic_s=stamp,
                          linear_mps=number("linear_mps", command.get("linear_mps")),
                          angular_rad_s=number("angular_rad_s", command.get("angular_rad_s")))
        if stamp > at:
            raise TrialFault("Command source is in the future")
        if stamp < self.armed_at:
            return None  # Prewarmed planner traffic cannot become post-ARM motion.
        if not 0 <= at - stamp < self.timeout_s:
            raise TrialFault("Command is stale at the original source")
        if self.latest is not None:
            if seq < self.latest["sequence"]:
                raise TrialFault("Command sequence moved backwards")
            if seq == self.latest["sequence"] and normalized != self.latest:
                raise TrialFault("Duplicate command changed content/source time")
            if seq > self.latest["sequence"] and stamp <= self.latest["received_monotonic_s"]:
                raise TrialFault("New command sequence did not advance its original source time")
        self.latest = normalized
        return dict(normalized)


class TrialWirePort:
    """One FD/PID/thread, one-use ARM/SETPOINT permits, complete v9 frames only."""
    def __init__(self, physical, *, clock=time.monotonic, limit=65536,
                 command_timeout_s=.25, feedback_timeout_s=.75):
        if type(limit) is not int or not 1024 <= limit <= 262144:
            raise ValueError("wire_limit must be an integer 1024..262144")
        self.physical, self.clock = physical, clock
        self.command_timeout_s = number("command_timeout_s", command_timeout_s, True)
        self.feedback_timeout_s = number("feedback_timeout_s", feedback_timeout_s, True)
        self.owner = (os.getpid(), threading.get_ident())
        self.events, self.limit, self.total_events = deque(maxlen=limit), limit, 0
        self.read_attempts, self.empty_reads = 0, 0
        self.overflow = False
        self.mode, self.arm_used, self.arm_permit = "stopped", False, None
        self.session, self.target_permit, self.last_sequence = 0, None, 0
        self.attempts = {str(kind): 0 for kind in ALLOWED_KINDS}
        self.complete_writes = {str(kind): 0 for kind in ALLOWED_KINDS}

    def owner_check(self):
        if (os.getpid(), threading.get_ident()) != self.owner:
            raise TrialFault("Original physical owner PID and thread required")

    def record(self, row):
        self.total_events += 1
        if len(self.events) == self.limit:
            self.overflow = True
        self.events.append(row)

    @property
    def timeout(self):
        return self.physical.timeout

    @timeout.setter
    def timeout(self, value):
        self.owner_check()
        self.physical.timeout = value

    @property
    def write_timeout(self):
        return self.physical.write_timeout

    @write_timeout.setter
    def write_timeout(self, value):
        self.owner_check()
        self.physical.write_timeout = value

    def permit_arm(self, session, challenge):
        self.owner_check()
        if self.arm_used or self.mode != "stopped":
            raise TrialFault("No automatic/repeated ARM within a trial")
        self.arm_permit = (session, challenge)

    def permit_target(self, session, left, right, note):
        self.owner_check()
        if self.mode != "armed" or session != self.session or not session:
            raise TrialFault("Physical target requires the current armed trial session")
        if any(type(value) is not int or abs(value) > 5 for value in (left, right)) or left < 0 or right > 0:
            raise TrialFault("Physical target violates integer forward-only wheel envelope")
        self.target_permit = (session, left, right, deepcopy(note))

    def revoke(self):
        self.owner_check()
        self.mode, self.session, self.arm_permit, self.target_permit = "stopping", 0, None, None

    def read(self, count):
        self.owner_check()
        if type(count) is not int or not 0 <= count <= 64:
            raise TrialFault("Read must be bounded to 64 bytes")
        start = self.clock()
        self.read_attempts += 1
        try:
            raw = self.physical.read(count)
        except BaseException as exc:
            self.record(dict(direction="rx", started_monotonic_s=start,
                completed_monotonic_s=self.clock(), raw_hex=None,
                error=type(exc).__name__ + ": " + str(exc)))
            raise
        if raw:
            self.record(dict(direction="rx", started_monotonic_s=start,
                             completed_monotonic_s=self.clock(), raw_hex=raw.hex() if type(raw) is bytes else None))
        else:
            self.empty_reads += 1
        return raw

    def write(self, raw):
        self.owner_check()
        if (type(raw) is not bytes or len(raw) != 20 or raw[:3] != b"\xaa\x55\x09"
                or raw[3] not in ALLOWED_KINDS or crc16(raw[:18]) != struct.unpack_from("<H", raw, 18)[0]):
            raise TrialFault("Trial wire requires a complete allowed CRC-verified 20-byte frame")
        kind, seq = raw[3], struct.unpack_from("<H", raw, 4)[0]
        left, right, session, challenge = struct.unpack_from("<hhII", raw, 6)
        if not seq or self.last_sequence and not 0 < ((seq - self.last_sequence) & 0xffff) < 0x8000:
            raise TrialFault("Physical request sequence did not advance uniquely")
        note = None
        if kind in (2, 3, 5):
            if left or right or session or challenge:
                raise TrialFault("Queries and STOP must have zero fields")
            if kind == 3:
                self.revoke()  # Terminal revocation occurs BEFORE the actual STOP write.
        elif kind == 6:
            if (self.arm_used or left or right or not session or not challenge
                    or self.arm_permit != (session, challenge) or self.mode != "stopped"):
                raise TrialFault("ARM does not match the one-use current challenge/session permit")
            self.arm_used, self.arm_permit, self.mode = True, None, "arming"
        else:
            permit = self.target_permit
            if (challenge or self.mode != "armed" or not permit
                    or (session, left, right) != permit[:3] or session != self.session):
                raise TrialFault("SETPOINT has no matching current one-use command permit")
            note, self.target_permit = permit[3], None
        if kind != 3 and self.mode not in ("stopping",) and len(self.events) >= self.limit - 512:
            raise TrialFault("Raw evidence budget nearly full; reserved STOP capacity required")
        self.last_sequence = seq
        event = dict(direction="tx", kind=kind, sequence=seq, raw_hex=raw.hex(),
                     started_monotonic_s=self.clock(), bytes_accepted=None,
                     write_attempted=False, command=note)
        try:
            if kind == 7:
                # Deepcopy, request construction, CRC and scheduling can all
                # consume time after permit_target(). Check ORIGINAL sources
                # at the final physical boundary, never renew either stamp.
                at = number("physical write clock", self.clock())
                command_age = at - number("original command source", note["original_command"]["received_monotonic_s"])
                feedback_age = at - number("original feedback source", note["feedback_source_monotonic_s"])
                if (self.mode != "armed" or session != self.session
                        or not 0 <= command_age < self.command_timeout_s
                        or not 0 <= feedback_age < self.feedback_timeout_s):
                    raise TrialFault("Physical SETPOINT boundary found expired/future command or feedback source")
                if (self.attempts["7"] == 0 and
                        at >= number("original READY lease deadline", note["ready_lease_deadline_monotonic_s"])):
                    raise TrialFault("First SETPOINT is outside the original READY 500ms lease")
                event["final_source_check"] = dict(monotonic_s=at,
                    command_age_s=command_age, feedback_age_s=feedback_age)
            self.attempts[str(kind)] += 1
            event["write_attempted"] = True
            count = self.physical.write(raw)
            event["bytes_accepted"] = count if type(count) is int else None
            if type(count) is not int or count != 20:
                raise OSError("Short/unknown physical write; no completion or retry")
            self.complete_writes[str(kind)] += 1
            return count
        except BaseException as exc:
            event["error"] = type(exc).__name__ + ": " + str(exc)
            raise
        finally:
            event["completed_monotonic_s"] = self.clock()
            self.record(event)


def run_trial_session(physical_port, api, stop_api, odom_factory, *, task_id,
                      supervisor, command_provider, cancelled=lambda: False,
                      on_phase=None, odom_sink=None, duration_s=200., warmup_timeout_s=100.,
                      post_stop_observer=None, post_stop_observation_timeout_s=2.,
                      feedback_timeout_s=.75, command_timeout_s=.25, apply_timeout_s=.30,
                      motion_sources_timeout_s=.20,
                      arm_timeout_s=4., zero_proof_timeout_s=2., zero_speed_rpm=.2,
                      max_linear_m_s=.03, max_angular_deg_s=6.9,
                      diameter_m=.2, separation_m=.3, row_limit=32768, wire_limit=65536,
                      clock=time.monotonic, sleep=time.sleep):
    """Run STOP -> live warmup -> one ARM -> selected commands -> new STOP.

    supervisor(now,phase) returns {ready:bool,complete:bool,failure:str|None}.
    command_provider(now) returns None or {task_id,sequence,received_monotonic_s,
    linear_mps,angular_rad_s}. Callback time is an original arrival, never the
    provider polling time. on_phase must be a lightweight atomic notification.
    odom_sink(odom,raw_packet,request_start,completed) must only enqueue in RAM.
    The caller owns locator/exclusive_port, ROS child cleanup and disk output.
    """
    if type(task_id) is not str or not task_id or len(task_id) > 128:
        raise ValueError("Bounded nonempty task_id required")
    for name, value in (("duration_s", duration_s), ("warmup_timeout_s", warmup_timeout_s),
                        ("feedback_timeout_s", feedback_timeout_s), ("command_timeout_s", command_timeout_s),
                        ("apply_timeout_s", apply_timeout_s), ("arm_timeout_s", arm_timeout_s),
                        ("motion_sources_timeout_s", motion_sources_timeout_s),
                        ("post_stop_observation_timeout_s", post_stop_observation_timeout_s),
                        ("zero_proof_timeout_s", zero_proof_timeout_s), ("max_linear_m_s", max_linear_m_s),
                        ("max_angular_deg_s", max_angular_deg_s), ("diameter_m", diameter_m),
                        ("separation_m", separation_m)):
        number(name, value, True)
    if (duration_s > 200 or warmup_timeout_s > 180 or feedback_timeout_s != .75
            or command_timeout_s > .25 or apply_timeout_s > .30 or arm_timeout_s > 4.
            or motion_sources_timeout_s > .20
            or post_stop_observation_timeout_s > 2.
            or max_linear_m_s > .03 or max_angular_deg_s > 6.9
            or not 0 <= number("zero_speed_rpm", zero_speed_rpm) <= .2
            or type(row_limit) is not int or not 128 <= row_limit <= 131072):
        raise ValueError("Parameters exceed this explicit bounded trial envelope")
    if (api.VERSION, api.IDENTITY, api.LEASE_MS, api.MAX_RPM) != (9, MCU_IDENTITY, 500, 5):
        raise ValueError("Reviewed v9/500ms/integer5RPM Client constants required")
    wire = TrialWirePort(physical_port, clock=clock, limit=wire_limit,
                         command_timeout_s=command_timeout_s, feedback_timeout_s=feedback_timeout_s)
    client, odom = api.Client(wire), odom_factory()
    source, rows = MotionSourceTracker(feedback_timeout_s), deque(maxlen=row_limit)
    result = dict(schema="p2p-base-motion-trial-v1", task_id=task_id,
        scope="one_bounded_forward_only_motion_trial", motion_requested=True,
        navigation_validated=False, physical_clearance_validated=False,
        first_failure=None, initial_stop_chain=None, final_stop_chain=None,
        goal_completion_reported=False, motion_started=False, phases=[],
        trial_complete=False, firmware_constants=dict(version=9, identity=MCU_IDENTITY,
            integer_rpm_cap=5, target_lease_ms=500, CAN_max_age_s=.25,
            hardware_firmware_bytes_verified=False),
        engineering=dict(feedback_source_timeout_s=feedback_timeout_s,
            command_source_timeout_s=command_timeout_s, apply_timeout_s=apply_timeout_s,
            motion_sources_timeout_s=motion_sources_timeout_s,
            apply_timeout_basis="oldest_not_yet_applied_full_write_start",
            arm_timeout_s=arm_timeout_s, zero_speed_rpm=zero_speed_rpm,
            post_stop_observation_timeout_s=post_stop_observation_timeout_s,
            arm_encoder_change_tolerance_counts=0,
            geometry_estimated=True, diameter_m=diameter_m, separation_m=separation_m))
    phase, last_clock, row_count = "initial_stop", None, 0
    session, armed_at, latest_command, ready_feedback = 0, None, None, False
    sent, sent_keys, unapplied = {}, deque(), deque()
    sent_order, applied_order, last_sent_sequence = 0, -1, None
    arm_encoder = None
    metrics = dict(feedback_responses=0, distinct_feedback_samples=0, source_age_max_s=0.,
                   setpoints=0, nonzero_setpoints=0, setpoint_gap_max_s=0., callback_max_s=0.)
    last_setpoint_start = None

    def now():
        nonlocal last_clock
        at = number("monotonic clock", clock())
        if at < 0 or last_clock is not None and at < last_clock:
            raise TrialFault("Monotonic clock moved backwards")
        last_clock = at
        return at

    def enter(name, notify=True):
        nonlocal phase
        phase = name
        at = now()
        result["phases"].append(dict(phase=name, monotonic_s=at))
        if notify and on_phase is not None:
            on_phase(name, at)
        return at

    def cancelled_check():
        value = cancelled()
        if type(value) is not bool or value:
            raise TrialFault("Explicit trial cancellation; stopping")

    def health(require_ready=False):
        cancelled_check()
        began = now()
        state = supervisor(began, phase)
        metrics["callback_max_s"] = max(metrics["callback_max_s"], now() - began)
        if (type(state) is not dict or type(state.get("ready")) is not bool
                or type(state.get("complete")) is not bool
                or state.get("failure") is not None and type(state["failure"]) is not str):
            raise TrialFault("Supervisor must return explicit ready/complete/failure")
        if state.get("failure"):
            raise TrialFault("Live localization/lidar/supervision failure: " + state["failure"])
        cancelled_check()
        if phase in ("arming", "waiting_motion_sources") and not state["ready"]:
            if state.get("readiness_reason") != "awaiting_post_arm_tf":
                raise TrialFault("Zero-target ARM wait lost a non-TF readiness condition")
        if require_ready and not state["ready"]:
            raise TrialFault("Current localization/lidar supervision is not ready")
        return state

    def capture(kind, packet, *, deliver_odometry=True, parking_after_tick_ms=None):
        nonlocal row_count, ready_feedback, arm_encoder, applied_order
        done = client.last_exchange_completion
        if not done or done["kind"] != kind or done["bytes_accepted"] != 20:
            raise TrialFault("Full-write matching physical completion missing")
        start, end = done["started_monotonic_s"], done["completed_monotonic_s"]
        if not 0 <= start <= end <= now():
            raise TrialFault("Invalid physical completion timing")
        row = dict(phase=phase, kind=kind, request_start_monotonic_s=start,
                   completed_monotonic_s=end, request_hex=done["request_hex"], packet=dict(packet),
                   reply_hex=client.trace[-1]["reply_hex"])
        rows.append(row)
        row_count += 1
        client.trace[:] = client.trace[-32:]
        if row_count >= row_limit - 128 and phase not in ("initial_stop", "final_stop"):
            raise TrialFault("Response evidence RAM budget exhausted; stopping")
        if kind != 5 and packet.get("identity") != MCU_IDENTITY:
            raise TrialFault("Unexpected MCU identity")
        if kind == 5:
            api.fresh(packet)
            if parking_after_tick_ms is not None:
                source_deltas = {key: (packet[key] - parking_after_tick_ms) & 0xffffffff
                    for key in ("sample_tick_ms", "left_tick_ms", "right_tick_ms", "speed_tick_ms")}
                row["parking_source_after_status"] = all(0 < delta < 0x80000000
                    for delta in source_deltas.values())
                row["parking_source_deltas_ms"] = source_deltas
                if not row["parking_source_after_status"]:
                    # Flags describe current MCU state, while the four CAN
                    # fields may still belong to the earlier moving sample.
                    # Retain the raw row without treating it as post-STOP
                    # motion, a tracker renewal, odometry, or a zero proof.
                    row["parking_source_rejection"] = "CAN source is not strictly after healthy stopped STATUS"
                    row["excluded_from_feedback_tracker"] = True
                    row["excluded_from_odometry"] = True
                    return row
            if phase in ("initial_stop", "waiting_ready", "final_stop", "post_stop_observation"):
                if packet["controller_state"] != 1 or max(abs(packet["speed_left_rpm"]), abs(packet["speed_right_rpm"])) > zero_speed_rpm:
                    raise TrialFault("Pre/post motion static feedback is not zero and DISARMED")
            elif phase in ("waiting_motion_sources", "armed", "normal_quiesce"):
                if packet["controller_state"] not in (5, 10):
                    raise TrialFault("Motion feedback is not READY/RUNNING")
                if phase == "waiting_motion_sources" and (packet["controller_state"] != 10 or
                        max(abs(packet["speed_left_rpm"]), abs(packet["speed_right_rpm"])) > zero_speed_rpm):
                    raise TrialFault("READY source handoff must remain at measured zero")
                if not ready_feedback:
                    if max(abs(packet["speed_left_rpm"]), abs(packet["speed_right_rpm"])) > zero_speed_rpm:
                        raise TrialFault("ARM did not start at measured zero RPM")
                    current_encoder = (packet["position_left_raw"], packet["position_right_raw"])
                    if arm_encoder is not None and current_encoder != arm_encoder:
                        raise TrialFault("Encoder moved across the zero-target ARM boundary")
                    ready_feedback = True
                    result["post_ARM_first_zero_feedback"] = dict(packet)
                left_rpm, right_rpm = packet["speed_left_rpm"], packet["speed_right_rpm"]
                measured_v = (left_rpm - right_rpm) * math.pi * diameter_m / 120.
                measured_w_deg = (-right_rpm - left_rpm) * (diameter_m / separation_m) * 3.
                row["measured_twist_estimate"] = dict(linear_mps=measured_v,
                    angular_deg_s=measured_w_deg, geometry_calibrated=False)
                if left_rpm < -zero_speed_rpm or right_rpm > zero_speed_rpm:
                    raise TrialFault("Actual raw wheel feedback reverses beyond zero-speed tolerance")
                if abs(measured_v) > max_linear_m_s or abs(measured_w_deg) > max_angular_deg_s:
                    raise TrialFault("Actual raw RPM estimated chassis speed exceeds trial ceilings")
            measured = source.accept(packet, start, end)
            row.update(measured)
            observation = odom.update(packet) if measured["distinct"] else None
            if observation is not None:
                if observation.get("discontinuity"):
                    raise TrialFault("Measured wheel sample gap; no invented odometry")
                observation = dict(observation, source_monotonic_s=measured["source_monotonic_s"])
                row["wheel_odometry"] = observation
                if odom_sink is not None and deliver_odometry:
                    try:
                        odom_sink(dict(observation), dict(packet), start, end)
                    except BaseException as exc:
                        if phase != "final_stop":
                            raise
                        result.setdefault("final_stop_callback_errors", []).append(type(exc).__name__ + ": " + str(exc))
            metrics["feedback_responses"] += 1
            metrics["distinct_feedback_samples"] += int(measured["distinct"])
            metrics["source_age_max_s"] = max(metrics["source_age_max_s"], measured["source_age_s"])
        elif phase in ("waiting_ready", "post_stop_observation") and kind == 2:
            if not stop_api.stopped(packet):
                raise TrialFault("Waiting state lost DISARMED/zero/guard status")
        elif phase in ("waiting_motion_sources", "armed", "normal_quiesce") and kind in (2, 7):
            if (packet["state"] not in (5, 10) or packet["session"] != session
                    or packet["error"] or packet["uart_critical_total"] or packet["guard_flags"] & 7 != 1):
                raise TrialFault("Armed STATUS/session/guard/fault changed")
            if phase == "waiting_motion_sources" and (packet["state"] != 10 or packet["target_pack"]):
                raise TrialFault("READY handoff left zero-target READY")
            expected_rx = last_sent_sequence
            if packet["last_rx_seq"] != expected_rx:
                raise TrialFault("MCU did not acknowledge the latest accepted command sequence")
            applied = sent.get(packet["last_applied_seq"])
            if not applied or applied["pack"] != packet["target_pack"] or applied["order"] < applied_order:
                raise TrialFault("Applied sequence/target differs from sent command history")
            if applied["order"] > applied_order:
                applied_order = applied["order"]
                while unapplied and unapplied[0][0] <= applied_order:
                    unapplied.popleft()
            if kind == 7:
                sent[packet["sequence"]]["acceptance_ACK_completed_monotonic_s"] = end
            row["applied_command"] = dict(applied)
            row["accepted_not_yet_applied"] = packet["last_applied_seq"] != expected_rx
            if not 0 <= packet["setpoint_age_ms"] < api.LEASE_MS:
                raise TrialFault("MCU target lease expired or is invalid")
        return row

    def query(kind, *, guarded=False, parking_after_tick_ms=None, **fields):
        if guarded:
            health(phase == "armed")
        ticket = client.begin_exchange(kind, **fields)
        while True:
            if guarded:
                health(phase == "armed")
                if source.source is not None:
                    source.check_age(now())
            packet = client.poll_exchange(ticket["id"])
            if packet is not None:
                return capture(kind, packet, parking_after_tick_ms=parking_after_tick_ms)
            sleep(.001)

    def stop_chain():
        nonlocal source
        ack = None
        def stop():
            nonlocal ack
            packet = client.stop()
            ack = capture(3, packet)
            return packet
        observed = []
        state = stop_api.verify_stop(SimpleNamespace(stop=stop, stopping_status=client.stopping_status),
            lambda packet: observed.append(capture(2, packet)), clock=clock, sleep=sleep)
        last = observed[-1]
        if last["request_start_monotonic_s"] < ack["completed_monotonic_s"]:
            raise TrialFault("Parking STATUS preceded the fresh STOP ACK")
        source = MotionSourceTracker(feedback_timeout_s)
        odom.invalidate()
        proof = dict(stop_ack=ack, stopped_status=last, zero_samples=[], complete=False)
        deadline = now() + zero_proof_timeout_s
        while now() < deadline:
            row = query(5, parking_after_tick_ms=state["tick_ms"])
            packet = row["packet"]
            if now() >= deadline:
                break
            if row["parking_source_after_status"] and row["distinct"]:
                proof["zero_samples"].append(dict(sample_sequence=packet["sample_sequence"],
                    source_monotonic_s=row["source_monotonic_s"],
                    completed_monotonic_s=row["completed_monotonic_s"],
                    left_rpm=packet["speed_left_rpm"], right_rpm=packet["speed_right_rpm"]))
                if len(proof["zero_samples"]) == 5:
                    proof["complete"] = True
                    return proof
            sleep(.010)
        raise TrialFault("Five different new zero sources after parking STATUS not observed")

    began = now()
    pending = None
    try:
        result["nonblocking_configuration"] = client.enable_nonblocking()
        enter("initial_stop")
        result["initial_stop_chain"] = stop_chain()
        wire.mode = "stopped"
        warm_start = enter("waiting_ready")
        next_feedback = next_status = warm_start
        while True:
            current = health()
            source.check_age(now())
            if current["complete"]:
                raise TrialFault("Goal completion cannot precede this trial ARM")
            if now() - warm_start >= warmup_timeout_s:
                raise TrialFault("Current localization/planner readiness timed out")
            if current["ready"] and pending is None:
                break
            if pending is not None:
                packet = client.poll_exchange(pending["id"])
                if packet is not None:
                    capture(pending["kind"], packet)
                    pending = None
            if pending is None:
                at = now()
                if at >= next_feedback:
                    pending = client.begin_exchange(5); next_feedback = at + .025
                elif at >= next_status:
                    pending = client.begin_exchange(2); next_status = at + .100
            sleep(.001)
        arm_encoder = (source.packet["position_left_raw"], source.packet["position_right_raw"])
        # ARM preflight is a verified zero-target boundary, not integrated
        # movement. Preserve pose but clear the preceding trapezoid interval.
        odom.invalidate()
        enter("arming")  # Disable the caller's static anchor BEFORE wire kind6.
        stopped = query(2, guarded=True)
        if not stop_api.stopped(stopped["packet"]):
            raise TrialFault("Fresh pre-ARM state is not stopped/guarded")
        session = secrets.randbelow(0xffffffff) + 1
        wire.permit_arm(session, stopped["packet"]["challenge"])
        arm_ack = query(6, guarded=True, session=session, challenge=stopped["packet"]["challenge"])
        arm_seq = arm_ack["packet"]["sequence"]
        if (arm_ack["packet"]["session"] != session or arm_ack["packet"]["last_rx_seq"] != arm_seq
                or arm_ack["packet"]["target_pack"] != 0 or arm_ack["packet"]["state"] not in (2, 4, 10)):
            raise TrialFault("ARM acceptance differs from zero-target current session")
        result["arm_ack"] = arm_ack
        arm_deadline = now() + arm_timeout_s
        while True:
            row = query(2, guarded=True)
            state = row["packet"]
            if state["state"] == 10:
                if (state["session"] != session or state["target_pack"] or state["last_rx_seq"] != arm_seq
                        or state["last_applied_seq"] != arm_seq or state["guard_flags"] & 7 != 1
                        or state["stop_verified"] or state["uart_critical_total"]):
                    raise TrialFault("READY is not the ARM zero target/session/guard")
                result["arm_ready"] = row
                break
            if state["state"] not in (2, 4) or state["session"] != session or now() >= arm_deadline:
                raise TrialFault("ARM did not reach READY before its deadline")
            sleep(.010)
        client.session, wire.session = session, session
        sent[arm_seq] = dict(sequence=arm_seq, order=0, pack=0, left_rpm=0, right_rpm=0,
                             request_start_monotonic_s=arm_ack["request_start_monotonic_s"], origin="ARM_internal_zero")
        sent_keys.append(arm_seq)
        applied_order, last_sent_sequence = 0, arm_seq
        ready_lease_deadline = (row["request_start_monotonic_s"] +
                               (api.LEASE_MS - state["setpoint_age_ms"]) / 1000.)
        refresh_start = enter("waiting_motion_sources")
        refresh_deadline = min(refresh_start + motion_sources_timeout_s,
                               ready_lease_deadline - command_timeout_s)
        result["ready_source_handoff"] = dict(started_monotonic_s=refresh_start,
            deadline_monotonic_s=refresh_deadline,
            ready_lease_deadline_monotonic_s=ready_lease_deadline,
            reserved_new_command_time_s=command_timeout_s, complete=False)
        pending = None
        next_feedback = next_status = now()
        refresh_source = None
        while True:
            live = health()
            at = now()
            source.check_age(at)
            if at >= refresh_deadline:
                raise TrialFault("Timed out waiting for real post-READY odometry/TF inside original lease")
            if pending is not None:
                packet = client.poll_exchange(pending["id"])
                if packet is not None:
                    refreshed = capture(pending["kind"], packet)
                    if pending["kind"] == 5 and refreshed["distinct"] and refresh_source is None:
                        refresh_source = refreshed["source_monotonic_s"]
                        result["ready_source_handoff"]["first_feedback"] = refreshed
                    pending = None
            if pending is None:
                if refresh_source is not None and live["ready"]:
                    tf_source = number("live post-READY TF source", live.get("tf_source_monotonic_s"))
                    odom_source = number("live post-READY odometry source", live.get("odom_source_monotonic_s"))
                    # Only a clock-domain roundoff allowance; no source is
                    # restamped or granted extra lifetime by this comparison.
                    if tf_source + 1e-6 >= refresh_source and odom_source + 1e-6 >= refresh_source:
                        result["ready_source_handoff"].update(complete=True,
                            completed_monotonic_s=now(), tf_source_monotonic_s=tf_source,
                            odom_source_monotonic_s=odom_source)
                        break
                at = now()
                if at >= next_feedback:
                    pending = client.begin_exchange(5); next_feedback = at + .025
                elif at >= next_status:
                    pending = client.begin_exchange(2); next_status = at + .050
            sleep(.001)
        wire.mode = "armed"
        armed_at = enter("armed")
        result["armed_at_monotonic_s"] = armed_at
        commands = CommandTracker(task_id, armed_at, command_timeout_s)
        motion_deadline = armed_at + duration_s
        next_feedback = next_status = next_target = armed_at
        while True:
            current = health(True)
            at = now()
            source.check_age(at)
            if at >= motion_deadline:
                raise TrialFault("Bounded motion task duration reached without completion")
            if current["complete"]:
                result["goal_completion_reported"] = True
                break
            raw_command = command_provider(at)
            if raw_command is not None:
                accepted_command = commands.accept(raw_command, now())
                if accepted_command is not None:
                    latest_command = accepted_command
            if latest_command is None:
                if at - armed_at >= command_timeout_s:
                    raise TrialFault("No new selected-source velocity after ARM")
            elif not 0 <= now() - latest_command["received_monotonic_s"] < command_timeout_s:
                raise TrialFault("Selected-source velocity expired; duplicate reads do not renew it")
            if unapplied and now() - unapplied[0][1] >= apply_timeout_s:
                raise TrialFault("Accepted SETPOINT did not become applied within trial budget")
            if pending is not None:
                packet = client.poll_exchange(pending["id"])
                if packet is not None:
                    row = capture(pending["kind"], packet)
                    if pending["kind"] == 7:
                        row["command"] = pending["command"]
                    pending = None
            if pending is None:
                at = now()
                # First confirm enabled zero feedback; thereafter SETPOINT at
                # 20Hz takes priority, followed by feedback and 10Hz STATUS.
                if ready_feedback and latest_command is not None and at >= next_target:
                    health(True)
                    commands.accept(latest_command, now())
                    quantized = quantize_forward(latest_command, max_linear_m_s=max_linear_m_s,
                        max_angular_deg_s=max_angular_deg_s, diameter_m=diameter_m, separation_m=separation_m)
                    # Quantization and callbacks consume real time. Recheck the
                    # original sources immediately before authorizing this write.
                    source.check_age(now())
                    commands.accept(latest_command, now())
                    cancelled_check()
                    left, right = quantized["wire_target"]["left_rpm"], quantized["wire_target"]["right_rpm"]
                    note = dict(original_command=dict(latest_command), quantization=quantized,
                                feedback_source_monotonic_s=source.source,
                                ready_lease_deadline_monotonic_s=ready_lease_deadline)
                    wire.permit_target(session, left, right, note)
                    pending = client.begin_exchange(7, left, right, session=session)
                    pending["command"] = note
                    sent_order += 1
                    last_sent_sequence = pending["sequence"]
                    sent[pending["sequence"]] = dict(sequence=pending["sequence"], order=sent_order,
                        pack=(left & 0xffff) | ((right & 0xffff) << 16), left_rpm=left, right_rpm=right,
                        request_start_monotonic_s=pending["started_monotonic_s"], origin="selected_command")
                    sent_keys.append(pending["sequence"])
                    unapplied.append((sent_order, pending["started_monotonic_s"]))
                    # 200 seconds cannot wrap this bounded history at 20Hz;
                    # retain a generous 1024-command application comparison tail.
                    if len(sent) > 1024:
                        del sent[sent_keys.popleft()]
                    if last_setpoint_start is not None:
                        metrics["setpoint_gap_max_s"] = max(metrics["setpoint_gap_max_s"], at - last_setpoint_start)
                    last_setpoint_start = at
                    metrics["setpoints"] += 1
                    metrics["nonzero_setpoints"] += int(bool(left or right))
                    result["motion_started"] |= bool(left or right)
                    next_target = at + .050
                elif at >= next_feedback:
                    pending = client.begin_exchange(5); next_feedback = at + .025
                elif at >= next_status:
                    pending = client.begin_exchange(2); next_status = at + .100
            sleep(.001)
        enter("normal_quiesce")
        quiesce = dict(pending_at_completion=deepcopy(pending), queries_started=0, complete=False,
                       deadline_basis="original_pending_Client_deadline_only", started_monotonic_s=now())
        result["normal_quiesce"] = quiesce
        while pending is not None:
            health(True)
            source.check_age(now())
            packet = client.poll_exchange(pending["id"])
            if packet is not None:
                capture(pending["kind"], packet)
                pending = None
            else:
                sleep(.001)
        quiesce.update(complete=True, duration_s=now() - quiesce["started_monotonic_s"])
    except BaseException as exc:
        result["first_failure"] = dict(phase=phase, error_type=type(exc).__name__, message=str(exc),
            monotonic_s=clock(), client_failure=deepcopy(client.last_exchange_failure))
    finally:
        try:
            # Revoke the physical ARM/SETPOINT fence FIRST. Only before any
            # ARM attempt, a fully written read-only query can finish within
            # its ORIGINAL Client deadline. No new request or callback runs
            # in this drain. Once ARM was attempted, STOP remains immediate.
            wire.revoke()
            old_query = client.pending_exchange
            if (phase == "waiting_ready" and not wire.arm_used and not client.session
                    and wire.attempts["6"] == 0 and wire.attempts["7"] == 0
                    and old_query is not None and old_query["kind"] in (2, 5)
                    and old_query["write_attempted"] and old_query["bytes_accepted"] == 20):
                drain = dict(pending=deepcopy(old_query), queries_started=0,
                    deadline_basis="original_pending_Client_deadline_only",
                    started_monotonic_s=now(), complete=False,
                    motion_authority_revoked=True, odometry_callback_delivered=False)
                result["pre_arm_stop_quiesce"] = drain
                try:
                    while True:
                        packet = client.poll_exchange(old_query["id"])
                        if packet is not None:
                            capture(old_query["kind"], packet, deliver_odometry=False)
                            drain["complete"] = True
                            break
                        sleep(.001)
                except BaseException as drain_exc:
                    drain["error"] = dict(error_type=type(drain_exc).__name__,
                        message=str(drain_exc), client_failure=deepcopy(client.last_exchange_failure))
                finally:
                    drain["duration_s"] = clock() - drain["started_monotonic_s"]
            # No callback, lock, disk write or source wait precedes an armed
            # fault STOP; the never-ARM read-only drain above is the sole case.
            phase = "final_stop"
            result["final_stop_chain"] = stop_chain()
        except BaseException as exc:
            phase = "final_stop"
            result["final_stop_error"] = dict(error_type=type(exc).__name__, message=str(exc),
                                               client_failure=deepcopy(client.last_exchange_failure))
            # Keep the first failure and its failed parking proof. Authority
            # remains revoked. One NEW complete STOP may obtain an independent
            # confirmation after an unacknowledged/failed STOP; this never
            # retries an old frame, completes a partial query, or resumes motion.
            result["final_stop_recovery_attempted"] = True
            try:
                wire.revoke()
                result["final_stop_recovery_chain"] = stop_chain()
            except BaseException as recovery_exc:
                result["final_stop_recovery_error"] = dict(error_type=type(recovery_exc).__name__,
                    message=str(recovery_exc), client_failure=deepcopy(client.last_exchange_failure))
        if (post_stop_observer is not None and result["goal_completion_reported"]
                and result["first_failure"] is None and result["final_stop_chain"]
                and result["final_stop_chain"]["complete"]):
            # The true STOP proof comes FIRST. This terminal tail can only
            # query the disabled base, producing current measured odometry for
            # new laser TF/AMCL evidence. Motion authority is never restored.
            observation = dict(complete=False, queries_only=True, queries_started=0,
                started_monotonic_s=clock(), timeout_s=post_stop_observation_timeout_s,
                stopped_status_monotonic_s=result["final_stop_chain"]["stopped_status"]["completed_monotonic_s"])
            result["post_stop_observation"] = observation
            try:
                enter("post_stop_observation")
                deadline = observation["started_monotonic_s"] + post_stop_observation_timeout_s
                next_feedback = next_status = now()
                pending = None
                satisfied = False
                while True:
                    cancelled_check()
                    at = now()
                    source.check_age(at)
                    if not satisfied and at >= deadline:
                        raise TrialFault("Post-STOP current localization observation timed out")
                    if not satisfied:
                        state = post_stop_observer(at, result["final_stop_chain"])
                        if (type(state) is not dict or type(state.get("complete")) is not bool
                                or state.get("failure") is not None and type(state["failure"]) is not str):
                            raise TrialFault("Post-STOP observer must return complete/failure")
                        if state.get("failure"):
                            raise TrialFault("Post-STOP observation failed: " + state["failure"])
                        satisfied = state["complete"]
                        if satisfied:
                            if now() >= deadline:
                                raise TrialFault("Post-STOP observer completed after its original deadline")
                            observation["validated_at_monotonic_s"] = now()
                            observation["observation_duration_s"] = now() - observation["started_monotonic_s"]
                            observation["normal_quiesce"] = dict(pending=deepcopy(pending),
                                queries_started=0, deadline_basis="original_pending_Client_deadline_only",
                                started_monotonic_s=now())
                    if pending is not None:
                        packet = client.poll_exchange(pending["id"])
                        if packet is not None:
                            capture(pending["kind"], packet)
                            pending = None
                    if satisfied and pending is None:
                        observation["complete"] = True
                        observation["normal_quiesce"]["duration_s"] = now() - observation["normal_quiesce"]["started_monotonic_s"]
                        break
                    if pending is None:
                        at = now()
                        if at >= next_feedback:
                            pending = client.begin_exchange(5); next_feedback = at + .025
                        elif at >= next_status:
                            pending = client.begin_exchange(2); next_status = at + .100
                        if pending is not None:
                            observation["queries_started"] += 1
                    sleep(.001)
            except BaseException as exc:
                observation["failure"] = dict(error_type=type(exc).__name__, message=str(exc))
                if result["first_failure"] is None:
                    result["first_failure"] = dict(phase="post_stop_observation", error_type=type(exc).__name__,
                        message=str(exc), monotonic_s=clock())
                # A NEW fault/cancel in this otherwise read-only tail still
                # sends a fresh immediate STOP. Keep the earlier proof intact.
                phase = "final_stop"
                try:
                    wire.revoke()
                    result["post_stop_fault_stop_chain"] = stop_chain()
                except BaseException as stop_exc:
                    result["post_stop_fault_stop_error"] = dict(error_type=type(stop_exc).__name__, message=str(stop_exc))
            finally:
                observation["ended_monotonic_s"] = clock()
        try:
            enter("finished")
        except BaseException as exc:
            result["phase_notification_error"] = type(exc).__name__ + ": " + str(exc)
        client.trace[:] = client.trace[-32:]
    result.update(elapsed_s=clock() - began, metrics=metrics, counts=wire.attempts,
        complete_write_counts=wire.complete_writes, rows=list(rows), total_rows=row_count,
        dropped_response_rows=max(0, row_count - len(rows)), wire_events=list(wire.events),
        total_wire_events=wire.total_events, dropped_wire_events=max(0, wire.total_events - len(wire.events)),
        physical_read_attempts=wire.read_attempts, empty_nonblocking_reads=wire.empty_reads,
        raw_evidence_overflow=wire.overflow)
    result["trial_complete"] = bool(result["goal_completion_reported"] and not result["first_failure"]
        and result["initial_stop_chain"] and result["initial_stop_chain"]["complete"]
        and result["final_stop_chain"] and result["final_stop_chain"]["complete"]
        and not result.get("phase_notification_error") and not wire.overflow
        and not result.get("final_stop_callback_errors")
        and not result["dropped_response_rows"])
    result["motion_trial_complete"] = result["trial_complete"]
    return result
