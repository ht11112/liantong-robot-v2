"""Bounded v9 turn/drive owner. Import opens nothing; default is query only.

The caller opens the precisely located device with exclusive_port on this same
thread and writes the returned evidence only after closing it. Geometry is an
uncalibrated estimate. ACKs acknowledge acceptance, never physical standstill.
"""
import math
import secrets
import struct
import time

from base_static_adapter import QueryOnlyPort, FeedbackSourceTracker, crc16, MCU_IDENTITY
from simple_p2p import Pose
from simple_p2p_history import PoseHistory


class BaseFault(RuntimeError):
    pass


class AppliedTargets:
    """Bounded accepted/current-applied identities, observed equally in 2/7."""
    def __init__(self, ready_packet):
        self.session = ready_packet['session']
        self.order, self.applied_order = 0, -1
        self.targets = {ready_packet['last_applied_seq']:dict(order=-1,pack=0,accepted_monotonic_s=None)}
        self.observe(ready_packet)

    def register(self, sequence, left, right, accepted_at):
        if (type(sequence) is not int or not 1 <= sequence <= 65535
                or sequence in self.targets or not finite(accepted_at)):
            raise BaseFault('Invalid or reused accepted SETPOINT sequence')
        self.targets[sequence] = dict(order=self.order,pack=(left & 65535)|((right & 65535)<<16),
                                      accepted_monotonic_s=accepted_at)
        self.order += 1

    def observe(self, packet, at=None):
        if at is not None:
            self.check_deadline(at)  # Late evidence cannot erase an expired pending deadline.
        if (packet['identity'] != MCU_IDENTITY or packet['session'] != self.session
                or packet['state'] not in (5,10) or packet['error']
                or packet['reserved'] or packet['status'] or packet['uart_critical_short']
                or packet['uart_critical_total']):
            raise BaseFault('Applied observation lost healthy MCU/session identity')
        applied = packet['last_applied_seq']
        item = self.targets.get(applied)
        if item is None or item['pack'] != packet['target_pack'] or item['order'] < self.applied_order:
            raise BaseFault('Applied command/target identity mismatch or sequence moved backwards')
        self.applied_order = item['order']
        for sequence in list(self.targets):
            if self.targets[sequence]['order'] < self.applied_order:
                del self.targets[sequence]

    def check_deadline(self, at):
        if any(item['order'] > self.applied_order and at >= item['accepted_monotonic_s']+.3
               for item in self.targets.values()):
            raise BaseFault('Accepted target not observed applied within 300ms')


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def quantize(command, config):
    """Enumerate integer RPM; never round past either requested axis limit."""
    v, w = command.linear_m_s, command.angular_rad_s
    if not all(finite(x) for x in (v, w)) or v < 0:
        raise BaseFault("Invalid/reverse controller intent")
    if v > min(.03, config.max_linear_m_s) or abs(w) > min(math.radians(6.9), config.max_angular_rad_s):
        raise BaseFault("Controller intent exceeds configured trial limits")
    if v and w or command.phase == "drive" and w or command.phase == "turn" and v:
        raise BaseFault("Only separate straight and in-place turn intents are supported")
    if command.phase not in ("turn", "drive", "stop", "fault"):
        raise BaseFault("Unknown controller phase")
    if command.phase in ("stop", "fault") and (v or w):
        raise BaseFault("Terminal intent must be zero")
    d, b = config.wheel_diameter_m, config.wheel_separation_m
    if not all(finite(x) and x > 0 for x in (d, b)):
        raise BaseFault("Invalid wheel geometry")
    candidates = []
    for rpm in range(6):
        speed, rate = rpm * math.pi*d/60., 2*rpm*math.pi*d/(60.*b)
        if v and speed <= v:
            candidates.append((rpm, -rpm, speed, 0.))
        elif w and rate <= abs(w):
            sign = 1 if w > 0 else -1
            candidates.append((-sign*rpm, -sign*rpm, 0., sign*rate))
        elif not v and not w:
            candidates.append((0, 0, 0., 0.))
    left, right, actual_v, actual_w = candidates[-1] if candidates else (0, 0, 0., 0.)
    return dict(left_rpm=left, right_rpm=right, linear_m_s=actual_v,
                angular_rad_s=actual_w, requested_linear_m_s=v, requested_angular_rad_s=w)


class SourceTracker(FeedbackSourceTracker):
    def accept(self, packet, request_start, completed):
        # Firmware puts *live* READY/RUNNING state in otherwise frozen CAN data.
        original = self.packet
        if original is not None and self.sequence == packet["sample_sequence"]:
            self.packet = dict(original, flags=(original["flags"] & ~0xff00) | (packet["flags"] & 0xff00))
        try:
            return super().accept(packet, request_start, completed)
        finally:
            if self.sequence == packet["sample_sequence"] and self.packet is not None:
                self.packet = dict(self.packet, flags=packet["flags"])


class SessionPort(QueryOnlyPort):
    """One owner, complete CRC-checked frames, one ARM, explicit target permits."""
    def __init__(self, physical, execute=False, clock=time.monotonic, limit=131072):
        super().__init__(physical)
        self.execute, self.clock, self.limit = execute, clock, limit
        self.events, self.phase, self.arm_count, self.permit = [], "query", 0, None
        self.attempts = {str(k): 0 for k in (2, 3, 5, 6, 7)}
        self.complete_writes = dict(self.attempts)
        self.finishing = False

    def reserve(self):
        self._check_owner()
        if len(self.events) >= self.limit - (0 if self.finishing else 1024):
            raise BaseFault("Raw evidence RAM limit reached")

    def write(self, raw):
        self._check_owner()
        if (type(raw) is not bytes or len(raw) != 20 or raw[:3] != b"\xaa\x55\x09"
                or int.from_bytes(raw[18:], "little") != crc16(raw[:18])):
            raise BaseFault("Only complete valid v9 frames permitted")
        kind, seq, left, right, session, challenge = struct.unpack_from("<BHhhII", raw, 3)
        if not seq or str(kind) not in self.attempts:
            raise BaseFault("Request kind/sequence forbidden")
        if kind not in (2, 3, 5, 6, 7) or not self.execute and kind not in (2, 5):
            raise BaseFault("Query mode permanently forbids STOP/ARM/SETPOINT")
        if kind != 7 and (left or right) or kind not in (3, 6, 7) and session or kind != 6 and challenge:
            raise BaseFault("Unexpected wire fields")
        if kind == 6:
            if self.phase != "arming" or self.arm_count or not session or not challenge:
                raise BaseFault("Only one explicit ARM permitted")
        if kind == 7:
            permit, self.permit = self.permit, None
            at = self.clock()
            if (self.phase != "active" or not session or max(abs(left), abs(right)) > 5
                    or permit is None or (left, right, session) != permit["target"]
                    or any(not 0 <= at-permit[key] < budget for key, budget in
                           (("command_source", .25), ("feedback_source", .75), ("scan_source", .75)))
                    or at >= permit["lease_deadline"]):
                raise BaseFault("SETPOINT lacks current measured-source/scan/session permit")
        self.reserve()
        event = dict(direction="tx", kind=kind, sequence=seq, raw_hex=raw.hex(),
                     started_monotonic_s=self.clock(), bytes_accepted=None)
        self.events.append(event)
        try:
            self.attempts[str(kind)] += 1
            if kind == 6:
                self.arm_count += 1
            written = self._physical.write(raw)
            event["bytes_accepted"] = written
            if type(written) is not int or written != 20:
                raise BaseFault("Short/unknown physical write; never retry the old frame")
            self.complete_writes[str(kind)] += 1
            return written
        except BaseException as exc:
            event["error"] = str(exc)
            raise
        finally:
            event["completed_monotonic_s"] = self.clock()

    def read(self, count):
        self.reserve()
        started = self.clock()
        raw = super().read(count)
        if type(raw) is not bytes or len(raw) > count:
            raise BaseFault("Invalid bounded read")
        if raw:
            self.events.append(dict(direction="rx", raw_hex=raw.hex(),
                started_monotonic_s=started, completed_monotonic_s=self.clock()))
        return raw


def run_session(physical, api, core, controller, *, scan_provider, boot_id,
                execute=False, cancelled=lambda: False, clock=time.monotonic,
                sleep=time.sleep, wall_clock=time.time, evidence_limit=131072,
                expected_start_encoders=None, event_sink=None):
    """Use one caller-owned FD/thread. execute=False only observes software zero.

    scan_provider is called only for execution and must be a short RAM read.
    STOP phases do not call external callbacks before sending the physical STOP.
    Source ages use original request start minus worst CAN age, never receipt.
    """
    if type(execute) is not bool or not isinstance(boot_id, str) or not boot_id:
        raise ValueError("Explicit execute bool and boot identity required")
    if (api.VERSION, api.IDENTITY, api.LEASE_MS, api.MAX_RPM) != (9, MCU_IDENTITY, 500, 5):
        raise ValueError("Reviewed v9 protocol required")
    if type(evidence_limit) is not int or not 2048 <= evidence_limit <= 262144:
        raise ValueError("Evidence limit must be 2048..262144")
    if expected_start_encoders is not None and (type(expected_start_encoders) is not dict
            or set(expected_start_encoders) != {"left_raw", "right_raw"}
            or any(type(value) is not int or not 0 <= value <= 0xffffffff
                   for value in expected_start_encoders.values())):
        raise ValueError("Expected original encoder fields must be uint32 left_raw/right_raw")
    cfg = controller.config
    if not finite(cfg.duration_s) or not 0 < cfg.duration_s <= 200:
        raise ValueError("Trial duration must be positive and <=200 seconds")
    class RawRpm(core.MeasuredOdometry):
        DEADBAND_RPM = 0.
    odom = RawRpm(cfg.wheel_diameter_m, cfg.wheel_separation_m, calibrated=False)
    port = SessionPort(physical, execute, clock, evidence_limit)
    client, source = api.Client(port), SourceTracker(.75)
    history = PoseHistory(capacity=256,epoch=boot_id)
    rows, intents = [], []
    control_updates = []
    control_metrics = dict(control_ticks=0,skipped_control_ticks=0,feedback_distinct=0,
        control_period_s=.05,feedback_period_s=.04,status_period_s=.2,
        same_target_keepalive_s=.1,minimum_setpoint_period_s=.05,
        max_feedback_request_gap_s=0.,max_status_request_gap_s=0.,max_setpoint_request_gap_s=0.,
        max_inflight_transactions=0,control_ticks_while_inflight=0,max_control_interval_s=0.,
        max_control_sample_age_s=0.,feedback_requests=0,status_requests=0,setpoint_requests=0,
        arm_history_reset_count=0)
    measurement_anchors = []
    result = dict(schema="simple-p2p-base-v1", boot_id=boot_id, execute=execute,
        origin=None, first_failure=None, initial_stop_chain=None, final_stop_chain=None,
        session_complete=False, motion_complete=False, geometry_calibrated=False,
        geometry=dict(diameter_m=odom.diameter, separation_m=odom.separation),
        host_source_timeout_s=.75, CAN_source_timeout_s=.25, zero_speed_rpm=.2,
        max_setpoint_hz=20, MCU_lease_s=.5, arm_handoff=None,
        encoder_unchanged_since_origin=True, first_origin_encoder_change=None,
        expected_start_encoders=expected_start_encoders, event_sink_errors=[])
    phase, latest, last_scan, scan_phase = "origin", None, None, None
    began, last_time, lease_deadline = clock(), None, float("inf")
    applied = None
    measurement_tick = measurement_time = None
    odometry_failure = None

    def now():
        nonlocal last_time
        at = clock()
        if not finite(at) or at < 0 or last_time is not None and at < last_time:
            raise BaseFault("Monotonic clock invalid/backwards")
        last_time = at
        return at

    def fail(exc, where):
        if result["first_failure"] is None:
            result["first_failure"] = dict(phase=where, message=str(exc),
                error_type=type(exc).__name__, monotonic_s=clock())

    def emit(kind, packet=None, measurement=None, pose=None):
        if event_sink is None:
            return
        event = dict(event=kind, phase=phase, boot_id=boot_id, execute=execute,
            event_monotonic_s=clock(), source_monotonic_s=None if measurement is None else measurement["source_monotonic_s"],
            raw_packet=None if packet is None else dict(packet),
            pose=None if pose is None else {key: pose[key] for key in ("x", "y", "yaw")})
        if pose is not None:
            event.update(measurement_monotonic_s=pose.get('measurement_monotonic_s'),
                pose_sample_sequence=pose.get('sample_sequence'),history_summary=history.summary(),
                control_summary=dict(control_metrics))
        try:
            event_sink(event)  # The caller must only replace a bounded RAM slot.
        except BaseException as exc:
            if len(result["event_sink_errors"]) < 8:
                result["event_sink_errors"].append(type(exc).__name__+": "+str(exc))
            if port.phase != "stopping":
                raise BaseFault("RAM event callback failed: "+str(exc))

    def expected_encoder(packet):
        if expected_start_encoders is not None and any(packet[field] != expected_start_encoders[key]
                for field, key in (("position_left_raw", "left_raw"), ("position_right_raw", "right_raw"))):
            raise BaseFault("Expected start encoder changed; reset the local frame before a new task")

    def health():
        nonlocal last_scan
        at = now()
        cancellation = cancelled()
        if type(cancellation) is not bool or cancellation:
            raise BaseFault("Session cancelled")
        if at-began >= cfg.duration_s+10:
            raise BaseFault("Bounded session deadline reached")
        if source.source is not None:
            source.check_age(at)
        if execute:
            last_scan = scan_provider(at, scan_phase or phase)
            if (not isinstance(last_scan, dict) or last_scan.get("ok") is not True
                    or not finite(last_scan.get("source_monotonic_s"))
                    or not 0 <= at-last_scan["source_monotonic_s"] < .75):
                raise BaseFault("Scan/clearance rejected: " + str(last_scan.get("reason", "invalid") if isinstance(last_scan, dict) else "invalid"))
        if port.phase == "active" and at >= lease_deadline:
            raise BaseFault("Original MCU lease deadline reached")
        if port.phase == "active" and applied is not None:
            applied.check_deadline(at)

    def record(kind, packet):
        done = client.last_exchange_completion
        if not done or done["bytes_accepted"] != 20 or len(rows) >= 40000:
            raise BaseFault("Missing full transaction or response RAM exhausted")
        row = dict(phase=phase, kind=kind, packet=dict(packet),
            request_start_monotonic_s=done["started_monotonic_s"],
            completed_monotonic_s=done["completed_monotonic_s"],
            request_hex=done["request_hex"], reply_hex=client.trace[-1]["reply_hex"])
        rows.append(row)
        client.trace[:] = client.trace[-8:]
        if kind == 5 and result["origin"] is not None:
            original = result["origin"]["raw_packet"]
            if any(packet[key] != original[key] for key in ("position_left_raw", "position_right_raw")):
                result["encoder_unchanged_since_origin"] = False
                if result["first_origin_encoder_change"] is None:
                    result["first_origin_encoder_change"] = row
        if kind != 5:
            emit("phase")
        return row

    def exchange(kind, left=0, right=0, session=0, challenge=0, guard=True, stopping=False):
        ticket = client.begin_exchange(kind, left, right, session, challenge,
                                       _stopping_observation=stopping)
        while True:
            if guard:
                health()
            packet = client.poll_exchange(ticket["id"])
            if packet is not None:
                return packet, record(kind, packet)
            sleep(.001)

    def stopped(packet):
        return (packet["state"] == 1 and not packet["session"] and not packet["target_pack"]
            and not packet["error"] and packet["stop_verified"] == 1
            and packet["guard_flags"] & 7 == 5 and not packet["uart_critical_total"])

    def accumulated(packet, observed, measurement, reset=False):
        nonlocal latest,measurement_tick,measurement_time,odometry_failure
        if observed is None or observed['discontinuity']:
            history.invalidate()
            odometry_failure = 'Measured MCU odometry discontinuity; missing motion not integrated'
            raise BaseFault(odometry_failure)
        if odometry_failure is not None:
            raise BaseFault(odometry_failure)
        if reset:
            history.reset(epoch=boot_id)
            control_metrics['arm_history_reset_count'] += 1
        if measurement_tick is None:
            measurement_time = measurement['source_monotonic_s']
        else:
            delta=(packet['speed_tick_ms']-measurement_tick)&0xffffffff
            if not 0 < delta < 0x80000000:
                history.invalidate()
                odometry_failure='MCU measurement clock did not advance'
                raise BaseFault(odometry_failure)
            measurement_time += delta/1000.
        if measurement_tick is None or reset:
            measurement_anchors.append(dict(sample_sequence=packet['sample_sequence'],
                mcu_speed_tick_ms=packet['speed_tick_ms'],source_monotonic_s=measurement['source_monotonic_s'],
                measurement_monotonic_s=measurement_time,
                reason='verified_zero_ARM_clock_continuation' if reset else 'origin'))
        measurement_tick = packet['speed_tick_ms']
        latest = dict(observed,source_monotonic_s=measurement['source_monotonic_s'],
                      measurement_monotonic_s=measurement_time)
        history.push(latest,epoch=boot_id)
        return latest

    def feedback(packet, row, parking_tick=None, integrate=True):
        nonlocal latest
        api.fresh(packet)
        if parking_tick is not None and not all(0 < (packet[k]-parking_tick) & 0xffffffff < 0x80000000
                for k in ("sample_tick_ms", "left_tick_ms", "right_tick_ms", "speed_tick_ms")):
            row["excluded_reason"] = "CAN source not strictly after stopped STATUS"
            return None
        measurement = source.accept(packet, row["request_start_monotonic_s"], row["completed_monotonic_s"])
        row.update(measurement)
        if not measurement["distinct"]:
            return None
        if integrate:
            observed = odom.update(packet)
            row['odometry_observation'] = None if observed is None else dict(observed)
            accumulated(packet,observed,measurement)
            row["odometry"] = dict(latest)
        emit("odometry", packet, measurement,
             latest if latest is not None and latest["sample_sequence"] == packet["sample_sequence"] else None)
        return packet

    def stop_chain(integrate_parking=False):
        nonlocal phase, source, latest
        port.phase, port.permit, port.finishing = "stopping", None, True
        client.cancel_for_stop()
        proof = dict(complete=False, ack=None, stopped_status=None, zero_samples=[],
                     parking_position_error=odometry_failure, position_verified=False,
                     settling_nonzero_samples=0, zero_run_restarts=0)
        motion_source = source
        try:
            ack, ack_row = exchange(3, guard=False)
            proof["ack"] = ack_row
            client.needs_stop = False
            deadline = now()+2.
            while True:
                if now() >= deadline:
                    raise BaseFault("STOP healthy STATUS timed out")
                state, status_row = exchange(2, guard=False, stopping=True)
                if stopped(state):
                    proof["stopped_status"] = status_row
                    break
                sleep(.01)
            source = SourceTracker(.75)
            if not integrate_parking:
                odom.invalidate()
            deadline = now()+2.
            while len(proof["zero_samples"]) < 5:
                if now() >= deadline:
                    raise BaseFault("Five new post-STATUS zero samples timed out")
                packet, row = exchange(5, guard=False)
                if now() >= deadline:
                    raise BaseFault("Post-STATUS zero deadline reached")
                if integrate_parking and proof["parking_position_error"] is None:
                    try:
                        api.fresh(packet)
                        # Live DISARMED flags may change on the last CAN sample.
                        # The healthy STATUS and fresh() validate that transition;
                        # no timestamp, wheel/status content or source is renewed.
                        if motion_source.sequence == packet["sample_sequence"]:
                            motion_source.packet = dict(motion_source.packet, flags=packet["flags"])
                        measurement = motion_source.accept(packet,
                            row["request_start_monotonic_s"], row["completed_monotonic_s"])
                        row["parking_motion_source"] = measurement
                        if measurement["distinct"]:
                            observed = odom.update(packet)
                            if observed is None or observed["discontinuity"]:
                                history.invalidate()
                                raise BaseFault("Parking MCU odometry discontinuity; final position unverified")
                            accumulated(packet,observed,measurement)
                            row["parking_odometry"] = dict(latest)
                    except BaseException as exc:
                        proof["parking_position_error"] = type(exc).__name__+": "+str(exc)
                sample = feedback(packet, row, state["tick_ms"], integrate=False)
                if sample is not None:
                    if sample["controller_state"] != 1:
                        raise BaseFault("New post-STATUS feedback lost DISARMED state")
                    if max(abs(sample["speed_left_rpm"]), abs(sample["speed_right_rpm"])) > .2:
                        proof["settling_nonzero_samples"] += 1
                        if proof["zero_samples"]:
                            proof["zero_run_restarts"] += 1
                        proof["zero_samples"].clear()
                        row["zero_proof_rejected_reason"] = "Healthy DISARMED wheel speed still above 0.2 RPM; original deadline unchanged"
                    else:
                        proof["zero_samples"].append(dict(sample))
                sleep(.005)
            proof["complete"] = True
            proof["position_verified"] = bool(integrate_parking
                and proof["parking_position_error"] is None and latest is not None
                and latest["sample_sequence"] == proof["zero_samples"][-1]["sample_sequence"])
            if integrate_parking:
                proof["final_odometry"] = None if latest is None else dict(latest)
            return proof
        except BaseException as exc:
            proof["error"] = type(exc).__name__+": "+str(exc)
            return proof

    def observe_origin():
        state, _ = exchange(2, guard=False)
        if not stopped(state):
            raise BaseFault("Origin requires healthy guarded DISARMED/session0/target0")
        deadline = now()+.75
        while result["origin"] is None:
            if now() >= deadline:
                raise BaseFault("Initial measured zero source timed out")
            packet, row = exchange(5, guard=False)
            if packet["controller_state"] != 1 or max(abs(packet["speed_left_rpm"]), abs(packet["speed_right_rpm"])) > .2:
                raise BaseFault("Origin requires actual zero speed within 0.2 RPM")
            if feedback(packet, row) is not None:
                expected_encoder(packet)
                result["origin"] = dict(pose=dict(x=0., y=0., yaw=0.), boot_id=boot_id,
                    source_monotonic_s=source.source, received_monotonic_s=now(),
                    received_wall_s=wall_clock(), raw_packet=dict(packet),
                    software_zero_only=True, MCU_encoder_reset=False)
                emit("origin", packet, dict(source_monotonic_s=source.source), latest)

    try:
        result["nonblocking"] = client.enable_nonblocking()
        observe_origin()
        if not execute:
            result["session_complete"] = True
        else:
            phase = "initial_stop"
            result["initial_stop_chain"] = stop_chain()
            if not result["initial_stop_chain"]["complete"]:
                raise BaseFault(result["initial_stop_chain"]["error"])
            port.finishing = False
            preview = controller.tick(Pose(latest["x"], latest["y"], latest["yaw"]), now())
            quantize(preview, cfg)
            scan_phase = preview.phase
            health()  # Actual first turn intent requires swept-area clearance before ARM.
            if preview.phase == "fault":
                raise BaseFault("Controller fault: "+preview.reason)
            if preview.phase == "stop" and preview.reason in ("arrived", "cancelled"):
                result["termination_reason"] = preview.reason
                result["motion_complete"] = preview.reason == "arrived"
                return result
            phase, port.phase = "arming", "arming"
            health()
            state, _ = exchange(2)
            if not stopped(state):
                raise BaseFault("ARM preflight requires new healthy stopped STATUS")
            before = result["initial_stop_chain"]["zero_samples"][-1]
            if expected_start_encoders is not None:
                prearm_deadline = now()+.2
                while True:
                    if now() >= prearm_deadline:
                        raise BaseFault("New pre-ARM encoder verification timed out")
                    packet, row = exchange(5)
                    sample = feedback(packet,row,integrate=False)
                    if sample is not None:
                        if sample["controller_state"] != 1 or max(abs(sample["speed_left_rpm"]),abs(sample["speed_right_rpm"])) > .2:
                            raise BaseFault("Pre-ARM encoder verification requires actual stopped DISARMED feedback")
                        before = sample
                        result["expected_start_verification"] = row
                        break
                    sleep(.001)
            expected_encoder(before)
            if expected_start_encoders is not None and not result["encoder_unchanged_since_origin"]:
                raise BaseFault("Start encoder changed during preparation; reset the local frame")
            session = secrets.randbelow(0xffffffff)+1
            _, arm_row = exchange(6, session=session, challenge=state["challenge"])
            arm_deadline, ready_lower_bound = now()+4., arm_row["request_start_monotonic_s"]
            while True:
                if now() >= arm_deadline:
                    raise BaseFault("ARM timed out")
                state, row = exchange(2)
                if state["state"] == 10 and state["session"] == session and not state["target_pack"]:
                    lease_deadline = ready_lower_bound+.5
                    client.session = session
                    break
                if state["state"] not in (2, 4) or state["session"] != session or state["target_pack"]:
                    raise BaseFault("ARM did not remain zero-target/expected session")
                ready_lower_bound = row["request_start_monotonic_s"]
                sleep(.01)
            phase = "ready_feedback"
            refresh_deadline = min(now()+.2, lease_deadline-.05)
            while True:
                if now() >= refresh_deadline:
                    raise BaseFault("READY true new zero feedback deadline reached")
                packet, row = exchange(5)
                api.fresh(packet)
                if packet["sample_sequence"] == source.sequence:
                    row["excluded_reason"] = "Waiting for first new post-READY CAN sample"
                    sleep(.001)
                    continue
                if measurement_tick is not None and packet['speed_tick_ms']==measurement_tick:
                    row['excluded_reason']='Waiting for advancing true MCU speed sample after READY'
                    sleep(.001)
                    continue
                sample = feedback(packet, row, integrate=False)
                if sample is not None:
                    if (sample["controller_state"] != 10 or max(abs(sample["speed_left_rpm"]), abs(sample["speed_right_rpm"])) > .2
                            or any(sample[k] != before[k] for k in ("position_left_raw", "position_right_raw"))):
                        raise BaseFault("ARM zero-target handoff moved encoder/real wheels")
                    result["arm_handoff"] = dict(before=dict(before), after=dict(sample),
                        skipped_interval_integrated=False, ready_lease_deadline=lease_deadline)
                    # ARM states 2/4 do not permit original Client feedback use.
                    # No motion target existed; establish the next measured interval.
                    odom.invalidate()
                    accumulated(sample,odom.update(sample),dict(source_monotonic_s=source.source),reset=True)
                    emit("odometry", sample, dict(source_monotonic_s=source.source), latest)
                    break
                sleep(.001)
            phase, port.phase = "motion", "active"
            applied = AppliedTargets(state)
            next_control = now()
            next_feedback,next_status = next_control+.04,next_control+.2
            current_axis, barrier, barrier_count, barrier_zero_tick = None, None, 0, None
            pending = desired = terminal = None
            last_send = last_pair = None
            last_starts = {}
            while True:
                health()
                at = now()
                # Acquisition owns every returned sample; no raw/latest mailbox
                # sits before MCU-time integration. Only control intents coalesce.
                if pending is not None:
                    packet = client.poll_exchange(pending['ticket']['id'])
                    if packet is not None:
                        kind = pending['ticket']['kind']
                        row = record(kind,packet)
                        if kind == 7:
                            target = pending['target']  # Immutable send-time intent.
                            if packet['last_rx_seq'] != packet['sequence'] or packet['state'] not in (5,10):
                                raise BaseFault('SETPOINT acceptance/state mismatch')
                            applied.register(packet['sequence'],target['left_rpm'],target['right_rpm'],row['completed_monotonic_s'])
                            applied.observe(packet,row['completed_monotonic_s'])
                            row['quantized_command'] = dict(target)
                            row['control_pose'] = dict(pending['control_pose'])
                            row['control_tick_monotonic_s'] = pending['control_time']
                            intents.append(dict(phase=pending['command'].phase,reason=pending['command'].reason,**target))
                            if pending['barrier'] is not None and barrier == pending['barrier'] and barrier_zero_tick is None:
                                barrier_zero_tick = packet['tick_ms']
                            if pending['intended'] is not None and pending['barrier'] is None:
                                current_axis = pending['intended']
                        elif kind == 5:
                            sample = feedback(packet,row)
                            if sample is not None:
                                control_metrics['feedback_distinct'] += 1
                            if (sample is not None and barrier is not None and barrier_zero_tick is not None
                                    and all(0 < (sample[k]-barrier_zero_tick)&0xffffffff < 0x80000000
                                            for k in ('sample_tick_ms','left_tick_ms','right_tick_ms','speed_tick_ms'))):
                                if max(abs(sample['speed_left_rpm']),abs(sample['speed_right_rpm'])) <= .2:
                                    barrier_count += 1
                                    if barrier_count >= 5:
                                        current_axis,barrier = barrier,None
                                else:
                                    barrier_count = 0
                        else:
                            applied.observe(packet,row['completed_monotonic_s'])
                        pending = None
                        if terminal is not None:
                            result['normal_quiesce']['complete'] = True
                            result['normal_quiesce']['completed_monotonic_s'] = now()
                at = now()
                if terminal is None and at >= next_control:
                    pose = history.latest(at,max_age=.75)
                    command = controller.tick(Pose(pose['x'],pose['y'],pose['yaw']),at)
                    control_metrics['skipped_control_ticks'] += int((at-next_control)/.05)
                    control_metrics['control_ticks'] += 1
                    if pending is not None:
                        control_metrics['control_ticks_while_inflight'] += 1
                    if len(control_updates) >= 5000:
                        raise BaseFault('Bounded control evidence exhausted')
                    if control_updates:
                        control_metrics['max_control_interval_s'] = max(control_metrics['max_control_interval_s'],
                            at-control_updates[-1]['monotonic_s'])
                    control_metrics['max_control_sample_age_s'] = max(control_metrics['max_control_sample_age_s'],
                        at-pose['source_monotonic_s'])
                    control_updates.append(dict(monotonic_s=at,source_monotonic_s=pose['source_monotonic_s'],
                        measurement_monotonic_s=pose['measurement_monotonic_s'],sample_sequence=pose['sample_sequence'],
                        x=pose['x'],y=pose['y'],yaw=pose['yaw'],phase=command.phase,reason=command.reason,
                        linear_m_s=command.linear_m_s,angular_rad_s=command.angular_rad_s))
                    next_control = at+.05  # Never replay a queue of missed control ticks.
                    if command.phase == "fault":
                        raise BaseFault("Controller fault: "+command.reason)
                    if command.phase == "stop" and command.reason in ("arrived", "cancelled"):
                        result["termination_reason"] = command.reason
                        result["motion_complete"] = command.reason == "arrived"
                        terminal = command
                        if command.reason == 'cancelled':
                            break
                        result['normal_quiesce'] = dict(pending=None if pending is None else pending['ticket'],
                            started_monotonic_s=at,complete=pending is None,queries_started=0,
                            deadline_basis='original_Client_transaction_only')
                        if pending is None:
                            result['normal_quiesce']['completed_monotonic_s'] = at
                    else:
                        phase = command.phase
                        target = quantize(command,cfg)
                        intended = command.phase if target['left_rpm'] or target['right_rpm'] else None
                        if intended and current_axis is not None and intended != current_axis and barrier is None:
                            barrier,barrier_count,barrier_zero_tick = intended,0,None
                        scan_phase = 'turn' if 'turn' in (current_axis,intended,barrier) else command.phase
                        desired = dict(command=command,target=target,intended=intended,control_time=at,control_pose=pose)
                if terminal is not None:
                    if pending is None:
                        break
                    sleep(.001)
                    continue
                if pending is None:
                    at = now()
                    # A control tick may have changed desired while the prior
                    # target ACK was in flight. Recheck at the physical send
                    # boundary, using axes already sent (possibly applied).
                    if (desired is not None and desired['intended'] is not None and current_axis is not None
                            and desired['intended'] != current_axis and barrier is None):
                        barrier,barrier_count,barrier_zero_tick=desired['intended'],0,None
                    if desired is not None:
                        scan_phase='turn' if 'turn' in (current_axis,desired['intended'],barrier) else desired['command'].phase
                    target = None if desired is None else dict(desired['target'])
                    if target is not None and barrier is not None:
                        target.update(left_rpm=0,right_rpm=0,linear_m_s=0.,angular_rad_s=0.)
                    pair = None if target is None else (target['left_rpm'],target['right_rpm'])
                    send_due = (at if last_send is None else last_send+(.05 if pair != last_pair else .1))
                    due = []
                    if target is not None and at >= send_due:
                        due.append((send_due,7))
                    if at >= next_feedback:
                        due.append((next_feedback,5))
                    if at >= next_status:
                        due.append((next_status,2))
                    if due:
                        _,kind = min(due)
                        health()
                        if kind == 7:
                            port.permit = dict(target=(pair[0],pair[1],client.session),
                                command_source=desired['control_time'],feedback_source=source.source,
                                scan_source=last_scan['source_monotonic_s'],lease_deadline=lease_deadline)
                            ticket = client.begin_exchange(7,pair[0],pair[1],session=client.session)
                            pending = dict(desired,ticket=ticket,target=target,barrier=barrier)
                            if desired['intended'] is not None and barrier is None:
                                # Full write can take effect before its ACK.
                                # Retain this axis for clearance and transitions.
                                current_axis=desired['intended']
                            last_send,last_pair = ticket['started_monotonic_s'],pair
                            lease_deadline = last_send+.5
                        else:
                            ticket = client.begin_exchange(kind)
                            pending = dict(ticket=ticket)
                            if kind == 5:
                                next_feedback = ticket['started_monotonic_s']+.04
                            else:
                                next_status = ticket['started_monotonic_s']+.2
                        gap_key={2:'max_status_request_gap_s',5:'max_feedback_request_gap_s',7:'max_setpoint_request_gap_s'}[kind]
                        if kind in last_starts:
                            control_metrics[gap_key] = max(control_metrics[gap_key],ticket['started_monotonic_s']-last_starts[kind])
                        last_starts[kind] = ticket['started_monotonic_s']
                        count_key={2:'status_requests',5:'feedback_requests',7:'setpoint_requests'}[kind]
                        control_metrics[count_key] += 1
                        control_metrics['max_inflight_transactions'] = 1
                applied.check_deadline(now())
                sleep(.001)
    except BaseException as exc:
        fail(exc, phase)
    finally:
        if execute:
            phase = "final_stop"
            result["final_stop_chain"] = stop_chain(integrate_parking=port.arm_count > 0)
            if not result["final_stop_chain"]["complete"]:
                fail(BaseFault(result["final_stop_chain"]["error"]), phase)
            if result["motion_complete"] and not port.arm_count:
                proof = result["final_stop_chain"]
                unchanged = bool(result["origin"] and result["initial_stop_chain"]
                    and result["initial_stop_chain"]["complete"] and proof["complete"]
                    and result["encoder_unchanged_since_origin"])
                result["position_verification"] = dict(method="encoder_unchanged_since_origin",
                    all_observed_encoders_unchanged=unchanged,
                    origin_sample_sequence=None if result["origin"] is None else result["origin"]["raw_packet"]["sample_sequence"],
                    final_post_status_sample=None if not proof["zero_samples"] else dict(proof["zero_samples"][-1]),
                    pose=dict(x=0., y=0., yaw=0.) if unchanged else None,
                    missing_motion_integrated=False)
                proof["position_verified"] = unchanged
                if not unchanged:
                    proof["parking_position_error"] = "Encoder changed since software origin or STOP evidence incomplete; no-ARM final pose unverified"
            result["position_verified"] = result["final_stop_chain"]["position_verified"]
            if result["motion_complete"]:
                if result["position_verified"]:
                    distance = math.hypot(controller.goal_x-latest["x"], controller.goal_y-latest["y"])
                    result["final_distance_m"] = distance
                    if distance > cfg.arrival_tolerance_m:
                        fail(BaseFault("Final measured stopped pose is outside arrival tolerance"), phase)
                        result["motion_complete"] = False
                else:
                    fail(BaseFault(result["final_stop_chain"]["parking_position_error"] or
                        "Final parking position not verified"), phase)
                    result["motion_complete"] = False
        if result["event_sink_errors"]:
            fail(BaseFault("RAM event callback failed: "+result["event_sink_errors"][0]), phase)
        result.update(rows=rows, raw_events=port.events, command_intents=intents,
            query_counts=dict(port.attempts), complete_write_counts=dict(port.complete_writes),
            elapsed_s=clock()-began, latest_odometry=latest, arm_count=port.arm_count)
        result.update(history=history.summary(),controller_updates=control_updates,control_metrics=control_metrics)
        result['history_timebase'] = dict(method='origin_first_source_plus_MCU_speed_tick_modular_delta',
            freshness_clock='unmodified_source_monotonic_s',anchors=measurement_anchors,
            interpolation_used_by_controller=False,missing_motion_recovered=False)
        result['odometry_continuity_validated'] = bool(result['origin'] and odometry_failure is None
            and not (result.get('final_stop_chain') or {}).get('parking_position_error'))
        result["session_complete"] = bool(not result["first_failure"] and result["origin"]
            and (not execute or result["motion_complete"] and result["final_stop_chain"]["complete"]))
    return result
