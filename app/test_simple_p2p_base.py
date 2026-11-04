"""Real frozen v9 Client/core against an in-memory MCU; no device imports."""
import importlib.util
import math
from pathlib import Path
import struct
import threading
import unittest
from unittest.mock import patch

import simple_p2p_base as base
from simple_p2p import Config, Command


def load(name):
    path = Path(__file__).resolve().parent / "deps" / (name+".py")
    spec = importlib.util.spec_from_file_location("simple_test_"+name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


api, core = load("control_client"), load("core")


class Clock:
    def __init__(self): self.value = 100.
    def monotonic(self): return self.value
    def sleep(self, seconds): self.value += seconds


class MCU:
    def __init__(self, clock, mode="healthy", delay=0., sample_period_ms=50):
        self.clock, self.mode, self.delay = clock, mode, delay
        self.sample_period_ms = sample_period_ms
        self.timeout = self.write_timeout = .1
        self.writes, self.queue, self.buffer = [], [], bytearray()
        self.state, self.session, self.target, self.rx, self.applied = 1, 0, (0, 0), 0, 0
        self.arm_at, self.stop_tick, self.stop_count = None, None, 0
        self.first_old = False
        self.speed_sample, self.sampled_speed = None, (0, 0)
        self.last_stopped_status_tick = None
        self.applied_target = (0,0)
        self.parking_sequence, self.parking_sources, self.parking_speed = None,0,(0,0)
        self.query_overlap_count = 0

    def write(self, raw):
        if raw[3] != 3 and self.queue:
            self.query_overlap_count += 1
        self.writes.append((self.clock.value, bytes(raw)))
        kind, sequence, left, right, session, challenge = struct.unpack_from("<BHhhII", raw, 3)
        at = self.clock.value
        tick = int(at*1000)
        if kind == 3:
            self.stop_count += 1
            self.state, self.session, self.target = 1, 0, (0, 0)
            self.stop_tick = tick
            self.first_old = self.stop_count == 1 and self.mode in ("old_parking", "new_parking_nonzero")
        elif kind == 6:
            self.state, self.session, self.rx, self.arm_at = 2, session, sequence, at
        elif kind == 7:
            self.state, self.target, self.rx = 5, (left, right), sequence
            if self.mode not in ("not_applied","ack_progress"):
                self.applied, self.applied_target = sequence,(left,right)
        if self.state == 2 and at-self.arm_at >= (.8 if self.mode == "slow_arm" else .09):
            self.state = 10
            self.applied,self.applied_target = self.rx,(0,0)
        if kind == 5:
            sample_tick = int(tick/self.sample_period_ms)*self.sample_period_ms
            sample_seq = sample_tick//self.sample_period_ms
            if self.mode == "gap" and self.arm_at is not None and self.target != (0, 0):
                sample_seq += 2
            if self.mode in ("parking_gap","settle_with_gap") and self.stop_count == 2:
                sample_seq += 2
            if self.mode == "frozen": sample_tick, sample_seq = 100000, 2000
            age = 251 if self.mode == "old_can" else 0
            l, r = self.target
            if self.speed_sample != sample_tick:
                self.speed_sample, self.sampled_speed = sample_tick, (l, r)
            l, r = self.sampled_speed
            if self.mode == "origin_nonzero" and self.arm_at is None: l = 1
            if self.first_old and (self.mode != "new_parking_nonzero" or tick > self.last_stopped_status_tick):
                sample_tick = self.stop_tick-10 if self.mode == "old_parking" else self.last_stopped_status_tick+1
                sample_seq, l, r = sample_tick//self.sample_period_ms, -.5, 0
                self.first_old = False
            if (self.stop_count == 2 and self.mode in ('settle_then_zero','zero_interrupted','continuous_moving','settle_with_gap')
                    and self.last_stopped_status_tick is not None and sample_tick > self.last_stopped_status_tick):
                if sample_seq != self.parking_sequence:
                    self.parking_sequence,self.parking_sources = sample_seq,self.parking_sources+1
                    moving = (self.mode == 'continuous_moving' or
                        self.mode in ('settle_then_zero','settle_with_gap') and self.parking_sources <= 3 or
                        self.mode == 'zero_interrupted' and self.parking_sources == 3)
                    self.parking_speed = (.4,0.) if moving else (0.,0.)
                l,r = self.parking_speed
            raw_speed = (int(round(l*10)) & 65535) | ((int(round(r*10)) & 65535)<<16)
            flags = (self.state<<8) | (5 if self.state == 1 else 1)
            enabled = 0 if self.state == 1 else 0x70007
            if self.mode == 'parking_lost_state' and self.stop_count == 2 and sample_tick > self.last_stopped_status_tick:
                flags,enabled=(5<<8)|1,0x70007
            words = [tick, sample_seq, sample_tick-age, 31, 0, 0, raw_speed,
                     sample_tick-age, sample_tick-age, sample_tick-age, enabled, 0, flags]
            if self.mode == 'source_rewinds' and sample_seq % 2 == 0:
                words[7] -= 150
                words[8] -= 150
            if self.mode == 'arm_source_rewinds' and self.arm_at is not None:
                words[7] -= 200
                words[8] -= 200
            if self.mode == "encoder_changed" and self.stop_count:
                words[4] = 1
        else:
            if kind == 2 and self.state == 1 and self.stop_count:
                self.last_stopped_status_tick = tick
            target_pack = (self.target[0] & 65535) | ((self.target[1] & 65535)<<16)
            applied = self.applied
            if self.mode == "not_applied": target_pack = 0
            if self.mode == 'ack_progress' and kind == 7:
                target_pack = (self.applied_target[0]&65535)|((self.applied_target[1]&65535)<<16)
            flags = (self.state<<8) | (5 if self.state == 1 else 1)
            words = [tick, self.state, 0, self.session, self.rx, applied, target_pack,
                     int(self.state == 1), flags, api.IDENTITY, 0, 54321, 0]
        body = struct.pack("<BBBBHBB13IH", 0xaa, 0x55, 9, kind|0x80, sequence, 0, 0, *words, 0)
        reply = body+struct.pack("<H", api.crc16(body))
        if self.mode == 'ack_progress' and kind == 7:
            self.applied,self.applied_target=sequence,(left,right)
        if self.mode == "bad_crc" and kind == 5: reply = reply[:-1]+bytes([reply[-1]^1])
        if self.mode == "short" and kind == 5: return 10
        # A healthy stopped STATUS requires a *new* disabled-driver CAN sample.
        # Delay STOP ACK to that sample boundary instead of changing frozen CAN
        # status_raw from 7 to 0 under the same sample_sequence.
        ready_at = at+(self.delay(kind) if callable(self.delay) else self.delay)
        if kind == 3:
            period=self.sample_period_ms/1000.
            ready_at = max(ready_at,(math.floor(at/period)+1)*period+.002)
        self.queue.append((ready_at, reply))
        return 20

    def read(self, count):
        ready = [entry for entry in self.queue if entry[0] <= self.clock.value]
        self.queue = [entry for entry in self.queue if entry[0] > self.clock.value]
        for _, raw in ready: self.buffer.extend(raw)
        raw = bytes(self.buffer[:count]); del self.buffer[:count]
        return raw


class Script:
    config = Config()
    goal_x = goal_y = 0.
    def __init__(self, clock, phases=("drive",)):
        self.clock, self.phases, self.started = clock, phases, None
    def tick(self, pose, now):
        if self.started is None: self.started = now
        elapsed = now-self.started
        index = int(elapsed/.65)
        if index >= len(self.phases): return Command("stop", 0., 0., "arrived", 0., 0.)
        phase = self.phases[index]
        return Command(phase, .03 if phase == "drive" else 0.,
                       math.radians(6.9) if phase == "turn" else 0., "moving", .5, .4)


class BaseTests(unittest.TestCase):
    def run_case(self, mode="healthy", execute=False, phases=("drive",), scan=None, cancelled=None, delay=0., goal_x=0.,
                 expected=None,event_sink=None,sample_period_ms=50,controller_factory=None,cancel_pending=False):
        clock = Clock(); port = MCU(clock, mode, delay,sample_period_ms)
        controller = Script(clock, phases) if controller_factory is None else controller_factory(clock)
        controller.goal_x = goal_x
        scan = scan or (lambda now, phase: dict(ok=True, reason="clear", source_monotonic_s=now-.01))
        if cancel_pending:
            cancelled=lambda:any(raw[3]==7 for _,raw in port.writes) and bool(port.queue)
        with patch.object(api.time, "monotonic", clock.monotonic), patch.object(api.time, "sleep", clock.sleep):
            result = base.run_session(port, api, core, controller, scan_provider=scan,
                boot_id="this-boot", execute=execute, cancelled=cancelled or (lambda:False),
                clock=clock.monotonic, sleep=clock.sleep, wall_clock=lambda:1000.+clock.value,
                expected_start_encoders=expected,event_sink=event_sink)
        return result, port

    def test_origin_only_queries_no_scan_and_first_true_sample_zero(self):
        def forbidden(*args): raise AssertionError("Default must not request scan")
        result, port = self.run_case(scan=forbidden)
        self.assertTrue(result["session_complete"], result["first_failure"])
        self.assertEqual([raw[3] for _, raw in port.writes], [2, 5])
        self.assertEqual(result["origin"]["pose"], dict(x=0., y=0., yaw=0.))
        self.assertFalse(result["origin"]["MCU_encoder_reset"])
        self.assertEqual(result["arm_count"], 0)
        self.assertIsNone(result["final_stop_chain"])
        self.assertEqual(len(result["raw_events"]), 4)

    def test_origin_nonzero_fault_can_crc_or_short_never_controls(self):
        for mode in ("origin_nonzero", "old_can", "bad_crc", "short"):
            with self.subTest(mode=mode):
                result, port = self.run_case(mode)
                self.assertFalse(result["session_complete"])
                self.assertIsNotNone(result["first_failure"])
                self.assertTrue(all(raw[3] in (2, 5) for _, raw in port.writes))
                self.assertFalse(result['odometry_continuity_validated'])

    def test_quantization_exact_below_at_above_and_selected_caps(self):
        cfg = Config()
        for phase, unit in (("drive", math.pi*.2/60.), ("turn", 2*math.pi*.2/(60*.3))):
            for factor, expected in ((1-1e-13, 0), (1., 1), (1+1e-13, 1)):
                command = Command(phase, unit*factor if phase == "drive" else 0.,
                                  unit*factor if phase == "turn" else 0., "test", 1., 0.)
                result = base.quantize(command, cfg)
                self.assertEqual(abs(result["left_rpm"]), expected)
                self.assertLessEqual(result["linear_m_s"], command.linear_m_s)
                self.assertLessEqual(abs(result["angular_rad_s"]), abs(command.angular_rad_s))
        straight = base.quantize(Command("drive", .03, 0., "test", 1., 0.), cfg)
        turn = base.quantize(Command("turn", 0., math.radians(6.9), "test", 1., 0.), cfg)
        self.assertEqual((straight["left_rpm"], straight["right_rpm"]), (2, -2))
        self.assertEqual((turn["left_rpm"], turn["right_rpm"]), (-1, -1))

    def test_execute_single_arm_measured_stop_bookends_and_20hz(self):
        result, port = self.run_case(execute=True)
        self.assertTrue(result["session_complete"], result["first_failure"])
        self.assertEqual(result["arm_count"], 1)
        for key in ("initial_stop_chain", "final_stop_chain"):
            proof = result[key]
            self.assertTrue(proof["complete"])
            self.assertEqual(len({p["sample_sequence"] for p in proof["zero_samples"]}), 5)
            self.assertTrue(all(max(abs(p["speed_left_rpm"]), abs(p["speed_right_rpm"])) <= .2 for p in proof["zero_samples"]))
        targets = [(at, struct.unpack_from("<hh", raw, 6)) for at, raw in port.writes if raw[3] == 7]
        self.assertTrue(targets)
        self.assertTrue(all(pair == (2, -2) for _, pair in targets))
        self.assertTrue(all(b[0]-a[0] >= .05-1e-10 for a,b in zip(targets,targets[1:])))
        self.assertLess(result["arm_handoff"]["after"]["position_left_raw"]+0., 1.)

    def test_axis_change_requires_zero_setpoints_and_five_new_actual_zeros(self):
        result, port = self.run_case(execute=True, phases=("turn", "drive"))
        self.assertTrue(result["session_complete"], result["first_failure"])
        targets = [row for row in result["rows"] if row["kind"] == 7]
        first_drive = next(i for i,row in enumerate(targets) if row["quantized_command"]["left_rpm"] > 0)
        self.assertTrue(any(row["quantized_command"]["left_rpm"] < 0 for row in targets[:first_drive]))
        zeros = [row for row in targets[:first_drive] if row["quantized_command"]["left_rpm"] == 0]
        self.assertGreaterEqual(len(zeros), 1)  # Ten-Hz zero keepalive; five real zero FB remain required.
        between = [row for row in result["rows"] if row["kind"] == 5
            and zeros[0]["completed_monotonic_s"] < row["request_start_monotonic_s"] < targets[first_drive]["request_start_monotonic_s"]
            and row.get("distinct") and row["packet"]["speed_left_rpm"] == 0]
        self.assertGreaterEqual(len(between), 5)
        self.assertEqual(result["arm_count"], 1)

    def test_old_pre_status_nonzero_is_excluded_and_new_nonzero_restarts_settling(self):
        result, _ = self.run_case("old_parking", execute=True)
        self.assertTrue(result["session_complete"], result["first_failure"])
        excluded = [row for row in result["rows"] if row.get("excluded_reason", "").startswith("CAN source")]
        self.assertGreaterEqual(len(excluded), 1)
        self.assertTrue(all("source_monotonic_s" not in row for row in excluded))
        result, port = self.run_case("new_parking_nonzero", execute=True)
        self.assertTrue(result["session_complete"],result["first_failure"])
        self.assertEqual(result["initial_stop_chain"]["settling_nonzero_samples"],1)
        self.assertEqual(len(result["initial_stop_chain"]["zero_samples"]),5)

    def test_moving_gap_frozen_source_slow_arm_and_unapplied_target_stop_once_without_rearm(self):
        for mode, reason in (("gap", "discontinuity"),
                             ("slow_arm", "Host feedback source age"), ("not_applied", "not observed applied")):
            with self.subTest(mode=mode):
                result, port = self.run_case(mode, execute=True)
                self.assertFalse(result["session_complete"])
                self.assertIn(reason, result["first_failure"]["message"])
                self.assertLessEqual(result["arm_count"], 1)
                self.assertEqual(port.writes[-1][1][3], 5)
                self.assertTrue(any(raw[3] == 3 for _, raw in port.writes))

    def test_scan_rejection_in_turn_never_sends_nonzero_and_first_cause_survives_stop(self):
        seen = []
        def gate(now, phase):
            seen.append(phase)
            return dict(ok=phase != "turn", reason="turn clearance missing", source_monotonic_s=now)
        result, port = self.run_case(execute=True, phases=("turn",), scan=gate)
        self.assertIn("turn", seen)
        self.assertIn("turn clearance missing", result["first_failure"]["message"])
        self.assertEqual(sum(raw[3] == 7 for _, raw in port.writes), 0)
        self.assertEqual(result["arm_count"],0)
        self.assertTrue(result["final_stop_chain"]["complete"])

    def test_stopped_position_uses_parking_samples_and_rejects_gap_or_outside_tolerance(self):
        result, _ = self.run_case(execute=True)
        self.assertTrue(result["position_verified"])
        self.assertTrue(any("parking_odometry" in row for row in result["rows"]))
        self.assertEqual(result["latest_odometry"]["sample_sequence"],
                         result["final_stop_chain"]["zero_samples"][-1]["sample_sequence"])
        result, _ = self.run_case(execute=True, goal_x=1.)
        self.assertFalse(result["session_complete"])
        self.assertFalse(result["motion_complete"])
        self.assertIn("outside arrival tolerance",result["first_failure"]["message"])
        self.assertTrue(result["final_stop_chain"]["complete"])
        result, _ = self.run_case("parking_gap",execute=True)
        self.assertTrue(result["final_stop_chain"]["complete"])
        self.assertFalse(result["position_verified"])
        self.assertFalse(result["motion_complete"])
        self.assertIn("Parking MCU odometry discontinuity",result["first_failure"]["message"])

    def test_no_arm_arrived_boundary_has_final_encoder_bound_position_evidence(self):
        result, port = self.run_case(execute=True, phases=(), goal_x=.1)
        self.assertTrue(result["session_complete"],result["first_failure"])
        self.assertTrue(result["position_verified"])
        self.assertEqual(result["final_distance_m"],.1)
        self.assertEqual(result["arm_count"],0)
        self.assertFalse(any(raw[3] in (6,7) for _,raw in port.writes))
        evidence=result["position_verification"]
        self.assertEqual(evidence["method"],"encoder_unchanged_since_origin")
        self.assertTrue(evidence["all_observed_encoders_unchanged"])
        self.assertEqual(evidence["final_post_status_sample"],result["final_stop_chain"]["zero_samples"][-1])
        self.assertEqual(evidence["pose"],dict(x=0.,y=0.,yaw=0.))

    def test_no_arm_encoder_change_cannot_be_reported_as_arrived(self):
        result, port = self.run_case("encoder_changed",execute=True,phases=(),goal_x=.1)
        self.assertEqual(result["arm_count"],0)
        self.assertFalse(any(raw[3] in (6,7) for _,raw in port.writes))
        self.assertTrue(result["final_stop_chain"]["complete"])
        self.assertFalse(result["session_complete"])
        self.assertFalse(result["motion_complete"])
        self.assertFalse(result["position_verified"])
        self.assertFalse(result["encoder_unchanged_since_origin"])
        self.assertIsNotNone(result["first_origin_encoder_change"])
        self.assertIsNone(result["position_verification"]["pose"])
        self.assertIn("Encoder changed",result["first_failure"]["message"])

    def test_expected_encoder_mismatch_at_origin_or_during_preparation_never_arms(self):
        for mode,expected in (("healthy",dict(left_raw=1,right_raw=0)),
                              ("encoder_changed",dict(left_raw=0,right_raw=0))):
            with self.subTest(mode=mode):
                result,port=self.run_case(mode,execute=True,expected=expected)
                self.assertFalse(result['session_complete'])
                self.assertIn('encoder',result['first_failure']['message'])
                self.assertFalse(any(raw[3] in (6,7) for _,raw in port.writes))
                self.assertTrue(result['final_stop_chain']['complete'])
        result,_=self.run_case(execute=True,expected=dict(left_raw=0,right_raw=0))
        self.assertTrue(result['session_complete'],result['first_failure'])
        self.assertEqual(result['expected_start_verification']['packet']['position_left_raw'],0)
        self.assertTrue(result['expected_start_verification']['distinct'])

    def test_ram_events_keep_original_feedback_source_and_callback_fault_still_stops(self):
        events=[]
        result,_=self.run_case(execute=True,event_sink=events.append)
        self.assertTrue(result['session_complete'],result['first_failure'])
        self.assertEqual(sum(event['event']=='origin' for event in events),1)
        for event in (event for event in events if event['source_monotonic_s'] is not None):
            self.assertLessEqual(event['source_monotonic_s'],event['event_monotonic_s'])
            self.assertIsNotNone(event['raw_packet'])
        def broken(event): raise ValueError('RAM callback fault')
        result,port=self.run_case(execute=True,event_sink=broken)
        self.assertFalse(result['session_complete'])
        self.assertIn('RAM event callback failed',result['first_failure']['message'])
        self.assertTrue(result['final_stop_chain']['complete'])
        self.assertFalse(any(raw[3] in (6,7) for _,raw in port.writes))

    def test_wire_forbids_unknown_bad_crc_second_arm_other_thread_and_stale_physical_target(self):
        clock = Clock(); physical = MCU(clock); wire = base.SessionPort(physical, True, clock.monotonic)
        for raw in (api.request(1, 1), api.request(8, 1), bytes(20), api.request(7, 1, 1, -1, session=8)):
            with self.assertRaises(base.BaseFault): wire.write(raw)
        wire.phase = "arming"
        wire.write(api.request(6, 1, session=8, challenge=9))
        with self.assertRaises(base.BaseFault): wire.write(api.request(6, 2, session=8, challenge=9))
        wire.phase = "active"
        for key, age in (("command_source", .25), ("feedback_source", .75), ("scan_source", .75)):
            wire.permit = dict(target=(1,-1,8), command_source=clock.value,
                feedback_source=clock.value, scan_source=clock.value, lease_deadline=clock.value+1)
            wire.permit[key] -= age
            with self.assertRaises(base.BaseFault): wire.write(api.request(7, 3, 1,-1,session=8))
        errors=[]
        def other():
            try: wire.write(api.request(2, 4))
            except Exception as exc: errors.append(exc)
        thread=threading.Thread(target=other); thread.start(); thread.join()
        self.assertIn("PID/thread", str(errors[0]))
        self.assertEqual(len(physical.writes), 1)

    def test_source_duplicate_and_late_new_sample_cannot_renew_original_750ms(self):
        tracker = base.SourceTracker(.75)
        packet = dict(sample_sequence=1, tick_ms=1000, sample_tick_ms=1000, left_tick_ms=1000,
            right_tick_ms=1000, speed_tick_ms=1000, position_left_raw=0, position_right_raw=0,
            speed_raw=0, status_raw=0x70007, fault_raw=0, flags=0xa01, valid_mask=31)
        self.assertEqual(tracker.accept(packet, 100.,100.)["source_monotonic_s"],100.)
        duplicate = tracker.accept(dict(packet, flags=0x501), 100.5,100.5)
        self.assertFalse(duplicate["distinct"])
        self.assertEqual(duplicate["source_monotonic_s"],100.)
        with self.assertRaisesRegex(Exception,"Host feedback source age"):
            tracker.accept(dict(packet,sample_sequence=2),100.75,100.75)

    def test_ack_applied_progress_is_evidence_during_status_gap_not_acceptance_alone(self):
        packet=dict(identity=api.IDENTITY,session=9,state=10,error=0,reserved=0,status=0,
            uart_critical_short=0,uart_critical_total=0,last_applied_seq=39,target_pack=0)
        tracker=base.AppliedTargets(packet)
        previous=39
        for index in range(12):
            sequence=53+index*3
            at=100.+index*.05
            tracker.register(sequence,2,-2,at)
            # A real ACK carries the prior target applied, never the new
            # accepted seq. There are deliberately no STATUS observations.
            tracker.observe(dict(packet,state=5,last_applied_seq=previous,
                target_pack=0 if previous==39 else 0xfffe0002),at)
            tracker.check_deadline(at+.049)
            previous=sequence
        self.assertEqual(len(tracker.targets),2)  # Current applied plus one pending.
        self.assertEqual(tracker.applied_order,10)

    def test_immediate_apply_and_unknown_pack_identity_or_regression_are_checked(self):
        packet=dict(identity=api.IDENTITY,session=9,state=10,error=0,reserved=0,status=0,
            uart_critical_short=0,uart_critical_total=0,last_applied_seq=39,target_pack=0)
        tracker=base.AppliedTargets(packet)
        tracker.register(53,2,-2,100.)
        current=dict(packet,state=5,last_applied_seq=53,target_pack=0xfffe0002)
        tracker.observe(current,100.)  # Register-before-observe permits immediate application.
        self.assertEqual(tracker.applied_order,0)
        for change in (dict(last_applied_seq=999),dict(target_pack=0),dict(last_applied_seq=39,target_pack=0),
                       dict(identity=1),dict(session=10)):
            with self.subTest(change=change),self.assertRaises(base.BaseFault):
                tracker.observe(dict(current,**change),100.1)

    def test_true_unapplied_or_late_application_still_fails_at_original_300ms(self):
        packet=dict(identity=api.IDENTITY,session=9,state=10,error=0,reserved=0,status=0,
            uart_critical_short=0,uart_critical_total=0,last_applied_seq=39,target_pack=0)
        tracker=base.AppliedTargets(packet);tracker.register(53,2,-2,100.)
        tracker.observe(packet,100.299)
        with self.assertRaisesRegex(base.BaseFault,'within 300ms'):
            tracker.observe(dict(packet,last_applied_seq=53,target_pack=0xfffe0002),100.301)
        with self.assertRaisesRegex(base.BaseFault,'within 300ms'):
            tracker.check_deadline(100.3)

    def test_twenty_ms_rtt_does_not_starve_status_or_ignore_ack_application(self):
        result,_=self.run_case('ack_progress',execute=True,delay=.021,sample_period_ms=100)
        self.assertEqual(result.get('termination_reason'),'arrived',result['first_failure'])
        active=[r for r in result['rows'] if r['phase'] in ('drive','turn','motion')]
        statuses=[r for r in active if r['kind']==2]
        feedback=[r for r in active if r['kind']==5]
        self.assertGreaterEqual(len(statuses),2)
        self.assertGreaterEqual(len(feedback),3)
        self.assertLess(max(b['request_start_monotonic_s']-a['request_start_monotonic_s']
                            for a,b in zip(statuses,statuses[1:])),.26)
        self.assertTrue(any(r['kind']==7 and r['packet']['last_applied_seq']!=r['packet']['sequence'] for r in active))
        if result['first_failure'] is not None:
            self.assertIn('Parking MCU odometry discontinuity',result['first_failure']['message'])

    def test_parking_nonzero_then_five_zero_and_interrupted_zero_run(self):
        for mode,nonzero,restarts in (('settle_then_zero',3,0),('zero_interrupted',1,1)):
            with self.subTest(mode=mode):
                result,_=self.run_case(mode,execute=True)
                proof=result['final_stop_chain']
                self.assertTrue(proof['complete'],proof.get('error'))
                self.assertEqual(proof['settling_nonzero_samples'],nonzero)
                self.assertEqual(proof['zero_run_restarts'],restarts)
                self.assertEqual(len({p['sample_sequence'] for p in proof['zero_samples']}),5)
                final_source=next(r for r in reversed(result['rows']) if r['kind']==5)
                self.assertLess(final_source['completed_monotonic_s']-proof['stopped_status']['completed_monotonic_s'],2.)

    def test_parking_continuous_motion_uses_original_two_seconds_but_state_loss_fails(self):
        result,_=self.run_case('continuous_moving',execute=True)
        proof=result['final_stop_chain']
        self.assertFalse(proof['complete'])
        self.assertIn('zero samples timed out',proof['error'])
        final_row=result['rows'][-1]
        self.assertGreaterEqual(final_row['completed_monotonic_s']-proof['stopped_status']['completed_monotonic_s'],1.99)
        self.assertLess(final_row['completed_monotonic_s']-proof['stopped_status']['completed_monotonic_s'],2.02)
        result,_=self.run_case('parking_lost_state',execute=True)
        self.assertFalse(result['final_stop_chain']['complete'])
        self.assertIn('lost DISARMED',result['final_stop_chain']['error'])

    def test_parking_gap_remains_position_failure_while_settled_zero_proof_completes(self):
        result,_=self.run_case('settle_with_gap',execute=True)
        proof=result['final_stop_chain']
        self.assertTrue(proof['complete'],proof.get('error'))
        self.assertEqual(len(proof['zero_samples']),5)
        self.assertFalse(proof['position_verified'])
        self.assertIn('Parking MCU odometry discontinuity',proof['parking_position_error'])
        self.assertFalse(result['session_complete'])
        self.assertFalse(result['motion_complete'])

    def test_control_reads_latest_accumulated_pose_while_serial_transaction_is_inflight(self):
        result,port=self.run_case('ack_progress',execute=True,delay=.021,sample_period_ms=100,
                                  phases=('drive','drive','drive'))
        self.assertEqual(result.get('termination_reason'),'arrived',result['first_failure'])
        active=[r for r in result['rows'] if r['phase'] in ('motion','turn','drive')]
        acquired=[r for r in active if r['kind']==5 and r.get('distinct')]
        self.assertTrue(all('odometry' in r for r in acquired))
        self.assertGreater(len(acquired),10)
        self.assertGreater(acquired[-1]['odometry']['x'],acquired[0]['odometry']['x'])
        for tick in result['controller_updates']:
            available=[r for r in acquired if r['completed_monotonic_s'] <= tick['monotonic_s']]
            if available:
                latest=available[-1]['odometry']
                self.assertEqual(tick['sample_sequence'],latest['sample_sequence'])
                self.assertEqual((tick['x'],tick['y'],tick['yaw']),(latest['x'],latest['y'],latest['yaw']))
        metrics=result['control_metrics']
        self.assertGreater(metrics['control_ticks_while_inflight'],0)
        self.assertEqual(metrics['max_inflight_transactions'],1)
        self.assertEqual(port.query_overlap_count,0)
        self.assertGreater(metrics['control_ticks'],len(acquired))
        starts=[r['request_start_monotonic_s'] for r in active if r['kind']==7]
        self.assertTrue(all(b-a>=.1-1e-10 for a,b in zip(starts,starts[1:])))

    def test_mcu_measurement_axis_does_not_rewrite_a_receding_conservative_source(self):
        result,_=self.run_case('source_rewinds',execute=True,sample_period_ms=100,
                               phases=('drive','drive'))
        self.assertEqual(result.get('termination_reason'),'arrived',result['first_failure'])
        acquired=[r['odometry'] for r in result['rows'] if r['phase'] in ('motion','drive','turn')
                  and r['kind']==5 and 'odometry' in r]
        self.assertTrue(any(b['source_monotonic_s']<a['source_monotonic_s'] for a,b in zip(acquired,acquired[1:])))
        for a,b in zip(acquired,acquired[1:]):
            self.assertAlmostEqual(b['measurement_monotonic_s']-a['measurement_monotonic_s'],
                ((b['mcu_speed_tick_ms']-a['mcu_speed_tick_ms'])&0xffffffff)/1000.)
        self.assertTrue(all(t['source_monotonic_s']<=t['monotonic_s'] for t in result['controller_updates']))

    def test_ack_uses_sent_snapshot_even_when_new_control_intent_arrives_before_reply(self):
        class Changing(Script):
            def __init__(self,clock):
                super().__init__(clock);self.calls=0
            def tick(self,pose,now):
                command=super().tick(pose,now);self.calls+=1
                if command.phase=='stop':return command
                return Command('drive',.03 if self.calls%2 else math.pi*.2/60.,0.,'changing',.5,0.)
        result,port=self.run_case(execute=True,delay=.06,sample_period_ms=100,controller_factory=Changing)
        commands=[r for r in result['rows'] if r['kind']==7]
        self.assertTrue(commands)
        self.assertGreater(result['control_metrics']['control_ticks_while_inflight'],0)
        for row in commands:
            physical=struct.unpack_from('<hh',bytes.fromhex(row['request_hex']),6)
            self.assertEqual(physical,(row['quantized_command']['left_rpm'],row['quantized_command']['right_rpm']))
            self.assertLessEqual(row['control_tick_monotonic_s'],row['request_start_monotonic_s'])
        self.assertNotIn('Applied command/target identity',str(result['first_failure']))
        self.assertEqual(port.query_overlap_count,0)

    def test_real_gap_invalidates_history_and_is_not_reanchored_during_stop(self):
        result,_=self.run_case('gap',execute=True,sample_period_ms=100)
        self.assertIn('discontinuity',result['first_failure']['message'])
        self.assertEqual(result['history']['count'],0)
        self.assertFalse(result['position_verified'])
        self.assertTrue(result['final_stop_chain']['complete'])
        self.assertFalse(result['session_complete'])

    def test_cancel_during_inflight_target_sends_stop_without_waiting_for_ack(self):
        result,port=self.run_case(execute=True,delay=.06,sample_period_ms=100,cancel_pending=True)
        self.assertIn('cancelled',result['first_failure']['message'])
        target_at=next(at for at,raw in port.writes if raw[3]==7)
        stop_at=next(at for at,raw in port.writes if raw[3]==3 and at>target_at)
        self.assertLess(stop_at-target_at,.01)
        self.assertEqual(result['arm_count'],1)
        self.assertFalse(result['motion_complete'])

    def test_normal_arrival_drains_only_original_transaction_before_stop(self):
        result,port=self.run_case('ack_progress',execute=True,delay=.021,sample_period_ms=100,
                                  phases=('drive','drive'))
        quiesce=result['normal_quiesce'];pending=quiesce['pending']
        self.assertIsNotNone(pending)
        self.assertTrue(quiesce['complete'])
        self.assertEqual(quiesce['queries_started'],0)
        self.assertLess(quiesce['completed_monotonic_s'],pending['deadline_monotonic_s'])
        after=[(at,raw) for at,raw in port.writes if at>=quiesce['started_monotonic_s']]
        self.assertEqual(after[0][1][3],3)
        self.assertGreaterEqual(after[0][0],quiesce['completed_monotonic_s'])

    def test_true_source_expiration_does_not_receive_a_new_history_timestamp(self):
        def slow_control(clock):
            class Paused(Script):
                def tick(self,pose,now):
                    command=super().tick(pose,now)
                    if self.started is not None and now-self.started>.15:
                        clock.sleep(.75)
                    return command
            return Paused(clock)
        result,_=self.run_case(execute=True,sample_period_ms=100,controller_factory=slow_control)
        self.assertFalse(result['session_complete'])
        self.assertIn('Host feedback source age',result['first_failure']['message'])
        self.assertTrue(all(tick['source_monotonic_s']<=tick['monotonic_s'] for tick in result['controller_updates']))
        self.assertFalse(result['history_timebase']['missing_motion_recovered'])

    def test_first_target_axis_change_while_ack_inflight_still_requires_five_zero_barrier(self):
        for first,second in (('drive','turn'),('turn','drive')):
            phases=[]
            def gate(now,phase):
                phases.append((now,phase))
                return dict(ok=True,reason='clear',source_monotonic_s=now)
            class Changing(Script):
                def __init__(self,clock):
                    super().__init__(clock);self.calls=0
                def tick(self,pose,now):
                    self.calls+=1
                    if self.calls>30:return Command('stop',0.,0.,'arrived',0.,0.)
                    axis=first if self.calls<=2 else second
                    return Command(axis,.03 if axis=='drive' else 0.,
                        math.radians(6.9) if axis=='turn' else 0.,'changing',1.,.5)
            with self.subTest(first=first,second=second):
                result,port=self.run_case(execute=True,delay=lambda kind:.06 if kind==7 else 0.,
                    sample_period_ms=100,controller_factory=Changing,scan=gate)
                targets=[r for r in result['rows'] if r['kind']==7]
                def axis(row):
                    q=row['quantized_command'];l,r=q['left_rpm'],q['right_rpm']
                    return None if not(l or r) else 'drive' if l==-r else 'turn'
                switched=next(r for r in targets if axis(r)==second)
                zeros=[r for r in targets if axis(r) is None and r['request_start_monotonic_s']<switched['request_start_monotonic_s']]
                self.assertTrue(zeros)
                zero=zeros[0];tick=zero['packet']['tick_ms']
                samples=[r for r in result['rows'] if r['kind']==5 and r.get('distinct')
                    and zero['completed_monotonic_s']<r['completed_monotonic_s']<=switched['request_start_monotonic_s']
                    and all(0<(r['packet'][k]-tick)&0xffffffff<0x80000000
                            for k in ('sample_tick_ms','left_tick_ms','right_tick_ms','speed_tick_ms'))
                    and max(abs(r['packet']['speed_left_rpm']),abs(r['packet']['speed_right_rpm']))<=.2]
                self.assertGreaterEqual(len(samples),5)
                self.assertEqual(port.query_overlap_count,0)
                if first=='turn':
                    first_send=targets[0]['request_start_monotonic_s']
                    self.assertTrue(all(phase=='turn' for at,phase in phases
                        if first_send<=at<samples[4]['completed_monotonic_s']))

    def test_arm_history_cut_preserves_mcu_axis_when_new_source_is_below_old_fence(self):
        result,port=self.run_case('arm_source_rewinds',execute=True,sample_period_ms=10)
        self.assertIsNotNone(result['arm_handoff'])
        self.assertTrue(any(raw[3]==7 for _,raw in port.writes))
        anchors=result['history_timebase']['anchors']
        self.assertEqual(len(anchors),2)
        self.assertLess(anchors[1]['source_monotonic_s'],anchors[0]['measurement_monotonic_s'])
        self.assertGreater(anchors[1]['measurement_monotonic_s'],anchors[0]['measurement_monotonic_s'])
        self.assertEqual(result['controller_updates'][0]['source_monotonic_s'],anchors[1]['source_monotonic_s'])
        self.assertNotIn('measurement time must strictly increase',str(result['first_failure']))
        self.assertEqual(result['history_timebase']['method'],'origin_first_source_plus_MCU_speed_tick_modular_delta')


if __name__ == "__main__": unittest.main()
