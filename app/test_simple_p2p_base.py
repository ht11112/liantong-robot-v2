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
    def __init__(self, clock, mode="healthy", delay=0.):
        self.clock, self.mode, self.delay = clock, mode, delay
        self.timeout = self.write_timeout = .1
        self.writes, self.queue, self.buffer = [], [], bytearray()
        self.state, self.session, self.target, self.rx, self.applied = 1, 0, (0, 0), 0, 0
        self.arm_at, self.stop_tick, self.stop_count = None, None, 0
        self.first_old = False
        self.speed_sample, self.sampled_speed = None, (0, 0)

    def write(self, raw):
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
            if self.mode != "not_applied": self.applied = sequence
        if self.state == 2 and at-self.arm_at >= (.8 if self.mode == "slow_arm" else .09):
            self.state = 10
        if kind == 5:
            sample_tick = int(tick/50)*50
            sample_seq = sample_tick//50
            if self.mode == "gap" and self.arm_at is not None and self.target != (0, 0):
                sample_seq += 2
            if self.mode == "parking_gap" and self.stop_count == 2:
                sample_seq += 2
            if self.mode == "frozen": sample_tick, sample_seq = 100000, 2000
            age = 251 if self.mode == "old_can" else 0
            l, r = self.target
            if self.speed_sample != sample_tick:
                self.speed_sample, self.sampled_speed = sample_tick, (l, r)
            l, r = self.sampled_speed
            if self.mode == "origin_nonzero" and self.arm_at is None: l = 1
            if self.first_old and (self.mode != "new_parking_nonzero" or tick > self.stop_tick):
                sample_tick = self.stop_tick-10 if self.mode == "old_parking" else self.stop_tick+1
                sample_seq, l, r = sample_tick//50, -.5, 0
                self.first_old = False
            raw_speed = (int(round(l*10)) & 65535) | ((int(round(r*10)) & 65535)<<16)
            flags = (self.state<<8) | (5 if self.state == 1 else 1)
            enabled = 0 if self.state == 1 else 0x70007
            words = [tick, sample_seq, sample_tick-age, 31, 0, 0, raw_speed,
                     sample_tick-age, sample_tick-age, sample_tick-age, enabled, 0, flags]
            if self.mode == "encoder_changed" and self.stop_count:
                words[4] = 1
        else:
            target_pack = (self.target[0] & 65535) | ((self.target[1] & 65535)<<16)
            applied = self.applied
            if self.mode == "not_applied": target_pack = 0
            flags = (self.state<<8) | (5 if self.state == 1 else 1)
            words = [tick, self.state, 0, self.session, self.rx, applied, target_pack,
                     int(self.state == 1), flags, api.IDENTITY, 0, 54321, 0]
        body = struct.pack("<BBBBHBB13IH", 0xaa, 0x55, 9, kind|0x80, sequence, 0, 0, *words, 0)
        reply = body+struct.pack("<H", api.crc16(body))
        if self.mode == "bad_crc" and kind == 5: reply = reply[:-1]+bytes([reply[-1]^1])
        if self.mode == "short" and kind == 5: return 10
        self.queue.append((at+self.delay, reply))
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
    def run_case(self, mode="healthy", execute=False, phases=("drive",), scan=None, cancelled=None, delay=0., goal_x=0.):
        clock = Clock(); port = MCU(clock, mode, delay)
        controller = Script(clock, phases)
        controller.goal_x = goal_x
        scan = scan or (lambda now, phase: dict(ok=True, reason="clear", source_monotonic_s=now-.01))
        with patch.object(api.time, "monotonic", clock.monotonic), patch.object(api.time, "sleep", clock.sleep):
            result = base.run_session(port, api, core, controller, scan_provider=scan,
                boot_id="this-boot", execute=execute, cancelled=cancelled or (lambda:False),
                clock=clock.monotonic, sleep=clock.sleep, wall_clock=lambda:1000.+clock.value)
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
        self.assertGreaterEqual(len(zeros), 4)
        between = [row for row in result["rows"] if row["kind"] == 5
            and zeros[0]["completed_monotonic_s"] < row["request_start_monotonic_s"] < targets[first_drive]["request_start_monotonic_s"]
            and row.get("distinct") and row["packet"]["speed_left_rpm"] == 0]
        self.assertGreaterEqual(len(between), 5)
        self.assertEqual(result["arm_count"], 1)

    def test_old_pre_status_nonzero_is_excluded_but_new_nonzero_fails(self):
        result, _ = self.run_case("old_parking", execute=True)
        self.assertTrue(result["session_complete"], result["first_failure"])
        excluded = [row for row in result["rows"] if row.get("excluded_reason", "").startswith("CAN source")]
        self.assertGreaterEqual(len(excluded), 1)
        self.assertTrue(all("source_monotonic_s" not in row for row in excluded))
        result, port = self.run_case("new_parking_nonzero", execute=True)
        self.assertFalse(result["session_complete"])
        self.assertIn("not physically stopped", result["first_failure"]["message"])
        self.assertEqual(result["arm_count"], 0)

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


if __name__ == "__main__": unittest.main()
