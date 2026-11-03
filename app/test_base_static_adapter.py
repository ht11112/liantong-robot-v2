"""Real reviewed Client/odometry with memory-only serial responses; no devices."""
import importlib.util
import hashlib
from pathlib import Path
import struct
import types
import unittest
from unittest.mock import patch

import base_static_adapter as adapter


ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent.parent


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


DEPS = ROOT / "deps"
if DEPS.exists():
    dependency_paths = {name: DEPS / (name + ".py") for name in ("control_client", "stop_observation", "core")}
    if not all(path.is_file() for path in dependency_paths.values()):
        raise RuntimeError("Present deps directory must contain all three reviewed files; no partial fallback")
else:
    dependency_paths = dict(
        control_client=PROJECT / "candidates/v9_nonblocking_20261102/transport/src/control_client.py",
        stop_observation=PROJECT / "output/v9_async_log_20261102/query_probe_package/stop_observation.py",
        core=PROJECT / "navigation-demo/green_base_bridge/src/green_base_bridge/core.py")
api = load("p2p_static_real_client", dependency_paths["control_client"])
stop_api = load("p2p_static_real_stop", dependency_paths["stop_observation"])
core = load("p2p_static_real_core", dependency_paths["core"])


class Clock:
    def __init__(self):
        self.value = 100.

    def monotonic(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class Port:
    def __init__(self, clock, mode="healthy"):
        self.clock, self.mode = clock, mode
        self.timeout, self.write_timeout = .01, .1
        self.writes, self.buffer = [], bytearray()
        self.injected = False
        self.stop_count = 0

    def read(self, count):
        if self.mode == "fragmented":
            count = min(count, 17)
        value = bytes(self.buffer[:count])
        del self.buffer[:count]
        return value

    def write(self, raw):
        self.writes.append(bytes(raw))
        kind = raw[3]
        sequence = struct.unpack_from("<H", raw, 4)[0]
        if kind == 3:
            self.stop_count += 1
        tick = int(self.clock.value * 1000)
        if kind == 5:
            index = int(self.clock.value / .05)
            sample_tick = index * 50
            speed = 0
            if self.mode == "moving_once" and self.stop_count == 1 and self.clock.value > 100.35 and not self.injected:
                speed, self.injected = 3, True
            if self.mode == "residual":
                speed = 1
            if self.mode == "fault_once" and self.stop_count == 1 and self.clock.value > 100.35 and not self.injected:
                speed, self.injected = 0, True
            if self.mode == "short_once" and not self.injected:
                self.injected = True
                return 10
            if self.mode == "crc_once" and not self.injected:
                self.injected = True
                self.buffer.extend(bytes(64))
                return 20
            age = 251 if self.mode == "stale_can" else 0
            words = [tick, index, sample_tick - age, 31, 0, 0, speed,
                     sample_tick - age, sample_tick - age, sample_tick - age, 0, 0, 261]
            if self.mode == "fault_once" and self.injected and self.stop_count == 1:
                words[11] = 1
        else:
            identity = 1 if self.mode == "identity" else api.IDENTITY
            words = [tick, 1, 0, 0, sequence, sequence, 0, 1, 5, identity, 0, 123, 0]
        header = struct.pack("<BBBBHBB13IH", 0xaa, 0x55, 9, kind | 0x80,
                             sequence, 0, 0, *words, 0)
        self.buffer.extend(header + struct.pack("<H", api.crc16(header)))
        return 20


class DelayedHalfDuplexPort(Port):
    """A deterministic contention model, not a claim about the RK wire trace.

    A query response occupies the modeled bus until +10 ms and is delivered to
    the host at +13 ms.  Transmitting STOP during that interval loses its ACK.
    This exposes the consequences of cancelling only the local Client wait.
    """
    def __init__(self, clock):
        super().__init__(clock)
        self.ready = []
        self.busy_until = 0.
        self.collisions = []
        self.write_times = []
        self.drop_feedback_after = None

    def write(self, raw):
        at = self.clock.value
        self.write_times.append((raw[3], at))
        if raw[3] == 3 and at < self.busy_until:
            self.collisions.append(dict(stop_at=at, query_bus_busy_until=self.busy_until))
            self.writes.append(bytes(raw))
            self.stop_count += 1
            return 20
        buffered = self.buffer
        self.buffer = bytearray()
        written = super().write(raw)
        response = bytes(self.buffer)
        self.buffer = buffered
        if not (raw[3] == 5 and self.drop_feedback_after is not None and at >= self.drop_feedback_after):
            self.ready.append((at + .013, response))
        self.busy_until = at + .010
        return written

    def read(self, count):
        waiting = []
        for at, response in self.ready:
            if self.clock.value >= at:
                self.buffer.extend(response)
            else:
                waiting.append((at, response))
        self.ready = waiting
        return super().read(count)


class AdapterTests(unittest.TestCase):
    def run_memory(self, mode="healthy", duration=.3, **kwargs):
        clock = Clock()
        port = Port(clock, mode)
        with patch.object(api, "time", clock):
            result = adapter.run_static_session(
                port, api, stop_api, core.MeasuredOdometry, duration_s=duration,
                clock=clock.monotonic, sleep=clock.sleep, **kwargs)
        return result, port, clock

    def test_new_stop_chain_before_window_and_after_window(self):
        result, port, _ = self.run_memory()
        self.assertTrue(result["static_link_complete"], result)
        self.assertFalse(result["navigation_validated"])
        self.assertFalse(result["localization_validated"])
        self.assertFalse(result["ARM_SETPOINT_allowed"])
        self.assertEqual((port.timeout, port.write_timeout), (0, 0))
        self.assertEqual(port.stop_count, 2)
        self.assertEqual(set(frame[3] for frame in port.writes), {2, 3, 5})
        initial, final = result["initial_stop_chain"], result["final_stop_chain"]
        for proof in (initial, final):
            self.assertTrue(proof["complete"])
            self.assertEqual(proof["distinct_zero_feedback_samples"], 5)
            self.assertGreaterEqual(proof["stopped_status"]["request_start_monotonic_s"],
                                    proof["stop_ack"]["completed_monotonic_s"])
            self.assertGreater(proof["last_zero_sample"]["sample_sequence"],
                               proof["first_zero_sample"]["sample_sequence"])
        self.assertGreaterEqual(result["active_start_monotonic_s"], initial["last_zero_sample"]["completed_monotonic_s"])
        self.assertGreater(final["stop_ack"]["request_start_monotonic_s"], result["active_start_monotonic_s"])
        self.assertGreater(result["metrics"]["feedback_response_hz"], 20)

    def test_port_rejects_all_arm_setpoint_even_zero_and_crc_bad_frames(self):
        port = Port(Clock())
        fence = adapter.QueryOnlyPort(port)
        for kind in (2, 3, 5):
            self.assertEqual(fence.write(api.request(kind, 1)), 20)
        forbidden = [api.request(1, 1), api.request(8, 1),
                     api.request(6, 1, session=1, challenge=1),
                     api.request(7, 1, session=1),
                     api.request(7, 1, 1, -1, session=1),
                     api.request(2, 1) + api.request(3, 2),
                     api.request(2, 1)[:-1], bytes(20), bytearray(api.request(2, 1))]
        bad = bytearray(api.request(2, 1))
        bad[-1] ^= 1
        forbidden.append(bytes(bad))
        # Recompute valid CRC for a STATUS carrying otherwise-forbidden fields.
        bad = bytearray(api.request(2, 1))
        bad[10] = 1
        bad[18:] = struct.pack("<H", adapter.crc16(bad[:18]))
        forbidden.append(bytes(bad))
        for raw in forbidden:
            with self.subTest(raw=bytes(raw).hex()):
                with self.assertRaises(ValueError):
                    fence.write(raw)
                self.assertEqual(len(port.writes), 3)

    def test_short_write_is_not_retried_and_final_stop_is_attempted(self):
        result, port, _ = self.run_memory("short_once")
        self.assertFalse(result["static_link_complete"])
        self.assertIn("Short/unknown", result["first_failure"]["message"])
        self.assertEqual(port.stop_count, 2)
        self.assertTrue(result["final_stop_chain"]["complete"])
        self.assertEqual(result["counts"]["5"], result["complete_write_counts"]["5"] + 1)

    def test_bad_response_crc_never_becomes_a_valid_sample(self):
        result, port, _ = self.run_memory("crc_once")
        self.assertFalse(result["static_link_complete"])
        self.assertEqual(result["first_failure"]["error_type"], "TimeoutError")
        self.assertTrue(result["final_stop_chain"]["complete"])
        self.assertEqual(port.stop_count, 2)

    def test_nonzero_feedback_stops_before_750ms_budget(self):
        result, port, _ = self.run_memory("moving_once", duration=5.)
        self.assertFalse(result["static_link_complete"])
        self.assertIn("zero-speed tolerance", result["first_failure"]["message"])
        self.assertLess(result["failure_monotonic_s"] - result["active_start_monotonic_s"], .2)
        self.assertEqual(port.stop_count, 2)

    def test_measured_point_one_rpm_residual_retained_within_stop_tolerance(self):
        result, _, _ = self.run_memory("residual")
        self.assertTrue(result["static_link_complete"], result)
        self.assertEqual(result["static_zero_speed_rpm"], .2)
        self.assertEqual(result["initial_stop_chain"]["last_zero_sample"]["speed_left_rpm"], .1)
        self.assertEqual(result["final_stop_chain"]["last_zero_sample"]["speed_left_rpm"], .1)

    def test_driver_fault_stops_without_using_host_timeout_as_grace(self):
        result, port, _ = self.run_memory("fault_once", duration=5.)
        self.assertFalse(result["static_link_complete"])
        self.assertIn("fault", result["first_failure"]["message"])
        self.assertLess(result["failure_monotonic_s"] - result["active_start_monotonic_s"], .2)
        self.assertEqual(port.stop_count, 2)

    def test_fragmented_matching_response_is_assembled_by_original_client(self):
        result, _, _ = self.run_memory("fragmented")
        self.assertTrue(result["static_link_complete"], result)

    def test_existing_250ms_can_check_is_not_relaxed_by_host_750(self):
        result, port, _ = self.run_memory("stale_can", feedback_timeout_s=.75)
        self.assertFalse(result["static_link_complete"])
        self.assertIn("250ms", result["first_failure"]["message"])
        self.assertEqual(result["host_feedback_source_timeout_s"], .75)
        self.assertEqual(result["existing_CAN_source_limit_s"], .25)
        self.assertEqual(port.stop_count, 2)

    def test_wrong_mcu_identity_cannot_open_valid_window(self):
        result, port, _ = self.run_memory("identity")
        self.assertFalse(result["active_window_complete"])
        self.assertFalse(result["static_link_complete"])
        self.assertIn("identity", result["first_failure"]["message"])
        self.assertGreaterEqual(port.stop_count, 2)

    def test_cancel_stops_without_waiting_for_feedback_deadline(self):
        clock = Clock()
        port = Port(clock)
        with patch.object(api, "time", clock):
            result = adapter.run_static_session(
                port, api, stop_api, core.MeasuredOdometry, duration_s=10.,
                clock=clock.monotonic, sleep=clock.sleep,
                cancelled=lambda: clock.value >= 100.40)
        self.assertIn("cancelled", result["first_failure"]["message"])
        self.assertLess(result["failure_monotonic_s"], 100.402)
        self.assertTrue(result["final_stop_chain"]["complete"])

    def test_duplicate_sample_does_not_refresh_first_source_origin(self):
        tracker = adapter.FeedbackSourceTracker(.75)
        packet = dict(sample_sequence=1, tick_ms=1000, sample_tick_ms=1000,
                      left_tick_ms=1000, right_tick_ms=1000, speed_tick_ms=1000,
                      position_left_raw=0, position_right_raw=0, speed_raw=0,
                      status_raw=0, fault_raw=0, flags=261, valid_mask=31)
        first = tracker.accept(packet, 100., 100.)
        later = tracker.accept(packet, 100.7, 100.7)
        self.assertEqual(first["source_monotonic_s"], later["source_monotonic_s"])
        self.assertFalse(later["distinct"])
        with self.assertRaises(adapter.StaticAdapterFault):
            tracker.accept(packet, 100.751, 100.751)
        self.assertEqual(tracker.distinct_samples, 1)

    def test_deadline_equality_stops_and_late_new_sample_cannot_refresh(self):
        tracker = adapter.FeedbackSourceTracker(.75)
        packet = dict(sample_sequence=1, tick_ms=1000, sample_tick_ms=1000,
                      left_tick_ms=1000, right_tick_ms=1000, speed_tick_ms=1000,
                      position_left_raw=0, position_right_raw=0, speed_raw=0,
                      status_raw=0, fault_raw=0, flags=261, valid_mask=31)
        tracker.accept(packet, 100., 100.)
        with self.assertRaises(adapter.StaticAdapterFault):
            tracker.check_age(100.75)
        fresh_new = dict(packet, sample_sequence=2)
        with self.assertRaises(adapter.StaticAdapterFault):
            tracker.accept(fresh_new, 100.75, 100.75)
        self.assertEqual(tracker.sequence, 1)

    def test_slow_stop_cleanup_starts_fresh_final_collection_without_clearing_failure(self):
        clock = Clock()
        port = Port(clock, "moving_once")
        original_verify = stop_api.verify_stop
        calls = []

        def slow_final(*args, **kwargs):
            calls.append(1)
            result = original_verify(*args, **kwargs)
            if len(calls) == 2:
                clock.sleep(.8)
            return result

        with patch.object(api, "time", clock), patch.object(stop_api, "verify_stop", slow_final):
            result = adapter.run_static_session(
                port, api, stop_api, core.MeasuredOdometry, duration_s=5.,
                clock=clock.monotonic, sleep=clock.sleep)
        self.assertIsNotNone(result["first_failure"])
        self.assertFalse(result["static_link_complete"])
        self.assertTrue(result["final_stop_chain"]["complete"], result)

    def test_duplicate_mutation_and_backwards_sequence_rejected(self):
        tracker = adapter.FeedbackSourceTracker()
        packet = dict(sample_sequence=5, tick_ms=1000, sample_tick_ms=1000,
                      left_tick_ms=1000, right_tick_ms=1000, speed_tick_ms=1000,
                      position_left_raw=0, position_right_raw=0, speed_raw=0,
                      status_raw=0, fault_raw=0, flags=261, valid_mask=31)
        tracker.accept(packet, 100., 100.)
        with self.assertRaises(adapter.StaticAdapterFault):
            tracker.accept(dict(packet, speed_raw=1), 100.1, 100.1)
        with self.assertRaises(adapter.StaticAdapterFault):
            tracker.accept(dict(packet, sample_sequence=4), 100.1, 100.1)

    def test_ram_log_bounded_without_losing_aggregate_counts(self):
        result, _, _ = self.run_memory(duration=1., row_limit=16)
        self.assertTrue(result["static_link_complete"])
        self.assertEqual(len(result["rows"]), 16)
        self.assertGreater(result["total_rows"], 16)
        self.assertGreater(result["dropped_old_rows"], 0)
        self.assertGreater(result["metrics"]["feedback_responses"], 16)

    def test_optional_sink_only_receives_real_new_measured_odometry(self):
        observations = []
        result, _, _ = self.run_memory(odom_sink=observations.append)
        self.assertTrue(result["static_link_complete"])
        self.assertGreater(len(observations), 5)
        self.assertTrue(all(row["linear_x"] == 0 and row["angular_z"] == 0 for row in observations))
        self.assertEqual(len({row["sample_sequence"] for row in observations}), len(observations))

    def test_invalid_duration_and_boolean_parameters_rejected_before_write(self):
        for value in (0, -1, True, float("nan"), float("inf"), 3601):
            port = Port(Clock())
            with self.assertRaises(ValueError):
                adapter.run_static_session(port, api, stop_api, core.MeasuredOdometry, duration_s=value)
            self.assertEqual(port.writes, [])

    def test_reviewed_client_sha_matches_prior_query_package(self):
        actual = hashlib.sha256(dependency_paths["control_client"].read_bytes()).hexdigest()
        self.assertEqual(actual, "a293816a54a52eed2a60dbb310b639c43763f14146797dc22e6dd8ab992b5e55")
        prior = PROJECT / "output/v9_async_log_20261102/query_probe_package/control_client.py"
        if prior.is_file():
            self.assertEqual(actual, hashlib.sha256(prior.read_bytes()).hexdigest())

    def run_delayed(self, duration=.005, cancel_at=None, drop_after=None):
        clock = Clock()
        port = DelayedHalfDuplexPort(clock)
        port.drop_feedback_after = drop_after
        with patch.object(api, "time", clock):
            result = adapter.run_static_session(
                port, api, stop_api, core.MeasuredOdometry, duration_s=duration,
                clock=clock.monotonic, sleep=clock.sleep,
                cancelled=lambda: cancel_at is not None and clock.value >= cancel_at)
        return result, port, clock

    def test_local_cancel_does_not_withdraw_query_or_prevent_half_duplex_contention(self):
        clock = Clock()
        port = DelayedHalfDuplexPort(clock)
        client = api.Client(adapter.QueryOnlyPort(port))
        with patch.object(api, "time", clock):
            client.enable_nonblocking()
            query = client.begin_exchange(5)
            clock.sleep(.005)
            with self.assertRaises(TimeoutError):
                client.stop()
        self.assertEqual(query["bytes_accepted"], 20)
        self.assertEqual(len(port.collisions), 1)
        self.assertEqual([kind for kind, _ in port.write_times], [5, 3])

    def test_normal_quiesce_finishes_original_query_before_stop_without_new_queries(self):
        result, port, _ = self.run_delayed()
        self.assertTrue(result["static_link_complete"], result)
        self.assertEqual(port.collisions, [])
        quiesce = result["normal_quiesce"]
        self.assertTrue(quiesce["complete"])
        self.assertEqual(quiesce["queries_started"], 0)
        self.assertIsNotNone(quiesce["pending_at_window_end"])
        self.assertGreater(quiesce["duration_s"], 0.)
        self.assertLess(quiesce["duration_s"], .020)
        pending = quiesce["pending_at_window_end"]
        quiet_rows = [row for row in result["rows"] if row["phase"] == "normal_quiesce"]
        self.assertEqual(len(quiet_rows), 1)
        self.assertEqual(quiet_rows[0]["request_hex"], pending["request_hex"])
        self.assertEqual(quiet_rows[0]["request_start_monotonic_s"], pending["started_monotonic_s"])
        self.assertLessEqual(quiet_rows[0]["completed_monotonic_s"], pending["deadline_monotonic_s"])
        # If the query is a duplicate, its source remains the active sample's
        # first origin; otherwise its request start (not drain time) is used.
        if quiet_rows[0]["distinct"]:
            self.assertLessEqual(quiet_rows[0]["source_monotonic_s"], pending["started_monotonic_s"])
        else:
            earlier = [row for row in result["rows"] if row["kind"] == 5 and row["phase"] == "initial_stop"][-1]
            self.assertEqual(quiet_rows[0]["source_monotonic_s"], earlier["source_monotonic_s"])
        active_end = result["active_start_monotonic_s"] + result["active_duration_s"]
        self.assertLessEqual(active_end, quiesce["started_monotonic_s"])
        self.assertLess(result["active_duration_s"], .007)
        self.assertEqual(result["metrics"]["feedback_responses"], 0)

    def test_cancel_in_normal_quiesce_stops_immediately_without_waiting_for_query(self):
        ordinary, _, _ = self.run_delayed()
        start = ordinary["active_start_monotonic_s"]
        result, port, _ = self.run_delayed(cancel_at=start + .007)
        self.assertEqual(result["first_failure"]["phase"], "normal_quiesce")
        self.assertIn("cancelled", result["first_failure"]["message"])
        self.assertFalse(result["normal_quiesce"]["complete"])
        self.assertLess(result["failure_monotonic_s"], start + .009)
        self.assertEqual(len(port.collisions), 1)
        self.assertLess(port.collisions[0]["stop_at"], start + .013)

    def test_cancel_during_active_window_never_waits_in_normal_quiesce(self):
        ordinary, _, _ = self.run_delayed()
        start = ordinary["active_start_monotonic_s"]
        result, port, _ = self.run_delayed(duration=5., cancel_at=start + .003)
        self.assertEqual(result["first_failure"]["phase"], "active")
        self.assertNotIn("normal_quiesce", result)
        self.assertFalse(result["active_window_complete"])
        self.assertEqual(len(port.collisions), 1)
        self.assertLess(port.collisions[0]["stop_at"], start + .005)

    def test_quiesce_keeps_original_300ms_deadline_and_does_not_relabel_failed_query(self):
        ordinary, _, _ = self.run_delayed()
        start = ordinary["active_start_monotonic_s"]
        result, port, _ = self.run_delayed(drop_after=start)
        self.assertFalse(result["static_link_complete"])
        self.assertTrue(result["active_window_complete"])
        self.assertEqual(result["first_failure"]["phase"], "normal_quiesce")
        self.assertEqual(result["first_failure"]["error_type"], "TimeoutError")
        quiesce = result["normal_quiesce"]
        self.assertFalse(quiesce["complete"])
        self.assertEqual(quiesce["queries_started"], 0)
        self.assertLess(quiesce["duration_s"], .301)
        self.assertLess(result["active_duration_s"], .007)
        self.assertLessEqual(result["failure_monotonic_s"] - quiesce["pending_at_window_end"]["deadline_monotonic_s"], .002)
        self.assertEqual(port.stop_count, 2)


if __name__ == "__main__":
    unittest.main()
