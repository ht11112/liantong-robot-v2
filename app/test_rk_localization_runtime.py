"""Pure seed/source/cleanup contracts; no ROS or physical I/O is imported."""
from copy import deepcopy
import json
import math
import subprocess
import unittest

import rk_localization_runtime as runtime


def request():
    return dict(schema="p2p-localization-seed-v1", boot_id="current-boot", nonce="abcdef12",
                map_id=runtime.MAP_ID, pose=[1., 2., .3], seed_candidate_accepted=True,
                extrinsics=[.13, 0., 0.], proposal_snapshot_stamps=[[100, 1], [100, 2], [100, 3]])


def scan(seconds=101, nanos=0, source_mono=50.1):
    return dict(header=dict(stamp_secs=seconds, stamp_nsecs=nanos, source_stamp=seconds + nanos / 1e9,
                            frame_id="p2p_abcdef12_laser", seq=1),
                source_monotonic_s=source_mono, ranges=[float("nan"), float("inf"), -float("inf"), 1.],
                angle_min=-math.pi, angle_increment=math.pi / 2., range_min=.1, range_max=50.)


def pose(stamp=101., covariance=None):
    sec = int(stamp)
    return dict(header=dict(stamp_secs=sec, stamp_nsecs=int(round((stamp - sec) * 1e9)),
                            frame_id="p2p_abcdef12_map"),
                position=dict(x=1., y=2.), orientation=dict(x=0., y=0., z=0., w=1.),
                covariance=[0.] * 36 if covariance is None else covariance)


def transform():
    return dict(source_stamp=101., translation=dict(x=1., y=2.), rotation=dict(x=0., y=0., z=0., w=1.))


class Child:
    def __init__(self, exit_code=None, timeout=False, terminate_error=False):
        self.pid = 123
        self.exit_code, self.timeout, self.terminate_error = exit_code, timeout, terminate_error
        self.calls = []

    def poll(self):
        return self.exit_code

    def terminate(self):
        self.calls.append("TERM")
        if self.terminate_error:
            raise OSError("synthetic cleanup failure")

    def kill(self):
        self.calls.append("KILL")

    def wait(self, timeout):
        self.calls.append("wait")
        if self.timeout and "KILL" not in self.calls:
            raise subprocess.TimeoutExpired("fixture", timeout)
        self.exit_code = -9 if "KILL" in self.calls else (-15 if self.exit_code is None else self.exit_code)
        return self.exit_code


class RuntimeContractTests(unittest.TestCase):
    def checked_seed(self, value):
        return runtime.validate_seed_request(value, boot_id="current-boot", nonce="abcdef12",
                                             expected_snapshot_keys=[[100, 1], [100, 2], [100, 3]])

    def test_current_seed_accepts_only_the_bound_identity_and_exact_original_sources(self):
        self.assertEqual(self.checked_seed(request()), [1., 2., .3])
        for key, value in (("boot_id", "old-boot"), ("nonce", "other-nonce"), ("map_id", "0" * 64),
                           ("seed_candidate_accepted", 1), ("proposal_snapshot_stamps", [[100, 1]] * 3),
                           ("extrinsics", [.13, True, 0]), ("extrinsics", [.2, 0., 0.])):
            candidate = request()
            candidate[key] = value
            with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                self.checked_seed(candidate)

    def test_pose_nan_infinity_boolean_and_wrong_units_rejected_before_publication(self):
        for values in ([float("nan"), 0., 0.], [0., float("inf"), 0.],
                       [True, 0., 0.], [0., 0., 180.], [101., 0., 0.]):
            candidate = request()
            candidate["pose"] = values
            with self.assertRaises(ValueError):
                self.checked_seed(candidate)

    def test_live_scans_must_all_follow_anchor_and_be_current_not_merely_newly_received(self):
        values = [scan(101, n, 50.1 + n / 10.) for n in (1, 2, 3)]
        args = dict(now_ros=101.1, anchor_source_monotonic_s=50., laser_frame="p2p_abcdef12_laser")
        self.assertEqual(runtime.current_three_scans(values, **args), values)
        values[0]["source_monotonic_s"] = 49.9
        with self.assertRaises(ValueError):
            runtime.current_three_scans(values, **args)
        values[0]["source_monotonic_s"] = 50.1
        with self.assertRaises(ValueError):
            runtime.current_three_scans(values, **dict(args, now_ros=101.76))
        values[1]["header"] = dict(values[0]["header"])
        with self.assertRaises(ValueError):
            runtime.current_three_scans(values, **args)

    def test_public_snapshot_retains_original_stamps_and_after_anchor_capture(self):
        scans = [scan(101, n, 50.1 + n / 10.) for n in (1, 2, 3)]
        snapshot = runtime.proposal_snapshot(scans, boot_id="current-boot", nonce="abcdef12",
                                             frames=runtime.names_for_nonce("abcdef12")["frames"], anchor={"valid": True})
        self.assertEqual(runtime.snapshot_keys(snapshot), [[101, 1], [101, 2], [101, 3]])
        self.assertEqual(snapshot["snapshots"][0]["ranges"], [None, "inf", "-inf", 1.])
        self.assertEqual(snapshot["snapshots"][0]["header"], scans[0]["header"])
        self.assertFalse(snapshot["source_time_rewritten"])
        json.dumps(snapshot, allow_nan=False)

    def test_every_live_scan_needs_endpoint_and_wall_threshold(self):
        class Scorer:
            def __init__(self): self.calls = 0
            def scan_match(self, *_):
                self.calls += 1
                return dict(used=80 if self.calls != 2 else 59, near_wall_ratio=.9)
        scorer = Scorer()
        with self.assertRaises(ValueError):
            runtime.scan_agreement(scorer, {}, [], [scan()] * 3, [1., 2., 0.])
        self.assertEqual(scorer.calls, 3)

    def test_preseed_default_amcl_pose_cannot_be_counted(self):
        with self.assertRaisesRegex(ValueError, "strictly after this seed"):
            runtime.quality_sample(pose(100.9), transform(), now_ros=101.1,
                                   seed_ros=101., map_frame="p2p_abcdef12_map")
        self.assertEqual(runtime.quality_sample(pose(), transform(), now_ros=101.1,
                                               seed_ros=100.9, map_frame="p2p_abcdef12_map")["source_key"], [101, 0])

    def test_covariance_and_tf_must_both_match_live_amcl(self):
        covariance = [0.] * 36
        covariance[0] = .11 ** 2
        with self.assertRaisesRegex(ValueError, "covariance"):
            runtime.quality_sample(pose(covariance=covariance), transform(), now_ros=101.1,
                                   seed_ros=100., map_frame="p2p_abcdef12_map")
        moved = transform()
        moved["translation"]["x"] += .04
        with self.assertRaisesRegex(ValueError, "differs"):
            runtime.quality_sample(pose(), moved, now_ros=101.1,
                                   seed_ros=100., map_frame="p2p_abcdef12_map")

    def test_five_distinct_postseed_quality_samples_and_later_counterevidence(self):
        evidence = runtime.ConsecutiveLocalizationEvidence(100., "abcdef12")
        for n in range(5):
            stamp = 100.1 + n * .2
            observation = dict(source_key=[100, 100000000 + n * 200000000], original_source_stamp=stamp)
            latest = evidence.observe(observation, now_ros=stamp + .01, now_monotonic_s=50. + n)
        self.assertIsNotNone(latest)
        original_time = latest["validated_at_monotonic_s"]
        duplicate = evidence.observe(observation, now_ros=stamp + .1, now_monotonic_s=99.)
        self.assertEqual(duplicate["validated_at_monotonic_s"], original_time)
        evidence.reject()
        self.assertIsNone(evidence.latest)
        self.assertIsNotNone(evidence.first)
        for n in range(4):
            t = 101.1 + n * .1
            self.assertIsNone(evidence.observe(dict(source_key=[101, 100000000 + n * 100000000],
                                                    original_source_stamp=t), now_ros=t + .01, now_monotonic_s=60. + n))

    def test_cleanup_records_term_kill_and_errors_without_stopping_other_children(self):
        healthy, hung, errored, ended = Child(), Child(timeout=True), Child(terminate_error=True), Child(exit_code=0)
        rows = runtime.cleanup_children([("healthy", healthy), ("hung", hung), ("errored", errored), ("ended", ended)],
                                        expected_natural_exit_names=("ended",))
        self.assertEqual(rows[0]["exit_code"], -15)
        self.assertTrue(rows[1]["forced_kill"])
        self.assertEqual(rows[1]["exit_code"], -9)
        self.assertTrue(rows[2]["errors"])
        self.assertEqual(rows[3]["exit_code"], 0)
        self.assertEqual(ended.calls, ["wait"])

    def test_crash_or_unexpected_early_exit_cannot_be_a_successful_cleanup(self):
        for code in (-11, 1, 0):
            row = runtime.cleanup_children([("amcl", Child(exit_code=code))],
                                            expected_natural_exit_names=("adapter",))[0]
            self.assertFalse(row["expected_termination"])
            self.assertTrue(row["errors"])
        adapter = runtime.cleanup_children([("adapter", Child(exit_code=0))],
                                            expected_natural_exit_names=("adapter",))[0]
        self.assertTrue(adapter["expected_termination"])

    def test_names_are_private_and_reject_namespace_or_shell_fragments(self):
        names = runtime.names_for_nonce("abcdef12")
        self.assertEqual(names["frames"]["base"], "p2p_abcdef12_base")
        self.assertTrue(all(value.startswith("/p2p_loc_abcdef12/") for value in names["topics"].values()))
        for value in ("../escape", "/global", "x;kill", "a b c d", True):
            with self.assertRaises(ValueError):
                runtime.names_for_nonce(value)

    def test_query_only_wire_guard_remains_the_only_physical_writer(self):
        from base_static_adapter import QueryOnlyPort, crc16
        import struct
        class Port:
            def __init__(self): self.writes = []
            def write(self, raw): self.writes.append(raw); return len(raw)
        physical = Port()
        fence = QueryOnlyPort(physical)
        for kind in (6, 7):
            body = struct.pack('<BBBBHhhII', 0xaa, 0x55, 9, kind, 1, 0, 0, 1, 1 if kind == 6 else 0)
            with self.assertRaises(ValueError):
                fence.write(body + struct.pack('<H', crc16(body)))
        self.assertEqual(physical.writes, [])


if __name__ == "__main__":
    unittest.main()
