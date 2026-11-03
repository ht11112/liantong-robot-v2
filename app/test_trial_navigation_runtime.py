"""Pure trial limits, source handoff and moving-pose tests; no ROS/hardware."""
from copy import deepcopy
import ast
import json
import math
from pathlib import Path
import signal
import sys
import tempfile
import threading
import time
from types import SimpleNamespace
import unittest

from trial_navigation_runtime import (TrialCommandState, heading_departure, json_safe,
    trial_planner_parameters, validate_moving_pose, validate_trial_config, within_position_tolerance,
    stop_owned_child, prearm_command_ready, require_post_stop_sources, health_sources_current,
    health_deadline_diagnostic, waiting_only_for_arm_tf, prearm_command_diagnostic,
    begin_preparation_nomotion, diagnostic_scan_agreement, ScanAgreementFault, scan_failure_evidence)


def config():
    return json.loads(Path(__file__).with_name("trial_config.json").read_text())


def pose():
    return dict(header=dict(stamp_secs=101, stamp_nsecs=0, frame_id="p2p_test_map"),
                position=dict(x=1., y=2.), orientation=dict(x=0., y=0., z=0., w=1.),
                covariance=[0.] * 36)


def transform():
    return dict(source_stamp=101., translation=dict(x=1., y=2.), rotation=dict(x=0., y=0., z=0., w=1.))


def nested_runtime_function(name, namespace):
    source = ast.parse(Path(__file__).with_name("trial_navigation_runtime.py").read_text())
    main = next(node for node in source.body if isinstance(node, ast.FunctionDef) and node.name == "main")
    function = next(node for node in main.body if isinstance(node, ast.FunctionDef) and node.name == name)
    exec(compile(ast.Module(body=[function], type_ignores=[]), "trial-main-injection", "exec"), namespace)
    return namespace[name]


class TrialNavigationTests(unittest.TestCase):
    def test_user_values_remain_exact_and_scope_is_one_attended_trial(self):
        current = config()
        self.assertEqual(validate_trial_config(current), current)
        for field, bad in (("forward_m", 1.), ("position_tolerance_m", .2),
                           ("allow_reverse", True), ("heading_required", True),
                           ("max_linear_m_s", .04), ("feedback_timeout_s", 1.)):
            changed = deepcopy(current)
            changed["user"][field] = bad
            with self.subTest(field=field), self.assertRaises(ValueError):
                validate_trial_config(changed)

    def test_unconfirmed_site_and_missing_engineering_budget_refuse(self):
        for kind in ("site", "age", "inplace", "geometry"):
            value = config()
            if kind == "site": value["onsite_confirmation"]["unoccupied"] = False
            if kind == "age": value["engineering"]["command_timeout_s"] = None
            if kind == "inplace": value["engineering"]["allow_in_place_rotation"] = True
            if kind == "geometry": value["geometry"]["laser_x_m"] = .2
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                validate_trial_config(value)

    def test_planner_has_user_tolerance_no_reverse_and_no_final_heading(self):
        names = dict(frames=dict(map="p2p_test_map", odom="p2p_test_odom", base="p2p_test_base"),
                     map_topic="/p2p_loc_test/map", scan_topic="/p2p_loc_test/scan_front180", odom_topic="/p2p_loc_test/wheel_odom")
        parameters = trial_planner_parameters(names, config())
        dwa = parameters["DWAPlannerROS"]
        self.assertEqual(dwa["max_vel_x"], .03)
        self.assertEqual(dwa["min_vel_x"], .006)
        self.assertAlmostEqual(dwa["max_vel_theta"], math.radians(6.9))
        self.assertEqual(dwa["xy_goal_tolerance"], .10)
        self.assertEqual(dwa["yaw_goal_tolerance"], math.pi)
        self.assertEqual(parameters["local_costmap"]["resolution"], .01)
        self.assertFalse(parameters["make_plan_clear_costmap"])
        self.assertEqual(parameters["global_costmap"]["transform_tolerance"], .50)
        self.assertEqual(parameters["local_costmap"]["transform_tolerance"], .50)

    def test_prearm_velocity_never_becomes_postarm_command(self):
        state = TrialCommandState("one-task", "/own_move_base")
        state.accept(.02, 0., 10., "/own_move_base")
        state.arm(10.1)
        self.assertIsNone(state.command())
        state.accept(.02, 0., 10.2, "/own_move_base")
        self.assertEqual(state.command()["received_monotonic_s"], 10.2)
        self.assertEqual(state.command()["task_id"], "one-task")
        self.assertEqual(state.command(), state.command())
        with self.assertRaises(RuntimeError):
            state.arm(10.3)

    def test_source_reverse_latched_and_overspeed_commands_refuse(self):
        for v, w, caller, latched in ((-.001, 0., "/own", False), (.031, 0., "/own", False),
            (.02, math.radians(7.), "/own", False), (.02, 0., "/other", False), (.02, 0., "/own", True)):
            state = TrialCommandState("task", "/own")
            self.assertFalse(state.accept(v, w, 1., caller, latched))
            self.assertIsNotNone(state.first_failure)

    def test_moving_pose_checks_sources_and_covariance_without_static_anchor(self):
        value = validate_moving_pose(pose(), transform(), now_ros=101.1, seed_ros=100., map_frame="p2p_test_map")
        self.assertEqual(value["pose"], [1., 2., 0.])
        for mode in ("old", "covariance", "tf"):
            p, tf = pose(), transform()
            if mode == "old": p["header"]["stamp_secs"] = 98
            if mode == "covariance": p["covariance"][0] = .11 ** 2
            if mode == "tf": tf["translation"]["x"] += .04
            with self.subTest(mode=mode), self.assertRaises(ValueError):
                validate_moving_pose(p, tf, now_ros=101.1, seed_ros=100., map_frame="p2p_test_map")

    def test_position_acceptance_ignores_final_heading_but_heading_corridor_is_separate(self):
        self.assertTrue(within_position_tolerance([.405, 0., 1.], [.5, 0., 0.], .1))
        self.assertFalse(within_position_tolerance([.399, 0., 0.], [.5, 0., 0.], .1))
        self.assertAlmostEqual(heading_departure([0., 0., -math.pi + .01], [0., 0., math.pi - .01]), .02)

    def test_complete_bad_covariance_is_strict_json_evidence(self):
        raw = pose()
        raw["covariance"][7] = float("nan")
        recorded = json_safe(raw)
        self.assertEqual(len(recorded["covariance"]), 36)
        self.assertEqual(recorded["covariance"][7], "nan")
        json.dumps(recorded, allow_nan=False)

    def test_cleanup_uses_owned_sigint_and_preserves_prior_crash(self):
        class Child:
            pid = 123
            def __init__(self, code=None): self.returncode, self.signals = code, []
            def poll(self): return self.returncode
            def send_signal(self, number):
                self.signals.append(number)
                self.returncode = -signal.SIGINT
            def wait(self, timeout): return self.returncode
        running = Child()
        row = stop_owned_child("own", running)
        self.assertEqual(running.signals, [signal.SIGINT])
        self.assertTrue(row["expected_termination"])
        crashed = Child(-6)
        row = stop_owned_child("own", crashed)
        self.assertFalse(row["expected_termination"])
        self.assertEqual(crashed.signals, [])
        self.assertEqual(row["exit_code"], -6)

    def test_prearm_requires_current_expressible_forward_command(self):
        for v, w, stamp, expected in ((0., 0., 10., False), (0., .02, 10., False),
                (.0001, 0., 10., False), (.02, 0., 10., True), (.02, 0., 9.75, False)):
            command = dict(linear_mps=v, angular_rad_s=w, received_monotonic_s=stamp)
            self.assertEqual(prearm_command_ready(command, 10., config()), expected)

    def test_post_stop_sources_must_all_advance_after_parking_status(self):
        fresh = dict(amcl_source_monotonic_s=20.1, tf_source_monotonic_s=20.2,
                     oldest_scan_source_monotonic_s=20.3)
        self.assertEqual(require_post_stop_sources(fresh, 20.), fresh)
        for key in fresh:
            stale = dict(fresh, **{key: 20.})
            with self.subTest(key=key), self.assertRaises(ValueError):
                require_post_stop_sources(stale, 20.)

    def test_cached_parked_health_cannot_complete_after_original_sources_expire(self):
        engineering = config()["engineering"]
        health = dict(odom_source_monotonic_s=10., oldest_scan_source_monotonic_s=10.,
                      amcl_source_monotonic_s=10., tf_source_monotonic_s=10., observed_monotonic_s=10.1)
        original = deepcopy(health)
        self.assertTrue(health_sources_current(health, 10.1, engineering))
        self.assertFalse(health_sources_current(health, 10.751, engineering))
        for key, stamp in (("odom_source_monotonic_s", 9.35),
                           ("oldest_scan_source_monotonic_s", 9.35),
                           ("amcl_source_monotonic_s", 8.1),
                           ("tf_source_monotonic_s", 9.59)):
            with self.subTest(key=key):
                self.assertFalse(health_sources_current(dict(health, **{key: stamp}), 10.1, engineering))
        self.assertEqual(health, original)

    def test_arming_tf_expiry_diagnostic_distinguishes_source_from_worker_heartbeat(self):
        now = 8461.676517974
        health = dict(odom_source_monotonic_s=8461.175741522,
            oldest_scan_source_monotonic_s=8461.40, amcl_source_monotonic_s=8460.6809,
            tf_source_monotonic_s=8461.175741522, observed_monotonic_s=8461.66, source_key=[1793677738, 808469000])
        value = health_deadline_diagnostic(health, now, config()["engineering"], 8461.67)
        self.assertTrue(value["heartbeat_current"])
        self.assertTrue(value["sources"]["odom"]["current"])
        self.assertTrue(value["sources"]["oldest_scan"]["current"])
        self.assertTrue(value["sources"]["amcl"]["current"])
        self.assertFalse(value["sources"]["tf"]["current"])
        self.assertAlmostEqual(value["sources"]["tf"]["age_s"], .500776452)

    def test_zero_target_arm_pipeline_can_wait_for_old_tf_but_never_calls_it_ready(self):
        p, tf = pose(), transform()
        tf["source_stamp"] = 100.5
        waiting = validate_moving_pose(p, tf, now_ros=101.1, seed_ros=100.,
                                      map_frame="p2p_test_map", allow_waiting_tf=True)
        self.assertTrue(waiting["awaiting_post_arm_tf"])
        self.assertEqual(waiting["tf_original_source_stamp"], 100.5)
        with self.assertRaises(ValueError):
            validate_moving_pose(p, tf, now_ros=101.1, seed_ros=100., map_frame="p2p_test_map")
        waiting.update(odom_source_monotonic_s=10., oldest_scan_source_monotonic_s=10.,
                       amcl_source_monotonic_s=10., tf_source_monotonic_s=9.5)
        self.assertTrue(waiting_only_for_arm_tf(waiting, 10.1, config()["engineering"], 10.05))
        unflagged = dict(waiting)
        del unflagged["awaiting_post_arm_tf"]
        self.assertTrue(waiting_only_for_arm_tf(unflagged, 10.1, config()["engineering"], 10.05))
        self.assertFalse(health_sources_current(waiting, 10.1, config()["engineering"]))
        stale = dict(waiting, odom_source_monotonic_s=9.)
        self.assertFalse(waiting_only_for_arm_tf(stale, 10.1, config()["engineering"], 10.05))
        p["covariance"][0] = .11 ** 2
        with self.assertRaises(ValueError):
            validate_moving_pose(p, tf, now_ros=101.1, seed_ros=100., map_frame="p2p_test_map", allow_waiting_tf=True)

    def test_real_preparation_empty_plan_is_preserved_and_never_retried(self):
        import trial_navigation_runtime as runtime
        from localization_ros_adapter import source_header
        def stamp(): return SimpleNamespace(secs=101, nsecs=0)
        def header(): return SimpleNamespace(seq=7, stamp=stamp(), frame_id="map")
        def target(): return SimpleNamespace(header=header(), pose=SimpleNamespace(
            position=SimpleNamespace(x=0., y=0., z=0.), orientation=SimpleNamespace(x=0., y=0., z=0., w=1.)))
        request_type = lambda: SimpleNamespace(start=target(), goal=target(), tolerance=None)
        empty = SimpleNamespace(header=header(), poses=[])
        calls = []
        def proxy(*_args):
            def invoke(req):
                calls.append(req)
                return SimpleNamespace(plan=empty)
            return invoke
        live = SimpleNamespace(header=header(), data=[0] * 100,
            info=SimpleNamespace(width=10, height=10, resolution=.1,
                origin=SimpleNamespace(position=SimpleNamespace(x=0., y=0., z=0.),
                    orientation=SimpleNamespace(x=0., y=0., z=0., w=1.))))
        live.header.stamp.to_sec = lambda: 101.
        with tempfile.TemporaryDirectory() as folder:
            state = dict(preparation_attempted=False, costmap=live, preparation_evidence=None)
            env = dict(state=state, user=dict(forward_m=.5), frames=dict(map="map"),
                forward_goal=runtime.forward_goal, GetPlanRequest=request_type,
                rospy=SimpleNamespace(Time=SimpleNamespace(now=lambda: SimpleNamespace(to_sec=lambda: 101.)), ServiceProxy=proxy),
                nav_node="/own/move_base", GetPlan=object, lock=threading.RLock(), source_header=source_header,
                math=math,
                yaw_from_quaternion=runtime.yaw_from_quaternion, atomic_json=runtime.atomic_json,
                time=time, output=Path(folder) / "trial_result.json", map_identity={"verified": True},
                json_safe=json_safe, info=dict(width=10, height=10, resolution=.1, origin=[0., 0., 0.]),
                cells=[0] * 100, validate_path=runtime.validate_path, eng=dict(max_trial_path_m=1.))
            # ROS Time.now also serves as the request's exact source header.
            env["rospy"].Time.now = lambda: SimpleNamespace(secs=101, nsecs=0, to_sec=lambda: 101.)
            prepare = nested_runtime_function("prepare_goal", env)
            joint = dict(post_seed_five_observations=[dict(pose=[.2, .2, 0.])] * 5)
            with self.assertRaisesRegex(ValueError, "bounded nonempty"):
                prepare(joint)
            evidence = json.loads((Path(folder) / "trial_preparation_evidence.json").read_text())
            self.assertEqual(evidence["returned_path"]["poses"], [])
            self.assertEqual(evidence["costmap"]["data"], [0] * 100)
            self.assertEqual(evidence["costmap_header"]["stamp_secs"], 101)
            self.assertEqual(evidence["request_start"], [.2, .2, 0.])
            self.assertEqual(evidence["request_goal"], [.7, .2, 0.])
            self.assertEqual(evidence["failure"]["error_type"], "ValueError")
            with self.assertRaisesRegex(RuntimeError, "Only one"):
                prepare(joint)
            self.assertEqual(len(calls), 1)

    def test_navigation_process_requires_post_seed_five_and_launches_once(self):
        launches, logs, children = [], [], []
        def popen(command, **_kwargs):
            launches.append(command)
            return SimpleNamespace(pid=123)
        with tempfile.TemporaryDirectory() as folder:
            state = dict(navigation_started=False, seed_ros=None, joint=None)
            env = dict(state=state, finished=threading.Event(), failures=[], logs=logs,
                children=children, output=Path(folder) / "trial_result.json", nav_ns="/private/navigation",
                cmd_topic="/private/trial_cmd", topics=dict(wheel_odom="/private/odom", map="/private/map"),
                subprocess=SimpleNamespace(Popen=popen, DEVNULL=-3), os=SimpleNamespace(environ={}),
                time=time, result={}, deepcopy=deepcopy)
            start = nested_runtime_function("start_navigation", env)
            with self.assertRaisesRegex(RuntimeError, "five accepted"):
                start()
            self.assertEqual(launches, [])
            state.update(seed_ros=100., joint=dict(post_seed_five_observations=[{}] * 5))
            start()
            start()
            self.assertEqual(len(launches), 1)
            self.assertIn("cmd_vel:=/private/trial_cmd", launches[0])
            for log in logs: log.close()

    def post_plan_fixture(self, produce_new):
        import trial_navigation_runtime as runtime
        clock = SimpleNamespace(at=100.)
        poses = [dict(header=dict(stamp_secs=101, stamp_nsecs=0))]
        state = dict(prepared=True, phase="waiting_ready", start_pose=[.2, .2, 0.], nomotion_inflight=False)
        calls = []
        class Finished:
            def is_set(self): return False
            def wait(self, seconds): clock.at += seconds
        def service(*_args):
            def invoke():
                calls.append("nomotion")
                if produce_new and calls.count("nomotion") == 2:
                    poses.append(dict(header=dict(stamp_secs=102, stamp_nsecs=0)))
            return invoke
        def quality(**_kwargs):
            calls.append("quality")
            self.assertEqual(poses[-1]["header"]["stamp_secs"], 102)
            return dict(pose=[.2, .2, 0.], source_key=[102, 0],
                odom_source_monotonic_s=clock.at-.01, oldest_scan_source_monotonic_s=clock.at-.1,
                amcl_source_monotonic_s=clock.at-.1, tf_source_monotonic_s=clock.at-.01)
        env = dict(state=state, lock=threading.RLock(), poses=poses, exact_stamp=runtime.exact_stamp,
            time=SimpleNamespace(monotonic=lambda: clock.at), eng=config()["engineering"],
            finished=Finished(), manual_cancel=threading.Event(), failures=[],
            rospy=SimpleNamespace(wait_for_service=lambda *_args, **_kwargs: None, ServiceProxy=service),
            topics=dict(request_nomotion_update="/private/nomotion"), Empty=object,
            pose_and_sources=quality, math=math, heading_departure=heading_departure,
            health_sources_current=health_sources_current)
        return nested_runtime_function("refresh_after_plan", env), state, clock, calls, poses

    def test_post_plan_old_source_waits_for_real_new_key_before_quality_or_goal(self):
        refresh, state, clock, calls, _poses = self.post_plan_fixture(True)
        checked = refresh()
        self.assertEqual(checked["source_key"], [102, 0])
        self.assertEqual(calls, ["nomotion", "nomotion", "quality"])
        self.assertFalse(state["prepared"])
        self.assertFalse(state["nomotion_inflight"])
        self.assertEqual(state["post_plan_refresh"]["previous_source_key"], [101, 0])
        self.assertLess(clock.at, 102.)

    def test_post_plan_duplicate_sources_timeout_without_renewal_or_quality_acceptance(self):
        refresh, state, clock, calls, poses = self.post_plan_fixture(False)
        with self.assertRaisesRegex(RuntimeError, "No distinct current AMCL"):
            refresh()
        self.assertNotIn("quality", calls)
        self.assertEqual(poses, [dict(header=dict(stamp_secs=101, stamp_nsecs=0))])
        self.assertFalse(state["prepared"])
        self.assertGreaterEqual(clock.at, 102.)
        self.assertLess(clock.at, 102.03)

    def test_nomotion_continues_after_goal_until_atomic_owner_handoff(self):
        state = dict(phase="waiting_ready", goal_sent=True, nomotion_finished=False,
                     nomotion_inflight=False, ready_handoff_reserved=False, prepared=True, anchor_enabled=True)
        self.assertTrue(begin_preparation_nomotion(state))
        self.assertTrue(state["nomotion_inflight"])
        self.assertFalse(state["prepared"])
        self.assertFalse(begin_preparation_nomotion(state))
        state.update(nomotion_inflight=False, prepared=True)
        self.assertTrue(begin_preparation_nomotion(state))
        state.update(nomotion_inflight=False, prepared=True, ready_handoff_reserved=True)
        self.assertFalse(begin_preparation_nomotion(state))
        phase = nested_runtime_function("on_phase", dict(state=state, lock=threading.RLock(), phase_history=[]))
        phase("arming", 101.)
        self.assertTrue(state["nomotion_finished"])
        self.assertFalse(state["anchor_enabled"])
        self.assertFalse(begin_preparation_nomotion(state))

    def test_command_diagnostics_distinguish_raw_zero_quantized_zero_and_rejection(self):
        examples = ((0., 0., "raw_dwa_zero"), (.0001, 0., "quantized_to_zero"),
                    (0., .02, "unsupported_forward_trial_command"), (.02, 0., "expressible_forward_integer_command"))
        for v, w, reason in examples:
            value = prearm_command_diagnostic(dict(linear_mps=v, angular_rad_s=w, received_monotonic_s=10.), 10., config())
            with self.subTest(reason=reason):
                self.assertEqual(value["reason"], reason)
                self.assertEqual(value["ready"], reason == "expressible_forward_integer_command")

    def test_bad_scan_retains_all_original_scores_and_raw_evidence_without_relaxing_threshold(self):
        scores = [dict(used=60, near_wall_ratio=.65), dict(used=59, near_wall_ratio=.9), dict(used=70, near_wall_ratio=.64)]
        class Core:
            def __init__(self, values): self.values, self.calls = iter(values), 0
            def scan_match(self, *_args):
                self.calls += 1
                return next(self.values)
        scans = [dict(header=dict(stamp_secs=101, stamp_nsecs=index, frame_id="laser"),
            ranges=[1., float("nan"), float("inf")], angle_min=-1., angle_increment=.01,
            range_min=.1, range_max=50., source_monotonic_s=10.+index*.1) for index in range(3)]
        core = Core(scores)
        with self.assertRaises(ScanAgreementFault) as raised:
            diagnostic_scan_agreement(core, {}, [], scans, [1., 2., 0.])
        self.assertEqual(core.calls, 3)
        self.assertEqual(raised.exception.scores, scores)
        evidence = scan_failure_evidence(raised.exception, scans, pose(), transform(), [1., 2., 0.],
            now_ros=101.5, now=10.5, phase="waiting_ready", frames=dict(map="map", laser="laser"))
        scans[0]["ranges"][0] = 99.
        recorded = json_safe(evidence)
        self.assertEqual(recorded["raw_scans"][0]["ranges"], [1., "nan", "inf"])
        self.assertEqual(recorded["raw_scans"][2]["header"]["stamp_nsecs"], 2)
        self.assertEqual(recorded["raw_scans"][0]["source_monotonic_s"], 10.)
        self.assertEqual(recorded["thresholds"], dict(minimum_endpoints=60, minimum_near_wall_ratio=.65))
        self.assertEqual(recorded["scores"], scores)
        json.dumps(recorded, allow_nan=False)
        exact = [dict(used=60, near_wall_ratio=.65)] * 3
        self.assertEqual(diagnostic_scan_agreement(Core(exact), {}, [], scans, [1., 2., 0.]), exact)

    def test_import_has_no_ros_or_serial_import(self):
        import importlib.util
        before = set(sys.modules)
        path = Path(__file__).with_name("trial_navigation_runtime.py")
        spec = importlib.util.spec_from_file_location("isolated_trial_nav", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertFalse({"rospy", "tf2_ros", "serial", "actionlib"} & (set(sys.modules) - before))


if __name__ == "__main__":
    unittest.main()
