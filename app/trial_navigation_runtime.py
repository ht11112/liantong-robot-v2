"""One attended half-metre move_base/DWA trial with one injected motion owner.

No frozen static runtime is modified. Stationary evidence is used only for the
new seed; arming permanently disables it and all forced nomotion updates.
ROS/file work runs outside the physical owner's callback/control thread.
"""
import argparse
from collections import deque
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

from config import loads_config
from global_pose_proposal import verified_map
from localization_anchor import StaticLocalizationAnchor
from rk_localization_runtime import (MAP_ID, names_for_nonce, exact_stamp,
    snapshot_keys, validate_seed_request, current_three_scans, scan_agreement,
    proposal_snapshot, quality_sample, ConsecutiveLocalizationEvidence,
    checked_module, atomic_json, yaw_from_quaternion)
from rk_path_preview import planner_parameters, forward_goal, validate_path


ROOT = Path(__file__).resolve().parent


def finite(value, name="number"):
    if type(value) not in (int, float):
        raise ValueError(name + " must be finite")
    try:
        value = float(value)
    except (ValueError, OverflowError):
        raise ValueError(name + " must be finite")
    if not math.isfinite(value):
        raise ValueError(name + " must be finite")
    return value


def json_safe(value):
    """Keep complete bad covariance evidence without invalid JSON extensions."""
    if type(value) is float and not math.isfinite(value):
        return "nan" if math.isnan(value) else "inf" if value > 0 else "-inf"
    if type(value) is dict:
        return {str(key): json_safe(item) for key, item in value.items()}
    if type(value) in (list, tuple, deque):
        return [json_safe(item) for item in value]
    return value


def validate_trial_config(config):
    if config.get("schema") != "p2p-half-metre-trial-v1":
        raise ValueError("Explicit half-metre trial configuration required")
    expected = dict(forward_m=.5, position_tolerance_m=.10, allow_reverse=False,
        heading_required=False, max_linear_m_s=.03, max_angular_deg_s=6.9,
        front_clearance_m=.20, feedback_timeout_s=.75, max_duration_s=200,
        connection_loss_action="continue_local_task")
    user = config.get("user") or {}
    for key, wanted in expected.items():
        actual = user.get(key)
        if type(wanted) is bool:
            valid = type(actual) is bool and actual is wanted
        elif type(wanted) is str:
            valid = actual == wanted
        else:
            valid = finite(actual, key) == wanted
        if not valid:
            raise ValueError("User-confirmed trial value differs: " + key)
    engineering = config.get("engineering") or {}
    for key in ("scan_timeout_s", "odom_timeout_s", "amcl_timeout_s", "command_timeout_s",
                "supervisor_timeout_s", "warmup_timeout_s", "arm_timeout_s", "apply_timeout_s",
                "zero_proof_timeout_s", "max_heading_departure_deg", "max_trial_path_m",
                "amcl_update_min_d_m", "amcl_update_min_a_rad", "side_front_margin_m", "motion_sources_timeout_s"):
        if finite(engineering.get(key), key) <= 0:
            raise ValueError("Positive explicit engineering budget required: " + key)
    if (not 0 <= finite(engineering.get("zero_speed_rpm")) <= .2 or
            type(engineering.get("zero_sample_count")) is not int or engineering["zero_sample_count"] != 5
            or engineering.get("allow_in_place_rotation") is not False
            or engineering.get("require_nonnegative_physical_wheels") is not True):
        raise ValueError("Explicit half-metre zero-speed/forward-wheel boundaries differ")
    if finite(engineering.get("costmap_transform_tolerance_s")) != .50:
        raise ValueError("Explicit trial costmap TF tolerance must match the existing 0.50s source gate")
    geometry = config.get("geometry") or {}
    for key in ("wheel_diameter_m", "wheel_separation_m", "planning_width_m",
                "front_from_axle_m", "rear_from_axle_m"):
        if finite(geometry.get(key), key) <= 0:
            raise ValueError("Positive geometry estimate required")
    for key, expected_value in (("laser_x_m", .13), ("laser_y_m", 0.), ("laser_z_m", .2), ("laser_yaw_rad", 0.)):
        if finite(geometry.get(key), key) != expected_value:
            raise ValueError("Seed/scan helper mounting assumptions differ: " + key)
    onsite = config.get("onsite_confirmation") or {}
    for key in ("robot_stopped", "original_map_room", "unoccupied",
                "front_and_surroundings_clear", "operator_present_with_manual_stop",
                "external_half_metre_floor_mark_available"):
        if onsite.get(key) is not True:
            raise ValueError("Current attended-trial confirmation missing: " + key)
    return deepcopy(config)


def trial_planner_parameters(names, config):
    # Reuse only costmap/GlobalPlanner parameters, never the preview's zero limits.
    params = planner_parameters(names, dict(geometry=config["geometry"]))
    user, engineering = config["user"], config["engineering"]
    params.update(controller_frequency=10., planner_frequency=1., planner_patience=3.,
                  controller_patience=2., oscillation_timeout=8., oscillation_distance=.03)
    params["DWAPlannerROS"].update(max_vel_x=user["max_linear_m_s"], min_vel_x=0.,
        max_vel_trans=user["max_linear_m_s"], min_vel_trans=0.,
        max_vel_theta=math.radians(user["max_angular_deg_s"]), min_vel_theta=0.,
        acc_lim_x=.10, acc_lim_y=0., acc_lim_trans=.10, acc_lim_theta=.30,
        xy_goal_tolerance=user["position_tolerance_m"], yaw_goal_tolerance=math.pi,
        latch_xy_goal_tolerance=True, trans_stopped_vel=.003, theta_stopped_vel=.01,
        sim_time=4., sim_granularity=.01, angular_sim_granularity=.02,
        vx_samples=12, vy_samples=1, vth_samples=21, path_distance_bias=32.,
        goal_distance_bias=20., occdist_scale=.05, forward_point_distance=.10,
        stop_time_buffer=.5, scaling_speed=.04, max_scaling_factor=.10,
        oscillation_reset_dist=.03, oscillation_reset_angle=.20)
    # This trial has no in-place phase. A small sampled forward minimum avoids
    # warming solely on rotate/zero candidates; no command is clamped later.
    params["DWAPlannerROS"].update(min_vel_x=.006, min_vel_trans=.006)
    for name in ("global_costmap", "local_costmap"):
        params[name]["transform_tolerance"] = engineering["costmap_transform_tolerance_s"]
    return params


class ScanAgreementFault(ValueError):
    def __init__(self, scores):
        super().__init__("Current scan/map agreement requires >=60 endpoints and >=0.65 wall ratio on every scan")
        self.scores = deepcopy(scores)


def diagnostic_scan_agreement(map_core, info, cells, current_scans, pose):
    """Same 60/.65 test as the frozen helper, retaining all rejected scores."""
    scores = [map_core.scan_match(info, cells, scan["ranges"], scan["angle_min"],
        scan["angle_increment"], scan["range_min"], scan["range_max"], pose) for scan in current_scans]
    if any(score.get("used", 0) < 60 or score.get("near_wall_ratio", 0.) < .65 for score in scores):
        raise ScanAgreementFault(scores)
    return scores


def scan_failure_evidence(error, current_scans, raw_pose, transform, checked_pose, *, now_ros, now, phase, frames):
    return dict(reason=str(error), scores=deepcopy(error.scores),
        thresholds=dict(minimum_endpoints=60, minimum_near_wall_ratio=.65),
        raw_scans=deepcopy(current_scans), raw_pose=deepcopy(raw_pose),
        tf=deepcopy(transform), checked_pose=list(checked_pose),
        observed_ros_s=now_ros, observed_monotonic_s=now, phase=phase,
        map_id=MAP_ID, frames=dict(frames), source_time_rewritten=False)


def validate_moving_pose(raw_pose, tf_data, *, now_ros, seed_ros, map_frame, allow_waiting_tf=False):
    """An old TF can only describe unready zero-target ARM pipeline state."""
    if not allow_waiting_tf or now_ros - finite(tf_data["source_stamp"]) <= .50:
        return quality_sample(raw_pose, tf_data, now_ros=now_ros, seed_ros=seed_ros, map_frame=map_frame)
    # Check AMCL independently when only the earlier odom->base TF may be old.
    # Keep its true stamp: this result must remain unready until a new real TF.
    header = raw_pose["header"]
    key = exact_stamp(header)
    stamp = key[0] + key[1] / 1e9
    if header.get("frame_id") != map_frame or not seed_ros < stamp <= now_ros or now_ros - stamp >= 2.:
        raise ValueError("AMCL needs a current original post-seed source")
    covariance = [finite(value) for value in raw_pose["covariance"]]
    if (len(covariance) != 36 or any(covariance[index] < 0 for index in (0, 7, 35))
            or max(covariance[0], covariance[7]) > .10 ** 2 or covariance[35] > math.radians(5.) ** 2):
        raise ValueError("AMCL position/yaw covariance exceeds diagnostic limits")
    point = [finite(raw_pose["position"][name]) for name in ("x", "y")]
    yaw = yaw_from_quaternion(raw_pose["orientation"])
    translation_error = math.hypot(finite(tf_data["translation"]["x"]) - point[0], finite(tf_data["translation"]["y"]) - point[1])
    tf_yaw = yaw_from_quaternion(tf_data["rotation"])
    yaw_error = abs(math.atan2(math.sin(tf_yaw - yaw), math.cos(tf_yaw - yaw)))
    if translation_error > .03 or yaw_error > .05:
        raise ValueError("Current map->base TF differs from this AMCL pose")
    return dict(source_key=key, original_source_stamp=stamp, pose=point + [yaw],
        covariance_position_std_m=[math.sqrt(covariance[0]), math.sqrt(covariance[7])],
        covariance_yaw_std_deg=math.degrees(math.sqrt(covariance[35])),
        tf_translation_error_m=translation_error, tf_yaw_error_rad=yaw_error,
        tf_original_source_stamp=tf_data["source_stamp"], awaiting_post_arm_tf=True)


def within_position_tolerance(pose, goal, tolerance):
    return math.hypot(finite(pose[0]) - finite(goal[0]), finite(pose[1]) - finite(goal[1])) <= finite(tolerance)


def heading_departure(pose, start):
    return abs(math.atan2(math.sin(pose[2] - start[2]), math.cos(pose[2] - start[2])))


def prearm_command_diagnostic(command, now, config):
    if command is None or not 0 <= now - command["received_monotonic_s"] < config["engineering"]["command_timeout_s"]:
        return dict(ready=False, reason="missing_or_expired_original_command")
    from base_trial_adapter import quantize_forward
    try:
        output = quantize_forward(command, max_linear_m_s=config["user"]["max_linear_m_s"],
            max_angular_deg_s=config["user"]["max_angular_deg_s"],
            diameter_m=config["geometry"]["wheel_diameter_m"],
            separation_m=config["geometry"]["wheel_separation_m"])
        wheels = output["physical_wheels"]
        ready = wheels["left_rpm"] >= 0 and wheels["right_rpm"] >= 0 and max(wheels.values()) > 0
        reason = ("expressible_forward_integer_command" if ready else
                  "raw_dwa_zero" if command["linear_mps"] == 0 and command["angular_rad_s"] == 0 else "quantized_to_zero")
        return dict(ready=ready, reason=reason, quantization=output)
    except (ValueError, RuntimeError) as exc:
        return dict(ready=False, reason="unsupported_forward_trial_command", error_type=type(exc).__name__, error=str(exc))


def prearm_command_ready(command, now, config):
    return prearm_command_diagnostic(command, now, config)["ready"]


def begin_preparation_nomotion(state):
    """Caller holds the short RAM lock; never run ROS while holding it."""
    if (state["phase"] != "waiting_ready" or state["nomotion_finished"] or
            state["nomotion_inflight"] or state["ready_handoff_reserved"]):
        return False
    state["prepared"], state["nomotion_inflight"] = False, True
    return True


def require_post_stop_sources(quality, stopped_status_monotonic_s):
    """Require independently new AMCL/TF/scan sources after physical parking."""
    boundary = finite(stopped_status_monotonic_s)
    for key in ("amcl_source_monotonic_s", "tf_source_monotonic_s", "oldest_scan_source_monotonic_s"):
        if finite(quality[key]) <= boundary:
            raise ValueError("Final parked localization source predates STOP/status: " + key)
    return quality


def health_sources_current(health, now, engineering):
    if health is None:
        return False
    return bool(all(0 <= now - stamp < timeout for stamp, timeout in
        ((health["odom_source_monotonic_s"], engineering["odom_timeout_s"]),
         (health["oldest_scan_source_monotonic_s"], engineering["scan_timeout_s"]),
         (health["amcl_source_monotonic_s"], engineering["amcl_timeout_s"])))
        and -.30 <= now - health["tf_source_monotonic_s"] <= .50)


def waiting_only_for_arm_tf(health, now, engineering, health_at):
    return bool(health is not None
        and health_at is not None and 0 <= now - health_at < engineering["supervisor_timeout_s"]
        and all(0 <= now - stamp < timeout for stamp, timeout in
            ((health["odom_source_monotonic_s"], engineering["odom_timeout_s"]),
             (health["oldest_scan_source_monotonic_s"], engineering["scan_timeout_s"]),
             (health["amcl_source_monotonic_s"], engineering["amcl_timeout_s"])))
        and now - health["tf_source_monotonic_s"] > .50)


def health_deadline_diagnostic(health, now, engineering, health_at):
    """Scalar fault snapshot taken in the owner's tick, not later ROS data."""
    heartbeat_age = None if health_at is None else now - health_at
    row = dict(checked_monotonic_s=now, health_at_monotonic_s=health_at,
        heartbeat_age_s=heartbeat_age, heartbeat_timeout_s=engineering["supervisor_timeout_s"],
        heartbeat_current=heartbeat_age is not None and 0 <= heartbeat_age < engineering["supervisor_timeout_s"],
        health_present=health is not None, sources={})
    if health is not None:
        row["health_observed_monotonic_s"] = health["observed_monotonic_s"]
        row["amcl_source_key"] = list(health["source_key"])
        for name, key, lower, upper, inclusive in (
            ("odom", "odom_source_monotonic_s", 0., engineering["odom_timeout_s"], False),
            ("oldest_scan", "oldest_scan_source_monotonic_s", 0., engineering["scan_timeout_s"], False),
            ("amcl", "amcl_source_monotonic_s", 0., engineering["amcl_timeout_s"], False),
            ("tf", "tf_source_monotonic_s", -.30, .50, True)):
            source, age = health[key], now - health[key]
            row["sources"][name] = dict(source_monotonic_s=source, age_s=age,
                minimum_age_s=lower, maximum_age_s=upper, maximum_inclusive=inclusive,
                current=lower <= age <= upper if inclusive else lower <= age < upper)
    row["sources_current"] = health_sources_current(health, now, engineering)
    return row


def stop_owned_child(name, child, timeout_s=4.):
    """SIGINT only this Popen child; preserve an already unexpected exit."""
    was_running = child.poll() is None
    row = dict(name=name, pid=child.pid, was_running=was_running,
               exit_code=child.poll(), forced_kill=False, expected_termination=False, errors=[])
    try:
        if was_running:
            child.send_signal(signal.SIGINT)
            row["signal"] = "INT"
            try:
                child.wait(timeout=timeout_s)
            except subprocess.TimeoutExpired:
                row["errors"].append("Owned child ignored SIGINT")
                child.terminate()
                row["signal"] = "TERM"
                try:
                    child.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    child.kill()
                    row["forced_kill"] = True
                    child.wait(timeout=2.)
        row["exit_code"] = child.wait(timeout=.1)
        row["expected_termination"] = bool(was_running and row["exit_code"] in (0, -signal.SIGINT)
                                            and not row["errors"] and not row["forced_kill"])
        if not row["expected_termination"]:
            row["errors"].append("Unexpected child exit: " + str(row["exit_code"]))
    except Exception as exc:
        row["errors"].append(type(exc).__name__ + ": " + str(exc))
    return row


class TrialCommandState:
    """Small serialized RAM handoff: producer receipt time is never renewed."""
    def __init__(self, task_id, expected_caller, max_linear=.03, max_angular_deg=6.9):
        self.task_id, self.expected_caller = task_id, expected_caller
        self.max_linear, self.max_angular = max_linear, math.radians(max_angular_deg)
        self.sequence, self.latest, self.armed_at = 0, None, None
        self.first_failure = None

    def accept(self, linear, angular, received_monotonic, caller, latched=False):
        received = finite(received_monotonic)
        linear, angular = finite(linear), finite(angular)
        if (caller != self.expected_caller or latched or linear < 0 or linear > self.max_linear
                or abs(angular) > self.max_angular):
            self.first_failure = self.first_failure or dict(reason="Invalid DWA command source/envelope",
                linear_mps=linear, angular_rad_s=angular, caller=caller, latched=latched,
                received_monotonic_s=received)
            return False
        self.sequence += 1
        self.latest = dict(task_id=self.task_id, sequence=self.sequence,
            received_monotonic_s=received, linear_mps=linear, angular_rad_s=angular)
        return True

    def arm(self, now):
        if self.armed_at is not None:
            raise RuntimeError("The trial cannot arm twice")
        self.armed_at = finite(now)

    def command(self):
        if (self.latest is None or self.armed_at is None or
                self.latest["received_monotonic_s"] < self.armed_at):
            return None
        return dict(self.latest)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-boot-id", required=True)
    parser.add_argument("--nonce", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--execute-half-metre-trial", action="store_true", required=True)
    args = parser.parse_args(argv)
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if boot != args.expected_boot_id:
        raise RuntimeError("Fresh RK boot identity differs")
    config = validate_trial_config(loads_config((ROOT / "trial_config.json").read_text()))
    user, eng, geometry = config["user"], config["engineering"], config["geometry"]
    manifest = loads_config((ROOT / "trial_package_manifest.json").read_text())
    for relative, expected in manifest["files"].items():
        path = ROOT / relative
        path.resolve().relative_to(ROOT.resolve())
        if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            raise RuntimeError("Trial package startup bytes differ: " + relative)
    # One local run claim; no live-data age or byte hash is used as a motor lease.
    with (ROOT / "trial_launch_claim.json").open("x") as out:
        json.dump(dict(boot_id=boot, nonce=args.nonce, started_monotonic_s=time.monotonic()), out)
    modules = {key: checked_module("deps/" + key + ".py", manifest["files"]["deps/" + key + ".py"], args.nonce)
               for key in ("control_client", "core", "rk_link", "stop_observation", "transport")}
    map_core = checked_module("deps/maploc_core.py", manifest["files"]["deps/maploc_core.py"], args.nonce)
    map_doc = loads_config((ROOT / "maps/map.json").read_text())
    info, cells, map_identity = verified_map(map_doc, (ROOT / "maps/map.pgm").read_bytes(), MAP_ID)
    amcl_parameters = loads_config((ROOT / "amcl_base.json").read_text())
    if any(key.startswith("initial_pose_") for key in amcl_parameters):
        raise RuntimeError("AMCL must not contain a historical pose")
    from base_trial_adapter import run_trial_session
    from trial_obstacle_gate import evaluate_forward_scan
    import rospy
    import tf2_ros
    import actionlib
    from nav_msgs.msg import Odometry, OccupancyGrid
    from nav_msgs.srv import GetPlan, GetPlanRequest
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
    from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
    from std_srvs.srv import Empty
    from localization_ros_adapter import source_header

    names = names_for_nonce(args.nonce)
    ns, frames, topics = names["namespace"], names["frames"], names["topics"]
    nav_ns, nav_node = ns + "/trial_navigation", ns + "/trial_navigation/move_base"
    cmd_topic = ns + "/trial_cmd_vel"
    planner_names = dict(frames=frames, map_topic=topics["map"], scan_topic=topics["scan_front180"], odom_topic=topics["wheel_odom"])
    planner_params = trial_planner_parameters(planner_names, config)
    amcl_parameters.update(global_frame_id=frames["map"], odom_frame_id=frames["odom"], base_frame_id=frames["base"],
        use_map_topic=True, first_map_only=True, save_pose_rate=-1., tf_broadcast=True,
        update_min_d=eng["amcl_update_min_d_m"], update_min_a=eng["amcl_update_min_a_rad"])
    rospy.init_node("p2p_trial_runtime_" + args.nonce, disable_signals=True)
    if rospy.get_param("/use_sim_time", False) or rospy.has_param(ns):
        raise RuntimeError("Actual ROS time and fresh trial namespace required")
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    seed_path, snapshot_path = output.with_name("seed_request.json"), output.with_name("scan_snapshot.json")
    if seed_path.exists():
        raise RuntimeError("Seed file predates this current localization trial")
    task_id = "half_metre_" + args.nonce
    commands = TrialCommandState(task_id, nav_node, user["max_linear_m_s"], user["max_angular_deg_s"])
    lock, finished, manual_cancel = threading.RLock(), threading.Event(), threading.Event()
    mailbox, scans, poses = deque(maxlen=1), deque(maxlen=8), deque(maxlen=128)
    prearm_commands = deque(maxlen=2048)
    children, logs, subscriptions = [], [], []
    failures, phase_history, quality_rejections = [], [], []
    state = dict(phase="initial_stop", phase_at=time.monotonic(), anchor=None, anchor_latest=None,
        anchor_enabled=False, owner_latest=None, map_verified=False, seed_consumed=False,
        seed_ros=None, seed_receipt=None, proposal_written=False, joint=None,
        prepared=False, health=None, health_at=None, goal_sent=False, goal_sent_at=None,
        start_pose=None, goal_pose=None, arrived=False, action_status=None, action_text=None,
        odom_published=0, path=None, nomotion_inflight=False, nomotion_finished=False,
        final_pose=None, first_bad_pose=None, latest_obstacle=None, final_post_stop_health=None,
        goal_cancelled=False, post_stop_status_monotonic_s=None, post_stop_request_ros_s=None,
        post_stop_last_rejection=None, pre_stop_amcl_source_key=None,
        navigation_started=False, navigation_started_at=None, preparation_attempted=False,
        preparation_evidence=None, navigation_start_attempted=False, post_plan_refresh=None,
        ready_handoff_reserved=False, prearm_command_count=0, first_bad_scan=None)
    result = dict(schema="p2p-half-metre-trial-result-v1", boot_id=boot, nonce=args.nonce,
        task_id=task_id, map_id=MAP_ID, frames=frames, topics=topics, trial_configuration=config,
        motion_requested=True, physical_trial_enabled=True, completed=False,
        firmware_modified=False, heading_acceptance_required=False, base_result=None,
        started_monotonic_s=time.monotonic(), saved_map_identity=map_identity)
    result["trial_costmap_transform_tolerance_s"] = eng["costmap_transform_tolerance_s"]
    for number in (signal.SIGINT, signal.SIGTERM):
        signal.signal(number, lambda *_: manual_cancel.set())
    odom_publisher = rospy.Publisher(topics["wheel_odom"], Odometry, queue_size=1)
    seed_publisher = rospy.Publisher(topics["initialpose"], PoseWithCovarianceStamped, queue_size=1)
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.))
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    action = actionlib.SimpleActionClient(nav_node, MoveBaseAction)

    def fail(reason, details=None):
        with lock:
            if not failures:
                failures.append(dict(reason=str(reason), at_monotonic_s=time.monotonic(), details=json_safe(details)))

    def scan_received(message):
        ros, mono = rospy.Time.now().to_sec(), time.monotonic()
        try:
            header = source_header(message.header)
            key = exact_stamp(header)
            if header["frame_id"] != frames["laser"]:
                raise ValueError("Trial laser frame differs")
            source = key[0] + key[1] / 1e9
            if not 0 <= ros - source < eng["scan_timeout_s"]:
                return
            value = dict(header=header, ranges=list(message.ranges), angle_min=message.angle_min,
                angle_increment=message.angle_increment, range_min=message.range_min,
                range_max=message.range_max, source_monotonic_s=mono - (ros - source))
            with lock:
                if not scans or key > exact_stamp(scans[-1]["header"]):
                    scans.append(value)
        except Exception as exc:
            fail("scan_callback: " + str(exc))

    def pose_received(message):
        # Capture all 36 raw covariance entries BEFORE any validity check.
        try:
            raw = dict(header=source_header(message.header), covariance=list(message.pose.covariance),
                position={key: getattr(message.pose.pose.position, key) for key in ("x", "y", "z")},
                orientation={key: getattr(message.pose.pose.orientation, key) for key in ("x", "y", "z", "w")},
                received_monotonic_s=time.monotonic(), received_ros_s=rospy.Time.now().to_sec())
            with lock:
                if not poses or exact_stamp(raw["header"]) > exact_stamp(poses[-1]["header"]):
                    poses.append(raw)
        except Exception as exc:
            fail("AMCL_callback: " + str(exc))

    def map_received(message):
        try:
            q = message.info.origin.orientation
            metadata = dict(width=message.info.width, height=message.info.height, resolution=message.info.resolution,
                origin=[message.info.origin.position.x, message.info.origin.position.y,
                        yaw_from_quaternion(dict(x=q.x, y=q.y, z=q.z, w=q.w))])
            if message.header.frame_id != frames["map"] or map_core.map_identity(metadata, list(message.data)) != MAP_ID:
                raise ValueError("Current private map identity/frame differs")
            state["map_verified"] = True
        except Exception as exc:
            fail("map_callback: " + str(exc))

    def velocity_received(message):
        received = time.monotonic()  # Preserve callback entry; do not renew later.
        header = getattr(message, "_connection_header", {}) or {}
        try:
            if any(abs(finite(value)) > 1e-9 for value in (message.linear.y, message.linear.z, message.angular.x, message.angular.y)):
                raise ValueError("Unsupported nonplanar command")
            with lock:
                accepted = commands.accept(message.linear.x, message.angular.z, received, header.get("callerid"), header.get("latching") == "1")
                if state["phase"] in ("initial_stop", "waiting_ready", "arming", "waiting_motion_sources"):
                    state["prearm_command_count"] += 1
                    value = dict(received_monotonic_s=received, phase=state["phase"], caller=header.get("callerid"),
                        latching=header.get("latching"), sequence=commands.sequence,
                        linear={key: getattr(message.linear, key) for key in ("x", "y", "z")},
                        angular={key: getattr(message.angular, key) for key in ("x", "y", "z")},
                        accepted_by_source_check=accepted)
                    value["readiness"] = (prearm_command_diagnostic(commands.latest, received, config) if accepted else
                                           dict(ready=False, reason="invalid_source_or_raw_envelope"))
                    cached_health = state["health"]
                    if cached_health is not None:
                        value["cached_supervisor_sources_at_receipt"] = dict(
                            tf_source_monotonic_s=cached_health["tf_source_monotonic_s"],
                            tf_age_s=received-cached_health["tf_source_monotonic_s"],
                            amcl_source_monotonic_s=cached_health["amcl_source_monotonic_s"],
                            amcl_age_s=received-cached_health["amcl_source_monotonic_s"],
                            odom_source_monotonic_s=cached_health["odom_source_monotonic_s"],
                            odom_age_s=received-cached_health["odom_source_monotonic_s"],
                            costmap_transform_tolerance_s=planner_params["global_costmap"]["transform_tolerance"],
                            is_cached_supervisor_evidence_not_dwa_internal_reason=True)
                    prearm_commands.append(value)
                if commands.first_failure:
                    fail("DWA_command_invalid", commands.first_failure)
        except Exception as exc:
            fail("DWA_command_callback: " + str(exc))

    def on_phase(phase, now):
        # Called by the physical owner: scalar/RAM state only, never ROS or disk.
        with lock:
            state["phase"], state["phase_at"] = phase, now
            phase_history.append(dict(phase=phase, at_monotonic_s=now))
            if phase == "waiting_ready":
                state["anchor_enabled"] = True
                state["anchor"] = StaticLocalizationAnchor(boot_id=boot, owner_id=args.nonce,
                    reference_after_monotonic_s=now, source_timeout_s=user["feedback_timeout_s"],
                    max_encoder_change_counts=0, max_encoder_travel_counts=0,
                    max_position_change_m=.002, max_yaw_change_rad=.003,
                    max_linear_speed_m_s=.002, max_angular_speed_rad_s=.01, max_wheel_speed_rpm=eng["zero_speed_rpm"])
            if phase in ("arming", "waiting_motion_sources", "armed", "normal_quiesce", "final_stop", "post_stop_observation", "finished"):
                state["anchor_enabled"] = False
                state["nomotion_finished"] = True
            if phase == "armed":
                commands.arm(now)
            if phase == "normal_quiesce" and state["health"] is not None:
                state["pre_stop_amcl_source_key"] = list(state["health"]["source_key"])

    def odom_sink(observation, packet, request_start, completed):
        # Same-owner raw/odom pairing before the lossy ROS mailbox.
        with lock:
            state["owner_latest"] = dict(observation)
            if state["anchor_enabled"] and state["anchor"] is not None:
                state["anchor_latest"] = state["anchor"].observe(packet, observation,
                    source_monotonic_s=observation["source_monotonic_s"], now_monotonic_s=completed,
                    boot_id=boot, owner_id=args.nonce)
        mailbox.append(dict(observation))

    def supervisor(now, phase):
        with lock:
            if failures:
                return dict(ready=False, complete=False, failure=failures[0]["reason"])
            fresh = state["health_at"] is not None and 0 <= now - state["health_at"] < eng["supervisor_timeout_s"]
            health = state["health"]
            fresh = bool(fresh and health_sources_current(health, now, eng))
            if phase in ("arming", "waiting_motion_sources") and waiting_only_for_arm_tf(health, now, eng, state["health_at"]):
                return dict(ready=False, complete=False, failure=None, readiness_reason="awaiting_post_arm_tf",
                    tf_source_monotonic_s=health["tf_source_monotonic_s"], odom_source_monotonic_s=health["odom_source_monotonic_s"])
            if phase in ("arming", "waiting_motion_sources", "armed") and not fresh:
                diagnostic = health_deadline_diagnostic(health, now, eng, state["health_at"])
                diagnostic["phase"] = phase
                fail("ROS supervisor source expired", diagnostic)
                return dict(ready=False, complete=False, failure="ROS supervisor source expired")
            ready = bool(state["prepared"] and fresh and state["health"] and not state["nomotion_inflight"])
            if phase == "waiting_ready" and ready:
                # Reserve only the immediate owner handoff after DWA is truly
                # expressible; this prevents a new service from racing ARM.
                state["ready_handoff_reserved"] = True
            return dict(ready=ready,
                        complete=state["arrived"], failure=None,
                        tf_source_monotonic_s=health["tf_source_monotonic_s"] if health else None,
                        odom_source_monotonic_s=health["odom_source_monotonic_s"] if health else None)

    def command_provider(_now):
        with lock:
            return commands.command()

    def post_stop_observer(now, proof):
        # The sole owner has already physically parked and continues only real
        # query/odom sources while the ROS worker observes the final pose.
        with lock:
            state["post_stop_status_monotonic_s"] = proof["stopped_status"]["completed_monotonic_s"]
            return dict(complete=health_sources_current(state["final_post_stop_health"], now, eng),
                        failure=failures[0]["reason"] if failures else None)

    def cancelled():
        return manual_cancel.is_set() or rospy.is_shutdown() or bool(failures)

    def publish_odometry():
        while not finished.is_set():
            try:
                value = mailbox.popleft()
            except IndexError:
                finished.wait(.005)
                continue
            try:
                msg = Odometry()
                msg.header.frame_id, msg.child_frame_id = frames["odom"], frames["base"]
                msg.header.stamp = rospy.Time.from_sec(rospy.Time.now().to_sec() - (time.monotonic() - value["source_monotonic_s"]))
                msg.header.seq = value["sample_sequence"]
                msg.pose.pose.position.x, msg.pose.pose.position.y = value["x"], value["y"]
                msg.pose.pose.orientation.z, msg.pose.pose.orientation.w = math.sin(value["yaw"] / 2.), math.cos(value["yaw"] / 2.)
                msg.twist.twist.linear.x, msg.twist.twist.angular.z = value["linear_x"], value["angular_z"]
                for index in (0, 7, 14, 21, 28, 35):
                    msg.pose.covariance[index] = msg.twist.covariance[index] = 1e3
                odom_publisher.publish(msg)
                state["odom_published"] += 1
            except Exception as exc:
                fail("odom_publication: " + str(exc))

    def current_data():
        with lock:
            return (deepcopy(poses[-1]) if poses else None, list(scans),
                    deepcopy(state["owner_latest"]), state["phase"])

    def pose_and_sources(require_anchor=False):
        raw, current_scans, odom, phase = current_data()
        now_ros, now = rospy.Time.now().to_sec(), time.monotonic()
        if raw is None or state["seed_ros"] is None or odom is None:
            raise ValueError("Current seeded AMCL/measured odometry missing")
        if not 0 <= now - odom["source_monotonic_s"] < eng["odom_timeout_s"]:
            raise ValueError("Measured owner odometry source expired")
        if len(current_scans) < 3:
            raise ValueError("Three current scans missing")
        for scan in current_scans[-3:]:
            if not 0 <= now - scan["source_monotonic_s"] < eng["scan_timeout_s"]:
                raise ValueError("Current scan source expired")
        tf = tf_buffer.lookup_transform(frames["map"], frames["base"], rospy.Time(0), rospy.Duration(0))
        q, t = tf.transform.rotation, tf.transform.translation
        tf_data = dict(source_stamp=tf.header.stamp.to_sec(), header=source_header(tf.header),
                       child_frame_id=tf.child_frame_id, translation=dict(x=t.x, y=t.y, z=t.z),
                       rotation=dict(x=q.x, y=q.y, z=q.z, w=q.w))
        try:
            quality = validate_moving_pose(raw, tf_data, now_ros=now_ros,
                seed_ros=state["seed_ros"], map_frame=frames["map"],
                allow_waiting_tf=phase in ("arming", "waiting_motion_sources"))
        except Exception as exc:
            payload = dict(reason=type(exc).__name__ + ": " + str(exc), raw_pose=raw,
                tf=tf_data, scans=[exact_stamp(scan["header"]) for scan in current_scans[-3:]],
                observed_ros_s=now_ros, observed_monotonic_s=now, phase=phase)
            if len(quality_rejections) < 128:
                quality_rejections.append(json_safe(payload))
            if state["first_bad_pose"] is None:
                state["first_bad_pose"] = json_safe(payload)
            raise
        try:
            scores = diagnostic_scan_agreement(map_core, info, cells, current_scans[-3:], quality["pose"])
        except ScanAgreementFault as exc:
            if state["first_bad_scan"] is None:
                # Worker RAM only; serialize full raw ranges and nonfinite
                # encodings after the physical owner's STOP/port closure.
                state["first_bad_scan"] = scan_failure_evidence(exc, current_scans[-3:], raw, tf_data,
                    quality["pose"], now_ros=now_ros, now=now, phase=phase, frames=frames)
            raise
        obstacle = evaluate_forward_scan(current_scans[-1], now,
            max_source_age_s=eng["scan_timeout_s"], body_width_m=geometry["planning_width_m"],
            front_m=geometry["front_from_axle_m"], rear_m=geometry["rear_from_axle_m"],
            laser_x_m=geometry["laser_x_m"], laser_y_m=geometry["laser_y_m"], laser_yaw_rad=geometry["laser_yaw_rad"],
            front_clearance_m=user["front_clearance_m"], side_margin_m=eng["side_front_margin_m"],
            expected_frame_id=frames["laser"])
        state["latest_obstacle"] = obstacle
        if not obstacle["ready"]:
            raise ValueError("Independent obstacle stop: " + str(obstacle.get("failure")))
        if state["start_pose"] is not None and heading_departure(quality["pose"], state["start_pose"]) > math.radians(eng["max_heading_departure_deg"]):
            raise ValueError("Half-metre trial left the configured heading corridor")
        quality.update(scan_scores=scores, scan_source_keys=[exact_stamp(scan["header"]) for scan in current_scans[-3:]],
                       observed_monotonic_s=now, obstacle=obstacle,
                       odom_source_monotonic_s=odom["source_monotonic_s"],
                       oldest_scan_source_monotonic_s=current_scans[-3]["source_monotonic_s"],
                       amcl_source_monotonic_s=now - (now_ros - quality["original_source_stamp"]),
                       tf_source_monotonic_s=now - (now_ros - quality["tf_original_source_stamp"]),
                       amcl_received_monotonic_s=raw["received_monotonic_s"],
                       amcl_received_ros_s=raw["received_ros_s"])
        if require_anchor:
            with lock:
                anchor = state["anchor"].check(now_monotonic_s=time.monotonic(), boot_id=boot, owner_id=args.nonce)
            if not anchor["valid"]:
                raise ValueError("Pre-arm stationary anchor not current")
            quality["anchor"] = anchor
        return quality

    def start_navigation():
        if state["navigation_started"]:
            return
        state["navigation_start_attempted"] = True
        if finished.is_set() or failures:
            raise RuntimeError("Navigation startup cancelled")
        if (state["seed_ros"] is None or not state["joint"] or
                len(state["joint"].get("post_seed_five_observations", [])) != 5):
            raise RuntimeError("Planner startup requires this seed and five accepted sources")
        log = output.with_name("trial_move_base.log").open("w")
        logs.append(log)
        command = ["/opt/ros/noetic/lib/move_base/move_base", "__name:=move_base", "__ns:=" + nav_ns,
            "cmd_vel:=" + cmd_topic, "odom:=" + topics["wheel_odom"], "map:=" + topics["map"],
            "move_base_simple/goal:=" + nav_ns + "/unused_simple_goal"]
        child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=log, env=dict(os.environ))
        children.append(("move_base", child))
        state["navigation_started"], state["navigation_started_at"] = True, time.monotonic()
        result["navigation_start"] = dict(at_monotonic_s=state["navigation_started_at"],
            after_seed_post_five=deepcopy(state["joint"]), pid=child.pid,
            preseed_scan_costmap_initialization_allowed=False)

    def navigation_warmed():
        if not state["navigation_started"]:
            return False
        if time.monotonic() - state["navigation_started_at"] >= 15.:
            raise RuntimeError("Post-seed private planner startup timed out")
        live = state.get("costmap")
        if live is None:
            return False
        if live.header.frame_id != frames["map"]:
            raise ValueError("Private planner costmap uses a different frame")
        if not 0 <= rospy.Time.now().to_sec() - live.header.stamp.to_sec() < 2.:
            return False
        try:
            rospy.wait_for_service(nav_node + "/make_plan", timeout=.02)
        except rospy.ROSException:
            return False
        if not action.wait_for_server(rospy.Duration(.02)):
            return False
        graph = rospy.get_master().getSystemState()
        if graph[0] != 1 or dict(graph[2][0]).get(cmd_topic) != [nav_node]:
            raise RuntimeError("Post-seed private cmd publisher ownership differs")
        return True

    def refresh_after_plan():
        # Heavy planning/evidence IO runs with no action and no ARM. A real new
        # AMCL source must reconnect the completed plan to current localization.
        with lock:
            baseline = exact_stamp(poses[-1]["header"]) if poses else None
            state["prepared"] = False
        if baseline is None:
            raise RuntimeError("AMCL source missing after planning")
        began = time.monotonic()
        deadline, next_update = began + eng["amcl_timeout_s"], began
        while time.monotonic() < deadline:
            if finished.is_set() or manual_cancel.is_set() or failures or state["phase"] != "waiting_ready":
                raise RuntimeError("Post-plan source refresh cancelled before goal")
            if time.monotonic() >= next_update:
                next_update = time.monotonic() + .28
                state["nomotion_inflight"] = True
                try:
                    rospy.wait_for_service(topics["request_nomotion_update"], timeout=.1)
                    rospy.ServiceProxy(topics["request_nomotion_update"], Empty)()
                finally:
                    state["nomotion_inflight"] = False
            if time.monotonic() >= deadline:
                raise RuntimeError("Post-plan AMCL refresh exceeded the unchanged 2s budget")
            with lock:
                key = exact_stamp(poses[-1]["header"]) if poses else None
            if key is not None and key > baseline:
                quality = pose_and_sources(require_anchor=True)
                if (math.hypot(quality["pose"][0] - state["start_pose"][0], quality["pose"][1] - state["start_pose"][1]) > .03
                        or heading_departure(quality["pose"], state["start_pose"]) > .05):
                    raise ValueError("Fresh post-plan pose shifted from the single planned start")
                if time.monotonic() >= deadline or not health_sources_current(quality, time.monotonic(), eng):
                    raise ValueError("Post-plan validation finished with an expired original source")
                state["post_plan_refresh"] = dict(started_monotonic_s=began,
                    completed_monotonic_s=time.monotonic(), previous_source_key=baseline,
                    accepted_new_source_key=list(quality["source_key"]), current_quality=quality)
                return quality
            finished.wait(.02)
        raise RuntimeError("No distinct current AMCL source after planning within the unchanged 2s budget")

    def prepare_goal(joint):
        if state["preparation_attempted"]:
            raise RuntimeError("Only one make_plan preparation attempt is allowed")
        state["preparation_attempted"] = True
        start = list(joint["post_seed_five_observations"][-1]["pose"])
        goal = forward_goal(start, user["forward_m"])
        state["start_pose"], state["goal_pose"] = start, goal
        req = GetPlanRequest()
        for target, value in ((req.start, start), (req.goal, goal)):
            target.header.frame_id, target.header.stamp = frames["map"], rospy.Time.now()
            target.pose.position.x, target.pose.position.y = value[:2]
            target.pose.orientation.z, target.pose.orientation.w = math.sin(value[2] / 2.), math.cos(value[2] / 2.)
        req.tolerance = 0.
        with lock:
            live = state.get("costmap")
        if (live is None or live.header.frame_id != frames["map"] or
                not 0 <= rospy.Time.now().to_sec() - live.header.stamp.to_sec() < 2.):
            raise ValueError("Private planner costmap missing/stale/wrong frame")
        q = live.info.origin.orientation
        grid = dict(width=live.info.width, height=live.info.height, resolution=live.info.resolution,
            origin=[live.info.origin.position.x, live.info.origin.position.y,
                    yaw_from_quaternion(dict(x=q.x, y=q.y, z=q.z, w=q.w))], data=list(live.data))
        evidence = dict(schema="p2p-single-preparation-evidence-v1", attempt_count=1,
            started_monotonic_s=time.monotonic(), request_start=start, request_goal=goal,
            request_start_header=source_header(req.start.header), request_goal_header=source_header(req.goal.header),
            request_tolerance_m=req.tolerance, costmap_header=source_header(live.header), costmap=grid,
            costmap_origin_position=dict(x=live.info.origin.position.x, y=live.info.origin.position.y, z=live.info.origin.position.z),
            costmap_origin_orientation=dict(x=q.x, y=q.y, z=q.z, w=q.w),
            saved_map_identity=map_identity, returned_path=None, validation=None, failure=None)
        state["preparation_evidence"] = evidence
        atomic_json(output.with_name("trial_preparation_evidence.json"), evidence)
        try:
            plan = rospy.ServiceProxy(nav_node + "/make_plan", GetPlan)(req).plan
            points = []
            for value in plan.poses:
                p, q = value.pose.position, value.pose.orientation
                points.append(dict(header=source_header(value.header), frame_id=value.header.frame_id, x=p.x, y=p.y, z=p.z,
                                   orientation=dict(x=q.x, y=q.y, z=q.z, w=q.w)))
            evidence["returned_path"] = dict(header=source_header(plan.header), frame_id=plan.header.frame_id, poses=points)
            atomic_json(output.with_name("trial_preparation_evidence.json"), json_safe(evidence))
            checked = validate_path(dict(frame_id=plan.header.frame_id, poses=points), start, goal,
                                    dict(info, data=cells), grid, frames["map"])
            if checked["path_length_m"] > eng["max_trial_path_m"]:
                raise ValueError("Trial path exceeds this short-route bound")
            evidence["validation"] = checked
        except Exception as exc:
            evidence["failure"] = dict(error_type=type(exc).__name__, message=str(exc), at_monotonic_s=time.monotonic())
            atomic_json(output.with_name("trial_preparation_evidence.json"), json_safe(evidence))
            raise
        evidence["completed_monotonic_s"] = time.monotonic()
        atomic_json(output.with_name("trial_preparation_evidence.json"), json_safe(evidence))
        state["path"] = checked
        refreshed = refresh_after_plan()
        message = MoveBaseGoal()
        message.target_pose = req.goal
        message.target_pose.header.stamp = rospy.Time.now()  # New goal publication time, never an observation source.

        def done(status, _result):
            state["action_status"], state["action_text"] = int(status), action.get_goal_status_text()

        action.send_goal(message, done_cb=done)  # Pre-ARM commands are discarded by the owner.
        state["goal_sent"], state["goal_sent_at"] = True, time.monotonic()
        return refreshed

    def work():
        evidence = None
        last_key, next_nomotion, next_status = None, 0., 0.
        while not finished.is_set():
            try:
                if time.monotonic() >= next_status:
                    next_status = time.monotonic() + .5
                    # Diagnostic-only write. Failure cannot renew any source or
                    # delay the owner's STOP/control thread.
                    with lock:
                        status = dict(schema="p2p-half-metre-trial-status-v1", task_id=task_id,
                            boot_id=boot, nonce=args.nonce, diagnostic_written_monotonic_s=time.monotonic(),
                            phase=state["phase"], current_goal=state["goal_pose"],
                            original_amcl=deepcopy(poses[-1]) if poses else None,
                            measured_odometry=deepcopy(state["owner_latest"]),
                            original_dwa_command=deepcopy(commands.latest),
                            failure=deepcopy(failures[0]) if failures else None,
                            diagnostic_only=True)
                    try:
                        atomic_json(output.with_name("trial_status.json"), json_safe(status))
                    except Exception as exc:
                        result["status_write_failure"] = type(exc).__name__ + ": " + str(exc)
                for name, child in children:
                    if child.poll() is not None:
                        raise RuntimeError("Owned ROS child exited: " + name + "/" + str(child.returncode))
                with lock:
                    phase = state["phase"]
                    anchor = state["anchor_latest"]
                if phase in ("normal_quiesce", "final_stop", "post_stop_observation", "finished"):
                    if state["goal_sent"] and not state["goal_cancelled"]:
                        action.cancel_goal()
                        state["goal_cancelled"] = True
                if phase in ("final_stop", "finished"):
                    finished.wait(.02)
                    continue
                if phase == "post_stop_observation":
                    boundary = state["post_stop_status_monotonic_s"]
                    if boundary is None:
                        finished.wait(.005)
                        continue
                    if state["post_stop_request_ros_s"] is None:
                        state["post_stop_request_ros_s"] = rospy.Time.now().to_sec()
                        result["post_stop_nomotion"] = dict(requested_ros_s=state["post_stop_request_ros_s"], once_only=True, done=False)
                        try:
                            rospy.wait_for_service(topics["request_nomotion_update"], timeout=.5)
                            rospy.ServiceProxy(topics["request_nomotion_update"], Empty)()
                            result["post_stop_nomotion"]["done"] = True
                        except Exception as exc:
                            fail("Post-STOP one-shot AMCL update failed: " + str(exc))
                    try:
                        health = require_post_stop_sources(pose_and_sources(require_anchor=False), boundary)
                        if (state["pre_stop_amcl_source_key"] is None or
                                health["source_key"] <= state["pre_stop_amcl_source_key"]):
                            raise ValueError("Waiting for a different AMCL source after physical parking")
                        if not within_position_tolerance(health["pose"], state["goal_pose"], user["position_tolerance_m"]):
                            raise ValueError("Fresh stopped pose is outside the user position tolerance")
                        with lock:
                            state["final_post_stop_health"], state["final_pose"] = health, health["pose"]
                    except Exception as exc:
                        state["post_stop_last_rejection"] = type(exc).__name__ + ": " + str(exc)
                    finished.wait(.02)
                    continue
                if phase == "waiting_ready" and not state["proposal_written"] and anchor and anchor["valid"] and anchor["accepted_samples"] >= 5:
                    current = current_three_scans(list(scans), now_ros=rospy.Time.now().to_sec(),
                        anchor_source_monotonic_s=anchor["anchor"]["source_monotonic_s"], laser_frame=frames["laser"])
                    atomic_json(snapshot_path, proposal_snapshot(current, boot_id=boot, nonce=args.nonce, frames=frames, anchor=anchor))
                    state["proposal_written"] = True
                if phase == "waiting_ready" and seed_path.is_file() and not state["seed_consumed"]:
                    state["seed_consumed"] = True
                    receipt = dict(accepted=False, published=False, received_monotonic_s=time.monotonic())
                    state["seed_receipt"] = receipt
                    try:
                        request = loads_config(seed_path.read_text())
                        pose = validate_seed_request(request, boot_id=boot, nonce=args.nonce,
                            expected_snapshot_keys=snapshot_keys(loads_config(snapshot_path.read_text())))
                        current = current_three_scans(list(scans), now_ros=rospy.Time.now().to_sec(),
                            anchor_source_monotonic_s=anchor["anchor"]["source_monotonic_s"], laser_frame=frames["laser"])
                        scores = scan_agreement(map_core, info, cells, current, pose)
                        with lock:
                            valid_anchor = state["anchor"].check(now_monotonic_s=time.monotonic(), boot_id=boot, owner_id=args.nonce)
                        if not valid_anchor["valid"]:
                            raise ValueError("Current seed anchor not valid")
                        message = PoseWithCovarianceStamped()
                        message.header.frame_id, message.header.stamp = frames["map"], rospy.Time.now()
                        message.pose.pose.position.x, message.pose.pose.position.y = pose[:2]
                        message.pose.pose.orientation.z, message.pose.pose.orientation.w = math.sin(pose[2] / 2.), math.cos(pose[2] / 2.)
                        message.pose.covariance[0] = message.pose.covariance[7] = .05 ** 2
                        message.pose.covariance[35] = .03 ** 2
                        state["seed_ros"] = message.header.stamp.to_sec()
                        seed_publisher.publish(message)
                        receipt.update(accepted=True, published=True, pose=pose, published_ros_s=state["seed_ros"],
                            scores=scores, anchor=valid_anchor, proposal_snapshot_stamps=request["proposal_snapshot_stamps"])
                        evidence = ConsecutiveLocalizationEvidence(state["seed_ros"], args.nonce)
                    except Exception as exc:
                        receipt["first_rejection"] = type(exc).__name__ + ": " + str(exc)
                        fail("seed_rejected", receipt)
                    atomic_json(output.with_name("seed_receipt.json"), json_safe(receipt))
                if state["seed_ros"] is not None:
                    if phase == "waiting_ready" and time.monotonic() >= next_nomotion:
                        with lock:
                            begin_update = begin_preparation_nomotion(state)
                        if begin_update:
                            next_nomotion = time.monotonic() + .28
                            try:
                                rospy.wait_for_service(topics["request_nomotion_update"], timeout=.1)
                                rospy.ServiceProxy(topics["request_nomotion_update"], Empty)()
                            finally:
                                with lock:
                                    state["nomotion_inflight"] = False
                    try:
                        quality = pose_and_sources(require_anchor=phase == "waiting_ready")
                        key = quality["source_key"]
                        if phase == "waiting_ready" and key != last_key:
                            last_key = key
                            joint = evidence.observe(quality, now_ros=rospy.Time.now().to_sec(), now_monotonic_s=time.monotonic())
                            state["joint"] = joint
                            if joint:
                                atomic_json(output.with_name("joint_evidence.json"), joint)
                                if not state["navigation_started"]:
                                    start_navigation()
                        if phase == "waiting_ready" and state["joint"] and not state["goal_sent"] and navigation_warmed():
                            # Planner starts after seed/five sources. Continue
                            # real nomotion updates while it warms, then recheck.
                            quality = pose_and_sources(require_anchor=True)
                            try:
                                quality = prepare_goal(state["joint"])
                            except Exception as exc:
                                fail("navigation_preparation: " + type(exc).__name__ + ": " + str(exc),
                                     dict(evidence_file=str(output.with_name("trial_preparation_evidence.json")),
                                          preparation_failure=(state["preparation_evidence"] or {}).get("failure")))
                                raise
                        if state["start_pose"] is not None and phase == "waiting_ready":
                            if (math.hypot(quality["pose"][0] - state["start_pose"][0], quality["pose"][1] - state["start_pose"][1]) > .03
                                    or heading_departure(quality["pose"], state["start_pose"]) > .05):
                                raise ValueError("Pre-arm pose shifted after the single goal was computed")
                        with lock:
                            state["health"], state["health_at"], state["final_pose"] = quality, time.monotonic(), quality["pose"]
                            command_ready = commands.latest is not None and time.monotonic() - commands.latest["received_monotonic_s"] < eng["command_timeout_s"]
                            expressible = phase != "waiting_ready" or prearm_command_ready(commands.latest, time.monotonic(), config)
                            state["prepared"] = bool(state["goal_sent"] and state["joint"] and command_ready and expressible and not state["nomotion_inflight"])
                            if phase == "armed" and within_position_tolerance(quality["pose"], state["goal_pose"], user["position_tolerance_m"]):
                                state["arrived"] = True
                        if state["arrived"]:
                            action.cancel_goal()
                        status = state["action_status"]
                        if status in (2, 4, 5, 8, 9) and not state["arrived"]:
                            raise RuntimeError("move_base ended before arrival: " + str(status) + "/" + str(state["action_text"]))
                        if status == 3 and not within_position_tolerance(quality["pose"], state["goal_pose"], user["position_tolerance_m"]):
                            raise RuntimeError("move_base success disagrees with the actual position tolerance")
                    except Exception as exc:
                        with lock:
                            state["prepared"] = False
                            state["health"] = None
                        if phase in ("arming", "waiting_motion_sources", "armed") or state["goal_sent"] or state["navigation_start_attempted"]:
                            fail("live_navigation_health: " + str(exc), dict(first_bad_pose=state["first_bad_pose"], obstacle=state["latest_obstacle"]))
                        elif evidence is not None:
                            evidence.reject()
                            state["joint"] = None
                finished.wait(.02)
            except Exception as exc:
                fail("ROS_supervisor: " + type(exc).__name__ + ": " + str(exc))
                finished.wait(.02)

    pub_thread = threading.Thread(target=publish_odometry, name="trial-odom-publisher", daemon=True)
    worker = threading.Thread(target=work, name="trial-ROS-supervisor", daemon=True)
    try:
        rospy.set_param(ns + "/map_server/frame_id", frames["map"])
        rospy.set_param(ns + "/amcl", amcl_parameters)
        rospy.set_param(nav_node, planner_params)
        subscriptions = [rospy.Subscriber(topics["map"], OccupancyGrid, map_received, queue_size=1),
            rospy.Subscriber(topics["scan_front180"], LaserScan, scan_received, queue_size=1, tcp_nodelay=True),
            rospy.Subscriber(topics["amcl_pose"], PoseWithCovarianceStamped, pose_received, queue_size=1, tcp_nodelay=True),
            rospy.Subscriber(cmd_topic, Twist, velocity_received, queue_size=1, tcp_nodelay=True),
            rospy.Subscriber(nav_node + "/global_costmap/costmap", OccupancyGrid,
                             lambda message: state.update(costmap=message), queue_size=1)]
        total_lifetime = eng["warmup_timeout_s"] + user["max_duration_s"] + 20.
        command_list = [
            ("map_server", ["/opt/ros/noetic/lib/map_server/map_server", str(ROOT / "maps/map.yaml"), "__name:=map_server", "__ns:=" + ns,
                "map:=" + topics["map"], "map_metadata:=" + ns + "/map_metadata", "static_map:=" + ns + "/static_map"]),
            ("amcl", ["/opt/ros/noetic/lib/amcl/amcl", "__name:=amcl", "__ns:=" + ns,
                "scan:=" + topics["scan_front180"], "map:=" + topics["map"], "amcl_pose:=" + topics["amcl_pose"],
                "initialpose:=" + topics["initialpose"], "request_nomotion_update:=" + topics["request_nomotion_update"],
                "static_map:=" + ns + "/static_map", "global_localization:=" + ns + "/global_localization", "set_map:=" + ns + "/set_map"]),
            ("adapter", [sys.executable, "-s", "-B", str(ROOT / "localization_ros_adapter.py"),
                "--scan-topic", "/lidar1/laser_fan", "--odom-topic", topics["wheel_odom"], "--scan-output-topic", topics["scan_front180"],
                "--amcl-topic", topics["amcl_pose"], "--laser-frame", frames["laser"], "--base-frame", frames["base"],
                "--odom-frame", frames["odom"], "--map-frame", frames["map"], "--scan-max-age-s", str(eng["scan_timeout_s"]),
                "--odom-max-age-s", str(eng["odom_timeout_s"]), "--amcl-max-age-s", str(eng["amcl_timeout_s"]),
                "--duration", str(total_lifetime), "--snapshot-scan-count", "0", "--output", str(output.with_name("trial_adapter_result.json"))])]
        for name, command in command_list:
            log = output.with_name("trial_" + name + ".log").open("w")
            logs.append(log)
            children.append((name, subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=log, env=dict(os.environ))))
        warm_deadline = time.monotonic() + 20.
        while time.monotonic() < warm_deadline:
            if failures or cancelled():
                raise RuntimeError("Trial startup cancelled/failed")
            if state["map_verified"] and len(scans) >= 3 and seed_publisher.get_num_connections() > 0:
                break
            time.sleep(.05)
        else:
            raise RuntimeError("Map/scan/AMCL seed connection did not warm")
        graph = rospy.get_master().getSystemState()
        if graph[0] != 1:
            raise RuntimeError("Current ROS graph unavailable")
        publishers = dict(graph[2][0])
        for topic, expected in ((topics["map"], ns + "/map_server"), (topics["amcl_pose"], ns + "/amcl")):
            if publishers.get(topic) != [expected]:
                raise RuntimeError("Private publisher ownership differs: " + topic)
        pub_thread.start()
        worker.start()
        target = modules["rk_link"].locate()
        result["target"] = target
        with modules["transport"].exclusive_port(target["port"]) as physical:
            result["base_result"] = run_trial_session(physical, modules["control_client"], modules["stop_observation"],
                lambda: modules["core"].MeasuredOdometry(geometry["wheel_diameter_m"], geometry["wheel_separation_m"], calibrated=False),
                task_id=task_id, supervisor=supervisor, command_provider=command_provider,
                cancelled=cancelled, on_phase=on_phase, odom_sink=odom_sink,
                post_stop_observer=post_stop_observer, post_stop_observation_timeout_s=2.,
                motion_sources_timeout_s=eng["motion_sources_timeout_s"],
                duration_s=user["max_duration_s"], warmup_timeout_s=eng["warmup_timeout_s"],
                feedback_timeout_s=user["feedback_timeout_s"], command_timeout_s=eng["command_timeout_s"],
                apply_timeout_s=eng["apply_timeout_s"], arm_timeout_s=eng["arm_timeout_s"],
                zero_proof_timeout_s=eng["zero_proof_timeout_s"], max_linear_m_s=user["max_linear_m_s"],
                zero_speed_rpm=eng["zero_speed_rpm"],
                max_angular_deg_s=user["max_angular_deg_s"], diameter_m=geometry["wheel_diameter_m"], separation_m=geometry["wheel_separation_m"])
        result["exclusive_port_closed_normally"] = True
        if not result["base_result"].get("trial_complete"):
            base_failure = (result["base_result"].get("first_failure") or
                            result["base_result"].get("final_stop_error") or
                            result["base_result"].get("post_stop_observation"))
            fail("Physical trial did not complete", base_failure)
        elif state["final_post_stop_health"] is None:
            fail("Final parked localization was not independently observed", state["post_stop_last_rejection"])
    except BaseException as exc:
        fail("trial_runtime: " + type(exc).__name__ + ": " + str(exc))
    finally:
        # run_trial_session has already attempted STOP before ROS shutdown.
        finished.set()
        if state["goal_sent"]:
            try:
                action.cancel_goal()
            except Exception as exc:
                fail("action cancellation cleanup: " + str(exc))
        result["children_cleanup"] = [stop_owned_child(name, child) for name, child in reversed(children)]
        for thread in (worker, pub_thread):
            if thread.ident is not None:
                thread.join(timeout=2.)
        result["worker_still_alive"], result["publication_thread_still_alive"] = worker.is_alive(), pub_thread.is_alive()
        for endpoint in subscriptions + [odom_publisher, seed_publisher]:
            try:
                endpoint.unregister()
            except Exception as exc:
                fail("ROS endpoint cleanup: " + str(exc))
        for log in logs:
            try:
                log.close()
            except Exception as exc:
                fail("Child log cleanup: " + str(exc))
        try:
            if rospy.has_param(ns):
                rospy.delete_param(ns)
            rospy.signal_shutdown("Bounded half-metre trial ended")
        except Exception as exc:
            fail("ROS namespace cleanup: " + str(exc))
    base = result.get("base_result") or {}
    final_pose = state["final_pose"]
    cleanup_ok = all(row["expected_termination"] and not row["forced_kill"] and not row["errors"] for row in result["children_cleanup"])
    result.update(first_failure=failures[0] if failures else None, seed_receipt=state["seed_receipt"],
        first_bad_pose=state["first_bad_pose"], quality_rejections=quality_rejections,
        first_bad_scan=json_safe(state["first_bad_scan"]),
        start_pose=state["start_pose"], goal_pose=state["goal_pose"], final_pose=final_pose,
        planner_path=state["path"], phase_history=phase_history, arrived_position=state["arrived"],
        latest_obstacle=json_safe(state["latest_obstacle"]), goal_sent=state["goal_sent"],
        original_covariance_history=json_safe(list(poses)), measured_odom_messages=state["odom_published"],
        final_health=json_safe(state["health"]), final_post_stop_health=json_safe(state["final_post_stop_health"]),
        post_stop_last_rejection=state["post_stop_last_rejection"],
        preparation_attempted=state["preparation_attempted"],
        post_plan_refresh=state["post_plan_refresh"],
        prearm_command_history=json_safe(list(prearm_commands)),
        prearm_command_history_limit=prearm_commands.maxlen,
        prearm_command_total=state["prearm_command_count"],
        prearm_command_history_dropped=max(0, state["prearm_command_count"] - len(prearm_commands)),
        preparation_evidence_file=str(output.with_name("trial_preparation_evidence.json")) if state["preparation_evidence"] else None,
        ended_monotonic_s=time.monotonic())
    result["completed"] = bool(base.get("trial_complete") and state["arrived"] and final_pose and state["final_post_stop_health"]
        and state["goal_pose"] and within_position_tolerance(final_pose, state["goal_pose"], user["position_tolerance_m"])
        and not failures and cleanup_ok and not worker.is_alive() and not pub_thread.is_alive())
    atomic_json(output, json_safe(result))
    print(json.dumps(dict(output=str(output), completed=result["completed"], first_failure=result["first_failure"],
                         motion_requested=True), allow_nan=False))
    _ = tf_listener
    return 0 if result["completed"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
