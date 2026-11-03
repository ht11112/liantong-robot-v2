"""Bounded saved-map localization observation with one permanent query-only owner.

No navigation, ARM or SETPOINT is supported. ROS and physical dependencies are
loaded only by main(). A seed is a one-shot, current-scan-checked proposal.
Final localization evidence is historical static evidence, never motion authority.
"""
import argparse
from collections import deque
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

from base_static_adapter import run_static_session
from config import loads_config
from localization_anchor import StaticLocalizationAnchor


ROOT = Path(__file__).resolve().parent
MAP_ID = "9e34ebc9f5a36fbbf64e3a6f7bbef4d23683cd97de1963bba00fe9f60507c746"
AGE_BUDGETS = dict(scan=.75, odom=.75, amcl=2.)
EXTRINSICS = [.13, 0., 0.]


def finite(value):
    try:
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("Finite built-in number required")
        return float(value)
    except OverflowError:
        raise ValueError("Finite built-in number required") from None


def names_for_nonce(nonce):
    if type(nonce) is not str or re.fullmatch(r"[A-Za-z0-9_]{6,48}", nonce) is None:
        raise ValueError("Nonce must have 6..48 letters, digits or underscores")
    ns = "/p2p_loc_" + nonce
    return dict(namespace=ns,
                frames={name: "p2p_" + nonce + "_" + name for name in ("map", "odom", "base", "laser")},
                topics={name: ns + "/" + name for name in
                        ("scan_front180", "amcl_pose", "wheel_odom", "initialpose", "map", "request_nomotion_update")})


def exact_stamp(header):
    seconds, nanos = header.get("stamp_secs"), header.get("stamp_nsecs")
    if (type(seconds) is not int or seconds < 0 or type(nanos) is not int
            or not 0 <= nanos < 1000000000):
        raise ValueError("Exact positive ROS scan stamp required")
    if seconds == 0 and nanos == 0:
        raise ValueError("Zero source stamp rejected")
    return [seconds, nanos]


def snapshot_keys(snapshot):
    scans = snapshot.get("snapshots") if type(snapshot) is dict else None
    if type(scans) is not list or len(scans) != 3 or snapshot.get("complete") is not True:
        raise ValueError("Exactly three completed proposal snapshots required")
    keys = [exact_stamp(scan["header"]) for scan in scans]
    if not keys[0] < keys[1] < keys[2]:
        raise ValueError("Proposal snapshot sources must strictly advance")
    return keys


def validate_seed_request(request, *, boot_id, nonce, expected_snapshot_keys):
    if type(request) is not dict:
        raise ValueError("Seed request object required")
    if (request.get("schema") != "p2p-localization-seed-v1" or request.get("boot_id") != boot_id
            or request.get("nonce") != nonce or request.get("map_id") != MAP_ID):
        raise ValueError("Seed identity/schema/nonce/map mismatch")
    if request.get("seed_candidate_accepted") is not True:
        raise ValueError("Seed must retain the accepted proposal marker")
    extrinsics = request.get("extrinsics")
    if (type(extrinsics) is not list or len(extrinsics) != 3
            or [finite(value) for value in extrinsics] != EXTRINSICS):
        raise ValueError("Proposal and runtime laser mounting assumptions differ")
    supplied = request.get("proposal_snapshot_stamps")
    if type(supplied) is not list or len(supplied) != 3:
        raise ValueError("Three exact proposal source stamps required")
    checked = []
    for key in supplied:
        if type(key) is not list or len(key) != 2:
            raise ValueError("Proposal source keys must be [seconds,nanoseconds]")
        checked.append(exact_stamp(dict(stamp_secs=key[0], stamp_nsecs=key[1])))
    if checked != expected_snapshot_keys:
        raise ValueError("Seed does not bind the actual captured proposal sources")
    pose = request.get("pose")
    if type(pose) is not list or len(pose) != 3:
        raise ValueError("Seed pose must be [x,y,yaw radians]")
    pose = [finite(value) for value in pose]
    if abs(pose[0]) > 100 or abs(pose[1]) > 100 or abs(pose[2]) > math.pi:
        raise ValueError("Seed pose units or bounds are invalid")
    return pose


def current_three_scans(scans, *, now_ros, anchor_source_monotonic_s, laser_frame):
    if len(scans) < 3:
        raise ValueError("Three current scans have not arrived")
    selected = list(scans)[-3:]
    keys = []
    for scan in selected:
        key = exact_stamp(scan["header"])
        keys.append(key)
        source = key[0] + key[1] / 1e9
        if (scan["header"].get("frame_id") != laser_frame
                or not 0 <= finite(now_ros) - source < AGE_BUDGETS["scan"]
                or finite(scan["source_monotonic_s"]) <= finite(anchor_source_monotonic_s)):
            raise ValueError("Seed/quality scans must be current original sources after the stationary anchor")
    if not keys[0] < keys[1] < keys[2]:
        raise ValueError("Current scan sources must be three different advancing stamps")
    return selected


def scan_agreement(map_core, info, cells, scans, pose):
    scores = [map_core.scan_match(info, cells, scan["ranges"], scan["angle_min"],
                                 scan["angle_increment"], scan["range_min"], scan["range_max"], pose)
              for scan in scans]
    if any(score.get("used", 0) < 60 or score.get("near_wall_ratio", 0.) < .65 for score in scores):
        raise ValueError("Current scan/map agreement requires >=60 endpoints and >=0.65 wall ratio on every scan")
    return scores


def proposal_snapshot(scans, *, boot_id, nonce, frames, anchor):
    """Serialize three real after-anchor cropped scans, never relay-time stamps."""
    snapshots = []
    for scan in scans:
        values = []
        for value in scan["ranges"]:
            value = float(value)
            values.append(None if math.isnan(value) else ("inf" if value > 0 else "-inf") if math.isinf(value) else value)
        item = {name: scan[name] for name in ("header", "angle_min", "angle_increment", "range_min", "range_max")}
        item.update(ranges=values, navigation_fov_deg=180,
                    source_monotonic_s=scan["source_monotonic_s"],
                    header_is_actual_front180_subscription=True)
        snapshots.append(item)
    return dict(scope="current_after_anchor_scan_proposal_input_only", complete=len(snapshots) == 3,
                required_scan_count=3, captured_scan_count=len(snapshots), snapshots=snapshots,
                boot_id=boot_id, nonce=nonce, map_id=MAP_ID, frames=dict(frames),
                mounting=dict(x_m=.13, y_m=0., z_m=.2, yaw_rad=0., assumed=True, measured=False),
                stationary_anchor=anchor, source_time_rewritten=False,
                hardware_execution_authorized=False)


def yaw_from_quaternion(q):
    values = [finite(q[key]) for key in ("x", "y", "z", "w")]
    if abs(sum(value * value for value in values) - 1.) > .002:
        raise ValueError("Non-normalized pose quaternion")
    x, y, z, w = values
    return math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))


def quality_sample(pose, transform, *, now_ros, seed_ros, map_frame):
    header = pose["header"]
    key = exact_stamp(header)
    stamp = key[0] + key[1] / 1e9
    if header.get("frame_id") != map_frame or not seed_ros < stamp <= now_ros or now_ros - stamp >= 2.:
        raise ValueError("AMCL needs a distinct current original source strictly after this seed")
    covariance = [finite(value) for value in pose["covariance"]]
    if (len(covariance) != 36 or any(covariance[index] < 0 for index in (0, 7, 35))
            or max(covariance[0], covariance[7]) > .10 ** 2 or covariance[35] > math.radians(5.) ** 2):
        raise ValueError("AMCL position/yaw covariance exceeds static diagnostic limits")
    point = [finite(pose["position"][key]) for key in ("x", "y")]
    yaw = yaw_from_quaternion(pose["orientation"])
    tf_stamp = finite(transform["source_stamp"])
    if not -.30 <= now_ros - tf_stamp <= .50:
        raise ValueError("Current TF source time is outside the static diagnostic window")
    tx, ty = (finite(transform["translation"][key]) for key in ("x", "y"))
    tf_yaw = yaw_from_quaternion(transform["rotation"])
    translation_error = math.hypot(tx - point[0], ty - point[1])
    yaw_error = abs(math.atan2(math.sin(tf_yaw - yaw), math.cos(tf_yaw - yaw)))
    if translation_error > .03 or yaw_error > .05:
        raise ValueError("Current map->base TF differs from this AMCL pose")
    return dict(source_key=key, original_source_stamp=stamp, pose=point + [yaw],
                covariance_position_std_m=[math.sqrt(covariance[0]), math.sqrt(covariance[7])],
                covariance_yaw_std_deg=math.degrees(math.sqrt(covariance[35])),
                tf_translation_error_m=translation_error, tf_yaw_error_rad=yaw_error,
                tf_original_source_stamp=tf_stamp)


class ConsecutiveLocalizationEvidence:
    """Five current post-seed observations; later counterevidence clears latest."""
    def __init__(self, seed_ros, nonce):
        self.seed_ros, self.nonce = finite(seed_ros), nonce
        self.window = deque(maxlen=5)
        self.first = self.latest = self.last_key = None

    def reject(self):
        self.window.clear()
        self.latest = None

    def observe(self, observation, *, now_ros, now_monotonic_s):
        key = observation["source_key"]
        stamp = finite(observation["original_source_stamp"])
        if not self.seed_ros < stamp <= now_ros or now_ros - stamp >= 2.:
            self.reject()
            raise ValueError("Only current original post-seed sources can accumulate")
        if self.last_key is not None:
            if key == self.last_key:
                return self.latest  # Never redate a duplicate validation.
            if key < self.last_key:
                self.reject()
                raise ValueError("AMCL source went backwards")
        self.last_key = list(key)
        self.window.append(deepcopy(observation))
        self.latest = None
        if len(self.window) == 5 and now_ros - self.window[0]["original_source_stamp"] < 2.:
            self.latest = dict(validated_at_monotonic_s=finite(now_monotonic_s), validated_at_ros_s=finite(now_ros),
                               post_seed_five_observations=list(self.window), seed_ros_s=self.seed_ros,
                               map_id=MAP_ID, nonce=self.nonce, stationary_diagnostic_only=True,
                               hardware_execution_authorized=False)
            if self.first is None:
                self.first = deepcopy(self.latest)
        return self.latest


def cleanup_children(children, *, timeout_s=3., expected_natural_exit_names=()):
    """Only caller-owned read-only children are signalled; no physical owner PID."""
    records = []
    for name, child in children:
        row = dict(name=name, pid=child.pid, was_running=child.poll() is None,
                   exit_code=child.poll(), forced_kill=False, expected_termination=False, errors=[])
        try:
            if row["was_running"]:
                child.terminate()
                row["signal"] = "TERM"
                try:
                    row["exit_code"] = child.wait(timeout=timeout_s)
                except subprocess.TimeoutExpired:
                    child.kill()
                    row["forced_kill"] = True
                    row["exit_code"] = child.wait(timeout=timeout_s)
            else:
                row["exit_code"] = child.wait(timeout=timeout_s)
            row["expected_termination"] = bool(
                not row["forced_kill"] and
                ((row["was_running"] and row["exit_code"] in (0, -15))
                 or (not row["was_running"] and name in expected_natural_exit_names and row["exit_code"] == 0)))
            if not row["expected_termination"]:
                row["errors"].append("Unexpected child exit/forced termination: " + str(row["exit_code"]))
        except Exception as exc:
            row["errors"].append(type(exc).__name__ + ": " + str(exc))
        records.append(row)
    return records


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False), encoding="utf-8")
    temporary.replace(path)


def checked_module(relative, expected, token):
    path = ROOT / relative
    if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise RuntimeError("Reviewed dependency differs: " + relative)
    name = "_p2p_loc_" + token + "_" + path.stem
    spec = importlib.util.spec_from_file_location(name, str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def adapter_report_agrees(report, frames):
    if (report.get("frames") != frames or report.get("runtime_error")
            or report.get("callback_error_count") != 0
            or report.get("engineering_age_budgets_s") != AGE_BUDGETS):
        return False
    for name in ("scan", "odom", "amcl"):
        channel = (report.get("channels") or {}).get(name) or {}
        if channel.get("missing", True) or channel.get("current_distinct_sources", 0) < (3 if name == "scan" else 5):
            return False
    return all((report.get("tf_diagnostics") or {}).get(frames[parent] + "->" + frames[child], {}).get("available") is True
               for parent, child in (("map", "odom"), ("map", "base"), ("odom", "base"), ("base", "laser")))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--expected-boot-id", required=True)
    parser.add_argument("--duration", required=True, type=float)
    parser.add_argument("--nonce", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--scan-snapshot-output", default="scan_snapshot.json")
    args = parser.parse_args(argv)
    if not 30 <= finite(args.duration) <= 120:
        raise ValueError("Localization observation duration must be 30..120 seconds")
    names = names_for_nonce(args.nonce)
    ns, frames, topics = names["namespace"], names["frames"], names["topics"]
    boot = Path("/proc/sys/kernel/random/boot_id").read_text().strip()
    if boot != args.expected_boot_id:
        raise RuntimeError("New RK boot requires a fresh inventory")
    manifest = loads_config((ROOT / "localization_package_manifest.json").read_text())
    for relative, digest in manifest["files"].items():
        path = ROOT / relative
        path.resolve().relative_to(ROOT.resolve())
        if path.is_symlink() or hashlib.sha256(path.read_bytes()).hexdigest() != digest:
            raise RuntimeError("Package startup SHA differs: " + relative)
    modules = {key: checked_module("deps/" + key + ".py", manifest["files"]["deps/" + key + ".py"], args.nonce)
               for key in ("control_client", "core", "rk_link", "stop_observation", "transport")}
    map_core = checked_module("deps/maploc_core.py", manifest["files"]["deps/maploc_core.py"], args.nonce)
    map_doc = loads_config((ROOT / "maps/map.json").read_text())
    info, cells = map_doc["info"], map_doc["data"]
    width, height, pgm_cells = map_core.pgm((ROOT / "maps/map.pgm").read_bytes())
    if (map_core.map_identity(info, cells) != MAP_ID or (width, height) != (info["width"], info["height"])
            or map_core.map_identity(info, pgm_cells) != MAP_ID):
        raise RuntimeError("Saved map JSON/PGM do not match the required old map identity")
    amcl_parameters = loads_config((ROOT / "amcl_base.json").read_text())
    if any(key.startswith("initial_pose_") for key in amcl_parameters):
        raise RuntimeError("AMCL configuration contains an old initial pose")
    amcl_parameters.update(global_frame_id=frames["map"], odom_frame_id=frames["odom"],
                           base_frame_id=frames["base"], use_map_topic=True, first_map_only=True,
                           save_pose_rate=-1., tf_broadcast=True)
    import rospy
    import tf2_ros
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from std_srvs.srv import Empty
    from localization_ros_adapter import amcl_diagnostics, source_header
    rospy.init_node("p2p_localization_runtime_" + args.nonce, disable_signals=True)
    if rospy.get_param("/use_sim_time", False) or rospy.has_param(ns):
        raise RuntimeError("Real ROS time and a fresh private parameter namespace are required")
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    snapshot_path = Path(args.scan_snapshot_output).resolve()
    seed_path = output.with_name("seed_request.json")
    if seed_path.exists():
        raise RuntimeError("A preexisting seed request is not a new current-session proposal")
    lock = threading.RLock()
    stop_event = threading.Event()
    cancelled_event = threading.Event()
    failure = []
    children, logs, subscriptions = [], [], []
    mailbox = deque(maxlen=1)
    scans, amcl_samples = deque(maxlen=8), deque(maxlen=32)
    state = dict(anchor=None, anchor_latest=None, anchor_frozen=None, anchor_enabled=True, stop_count=0,
                 seed_receipt=None, seed_consumed=False, seed_ros=None, latest_packet=None,
                 map_verified=False, published_odom=0, publication_errors=[], joint_latest=None,
                 joint_first=None, adapter_report=None, service_inflight=None,
                 adapter_started=None, adapter_duration=args.duration - 10.,
                 proposal_snapshot_written=False, quality_rejections=[])
    result = dict(schema="p2p-saved-map-static-localization-v1", boot_id=boot, nonce=args.nonce,
                  map_id=MAP_ID, frames=frames, topics=topics, navigation_enabled=False,
                  motion_requested=False, hardware_execution_authorized=False,
                  historical_static_localization_verified=False, actual_map_localization_valid=False,
                  engineering_age_budgets_s=dict(AGE_BUDGETS), age_budgets_are_navigation_defaults=False,
                  mounting_assumed=dict(x_m=.13, y_m=0., z_m=.2, yaw_rad=0., verified=False),
                  base_result=None, started_monotonic_s=time.monotonic())

    def fail(reason):
        with lock:
            if not failure:
                failure.append(dict(reason=reason, at_monotonic_s=time.monotonic()))

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda _s, _f: cancelled_event.set())
    publisher = rospy.Publisher(topics["wheel_odom"], Odometry, queue_size=1)
    seed_publisher = rospy.Publisher(topics["initialpose"], PoseWithCovarianceStamped, queue_size=1)
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    def scan_received(message):
        try:
            header = source_header(message.header)
            key = exact_stamp(header)
            ros, mono = rospy.Time.now().to_sec(), time.monotonic()
            source_ros = key[0] + key[1] / 1e9
            if header["frame_id"] != frames["laser"]:
                raise ValueError("Private scan has the wrong laser frame")
            if not 0 <= ros - source_ros < .75:
                return
            with lock:
                if scans and exact_stamp(scans[-1]["header"]) >= key:
                    return
                scans.append(dict(header=header, ranges=list(message.ranges), angle_min=message.angle_min,
                                  angle_increment=message.angle_increment, range_min=message.range_min,
                                  range_max=message.range_max, source_monotonic_s=mono - (ros - source_ros)))
        except Exception as exc:
            fail("scan callback: " + str(exc))

    def amcl_received(message):
        try:
            value = amcl_diagnostics(message, frames["map"])
            with lock:
                if not amcl_samples or exact_stamp(value["header"]) > exact_stamp(amcl_samples[-1]["header"]):
                    amcl_samples.append(value)
        except Exception as exc:
            fail("AMCL callback: " + str(exc))

    def map_received(message):
        try:
            if message.header.frame_id != frames["map"]:
                raise ValueError("Private map frame differs")
            q = message.info.origin.orientation
            metadata = dict(width=message.info.width, height=message.info.height,
                            resolution=message.info.resolution,
                            origin=[message.info.origin.position.x, message.info.origin.position.y,
                                    yaw_from_quaternion(dict(x=q.x, y=q.y, z=q.z, w=q.w))])
            if map_core.map_identity(metadata, list(message.data)) != MAP_ID:
                raise ValueError("Live map-server grid identity differs")
            with lock:
                state["map_verified"] = True
        except Exception as exc:
            fail("map callback: " + str(exc))

    def publish_loop():
        while not stop_event.is_set():
            try:
                observation = mailbox.popleft()
            except IndexError:
                stop_event.wait(.005)
                continue
            try:
                source_ros = rospy.Time.now().to_sec() - (time.monotonic() - observation["source_monotonic_s"])
                msg = Odometry()
                msg.header.stamp = rospy.Time.from_sec(source_ros)
                msg.header.seq = observation["sample_sequence"]
                msg.header.frame_id, msg.child_frame_id = frames["odom"], frames["base"]
                msg.pose.pose.position.x, msg.pose.pose.position.y = observation["x"], observation["y"]
                msg.pose.pose.orientation.z = math.sin(observation["yaw"] / 2.)
                msg.pose.pose.orientation.w = math.cos(observation["yaw"] / 2.)
                msg.twist.twist.linear.x, msg.twist.twist.angular.z = observation["linear_x"], observation["angular_z"]
                for index in (0, 7, 14, 21, 28, 35):
                    msg.pose.covariance[index] = msg.twist.covariance[index] = 1e3
                publisher.publish(msg)
                state["published_odom"] += 1
            except Exception as exc:
                fail("measured odometry publication: " + str(exc))

    def anchor_snapshot():
        with lock:
            checker = state["anchor"]
            return None if checker is None else checker.snapshot(time.monotonic())

    def live_scans(anchor):
        if not anchor or not anchor["valid"]:
            raise ValueError("Current stationary anchor is not ready")
        with lock:
            selected = list(scans)
        return current_three_scans(selected, now_ros=rospy.Time.now().to_sec(),
                                   anchor_source_monotonic_s=anchor["anchor"]["source_monotonic_s"],
                                   laser_frame=frames["laser"])

    def supervise():
        last_amcl_key = None
        evidence = None
        next_service = 0.
        while not stop_event.is_set():
            try:
                if not state["proposal_snapshot_written"]:
                    try:
                        anchor = anchor_snapshot()
                        if anchor and anchor["valid"] and anchor["accepted_samples"] >= 5:
                            current = live_scans(anchor)
                            atomic_json(snapshot_path, proposal_snapshot(current, boot_id=boot, nonce=args.nonce,
                                                                       frames=frames, anchor=anchor))
                            state["proposal_snapshot_written"] = True
                    except ValueError:
                        pass  # Warming sources do not create a proposal snapshot.
                if seed_path.is_file() and not state["seed_consumed"]:
                    state["seed_consumed"] = True
                    receipt = dict(accepted=False, published=False, received_monotonic_s=time.monotonic())
                    state["seed_receipt"] = receipt
                    try:
                        if seed_path.is_symlink() or seed_path.stat().st_size > 65536:
                            raise ValueError("Seed file is not a bounded regular current request")
                        request = loads_config(seed_path.read_text())
                        expected_keys = snapshot_keys(loads_config(snapshot_path.read_text()))
                        proposed = validate_seed_request(request, boot_id=boot, nonce=args.nonce,
                                                         expected_snapshot_keys=expected_keys)
                        anchor = anchor_snapshot()
                        current = live_scans(anchor)
                        scores = scan_agreement(map_core, info, cells, current, proposed)
                        # Recheck the anchor after calculation; do not refresh it.
                        anchor = anchor_snapshot()
                        if not anchor or not anchor["valid"]:
                            raise ValueError("Stationary anchor expired during seed validation")
                        current_three_scans(current, now_ros=rospy.Time.now().to_sec(),
                                            anchor_source_monotonic_s=anchor["anchor"]["source_monotonic_s"],
                                            laser_frame=frames["laser"])
                        message = PoseWithCovarianceStamped()
                        message.header.frame_id = frames["map"]
                        message.header.stamp = rospy.Time.now()
                        message.pose.pose.position.x, message.pose.pose.position.y = proposed[:2]
                        message.pose.pose.orientation.z = math.sin(proposed[2] / 2.)
                        message.pose.pose.orientation.w = math.cos(proposed[2] / 2.)
                        message.pose.covariance[0] = message.pose.covariance[7] = .05 ** 2
                        message.pose.covariance[35] = .03 ** 2
                        state["seed_ros"] = message.header.stamp.to_sec()
                        seed_publisher.publish(message)
                        receipt.update(accepted=True, published=True, pose=proposed,
                                       published_ros_s=state["seed_ros"], published_monotonic_s=time.monotonic(),
                                       proposal_snapshot_stamps=expected_keys,
                                       current_scan_sources=[exact_stamp(s["header"]) for s in current], scores=scores,
                                       initial_covariance_std=dict(x_m=.05, y_m=.05, yaw_rad=.03),
                                       covariance_is_static_trial_assumption=True,
                                       anchor=anchor, hardware_execution_authorized=False)
                    except Exception as exc:
                        receipt["first_rejection"] = type(exc).__name__ + ": " + str(exc)
                    atomic_json(output.with_name("seed_receipt.json"), receipt)
                if state["seed_ros"] is not None:
                    if evidence is None:
                        evidence = ConsecutiveLocalizationEvidence(state["seed_ros"], args.nonce)
                    at = time.monotonic()
                    if at >= next_service:
                        next_service = at + .28
                        state["service_inflight"] = at
                        try:
                            rospy.wait_for_service(topics["request_nomotion_update"], timeout=.10)
                            rospy.ServiceProxy(topics["request_nomotion_update"], Empty)()
                        finally:
                            state["service_inflight"] = None
                    with lock:
                        pose = deepcopy(amcl_samples[-1]) if amcl_samples else None
                    if pose is not None and exact_stamp(pose["header"]) != last_amcl_key:
                        last_amcl_key = exact_stamp(pose["header"])
                        try:
                            anchor = anchor_snapshot()
                            current = live_scans(anchor)
                            transform = tf_buffer.lookup_transform(frames["map"], frames["base"], rospy.Time(0), rospy.Duration(0))
                            q, t = transform.transform.rotation, transform.transform.translation
                            tf_data = dict(source_stamp=transform.header.stamp.to_sec(),
                                           translation=dict(x=t.x, y=t.y), rotation=dict(x=q.x, y=q.y, z=q.z, w=q.w))
                            observed = quality_sample(pose, tf_data, now_ros=rospy.Time.now().to_sec(),
                                                      seed_ros=state["seed_ros"], map_frame=frames["map"])
                            scores = scan_agreement(map_core, info, cells, current, observed["pose"])
                            anchor = anchor_snapshot()
                            if not anchor or not anchor["valid"]:
                                raise ValueError("Stationary anchor expired during quality calculation")
                            current_three_scans(current, now_ros=rospy.Time.now().to_sec(),
                                                anchor_source_monotonic_s=anchor["anchor"]["source_monotonic_s"],
                                                laser_frame=frames["laser"])
                            quality_sample(pose, tf_data, now_ros=rospy.Time.now().to_sec(),
                                           seed_ros=state["seed_ros"], map_frame=frames["map"])
                            observed.update(scan_scores=scores, scan_source_keys=[exact_stamp(s["header"]) for s in current],
                                            anchor=anchor, observed_monotonic_s=time.monotonic())
                            joint = evidence.observe(observed, now_ros=rospy.Time.now().to_sec(),
                                                     now_monotonic_s=time.monotonic())
                            state["joint_latest"] = joint
                            state["joint_first"] = deepcopy(evidence.first)
                            if joint:
                                atomic_json(output.with_name("joint_evidence.json"), joint)
                        except Exception as exc:
                            evidence.reject()
                            state["joint_latest"] = None
                            if len(state["quality_rejections"]) < 20:
                                state["quality_rejections"].append(dict(source_key=last_amcl_key,
                                    at_monotonic_s=time.monotonic(), reason=type(exc).__name__ + ": " + str(exc)))
                adapter = next((child for name, child in children if name == "adapter"), None)
                if adapter and adapter.poll() is not None:
                    report_path = output.with_name("localization_adapter_result.json")
                    if state["adapter_report"] is None and report_path.is_file():
                        state["adapter_report"] = loads_config(report_path.read_text())
                    # The observer intentionally finishes before the physical owner.
                    break
            except Exception as exc:
                fail("localization supervisor: " + type(exc).__name__ + ": " + str(exc))
                break
            stop_event.wait(.04)

    worker = threading.Thread(target=supervise, name="p2p-localization-supervisor", daemon=True)
    publication_thread = threading.Thread(target=publish_loop, name="p2p-localization-odom", daemon=True)
    api, stop_api, core = modules["control_client"], modules["stop_observation"], modules["core"]

    class ObservedClient(api.Client):
        def stop(self):
            # Owner-thread scalar/pointer stores only: never acquire the shared
            # diagnostic lock or compute a snapshot before the physical STOP.
            state["stop_count"] += 1
            if state["stop_count"] >= 2:
                state["anchor_enabled"] = False
                state["anchor_frozen"] = state["anchor_latest"]
            return super().stop()

        def stopping_status(self):
            packet = super().stopping_status()
            if state["stop_count"] == 1 and stop_api.stopped(packet) and state["anchor"] is None:
                state["post_stop_tick"] = packet["tick_ms"]
                state["anchor"] = StaticLocalizationAnchor(
                    boot_id=boot, owner_id=args.nonce,
                    reference_after_monotonic_s=self.last_exchange_completion["completed_monotonic_s"],
                    source_timeout_s=.75, max_encoder_change_counts=0, max_encoder_travel_counts=0,
                    max_position_change_m=.002, max_yaw_change_rad=.003,
                    max_linear_speed_m_s=.002, max_angular_speed_rad_s=.01, max_wheel_speed_rpm=.2)
            return packet

    class RecordingOdometry(core.MeasuredOdometry):
        def update(self, packet):
            state["latest_packet"] = dict(packet)
            return super().update(packet)

    def odom_sink(observation):
        with lock:
            checker, packet = state["anchor"], state["latest_packet"]
            if state["anchor_enabled"] and checker is not None:
                ticks = ("sample_tick_ms", "left_tick_ms", "right_tick_ms", "speed_tick_ms")
                if all(0 < ((packet[key] - state["post_stop_tick"]) & 0xffffffff) < 0x80000000 for key in ticks):
                    state["anchor_latest"] = checker.observe(
                        packet, observation, source_monotonic_s=observation["source_monotonic_s"],
                        now_monotonic_s=time.monotonic(), boot_id=boot, owner_id=args.nonce)
        mailbox.append(observation)

    def cancellation():
        if cancelled_event.is_set() or rospy.is_shutdown():
            fail("operator/runtime cancellation")
        at = time.monotonic()
        if state["service_inflight"] is not None and at - state["service_inflight"] >= 1.:
            fail("private nomotion service exceeded one second")
        for name, child in children:
            code = child.poll()
            if code is not None:
                expected_adapter_end = (name == "adapter" and code == 0 and state["adapter_started"] is not None
                                        and at - state["adapter_started"] >= state["adapter_duration"] - .5)
                if not expected_adapter_end:
                    fail("read-only child exited early: " + name + " code=" + str(code))
        return bool(failure)

    try:
        rospy.set_param(ns + "/map_server/frame_id", frames["map"])
        for key, value in amcl_parameters.items():
            rospy.set_param(ns + "/amcl/" + key, value)
        subscriptions = [rospy.Subscriber(topics["map"], OccupancyGrid, map_received, queue_size=1),
                         rospy.Subscriber(topics["scan_front180"], LaserScan, scan_received, queue_size=1, tcp_nodelay=True),
                         rospy.Subscriber(topics["amcl_pose"], PoseWithCovarianceStamped, amcl_received, queue_size=1, tcp_nodelay=True)]
        commands = [
            ("map_server", ["/opt/ros/noetic/lib/map_server/map_server", str(ROOT / "maps/map.yaml"),
                            "__name:=map_server", "__ns:=" + ns, "map:=" + topics["map"],
                            "map_metadata:=" + ns + "/map_metadata", "static_map:=" + ns + "/static_map"]),
            ("amcl", ["/opt/ros/noetic/lib/amcl/amcl", "__name:=amcl", "__ns:=" + ns,
                      "scan:=" + topics["scan_front180"], "map:=" + topics["map"],
                      "amcl_pose:=" + topics["amcl_pose"], "initialpose:=" + topics["initialpose"],
                      "request_nomotion_update:=" + topics["request_nomotion_update"],
                      "static_map:=" + ns + "/static_map", "global_localization:=" + ns + "/global_localization",
                      "set_map:=" + ns + "/set_map"]),
            ("adapter", [sys.executable, "-s", "-B", str(ROOT / "localization_ros_adapter.py"),
                         "--scan-topic", "/lidar1/laser_fan", "--odom-topic", topics["wheel_odom"],
                         "--scan-output-topic", topics["scan_front180"], "--amcl-topic", topics["amcl_pose"],
                         "--laser-frame", frames["laser"], "--base-frame", frames["base"],
                         "--odom-frame", frames["odom"], "--map-frame", frames["map"],
                         "--scan-max-age-s", ".75", "--odom-max-age-s", ".75", "--amcl-max-age-s", "2",
                         "--duration", str(state["adapter_duration"]), "--snapshot-scan-count", "3",
                         "--scan-snapshot-output", str(output.with_name("adapter_initial_scan_snapshot.json")),
                         "--output", str(output.with_name("localization_adapter_result.json"))])]
        for name, command in commands:
            log = output.with_name(name + ".log").open("w")
            logs.append(log)
            child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=log, env=dict(os.environ))
            children.append((name, child))
            if name == "adapter":
                state["adapter_started"] = time.monotonic()
        deadline = time.monotonic() + 20.
        while time.monotonic() < deadline:
            if cancellation():
                raise RuntimeError(failure[0]["reason"])
            with lock:
                warm = state["map_verified"] and len(scans) >= 3
            if warm and seed_publisher.get_num_connections() > 0:
                break
            time.sleep(.05)
        else:
            raise RuntimeError("Private map/scan/AMCL seed subscriber did not warm before opening serial")
        for key, value in amcl_parameters.items():
            if rospy.get_param(ns + "/amcl/" + key) != value:
                raise RuntimeError("Private AMCL parameter readback differs: " + key)
        graph = rospy.get_master().getSystemState()
        if graph[0] != 1:
            raise RuntimeError("ROS graph preflight failed")
        publishers = dict(graph[2][0])
        for topic, expected in ((topics["map"], ns + "/map_server"), (topics["amcl_pose"], ns + "/amcl")):
            if publishers.get(topic) != [expected]:
                raise RuntimeError("Private ROS publisher ownership differs: " + topic)
        result["preopen_map_and_graph_verified"] = True
        publication_thread.start()
        worker.start()
        target = modules["rk_link"].locate()
        result["target"] = target
        proxy = SimpleNamespace(Client=ObservedClient, VERSION=api.VERSION, IDENTITY=api.IDENTITY,
                                LEASE_MS=api.LEASE_MS, MAX_RPM=api.MAX_RPM, fresh=api.fresh)
        with modules["transport"].exclusive_port(target["port"]) as physical:
            result["base_result"] = run_static_session(
                physical, proxy, stop_api, lambda: RecordingOdometry(.2, .3, calibrated=False),
                duration_s=args.duration, feedback_timeout_s=.75, cancelled=cancellation,
                odom_sink=odom_sink, row_limit=10000, zero_speed_rpm=.2)
        result["exclusive_port_closed_normally"] = True
    except BaseException as exc:
        fail("runtime: " + type(exc).__name__ + ": " + str(exc))
    finally:
        # Serial owner has returned/closed before signalling read-only children.
        stop_event.set()
        result["children_cleanup"] = cleanup_children(list(reversed(children)), expected_natural_exit_names=("adapter",))
        for thread in (worker, publication_thread):
            if thread.ident is not None:
                thread.join(timeout=2.)
        result["worker_still_alive"] = worker.is_alive()
        result["publication_thread_still_alive"] = publication_thread.is_alive()
        for name, callback in ([('subscriber', subscription.unregister) for subscription in subscriptions]
                               + [('odom publisher', publisher.unregister), ('seed publisher', seed_publisher.unregister)]):
            try:
                callback()
            except Exception as exc:
                fail(name + " cleanup: " + str(exc))
        for log in logs:
            log.close()
        try:
            if rospy.has_param(ns):
                rospy.delete_param(ns)  # Only this fresh nonce namespace.
            result["private_parameters_removed"] = True
        except Exception as exc:
            fail("private parameter cleanup: " + str(exc))
        rospy.signal_shutdown("Bounded stationary localization observation ended")
    base = result.get("base_result") or {}
    first_failure = failure[0] if failure else None
    if base.get("first_failure") and (first_failure is None or base.get("failure_monotonic_s", float("inf")) < first_failure["at_monotonic_s"]):
        first_failure = dict(reason="base owner: " + str(base["first_failure"]),
                             at_monotonic_s=base.get("failure_monotonic_s"))
    result.update(first_failure=first_failure,
                  seed_receipt=state["seed_receipt"], anchor_frozen=state["anchor_frozen"],
                  joint_first=state["joint_first"], joint_latest=state["joint_latest"],
                  adapter_report=state["adapter_report"], measured_odom_messages=state["published_odom"],
                  ended_monotonic_s=time.monotonic(), map_live_identity_verified=state["map_verified"],
                  quality_rejections=state["quality_rejections"],
                  proposal_snapshots_after_anchor=state["proposal_snapshot_written"])
    cleanup_ok = all(item["expected_termination"] and not item["forced_kill"] and not item["errors"]
                     for item in result["children_cleanup"])
    result["historical_static_localization_verified"] = bool(
        base.get("static_link_complete") and not failure and state["joint_latest"]
        and state["anchor_frozen"] and state["anchor_frozen"]["valid"]
        and state["adapter_report"] and adapter_report_agrees(state["adapter_report"], frames)
        and cleanup_ok and not result["worker_still_alive"] and not result["publication_thread_still_alive"])
    atomic_json(output, result)
    print(json.dumps(dict(output=str(output), historical_static_localization_verified=result["historical_static_localization_verified"],
                          first_failure=result["first_failure"], navigation_enabled=False)))
    _ = tf_listener
    return 0 if result["historical_static_localization_verified"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
