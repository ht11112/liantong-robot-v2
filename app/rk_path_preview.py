"""Single GlobalPlanner make_plan preview beside a live static localization.

No serial, motor, action-goal, initialpose or velocity publication exists. A
private move_base child has every cmd_vel output remapped to shadow_unused.
The returned historical path is not proof of driving, clearance or arrival.
"""
import argparse
from copy import deepcopy
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time

from config import loads_config
from global_pose_proposal import SAVED_MAP_ID, map_identity, verified_map, endpoint_agreement


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


def names_for_preview(nonce, token):
    if type(nonce) is not str or re.fullmatch(r"[A-Za-z0-9_]{6,48}", nonce) is None:
        raise ValueError("Invalid localization nonce")
    if type(token) is not str or re.fullmatch(r"[A-Za-z0-9_]{1,48}", token) is None:
        raise ValueError("Invalid preview namespace token")
    loc = "/p2p_loc_" + nonce
    ns = "/p2p_preview_" + nonce + "_" + token
    return dict(namespace=ns, node=ns + "/move_base", shadow_topic=ns + "/shadow_unused",
                service=ns + "/move_base/make_plan", costmap_topic=ns + "/move_base/global_costmap/costmap",
                map_topic=loc + "/map", scan_topic=loc + "/scan_front180", odom_topic=loc + "/wheel_odom",
                amcl_topic=loc + "/amcl_pose",
                map_publisher=loc + "/map_server",
                frames={key: "p2p_" + nonce + "_" + key for key in ("map", "odom", "base", "laser")})


def stamp_key(key):
    if (type(key) not in (list, tuple) or len(key) != 2 or type(key[0]) is not int
            or key[0] < 0 or type(key[1]) is not int or not 0 <= key[1] < 1000000000
            or key == [0, 0] or tuple(key) == (0, 0)):
        raise ValueError("Exact nonzero source [seconds,nanoseconds] required")
    return (key[0], key[1])


def validate_joint(joint, boot_id, nonce, now_monotonic, now_ros):
    """Bind five post-seed observations and the latest live owner anchor."""
    now_monotonic, now_ros = finite(now_monotonic), finite(now_ros)
    if (type(joint) is not dict or joint.get("nonce") != nonce or joint.get("map_id") != SAVED_MAP_ID
            or joint.get("stationary_diagnostic_only") is not True
            or joint.get("hardware_execution_authorized") is not False):
        raise ValueError("Joint map/nonce/diagnostic identity differs")
    checked_at = finite(joint.get("validated_at_monotonic_s"), "joint validation time")
    checked_ros = finite(joint.get("validated_at_ros_s"), "joint ROS validation time")
    if not 0 <= now_monotonic - checked_at <= 2. or not 0 <= now_ros - checked_ros <= 2.:
        raise ValueError("Joint validation is stale or future")
    seed = finite(joint.get("seed_ros_s"), "seed time")
    rows = joint.get("post_seed_five_observations")
    if type(rows) is not list or len(rows) != 5:
        raise ValueError("Five post-seed localization observations required")
    previous = None
    for row in rows:
        key = stamp_key(row.get("source_key"))
        source = key[0] + key[1] / 1e9
        if (previous is not None and key <= previous or not seed < source <= checked_ros
                or checked_ros - source >= 2.
                or abs(finite(row.get("original_source_stamp")) - source) > 1e-6):
            raise ValueError("Post-seed source stamps are not distinct/current/consistent")
        previous = key
        pose = row.get("pose")
        if type(pose) is not list or len(pose) != 3:
            raise ValueError("Joint pose must be [x,y,yaw]")
        pose = [finite(value, "joint pose") for value in pose]
        if abs(pose[2]) > math.pi:
            raise ValueError("Joint yaw must be radians")
        anchor = row.get("anchor") or {}
        if (anchor.get("boot_id") != boot_id or anchor.get("owner_id") != nonce
                or anchor.get("valid") is not True or anchor.get("failure") is not None
                or anchor.get("anchor_established") is not True):
            raise ValueError("Current boot/owner stationary anchor differs")
        scores, scan_keys = row.get("scan_scores"), row.get("scan_source_keys")
        if type(scores) is not list or len(scores) != 3 or type(scan_keys) is not list or len(scan_keys) != 3:
            raise ValueError("Three original scan/map checks per localization observation required")
        keys = [stamp_key(value) for value in scan_keys]
        if not keys[0] < keys[1] < keys[2]:
            raise ValueError("Joint scan source stamps must advance")
        for score in scores:
            if (type(score.get("used")) is not int or score["used"] < 60
                    or not .65 <= finite(score.get("near_wall_ratio")) <= 1.):
                raise ValueError("Recorded live scan/map agreement is insufficient")
        if finite(row.get("tf_translation_error_m")) > .03 or finite(row.get("tf_yaw_error_rad")) > .05:
            raise ValueError("Recorded AMCL/TF disagreement")
    last = rows[-1]
    anchor_source = finite((last["anchor"].get("latest") or {}).get("source_monotonic_s"))
    if not 0 <= now_monotonic - anchor_source < .75:
        raise ValueError("The stationary owner's latest source is no longer current")
    if not 0 <= now_ros - finite(last["original_source_stamp"]) < 2.:
        raise ValueError("Latest AMCL source is no longer current")
    return dict(pose=[finite(value) for value in last["pose"]],
                source_key=list(stamp_key(last["source_key"])),
                validated_at_monotonic_s=checked_at, validated_at_ros_s=checked_ros,
                anchor_source_monotonic_s=anchor_source, boot_id=boot_id, nonce=nonce,
                map_id=SAVED_MAP_ID)


def forward_goal(pose, forward_m):
    if type(pose) not in (list, tuple) or len(pose) != 3:
        raise ValueError("Current [x,y,yaw] pose required")
    x, y, yaw = [finite(value, "pose") for value in pose]
    distance = finite(forward_m, "forward_m")
    if not 0 < distance <= 1.:
        raise ValueError("This preview accepts a positive forward distance up to 1m")
    return [x + distance * math.cos(yaw), y + distance * math.sin(yaw), yaw]


def yaw_from_quaternion(q):
    x, y, z, w = [finite(q[key], "quaternion") for key in ("x", "y", "z", "w")]
    if abs(x * x + y * y + z * z + w * w - 1.) > .002:
        raise ValueError("Normalized quaternion required")
    return math.atan2(2. * (w * z + x * y), 1. - 2. * (y * y + z * z))


def validate_live_tf(transform, pose, frames, now_ros):
    if transform.get("parent") != frames["map"] or transform.get("child") != frames["base"]:
        raise ValueError("Live TF does not use this map/base chain")
    age = finite(now_ros) - finite(transform.get("source_stamp"))
    if not -.30 <= age <= .50:
        raise ValueError("Live map->base TF is stale or unexpectedly future")
    tx, ty = finite(transform["translation"]["x"]), finite(transform["translation"]["y"])
    yaw = yaw_from_quaternion(transform["rotation"])
    distance = math.hypot(tx - pose[0], ty - pose[1])
    angle = abs(math.atan2(math.sin(yaw - pose[2]), math.cos(yaw - pose[2])))
    if distance > .03 or angle > .05:
        raise ValueError("Fresh joint pose and live TF differ")
    return dict(source_age_s=age, translation_error_m=distance, yaw_error_rad=angle)


def validate_live_amcl(value, joint, frames, now_ros):
    header = value["header"]
    key = stamp_key([header["stamp_secs"], header["stamp_nsecs"]])
    source = key[0] + key[1] / 1e9
    if (header.get("frame_id") != frames["map"] or
            not finite(joint["seed_ros_s"]) < source <= now_ros or now_ros - source >= 2.):
        raise ValueError("Direct latest AMCL source is not current/post-seed/in this map frame")
    covariance = [finite(number, "current AMCL covariance") for number in value["covariance"]]
    if (len(covariance) != 36 or any(covariance[index] < 0 for index in (0, 7, 35))
            or max(covariance[0], covariance[7]) > .10 ** 2
            or covariance[35] > math.radians(5.) ** 2):
        raise ValueError("Direct latest AMCL covariance invalidates older joint evidence")
    pose = joint["post_seed_five_observations"][-1]["pose"]
    x, y = finite(value["position"]["x"]), finite(value["position"]["y"])
    yaw = yaw_from_quaternion(value["orientation"])
    displacement = math.hypot(x - pose[0], y - pose[1])
    yaw_error = abs(math.atan2(math.sin(yaw - pose[2]), math.cos(yaw - pose[2])))
    if displacement > .03 or yaw_error > .05:
        raise ValueError("Direct latest AMCL differs from the joint preview pose")
    return dict(source_key=list(key), source_age_s=now_ros - source,
                translation_error_m=displacement, yaw_error_rad=yaw_error,
                covariance_position_std_m=[math.sqrt(covariance[0]), math.sqrt(covariance[7])],
                covariance_yaw_std_deg=math.degrees(math.sqrt(covariance[35])))


def validate_live_scan(value, pose, frames, now_ros, info, cells):
    header = value["header"]
    key = stamp_key([header["stamp_secs"], header["stamp_nsecs"]])
    source = key[0] + key[1] / 1e9
    if header.get("frame_id") != frames["laser"] or not 0 <= now_ros - source < .75:
        raise ValueError("Direct latest scan is not current/in this laser frame")
    amin, increment = finite(value["angle_min"]), finite(value["angle_increment"])
    lower, upper = max(.3, finite(value["range_min"])), min(8., finite(value["range_max"]))
    if increment == 0 or lower >= upper:
        raise ValueError("Direct scan geometry is invalid")
    points = []
    ranges = value["ranges"]
    for index in range(0, len(ranges), max(1, len(ranges) // 180)):
        raw = ranges[index]
        if type(raw) not in (int, float):
            raise ValueError("Direct scan contains nonnumeric ranges")
        beam = math.atan2(math.sin(amin + index * increment), math.cos(amin + index * increment))
        if math.isfinite(raw) and lower <= raw <= upper and abs(beam) <= math.pi / 2. + 1e-12:
            points.append((.13 + raw * math.cos(beam), raw * math.sin(beam)))
    score = endpoint_agreement(info, cells, points, pose)
    if score["used"] < 60 or score["near_wall_ratio"] < .65:
        raise ValueError("Direct current scan/map agreement invalidates older joint evidence")
    return dict(source_key=list(key), source_age_s=now_ros - source, score=score,
                assumed_laser_extrinsics=[.13, 0., 0.])


def planner_parameters(names, site):
    geometry = site["geometry"]
    width, front, rear = [finite(geometry[key], key) for key in
                          ("planning_width_m", "front_from_axle_m", "rear_from_axle_m")]
    if min(width, front, rear) <= 0:
        raise ValueError("Positive estimated planning geometry required")
    footprint = [[-rear, -width / 2.], [-rear, width / 2.], [front, width / 2.], [front, -width / 2.]]
    layers = [dict(name="static_layer", type="costmap_2d::StaticLayer"),
              dict(name="obstacle_layer", type="costmap_2d::ObstacleLayer"),
              dict(name="inflation_layer", type="costmap_2d::InflationLayer")]
    common = dict(robot_base_frame=names["frames"]["base"], transform_tolerance=.25,
        footprint=footprint, footprint_padding=.03, track_unknown_space=True,
        always_send_full_costmap=True, plugins=layers,
        static_layer=dict(map_topic=names["map_topic"], first_map_only=True,
                          subscribe_to_updates=False, track_unknown_space=True,
                          use_maximum=False, trinary_costmap=True),
        obstacle_layer=dict(observation_sources="lidar", combination_method=1,
            obstacle_range=2.5, raytrace_range=3., observation_persistence=0.,
            lidar=dict(topic=names["scan_topic"], data_type="LaserScan", marking=True,
                       clearing=True, expected_update_rate=.2, observation_persistence=0.)),
        inflation_layer=dict(inflation_radius=.35, cost_scaling_factor=3.))
    global_map = deepcopy(common)
    global_map.update(global_frame=names["frames"]["map"], rolling_window=False,
                      resolution=.05, update_frequency=5., publish_frequency=2.)
    local_map = deepcopy(common)
    local_map.update(global_frame=names["frames"]["odom"], rolling_window=True,
                     resolution=.01, width=3., height=3., update_frequency=5., publish_frequency=2.)
    return dict(base_global_planner="global_planner/GlobalPlanner",
        base_local_planner="dwa_local_planner/DWAPlannerROS", controller_frequency=5.,
        planner_frequency=0., recovery_behavior_enabled=False, clearing_rotation_allowed=False,
        recovery_behaviors=[], shutdown_costmaps=False, make_plan_clear_costmap=False,
        make_plan_add_unreachable_goal=False,
        GlobalPlanner=dict(allow_unknown=False, default_tolerance=0.),
        global_costmap=global_map, local_costmap=local_map,
        DWAPlannerROS=dict(odom_topic=names["odom_topic"], max_vel_x=0., min_vel_x=0.,
            max_vel_y=0., min_vel_y=0., max_vel_trans=0., min_vel_trans=0.,
            max_vel_theta=0., min_vel_theta=0., sim_time=4., vth_samples=21))


def grid_cell(grid, x, y):
    resolution = finite(grid["resolution"])
    width, height = grid["width"], grid["height"]
    if (resolution <= 0 or type(width) is not int or type(height) is not int
            or width <= 0 or height <= 0 or len(grid["data"]) != width * height):
        raise ValueError("Invalid grid metadata")
    ox, oy, yaw = [finite(value) for value in grid["origin"]]
    dx, dy = finite(x) - ox, finite(y) - oy
    gx = int(math.floor((math.cos(yaw) * dx + math.sin(yaw) * dy) / resolution))
    gy = int(math.floor((-math.sin(yaw) * dx + math.cos(yaw) * dy) / resolution))
    if not 0 <= gx < width or not 0 <= gy < height:
        raise ValueError("Path leaves an observed grid")
    value = grid["data"][gy * width + gx]
    if type(value) is not int or not -1 <= value <= 100:
        raise ValueError("Invalid grid cost")
    return value


def validate_path(path, start, goal, map_grid, costmap_grid, frame):
    for name, pose in (("start", start), ("goal", goal)):
        if (not 0 <= grid_cell(map_grid, pose[0], pose[1]) <= 25 or
                not 0 <= grid_cell(costmap_grid, pose[0], pose[1]) < 99):
            raise ValueError("Requested " + name + " is unknown/occupied/inscribed")
    if path.get("frame_id") not in ("", frame):
        raise ValueError("Plan header uses the wrong map frame")
    values = path.get("poses")
    if type(values) is not list or not 2 <= len(values) <= 100000:
        raise ValueError("A bounded nonempty path is required")
    points = []
    for value in values:
        if value.get("frame_id") != frame:
            raise ValueError("Every path pose must explicitly use this map frame")
        point = [finite(value[key], "path coordinate") for key in ("x", "y", "z")]
        yaw_from_quaternion(value["orientation"])
        points.append(point[:2])
    resolution = max(finite(map_grid["resolution"]), finite(costmap_grid["resolution"]))
    endpoint_tolerance, maximum_segment = 2. * resolution, 3. * resolution
    if math.hypot(points[0][0] - start[0], points[0][1] - start[1]) > endpoint_tolerance:
        raise ValueError("Path does not start at the requested current pose")
    if math.hypot(points[-1][0] - goal[0], points[-1][1] - goal[1]) > endpoint_tolerance:
        raise ValueError("Path does not reach the preview endpoint")
    length, max_cost, checks = 0., 0, 0
    spacing = min(map_grid["resolution"], costmap_grid["resolution"]) / 2.
    for index, point in enumerate(points):
        previous = points[index - 1] if index else point
        segment = math.hypot(point[0] - previous[0], point[1] - previous[1])
        if segment > maximum_segment:
            raise ValueError("Path has a discontinuous segment")
        length += segment
        count = max(1, int(math.ceil(segment / spacing)))
        for step in range(count + 1):
            t = step / count
            x, y = previous[0] + t * (point[0] - previous[0]), previous[1] + t * (point[1] - previous[1])
            if not 0 <= grid_cell(map_grid, x, y) <= 25:
                raise ValueError("Path center intersects unknown or occupied saved-map cells")
            cost = grid_cell(costmap_grid, x, y)
            if not 0 <= cost < 99:
                raise ValueError("Path center intersects unknown/lethal/inscribed live costmap")
            max_cost, checks = max(max_cost, cost), checks + 1
    return dict(points=points, path_length_m=length, pose_count=len(points),
        sampled_cell_checks=checks, maximum_path_cost=max_cost,
        endpoint_grid_tolerance_m=endpoint_tolerance, maximum_segment_m=maximum_segment,
        tolerances_are_path_representation_checks_not_arrival_policy=True,
        known_saved_map_and_nonlethal_live_costmap_centers=True,
        physical_clearance_validated=False, motion_feasibility_validated=False)


def stop_child(child):
    result = dict(pid=child.pid, exit_code=child.poll(), forced_kill=False, errors=[])
    try:
        if child.poll() is None:
            child.send_signal(signal.SIGINT)
            try:
                child.wait(timeout=4.)
            except subprocess.TimeoutExpired:
                child.terminate()
                try:
                    child.wait(timeout=2.)
                except subprocess.TimeoutExpired:
                    child.kill()
                    result["forced_kill"] = True
                    child.wait(timeout=2.)
        result["exit_code"] = child.wait(timeout=.1)
    except Exception as exc:
        result["errors"].append(type(exc).__name__ + ": " + str(exc))
    return result


def main(argv=None):
    started = time.monotonic()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--localization-directory", required=True)
    parser.add_argument("--nonce", required=True)
    parser.add_argument("--expected-boot-id", required=True)
    parser.add_argument("--forward-m", type=float, default=.5)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    directory = Path(args.localization_directory).resolve(strict=True)
    output = Path(args.output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    names = names_for_preview(args.nonce, str(os.getpid()))
    result = dict(schema="p2p-path-preview-v1", scope="historical_GlobalPlanner_path_preview_only",
        boot_id=args.expected_boot_id, nonce=args.nonce, map_id=SAVED_MAP_ID,
        preview_verified=False, motion_requested=False, hardware_execution_authorized=False,
        action_goal_published=False, cmd_vel_published_by_sidecar=False,
        serial_access_supported=False, localization_live_at_output=False,
        preview_namespace=names["namespace"], names=names, first_failure=None,
        cleanup_errors=[], make_plan_calls=0,
        final_heading_policy="undecided_current_heading_for_path_request_only")
    child, log, subscriptions, service_thread = None, None, [], None
    owns_parameters = False
    ros_ready = False
    try:
        forward_goal([0., 0., 0.], args.forward_m)
        if Path("/proc/sys/kernel/random/boot_id").read_text().strip() != args.expected_boot_id:
            raise RuntimeError("RK boot changed")
        map_doc = loads_config((directory / "maps/map.json").read_text())
        info, cells, identity = verified_map(map_doc, (directory / "maps/map.pgm").read_bytes(), SAVED_MAP_ID)
        saved_grid = dict(info, data=cells)
        site = loads_config((directory / "site_config.json").read_text())
        parameters = planner_parameters(names, site)
        result.update(saved_map_identity=identity, planner_parameters=parameters,
            geometry_basis="site_estimates_not_measured_and_not_new_navigation_defaults",
            local_planner_initialized_but_not_exercised=True)
        import rospy
        import tf2_ros
        from nav_msgs.msg import OccupancyGrid
        from nav_msgs.srv import GetPlan, GetPlanRequest
        from geometry_msgs.msg import PoseStamped
        from geometry_msgs.msg import PoseWithCovarianceStamped
        from sensor_msgs.msg import LaserScan
        rospy.init_node("p2p_path_preview_" + args.nonce + "_" + str(os.getpid()), disable_signals=False)
        ros_ready = True
        if rospy.get_param("/use_sim_time", False) or rospy.has_param(names["namespace"]):
            raise RuntimeError("Actual ROS time and a fresh preview namespace required")
        rospy.set_param(names["node"], parameters)
        owns_parameters = True
        lock, current = threading.RLock(), {}

        def received(name):
            def callback(message):
                with lock:
                    current[name] = message
            return callback

        subscriptions = [rospy.Subscriber(names["map_topic"], OccupancyGrid, received("map"), queue_size=1),
                         rospy.Subscriber(names["costmap_topic"], OccupancyGrid, received("costmap"), queue_size=1),
                         rospy.Subscriber(names["amcl_topic"], PoseWithCovarianceStamped, received("amcl"), queue_size=1),
                         rospy.Subscriber(names["scan_topic"], LaserScan, received("scan"), queue_size=1)]
        tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.))
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        log = output.with_name(output.stem + "_move_base.log").open("w")
        command = ["/opt/ros/noetic/lib/move_base/move_base", "__name:=move_base", "__ns:=" + names["namespace"],
            "cmd_vel:=" + names["shadow_topic"], "map:=" + names["map_topic"],
            "odom:=" + names["odom_topic"],
            "move_base_simple/goal:=" + names["namespace"] + "/simple_goal_unused"]
        result["child_command"] = command
        child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=log)
        result["child_pid"] = child.pid
        live_map_verified = False
        cached_map = None

        def grid_from_message(message):
            q = message.info.origin.orientation
            return dict(width=int(message.info.width), height=int(message.info.height),
                resolution=float(message.info.resolution),
                origin=[message.info.origin.position.x, message.info.origin.position.y,
                        yaw_from_quaternion(dict(x=q.x, y=q.y, z=q.z, w=q.w))], data=list(message.data))

        while time.monotonic() - started < 85. and not rospy.is_shutdown():
            if child.poll() is not None:
                raise RuntimeError("Own move_base exited during warmup: " + str(child.returncode))
            with lock:
                live_map, costmap = current.get("map"), current.get("costmap")
                live_amcl, live_scan = current.get("amcl"), current.get("scan")
            if live_map is not None and live_map is not cached_map:
                if live_map.header.frame_id != names["frames"]["map"]:
                    raise RuntimeError("Private map source frame differs")
                live_grid = grid_from_message(live_map)
                if map_identity({key: live_grid[key] for key in ("width", "height", "resolution", "origin")}, live_grid["data"]) != SAVED_MAP_ID:
                    raise RuntimeError("Actual private map identity differs")
                cached_map, live_map_verified = live_map, True
            try:
                rospy.wait_for_service(names["service"], timeout=.1)
                service_ready = True
            except rospy.ROSException:
                service_ready = False
            costmap_ready = (costmap is not None and costmap.header.frame_id == names["frames"]["map"]
                             and 0 <= rospy.Time.now().to_sec() - costmap.header.stamp.to_sec() <= 2.)
            if (live_map_verified and service_ready and costmap_ready and live_amcl is not None
                    and live_scan is not None and (directory / "joint_evidence.json").is_file() and
                    tf_buffer.can_transform(names["frames"]["map"], names["frames"]["base"], rospy.Time(0), rospy.Duration(0))):
                break
            time.sleep(.05)
        else:
            raise RuntimeError("Private planner/map/costmap/TF and new joint evidence did not warm within 85s")
        graph = rospy.get_master().getSystemState()
        if graph[0] != 1:
            raise RuntimeError("ROS graph unavailable")
        publishers, subscribers = dict(graph[2][0]), dict(graph[2][1])
        if publishers.get(names["map_topic"]) != [names["map_publisher"]]:
            raise RuntimeError("Private map publisher ownership differs")
        if publishers.get(names["costmap_topic"]) != [names["node"]]:
            raise RuntimeError("Private costmap publisher ownership differs")
        if subscribers.get(names["shadow_topic"]):
            raise RuntimeError("shadow_unused unexpectedly has subscribers")
        result["map_live_identity_verified"] = True
        for key in ("make_plan_clear_costmap", "make_plan_add_unreachable_goal"):
            if rospy.get_param(names["node"] + "/" + key) is not False:
                raise RuntimeError("No-clearing/no-unreachable planner parameter differs")

        def read_joint():
            path = directory / "joint_evidence.json"
            if path.is_symlink() or path.stat().st_size > 1024 * 1024:
                raise ValueError("Joint file must be bounded/current-session data")
            raw = loads_config(path.read_text())
            return raw, validate_joint(raw, args.expected_boot_id, args.nonce,
                                      time.monotonic(), rospy.Time.now().to_sec())

        joint, checked = read_joint()  # Only AFTER actual planner warmup.
        start, goal = checked["pose"], forward_goal(checked["pose"], args.forward_m)

        def header_record(header):
            return dict(frame_id=header.frame_id, stamp_secs=int(header.stamp.secs),
                        stamp_nsecs=int(header.stamp.nsecs))

        def live_checks(joint_value, pose):
            with lock:
                amcl, scan = current.get("amcl"), current.get("scan")
            if amcl is None or scan is None:
                raise ValueError("Direct current AMCL/scan missing")
            p, q = amcl.pose.pose.position, amcl.pose.pose.orientation
            amcl_value = dict(header=header_record(amcl.header), covariance=list(amcl.pose.covariance),
                position=dict(x=p.x, y=p.y), orientation=dict(x=q.x, y=q.y, z=q.z, w=q.w))
            scan_value = dict(header=header_record(scan.header), ranges=list(scan.ranges),
                angle_min=scan.angle_min, angle_increment=scan.angle_increment,
                range_min=scan.range_min, range_max=scan.range_max)
            transform = tf_buffer.lookup_transform(names["frames"]["map"], names["frames"]["base"], rospy.Time(0), rospy.Duration(0))
            q, t = transform.transform.rotation, transform.transform.translation
            tf_record = dict(parent=transform.header.frame_id, child=transform.child_frame_id,
                source_stamp=transform.header.stamp.to_sec(), translation=dict(x=t.x, y=t.y),
                rotation=dict(x=q.x, y=q.y, z=q.z, w=q.w))
            now_ros = rospy.Time.now().to_sec()
            return dict(tf=tf_record, tf_agreement=validate_live_tf(tf_record, pose, names["frames"], now_ros),
                amcl=validate_live_amcl(amcl_value, joint_value, names["frames"], now_ros),
                scan=validate_live_scan(scan_value, pose, names["frames"], now_ros, info, cells))

        result.update(joint_before_request=joint, start_pose=start, goal_pose=goal,
                      forward_m=args.forward_m, live_before_request=live_checks(joint, start))
        with lock:
            before_costmap = current.get("costmap")
        for name, value in (("start", start), ("goal", goal)):
            if (not 0 <= grid_cell(saved_grid, value[0], value[1]) <= 25 or
                    not 0 <= grid_cell(grid_from_message(before_costmap), value[0], value[1]) < 99):
                raise ValueError("Requested " + name + " is unknown/occupied/inscribed before planning")
        request = GetPlanRequest()
        for target, pose in ((request.start, start), (request.goal, goal)):
            target.header.frame_id = names["frames"]["map"]
            target.header.stamp = rospy.Time.now()
            target.pose.position.x, target.pose.position.y = pose[:2]
            target.pose.orientation.z, target.pose.orientation.w = math.sin(pose[2] / 2.), math.cos(pose[2] / 2.)
        request.tolerance = 0.
        finished, response, service_errors = threading.Event(), [], []

        def call_plan_once():
            try:
                response.append(rospy.ServiceProxy(names["service"], GetPlan, persistent=False)(request))
            except Exception as exc:
                service_errors.append(type(exc).__name__ + ": " + str(exc))
            finally:
                finished.set()

        result["make_plan_calls"] = 1
        service_thread = threading.Thread(target=call_plan_once, name="p2p-single-getplan", daemon=True)
        service_thread.start()
        remaining = min(5., 90. - (time.monotonic() - started))
        if remaining <= 0 or not finished.wait(remaining):
            raise RuntimeError("Single make_plan service call timed out; no retry")
        if service_errors:
            raise RuntimeError("Single make_plan failed: " + service_errors[0])
        after_raw, after = read_joint()
        yaw_change = abs(math.atan2(math.sin(after["pose"][2] - start[2]), math.cos(after["pose"][2] - start[2])))
        if math.hypot(after["pose"][0] - start[0], after["pose"][1] - start[1]) > .03 or yaw_change > .05:
            raise RuntimeError("Joint pose moved during planning")
        result["live_after_request"] = live_checks(after_raw, start)
        plan = response[0].plan
        path = dict(frame_id=plan.header.frame_id, poses=[])
        for value in plan.poses:
            p, q = value.pose.position, value.pose.orientation
            path["poses"].append(dict(frame_id=value.header.frame_id, x=p.x, y=p.y, z=p.z,
                                      orientation=dict(x=q.x, y=q.y, z=q.z, w=q.w)))
        with lock:
            costmap = current.get("costmap")
        if (costmap is None or costmap.header.frame_id != names["frames"]["map"] or
                not 0 <= rospy.Time.now().to_sec() - costmap.header.stamp.to_sec() <= 2.):
            raise RuntimeError("Planner costmap is no longer current")
        result["path"] = validate_path(path, start, goal, saved_grid, grid_from_message(costmap), names["frames"]["map"])
        result["joint_after_request"] = after
        if Path("/proc/sys/kernel/random/boot_id").read_text().strip() != args.expected_boot_id:
            raise RuntimeError("Boot identity changed")
        result["preview_verified"] = True
        _ = tf_listener, PoseStamped
    except BaseException as exc:
        result["first_failure"] = dict(type=type(exc).__name__, message=str(exc))
    finally:
        if child is not None:
            result["child_cleanup"] = stop_child(child)
            cleanup = result["child_cleanup"]
            if cleanup["forced_kill"] or cleanup["errors"] or cleanup["exit_code"] not in (0, -2):
                result["cleanup_errors"].append("Own move_base cleanup was not a clean SIGINT/zero exit")
        if service_thread is not None:
            service_thread.join(timeout=.2)
            result["service_thread_still_alive"] = service_thread.is_alive()
            if service_thread.is_alive():
                result["cleanup_errors"].append("Single service call thread did not exit")
        for subscription in subscriptions:
            try:
                subscription.unregister()
            except Exception as exc:
                result["cleanup_errors"].append(type(exc).__name__ + ": " + str(exc))
        if ros_ready and owns_parameters:
            try:
                if rospy.has_param(names["namespace"]):
                    rospy.delete_param(names["namespace"])
            except Exception as exc:
                result["cleanup_errors"].append(type(exc).__name__ + ": " + str(exc))
        if log is not None:
            log.close()
        if result["cleanup_errors"]:
            result["preview_verified"] = False
        result["elapsed_s"] = time.monotonic() - started
        output.write_text(json.dumps(result, indent=2, allow_nan=False), encoding="utf-8")
    print(json.dumps(dict(output=str(output), preview_verified=result["preview_verified"],
                         first_failure=result["first_failure"], motion_requested=False), allow_nan=False))
    return 0 if result["preview_verified"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
