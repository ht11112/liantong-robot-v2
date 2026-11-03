"""Isolated localization scan/TF adapter; no serial or navigation commands.

ROS imports occur only in run(). The raw laser's known incorrect frame label is
explicitly replaced by the caller's isolated laser frame, preserving source
time and coordinates. Mounting values are assumptions, never a calibration.
This module neither starts AMCL nor publishes an initial pose.
"""
import argparse
from collections import deque
from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time

from scan_sector import front_180_scan
from ros_sensor_adapter import source_header


def finite(value, name):
    if type(value) not in (int, float):
        raise ValueError(name + " must be finite")
    try:
        result = float(value)
    except (ValueError, OverflowError):
        raise ValueError(name + " must be finite")
    if not math.isfinite(result):
        raise ValueError(name + " must be finite")
    return result


def positive(value, name):
    result = finite(value, name)
    if result <= 0:
        raise ValueError(name + " must be positive")
    return result


def validate_frames(laser_frame, base_frame, odom_frame, map_frame):
    frames = {"laser": laser_frame, "base": base_frame,
              "odom": odom_frame, "map": map_frame}
    for frame in frames.values():
        if (not isinstance(frame, str) or not frame.startswith("p2p_") or
                frame.startswith("/") or any(c.isspace() for c in frame)):
            raise ValueError("Frames must be explicit isolated p2p_* names without leading slash/space")
    if len(set(frames.values())) != 4:
        raise ValueError("Laser/base/odom/map frames must all differ")
    return frames


def mounting_assumption(x=.13, y=0., z=.2, yaw=0.):
    return dict(x_m=finite(x, "laser x"), y_m=finite(y, "laser y"),
                z_m=finite(z, "laser z"), yaw_rad=finite(yaw, "laser yaw"),
                assumed=True, measured=False, calibration_verified=False)


def corrected_scan(message, laser_frame, mounting_yaw_rad=0.):
    """Correct only the frame label and rear mask; do not transform scan points."""
    original = source_header(message.header)
    cropped = front_180_scan(list(message.ranges), message.angle_min,
                             message.angle_increment, original["source_stamp"],
                             mounting_yaw_rad)
    result = deepcopy(message)
    result.header.frame_id = laser_frame
    result.ranges = cropped["ranges"]
    return result


def xyz(value):
    return {key: finite(getattr(value, key), key) for key in ("x", "y", "z")}


def quaternion(value):
    result = {key: finite(getattr(value, key), key) for key in ("x", "y", "z", "w")}
    norm = math.sqrt(sum(number * number for number in result.values()))
    if not math.isfinite(norm) or abs(norm - 1.) > .001:
        raise ValueError("Pose quaternion must already be normalized")
    return result


def odom_transform(message, odom_frame, base_frame):
    """Pure TransformStamped payload using measured pose and exact input stamp."""
    original = source_header(message.header)
    if original["frame_id"] != odom_frame or message.child_frame_id != base_frame:
        raise ValueError("Measured odometry must already use the configured odom/base frames")
    return dict(header=original, child_frame_id=base_frame,
                translation=xyz(message.pose.pose.position),
                rotation=quaternion(message.pose.pose.orientation))


def amcl_diagnostics(message, map_frame):
    original = source_header(message.header)
    if original["frame_id"] != map_frame:
        raise ValueError("AMCL pose frame differs from configured map frame")
    covariance = [finite(value, "AMCL covariance") for value in message.pose.covariance]
    if len(covariance) != 36:
        raise ValueError("AMCL covariance must have 36 entries")
    diagonal = [covariance[index] for index in (0, 7, 14, 21, 28, 35)]
    if any(value < 0 for value in diagonal):
        raise ValueError("AMCL covariance has a negative variance")
    symmetry_error = max(abs(covariance[row * 6 + column] - covariance[column * 6 + row])
                         for row in range(6) for column in range(6))
    return dict(header=original, position=xyz(message.pose.pose.position),
                orientation=quaternion(message.pose.pose.orientation), covariance=covariance,
                position_std_m=[math.sqrt(covariance[0]), math.sqrt(covariance[7])],
                yaw_std_deg=math.degrees(math.sqrt(covariance[35])),
                covariance_diagonal_nonnegative=True,
                covariance_symmetry_max_abs_error=symmetry_error,
                covariance_positive_semidefinite_assessed=False,
                localization_validated=False)


class SourceGate:
    """Accept only advancing, nonfuture, current source stamps; never restamp."""
    def __init__(self, max_age_s, earliest_source=None):
        self.max_age_s = positive(max_age_s, "engineering max source age")
        self.earliest_source = (None if earliest_source is None
                                else finite(earliest_source, "earliest_source"))
        self.last_header = None
        self.accepted = deque(maxlen=1000)
        self.counts = {key: 0 for key in ("accepted", "duplicate", "reordered",
                                         "future", "stale", "zero_stamp", "before_session")}
        self.last_rejection = None

    def accept(self, header, now_ros):
        original = source_header(header)
        now_ros = finite(now_ros, "now_ros")
        source = original["source_stamp"]
        key = (original["stamp_secs"], original["stamp_nsecs"])
        reason = None
        if source <= 0:
            reason = "zero_stamp"
        elif source > now_ros:
            reason = "future"
        elif self.earliest_source is not None and source < self.earliest_source:
            reason = "before_session"
        elif now_ros - source >= self.max_age_s:
            reason = "stale"
        elif self.last_header is not None:
            old = (self.last_header["stamp_secs"], self.last_header["stamp_nsecs"])
            if key == old:
                reason = "duplicate"
            elif key < old:
                reason = "reordered"
        self.last_rejection = reason
        if reason:
            self.counts[reason] += 1
            return False
        self.last_header = original
        self.accepted.append(deepcopy(original))
        self.counts["accepted"] += 1
        return True

    def report(self, now_ros):
        now_ros = finite(now_ros, "now_ros")
        current = [item for item in self.accepted
                   if 0 <= now_ros - item["source_stamp"] < self.max_age_s]
        last = deepcopy(self.last_header)
        return dict(engineering_max_age_s=self.max_age_s, counts=dict(self.counts),
                    missing=last is None, latest_header=last,
                    latest_source_age_s=None if last is None else now_ros - last["source_stamp"],
                    current_distinct_sources=len(current),
                    five_current_distinct_sources=len(current) >= 5,
                    last_rejection=self.last_rejection,
                    source_headers_current=current[-5:], source_time_rewritten=False)


def _json_range(raw):
    if type(raw) not in (int, float):
        raise ValueError("Scan ranges must be numeric")
    try:
        value = float(raw)
    except OverflowError:
        raise ValueError("Scan range is outside finite model range")
    if math.isnan(value):
        return None
    if math.isinf(value):
        return "inf" if value > 0 else "-inf"
    return value


def scan_snapshot(raw, cropped):
    return dict(raw_header=source_header(raw.header), header=source_header(cropped.header),
                angle_min=finite(cropped.angle_min, "angle_min"),
                angle_max=finite(cropped.angle_max, "angle_max"),
                angle_increment=finite(cropped.angle_increment, "angle_increment"),
                time_increment=finite(cropped.time_increment, "time_increment"),
                scan_time=finite(cropped.scan_time, "scan_time"),
                range_min=finite(cropped.range_min, "range_min"),
                range_max=finite(cropped.range_max, "range_max"),
                ranges=[_json_range(value) for value in cropped.ranges],
                nan_encoding=None, positive_infinity_encoding="inf",
                negative_infinity_encoding="-inf", navigation_fov_deg=180,
                phase="prepublication", output_header_seq_semantics="rospy_publisher_sequence")


def write_json_atomic(path, value):
    """Called only by the main sampler thread, never a ROS callback."""
    target = Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    temporary = target.with_name(target.name + ".tmp")
    temporary.write_text(json.dumps(value, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
    temporary.replace(target)


def _assign_xyz(target, values):
    for name in ("x", "y", "z"):
        setattr(target, name, values[name])


def _assign_quaternion(target, values):
    for name in ("x", "y", "z", "w"):
        setattr(target, name, values[name])


def run(args):
    duration = positive(args.duration, "duration")
    frames = validate_frames(args.laser_frame, args.base_frame, args.odom_frame, args.map_frame)
    mounting = mounting_assumption(args.laser_x, args.laser_y, args.laser_z, args.laser_yaw)
    for budget in (args.scan_max_age_s, args.odom_max_age_s, args.amcl_max_age_s):
        positive(budget, "engineering max source age")
    if type(args.snapshot_scan_count) is not int or not 0 <= args.snapshot_scan_count <= 100:
        raise ValueError("snapshot_scan_count must be an integer in 0..100")
    import rospy
    import tf2_ros
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped

    rospy.init_node("p2p_localization_adapter", anonymous=True)
    if rospy.get_param("/use_sim_time", False):
        raise ValueError("Real localization observation requires actual ROS time")
    input_topics = {"scan": rospy.resolve_name(args.scan_topic),
                    "odom": rospy.resolve_name(args.odom_topic),
                    "amcl": rospy.resolve_name(args.amcl_topic)}
    output_topic = rospy.resolve_name(args.scan_output_topic)
    if not output_topic.startswith("/p2p_loc_") or output_topic in input_topics.values():
        raise ValueError("Scan output must be a distinct isolated /p2p_loc_* topic")
    publisher = rospy.Publisher(args.scan_output_topic, LaserScan, queue_size=1, latch=False)
    dynamic_tf = tf2_ros.TransformBroadcaster()
    static_tf = tf2_ros.StaticTransformBroadcaster()
    buffer = tf2_ros.Buffer(cache_time=rospy.Duration(max(30., duration + 10.)))
    listener = tf2_ros.TransformListener(buffer)
    start_ros, started = rospy.Time.now().to_sec(), time.monotonic()
    gates = {"scan": SourceGate(args.scan_max_age_s),
             "odom": SourceGate(args.odom_max_age_s),
             "amcl": SourceGate(args.amcl_max_age_s, earliest_source=start_ros)}
    lock, accepting = threading.RLock(), [True]
    snapshots, latest, errors = [], {}, []
    published = {"scan": 0, "odom_tf": 0, "static_tf": 0}
    error_count, subscriptions = [0], []
    tf_latest, tf_successes = {}, {}
    snapshot_revision = -1
    runtime_error = None

    static_message = TransformStamped()
    static_message.header.stamp = rospy.Time.now()
    static_message.header.frame_id = frames["base"]
    static_message.child_frame_id = frames["laser"]
    _assign_xyz(static_message.transform.translation,
                {"x": mounting["x_m"], "y": mounting["y_m"], "z": mounting["z_m"]})
    _assign_quaternion(static_message.transform.rotation,
                       dict(x=0., y=0., z=math.sin(mounting["yaw_rad"] / 2.),
                            w=math.cos(mounting["yaw_rad"] / 2.)))
    static_tf.sendTransform(static_message)
    published["static_tf"] = 1

    def error(kind, exc):
        with lock:
            if accepting[0]:
                error_count[0] += 1
                if len(errors) < 30:
                    errors.append(dict(channel=kind, at_monotonic=time.monotonic(),
                                       error=type(exc).__name__ + ": " + str(exc)))

    def scan_received(message):
        try:
            corrected = corrected_scan(message, frames["laser"], mounting["yaw_rad"])
            sample = scan_snapshot(message, corrected)
            with lock:
                if not accepting[0] or not gates["scan"].accept(message.header, rospy.Time.now().to_sec()):
                    return
                if len(snapshots) < args.snapshot_scan_count:
                    snapshots.append(sample)
                latest["scan"] = dict(raw_header=sample["raw_header"],
                                      corrected_header=sample["header"], beam_count=len(corrected.ranges))
                publisher.publish(corrected)
                published["scan"] += 1
        except Exception as exc:
            error("scan", exc)

    def odom_received(message):
        try:
            payload = odom_transform(message, frames["odom"], frames["base"])
            transform = TransformStamped()
            transform.header = deepcopy(message.header)
            transform.child_frame_id = frames["base"]
            _assign_xyz(transform.transform.translation, payload["translation"])
            _assign_quaternion(transform.transform.rotation, payload["rotation"])
            with lock:
                if not accepting[0] or not gates["odom"].accept(message.header, rospy.Time.now().to_sec()):
                    return
                dynamic_tf.sendTransform(transform)
                latest["odom"] = payload
                published["odom_tf"] += 1
        except Exception as exc:
            error("odom", exc)

    def amcl_received(message):
        try:
            payload = amcl_diagnostics(message, frames["map"])
            with lock:
                if not accepting[0] or not gates["amcl"].accept(message.header, rospy.Time.now().to_sec()):
                    return
                latest["amcl"] = payload
        except Exception as exc:
            error("amcl", exc)

    def capture_tf():
        for parent, child in ((frames["map"], frames["odom"]),
                              (frames["map"], frames["base"]),
                              (frames["odom"], frames["base"]),
                              (frames["base"], frames["laser"])):
            key = parent + "->" + child
            try:
                # Zero timeout: diagnostics must not wait on missing TF.
                value = buffer.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(0))
                header = source_header(value.header)
                tf_latest[key] = dict(available=True, header=header,
                    source_age_s=rospy.Time.now().to_sec() - header["source_stamp"],
                    translation=xyz(value.transform.translation),
                    rotation=quaternion(value.transform.rotation),
                    static=(parent == frames["base"] and child == frames["laser"]))
                tf_successes[key] = tf_successes.get(key, 0) + 1
            except Exception as exc:
                tf_latest[key] = dict(available=False, error=type(exc).__name__ + ": " + str(exc))

    def snapshot_payload():
        with lock:
            return dict(scope="current_scan_proposal_input_only", required_scan_count=args.snapshot_scan_count,
                captured_scan_count=len(snapshots), complete=len(snapshots) >= args.snapshot_scan_count,
                output_topic=output_topic, frames=frames, mounting=mounting,
                snapshots=deepcopy(snapshots), hardware_execution_authorized=False,
                source_time_rewritten=False, output_monotonic=time.monotonic())

    try:
        subscriptions = [rospy.Subscriber(input_topics["scan"], LaserScan, scan_received, queue_size=1, tcp_nodelay=True),
                         rospy.Subscriber(input_topics["odom"], Odometry, odom_received, queue_size=1, tcp_nodelay=True),
                         rospy.Subscriber(input_topics["amcl"], PoseWithCovarianceStamped, amcl_received, queue_size=1, tcp_nodelay=True)]
        while not rospy.is_shutdown() and time.monotonic() - started < duration:
            capture_tf()
            if args.scan_snapshot_output:
                with lock:
                    revision = len(snapshots)
                if revision != snapshot_revision:
                    write_json_atomic(args.scan_snapshot_output, snapshot_payload())
                    snapshot_revision = revision
            time.sleep(.05)
    except Exception as exc:
        runtime_error = type(exc).__name__ + ": " + str(exc)
    finally:
        with lock:
            accepting[0] = False
        for subscription in subscriptions:
            subscription.unregister()
        publisher.unregister()
    now_ros, ended = rospy.Time.now().to_sec(), time.monotonic()
    with lock:
        result = dict(scope="isolated_TF_saved_map_localization_observer", frames=frames,
            input_topics=input_topics, scan_output_topic=output_topic,
            assumed_mounting=mounting, engineering_age_budgets_s={"scan": args.scan_max_age_s,
                "odom": args.odom_max_age_s, "amcl": args.amcl_max_age_s},
            age_budgets_are_user_navigation_defaults=False,
            channels={name: gate.report(now_ros) for name, gate in gates.items()},
            latest=deepcopy(latest), publications=dict(published),
            tf_diagnostics=deepcopy(tf_latest), tf_lookup_success_counts=dict(tf_successes),
            callback_error_count=error_count[0], callback_errors=deepcopy(errors),
            runtime_error=runtime_error, elapsed_s=ended - started,
            source_stamp_preserved=True, output_header_seq_semantics="rospy_publisher_sequence",
            initialpose_published=False, navigation_commands_supported=False,
            hardware_execution_authorized=False, actual_map_localization_valid=False,
            static_mounting_stamp_semantics="publication_time_for_static_transform_only",
            scan_snapshot=snapshot_payload(), output_ros_time=now_ros,
            output_monotonic=ended)
    if args.scan_snapshot_output:
        write_json_atomic(args.scan_snapshot_output, result["scan_snapshot"])
    write_json_atomic(args.output, result)
    # Keep listener/broadcasters alive through final evidence collection.
    _ = listener, dynamic_tf, static_tf
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("scan-topic", "odom-topic", "scan-output-topic", "amcl-topic",
                 "laser-frame", "base-frame", "odom-frame", "map-frame", "output"):
        parser.add_argument("--" + name, required=True)
    for name in ("duration", "scan-max-age-s", "odom-max-age-s", "amcl-max-age-s"):
        parser.add_argument("--" + name, required=True, type=float)
    parser.add_argument("--laser-x", type=float, default=.13)
    parser.add_argument("--laser-y", type=float, default=0.)
    parser.add_argument("--laser-z", type=float, default=.2)
    parser.add_argument("--laser-yaw", type=float, default=0.)
    parser.add_argument("--snapshot-scan-count", type=int, default=3)
    parser.add_argument("--scan-snapshot-output")
    args = parser.parse_args(argv)
    try:
        result = run(args)
    except (ValueError, OSError, ImportError) as exc:
        parser.exit(2, type(exc).__name__ + ": " + str(exc) + "\n")
    print(json.dumps(dict(output=str(Path(args.output).resolve()),
        runtime_error=result["runtime_error"], publications=result["publications"],
        hardware_execution_authorized=False), allow_nan=False))
    return 2 if result["runtime_error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
