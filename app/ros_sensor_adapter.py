"""Read-only ROS sensor sampling and a private front-180 LaserScan relay.

Only run() imports ROS. This module never starts localization, publishes TF,
initial poses, navigation goals, velocities, heartbeats or motor commands.
Header source times and frame names are preserved, never replaced by relay time.
"""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import threading
import time

from scan_sector import front_180_scan


def _finite(value, name):
    if type(value) not in (int, float):
        raise ValueError(name + " must be finite")
    try:
        value = float(value)
    except (OverflowError, ValueError):
        raise ValueError(name + " must be finite")
    if not math.isfinite(value):
        raise ValueError(name + " must be finite")
    return value


def source_header(header):
    """Keep exact ROS sec/nsec identity alongside the convenience float time."""
    sequence = header.seq
    seconds, nanoseconds = header.stamp.secs, header.stamp.nsecs
    if type(sequence) is not int or not 0 <= sequence <= 0xffffffff:
        raise ValueError("Header sequence must be uint32")
    if (type(seconds) is not int or seconds < 0 or
            type(nanoseconds) is not int or not 0 <= nanoseconds < 1000000000):
        raise ValueError("Invalid ROS source stamp")
    if not isinstance(header.frame_id, str):
        raise ValueError("Header frame_id must be a string")
    return {"seq": sequence, "stamp_secs": seconds, "stamp_nsecs": nanoseconds,
            "source_stamp": _finite(seconds + nanoseconds / 1e9, "source_stamp"),
            "frame_id": header.frame_id}


def crop_scan_message(message, lidar_yaw_rad=0.):
    """Return a deep-copy LaserScan, preserving its header, frame and metadata.

    Duck typing permits pure offline fixtures. Unknown rear beams become NaN;
    the source message and all original angular/timing fields remain unchanged.
    """
    header = source_header(message.header)
    cropped = front_180_scan(list(message.ranges), message.angle_min,
                             message.angle_increment, header["source_stamp"],
                             lidar_yaw_rad)
    result = deepcopy(message)
    result.ranges = cropped["ranges"]
    return result


def _xyz(value):
    return {key: _finite(getattr(value, key), key) for key in ("x", "y", "z")}


def _quaternion(value):
    return {key: _finite(getattr(value, key), key) for key in ("x", "y", "z", "w")}


def _pose(value):
    return {"position": _xyz(value.position), "orientation": _quaternion(value.orientation)}


def scan_summary(message):
    minimum, maximum = _finite(message.range_min, "range_min"), _finite(message.range_max, "range_max")
    if not 0 <= minimum < maximum:
        raise ValueError("Invalid LaserScan range bounds")
    counts = {"finite_in_range": 0, "finite_out_of_range": 0, "nan": 0,
              "positive_inf": 0, "negative_inf": 0}
    nearest = None
    for raw in message.ranges:
        if type(raw) not in (int, float):
            raise ValueError("LaserScan ranges must be numeric")
        try:
            distance = float(raw)
        except OverflowError:
            raise ValueError("LaserScan range cannot be represented")
        if math.isnan(distance):
            counts["nan"] += 1
        elif math.isinf(distance):
            counts["positive_inf" if distance > 0 else "negative_inf"] += 1
        elif minimum <= distance <= maximum:
            counts["finite_in_range"] += 1
            nearest = distance if nearest is None else min(nearest, distance)
        else:
            counts["finite_out_of_range"] += 1
    return dict(beam_count=len(message.ranges), counts=counts,
                nearest_valid_range_m=nearest,
                angle_min=_finite(message.angle_min, "angle_min"),
                angle_max=_finite(message.angle_max, "angle_max"),
                angle_increment=_finite(message.angle_increment, "angle_increment"),
                range_min=minimum, range_max=maximum)


def odom_summary(message):
    return dict(child_frame_id=message.child_frame_id,
                pose=_pose(message.pose.pose),
                twist={"linear": _xyz(message.twist.twist.linear),
                       "angular": _xyz(message.twist.twist.angular)})


def amcl_summary(message):
    covariance = [_finite(value, "AMCL covariance") for value in message.pose.covariance]
    if len(covariance) != 36:
        raise ValueError("AMCL covariance must have 36 entries")
    return dict(pose=_pose(message.pose.pose), covariance=covariance,
                localization_quality_assessed=False)


def map_summary(message):
    info = message.info
    if (type(info.width) is not int or type(info.height) is not int or
            info.width <= 0 or info.height <= 0 or len(message.data) != info.width * info.height):
        raise ValueError("OccupancyGrid dimensions/data length differ")
    resolution = _finite(info.resolution, "map resolution")
    if resolution <= 0:
        raise ValueError("Map resolution must be positive")
    unknown, free, other = 0, 0, 0
    for cell in message.data:
        if type(cell) is not int or not -1 <= cell <= 100:
            raise ValueError("Invalid OccupancyGrid cell")
        if cell == -1:
            unknown += 1
        elif cell == 0:
            free += 1
        else:
            other += 1
    return dict(width=info.width, height=info.height, resolution=resolution,
                origin=_pose(info.origin), cell_count=len(message.data),
                unknown_cells=unknown, zero_cost_cells=free,
                nonzero_known_cells=other, map_identity_verified=False)


def tf_summary(message):
    if not isinstance(message.child_frame_id, str) or not message.child_frame_id:
        raise ValueError("TF child frame must be nonempty")
    return {"child_frame_id": message.child_frame_id,
            "translation": _xyz(message.transform.translation),
            "rotation": _quaternion(message.transform.rotation)}


class SourceRegistry:
    """Observe new source stamps; duplicate receives never renew source evidence.

    Counts and latest payload are diagnostic, not navigation readiness. A TF may
    intentionally carry a future stamp; its negative age is retained and flagged.
    Zero-stamped static data has unknown age, never a fabricated receipt age.
    """
    def __init__(self):
        self.channels = {}

    def observe(self, name, header, payload, received_monotonic, static=False,
                received_ros=None):
        received = _finite(received_monotonic, "received_monotonic")
        original = source_header(header)
        channel = self.channels.setdefault(name, dict(received=0, new_sources=0,
            duplicates=0, reordered=0, latest=None, static=bool(static),
            max_source_age_at_receive_s=None))
        channel["received"] += 1
        source_age = None
        if received_ros is not None:
            received_ros = _finite(received_ros, "received_ros")
            if original["source_stamp"] != 0:
                source_age = received_ros - original["source_stamp"]
                old_max = channel["max_source_age_at_receive_s"]
                channel["max_source_age_at_receive_s"] = (source_age if old_max is None
                                                          else max(source_age, old_max))
        latest = channel["latest"]
        current_key = (original["stamp_secs"], original["stamp_nsecs"])
        if latest is not None:
            previous_key = (latest["header"]["stamp_secs"], latest["header"]["stamp_nsecs"])
            if current_key == previous_key:
                channel["duplicates"] += 1
                return False
            if current_key < previous_key:
                channel["reordered"] += 1
                return False
        channel["latest"] = dict(header=original, received_monotonic=received,
                                  received_ros=received_ros,
                                  source_age_at_receive_s=source_age,
                                  payload=deepcopy(payload))
        channel["new_sources"] += 1
        return True

    def report(self, now_ros, now_monotonic, expected=()):
        now_ros = _finite(now_ros, "now_ros")
        now_monotonic = _finite(now_monotonic, "now_monotonic")
        channels = deepcopy(self.channels)
        for name in expected:
            channels.setdefault(name, dict(received=0, new_sources=0, duplicates=0,
                                           reordered=0, latest=None, static=False))
        for channel in channels.values():
            latest = channel["latest"]
            channel["missing"] = latest is None
            if latest is None:
                continue
            source = latest["header"]["source_stamp"]
            latest["source_age_s"] = None if source == 0 else now_ros - source
            latest["source_time_known"] = source != 0
            latest["future_source_stamp"] = source > now_ros
            latest["receive_age_s"] = now_monotonic - latest["received_monotonic"]
        return channels


def validate_runtime_options(duration, output_scan_topic):
    duration = _finite(duration, "duration")
    if duration <= 0:
        raise ValueError("duration must be positive")
    if (not isinstance(output_scan_topic, str) or not output_scan_topic.startswith("~") or
            len(output_scan_topic) <= 1):
        raise ValueError("Output scan must be a private ~topic")
    return duration


def run(args):
    """Bounded live ROS sampling. Only the private LaserScan output is published."""
    duration = validate_runtime_options(args.duration, args.output_scan_topic)
    _finite(args.lidar_yaw_rad, "lidar_yaw_rad")
    import rospy
    from nav_msgs.msg import OccupancyGrid, Odometry
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from sensor_msgs.msg import LaserScan
    from tf2_msgs.msg import TFMessage

    rospy.init_node("p2p_sensor_observer", anonymous=True, disable_signals=False)
    input_topics = {"scan": args.scan_topic, "map": args.map_topic,
                    "odom": args.odom_topic, "amcl": args.amcl_topic,
                    "tf": args.tf_topic, "tf_static": args.tf_static_topic}
    output_topic = rospy.resolve_name(args.output_scan_topic)
    resolved_inputs = {name: rospy.resolve_name(topic) for name, topic in input_topics.items()}
    if (not output_topic.startswith(rospy.get_name().rstrip("/") + "/") or
            output_topic in resolved_inputs.values()):
        raise ValueError("Resolved scan output must remain private and distinct from inputs")
    registry, lock, errors = SourceRegistry(), threading.RLock(), []
    accepting = [True]
    tf_messages = {"dynamic": 0, "static": 0}
    published = [0]
    publisher = rospy.Publisher(args.output_scan_topic, LaserScan, queue_size=1, latch=False)
    subscriptions = []
    started = time.monotonic()
    runtime_error = None

    def error(channel, exc):
        with lock:
            if accepting[0] and len(errors) < 20:
                errors.append(dict(channel=channel, received_monotonic=time.monotonic(),
                                   error=type(exc).__name__ + ": " + str(exc)))

    def sensor_callback(name, summarize, is_map=False):
        def receive(message):
            received = time.monotonic()
            received_ros = rospy.Time.now().to_sec()
            try:
                payload = summarize(message)
                with lock:
                    if accepting[0]:
                        registry.observe(name, message.header, payload, received,
                                         static=is_map, received_ros=received_ros)
            except Exception as exc:
                error(name, exc)
        return receive

    def scan_callback(message):
        received = time.monotonic()
        received_ros = rospy.Time.now().to_sec()
        try:
            cropped = crop_scan_message(message, args.lidar_yaw_rad)
            raw_payload, crop_payload = scan_summary(message), scan_summary(cropped)
            with lock:
                if not accepting[0]:
                    return
                fresh = registry.observe("scan", message.header, raw_payload, received,
                                         received_ros=received_ros)
                registry.observe("front180", cropped.header, crop_payload, received,
                                 received_ros=received_ros)
                # Preserve source stamp/frame. rospy may regenerate output header.seq;
                # registry diagnostics above keep the original input sequence.
                if fresh:
                    publisher.publish(cropped)
                    published[0] += 1
        except Exception as exc:
            error("scan", exc)

    def tf_callback(static):
        def receive(message):
            received = time.monotonic()
            received_ros = rospy.Time.now().to_sec()
            with lock:
                if not accepting[0]:
                    return
                tf_messages["static" if static else "dynamic"] += 1
            for transform in message.transforms:
                try:
                    payload = tf_summary(transform)
                    key = ("tf_static:" if static else "tf:") + transform.header.frame_id + "->" + transform.child_frame_id
                    with lock:
                        if accepting[0]:
                            registry.observe(key, transform.header, payload, received,
                                             static=static, received_ros=received_ros)
                except Exception as exc:
                    error("tf_static" if static else "tf", exc)
        return receive

    try:
        for key, kind, callback in (
                ("scan", LaserScan, scan_callback),
                ("map", OccupancyGrid, sensor_callback("map", map_summary, True)),
                ("odom", Odometry, sensor_callback("odom", odom_summary)),
                ("amcl", PoseWithCovarianceStamped, sensor_callback("amcl", amcl_summary)),
                ("tf", TFMessage, tf_callback(False)),
                ("tf_static", TFMessage, tf_callback(True))):
            subscriptions.append(rospy.Subscriber(input_topics[key], kind, callback, queue_size=1,
                                                   tcp_nodelay=True))
        deadline = started + duration
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            time.sleep(min(.05, max(0., deadline - time.monotonic())))
    except Exception as exc:
        runtime_error = type(exc).__name__ + ": " + str(exc)
    finally:
        with lock:
            accepting[0] = False
        for subscription in subscriptions:
            subscription.unregister()
        publisher.unregister()
    ended, ros_now = time.monotonic(), rospy.Time.now().to_sec()
    with lock:
        result = dict(scope="ros_readonly_sensor_sampling", hardware_execution_authorized=False,
            motion_commands_supported=False, localization_quality_assessed=False,
            requested_duration_s=duration, elapsed_s=ended - started,
            topics=resolved_inputs, output_scan_topic=output_topic,
            navigation_fov_deg=180, lidar_yaw_rad=args.lidar_yaw_rad,
            source_stamp_and_frame_preserved=True,
            output_header_seq_semantics="rospy_publisher_sequence",
            front180_diagnostic_is_prepublication=True, tf_published=False,
            use_sim_time=bool(rospy.get_param("/use_sim_time", False)),
            tf_streams={key: dict(messages_received=count, missing=count == 0)
                        for key, count in tf_messages.items()},
            published_front180_messages=published[0], channels=registry.report(
                ros_now, ended, expected=("scan", "front180", "map", "odom", "amcl")),
            callback_errors=deepcopy(errors), runtime_error=runtime_error,
            output_ros_time=ros_now, output_monotonic=ended)
    target = Path(args.output)
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(result, indent=2, sort_keys=True, allow_nan=False), encoding="utf-8")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scan-topic", required=True, help="Actual existing raw LaserScan topic")
    parser.add_argument("--output-scan-topic", default="~front180_scan")
    parser.add_argument("--map-topic", default="/map")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--amcl-topic", default="/amcl_pose")
    parser.add_argument("--tf-topic", default="/tf")
    parser.add_argument("--tf-static-topic", default="/tf_static")
    parser.add_argument("--lidar-yaw-rad", type=float, default=0., help="Explicit mounting yaw estimate in radians")
    parser.add_argument("--duration", type=float, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    try:
        result = run(args)
    except (ValueError, OSError, ImportError) as exc:
        parser.exit(2, type(exc).__name__ + ": " + str(exc) + "\n")
    print(json.dumps(dict(output=str(Path(args.output).resolve()),
                          runtime_error=result["runtime_error"],
                          published_front180_messages=result["published_front180_messages"],
                          hardware_execution_authorized=False), allow_nan=False))
    return 2 if result["runtime_error"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
