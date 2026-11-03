"""Offline front-180-degree navigation scan crop; no ROS or device I/O."""
import math


def _finite(value, name):
    try:
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(name + " must be finite")
    except OverflowError:
        raise ValueError(name + " must be finite")
    return value


def front_180_scan(ranges, angle_min, angle_increment, source_stamp,
                   lidar_yaw_rad=0.):
    """Mask rear beams as NaN, preserving layout and the original source stamp.

    Angles are radians in the laser frame; lidar_yaw_rad is the measured/assumed
    mounting yaw in base_link. NaN and Inf in the visible sector are preserved;
    downstream health/obstacle consumers must interpret them, never this crop.
    """
    for name, value in (("angle_min", angle_min), ("angle_increment", angle_increment),
                        ("source_stamp", source_stamp), ("lidar_yaw_rad", lidar_yaw_rad)):
        _finite(value, name)
    if angle_increment == 0 or source_stamp < 0:
        raise ValueError("Nonzero angular increment and nonnegative source stamp required")
    if type(ranges) not in (list, tuple) or not ranges:
        raise ValueError("A nonempty sequence of ranges is required")
    cropped, visible = [], 0
    for index, distance in enumerate(ranges):
        if type(distance) not in (int, float):
            raise ValueError("Ranges must be numeric (NaN/Inf are permitted)")
        angle = angle_min + index * angle_increment + lidar_yaw_rad
        _finite(angle, "beam angle")
        angle = math.atan2(math.sin(angle), math.cos(angle))
        if abs(angle) <= math.pi / 2 + 1e-12:
            cropped.append(distance)
            visible += 1
        else:
            cropped.append(float("nan"))
    return dict(ranges=cropped, angle_min=angle_min, angle_increment=angle_increment,
                source_stamp=source_stamp, navigation_fov_deg=180,
                geometrically_visible_beams=visible, rear_is_unobserved=True,
                mounting_yaw_rad=lidar_yaw_rad, source_stamp_preserved=True)
