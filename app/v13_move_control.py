"""Pure MOVE line tracking; metres, radians, and physical wheel millirpm.

The defaults are provisional trial settings, not calibrated vehicle gains.
Pose and angular velocity must already share the map/body frames and have
passed the runtime's timestamp and session checks. This module has no clock,
I/O, state estimator, or authority to start a segment.
"""

from dataclasses import dataclass
import math


WHEEL_DIAMETER_M = .20
WHEEL_TRACK_M = .30


def _angle(value):
    return math.atan2(math.sin(value), math.cos(value))


@dataclass(frozen=True)
class LineReference:
    x: float
    y: float
    yaw: float
    length_m: float

    @classmethod
    def from_plan(cls, plan, index):
        """Reconstruct a zero-based MOVE's nominal line from the exact queue.

        Reconstructing preceding TURNs retains their residual heading error;
        using the measured heading at MOVE entry would erase that error.
        """
        segment = plan['segments'][index]
        if segment['kind'] != 'move':
            raise ValueError('LineReference requires a MOVE segment')
        x, y, yaw = (plan['start'][key] for key in ('x', 'y', 'yaw'))
        for previous in plan['segments'][:index]:
            if previous['kind'] == 'turn':
                yaw = _angle(yaw + previous['angle_rad'])
            else:
                x += previous['distance_m'] * math.cos(yaw)
                y += previous['distance_m'] * math.sin(yaw)
        return cls(x, y, _angle(yaw), segment['distance_m'])


@dataclass
class MoveController:
    """Track one planned line with a bounded heading return and gyro damping.

    Default lookahead is 0.5 m, maximum return angle 8 degrees, heading gain
    0.8 /s, yaw-rate damping 0.3, and wheel difference limit 1000 millirpm.
    The caller may select another validated trial limit, up to the MCU's
    2000 millirpm limit. Gains, geometry, and deadbands are not calibration.
    """

    reference: LineReference
    lookahead_m: float = .5
    max_return_angle_rad: float = math.radians(8.)
    heading_gain: float = .8
    yaw_rate_gain: float = .3
    max_differential_millirpm: int = 1000
    heading_deadband_rad: float = math.radians(.2)
    cross_deadband_m: float = .005

    def command(self, pose, angular_z_rad_s):
        """Return diagnostics and signed physical right-minus-left wheel RPM.

        A positive differential turns left. The MCU applies it by reducing
        the inner wheel, retaining its forward base speed on the outer wheel.
        """
        line = self.reference
        dx, dy = pose['x'] - line.x, pose['y'] - line.y
        c, s = math.cos(line.yaw), math.sin(line.yaw)
        cross = -s * dx + c * dy
        along = c * dx + s * dy
        line_error = _angle(line.yaw - pose['yaw'])
        in_deadband = (abs(line_error) <= self.heading_deadband_rad
                       and abs(cross) <= self.cross_deadband_m)
        return_angle = max(-self.max_return_angle_rad, min(
            self.max_return_angle_rad, math.atan2(cross, self.lookahead_m)))
        desired_yaw = _angle(line.yaw - return_angle)
        heading_error = _angle(desired_yaw - pose['yaw'])
        omega = 0. if in_deadband else (
            self.heading_gain * heading_error - self.yaw_rate_gain * angular_z_rad_s)
        differential = omega * WHEEL_TRACK_M / (math.pi * WHEEL_DIAMETER_M) * 60000.
        differential = round(max(-self.max_differential_millirpm,
                                 min(self.max_differential_millirpm, differential)))
        return dict(differential_millirpm=differential, cross_track_m=cross,
                    along_track_m=along, desired_yaw_rad=desired_yaw,
                    heading_error_rad=heading_error)
