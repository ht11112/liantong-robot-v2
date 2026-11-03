"""Odom-frame point-to-point command intent, without IO or a motion owner.

x is forward and y is left. Turn in place, then drive with equal physical
wheel speeds. A zero Command is a request, never proof of measured stopping.
The outer owner must enforce stop/turn/drive transitions and real feedback.
"""
from dataclasses import dataclass
import math


def _finite(value, name):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except (OverflowError, TypeError, ValueError):
        valid = False
    if not valid:
        raise ValueError(name + ' must be a finite number')
    return float(value)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


@dataclass(frozen=True)
class Config:
    max_linear_m_s: float = .03
    max_angular_rad_s: float = math.radians(6.9)
    minimum_linear_m_s: float = math.pi*.2/60.  # Nominal 1 physical RPM.
    minimum_angular_rad_s: float = math.radians(4.)  # D=.2, B=.3, +/-1 RPM.
    arrival_tolerance_m: float = .1
    duration_s: float = 200.
    align_tolerance_rad: float = math.radians(3.)
    realign_threshold_rad: float = math.radians(8.)
    slowdown_distance_m: float = .3
    slowdown_angle_rad: float = math.radians(20.)
    wheel_diameter_m: float = .2
    wheel_separation_m: float = .3

    def __post_init__(self):
        for name in self.__dataclass_fields__:
            value = _finite(getattr(self, name), name)
            if value <= 0:
                raise ValueError(name + ' must be positive')
            object.__setattr__(self, name, value)
        if self.max_linear_m_s > .03 or self.max_angular_rad_s > math.radians(6.9):
            raise ValueError('Configured speeds exceed the selected .03m/s / 6.9deg/s caps')
        if self.minimum_linear_m_s > self.max_linear_m_s or self.minimum_angular_rad_s > self.max_angular_rad_s:
            raise ValueError('Minimum commanded speed must not exceed its maximum')
        if self.arrival_tolerance_m > .1 or self.duration_s > 200.:
            raise ValueError('Arrival tolerance <=.1m and duration <=200s required')
        if not self.align_tolerance_rad < self.realign_threshold_rad < math.pi/2:
            raise ValueError('Require align tolerance < realign threshold <90deg')
        if self.slowdown_distance_m <= self.arrival_tolerance_m:
            raise ValueError('Slowdown distance must exceed arrival tolerance')
        if not self.align_tolerance_rad <= self.slowdown_angle_rad <= math.pi:
            raise ValueError('Slowdown angle must be between alignment tolerance and pi')


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    yaw: float

    def __post_init__(self):
        for name in ('x', 'y', 'yaw'):
            object.__setattr__(self, name, _finite(getattr(self, name), 'pose.'+name))


@dataclass(frozen=True)
class Command:
    phase: str
    linear_m_s: float
    angular_rad_s: float
    reason: str
    distance_m: float
    heading_error_rad: float


class SimplePointController:
    """Single target, no reverse or final-yaw goal. Terminal commands latch.

    The speed minima are continuous intentions for nominal geometry. Integer
    wheel quantization and all physical/source/obstacle checks belong to the
    owner and must still respect the configured maxima.
    """
    def __init__(self, goal_x, goal_y, config=Config(), started_at=0):
        if not isinstance(config, Config):
            raise ValueError('A validated Config is required')
        self.goal_x = _finite(goal_x, 'goal_x')
        self.goal_y = _finite(goal_y, 'goal_y')
        self.started_at = _finite(started_at, 'started_at')
        if self.started_at < 0:
            raise ValueError('started_at must be nonnegative')
        self.config = config
        self._last_now = self.started_at
        self._phase, self._terminal = 'turn', None
        self._cancelled = False
        self._distance = self._heading = 0.

    def cancel(self):
        """Latch cancellation intent; tick returns zero, not a physical STOP ACK."""
        self._cancelled = True

    def _command(self, phase, linear, angular, reason):
        return Command(phase, linear, angular, reason, self._distance, self._heading)

    def _finish(self, phase, reason):
        self._terminal = self._command(phase, 0., 0., reason)
        return self._terminal

    def tick(self, pose, now):
        if self._terminal is not None:
            return self._terminal
        try:
            now = _finite(now, 'now')
        except ValueError:
            return self._finish('fault', 'invalid_time')
        if now < self._last_now:
            return self._finish('fault', 'time_went_backwards')
        self._last_now = now
        if self._cancelled:
            return self._finish('stop', 'cancelled')
        if now-self.started_at >= self.config.duration_s:
            return self._finish('fault', 'timeout')
        if not isinstance(pose, Pose):
            return self._finish('fault', 'invalid_pose')
        dx, dy = self.goal_x-pose.x, self.goal_y-pose.y
        distance = math.hypot(dx, dy)
        if not math.isfinite(distance):
            return self._finish('fault', 'invalid_geometry')
        self._distance = distance
        self._heading = _wrap(math.atan2(dy, dx)-pose.yaw)
        if distance <= self.config.arrival_tolerance_m:
            return self._finish('stop', 'arrived')
        error = abs(self._heading)
        if self._phase == 'drive' and error > self.config.realign_threshold_rad:
            self._phase = 'turn'
            # One explicit zero intent before turning. The owner must wait for
            # measured zero independently; this tick cannot prove it occurred.
            return self._command('turn', 0., 0., 'realigning')
        if self._phase == 'turn' and error <= self.config.align_tolerance_rad:
            self._phase = 'drive'
        if self._phase == 'turn':
            angular = max(self.config.minimum_angular_rad_s,
                self.config.max_angular_rad_s*min(1., error/self.config.slowdown_angle_rad))
            return self._command('turn', 0., math.copysign(angular, self._heading), 'aligning')
        linear = max(self.config.minimum_linear_m_s,
            self.config.max_linear_m_s*min(1., distance/self.config.slowdown_distance_m))
        return self._command('drive', linear, 0., 'approaching')
