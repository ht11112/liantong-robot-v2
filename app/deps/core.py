"""Host-only safety policy and measured-speed odometry. No ROS or serial I/O."""
from dataclasses import dataclass
import math


def wheels_from_twist(v, w, diameter=.2, separation=.3):
    if not all(math.isfinite(x) for x in (v, w, diameter, separation)) or min(diameter, separation) <= 0:
        raise ValueError('Non-finite velocity or invalid geometry')
    factor = 60 / (math.pi * diameter)
    left = (v - w * separation / 2) * factor
    right = -(v + w * separation / 2) * factor
    scale = max(1., abs(left) / 5, abs(right) / 5)
    return int(round(left / scale)), int(round(right / scale))


@dataclass(frozen=True)
class Policy:
    enable_motion: bool = False
    site_motion_authorized: bool = False
    diameter: float = .2
    separation: float = .3
    cmd_timeout: float = .20
    operator_timeout: float = .35
    scan_timeout: float = .30
    feedback_timeout: float = .25

    def __post_init__(self):
        if type(self.enable_motion) is not bool or type(self.site_motion_authorized) is not bool:
            raise ValueError('Motion flags must be explicit booleans')
        for name, cap in (('cmd_timeout', .25), ('operator_timeout', .4),
                          ('scan_timeout', .35), ('feedback_timeout', .25)):
            if not math.isfinite(getattr(self, name)) or not .05 <= getattr(self, name) <= cap:
                raise ValueError('%s is outside the fail-closed bound' % name)
        if not all(math.isfinite(x) and .05 <= x <= 2 for x in (self.diameter, self.separation)):
            raise ValueError('Invalid geometry')


class SafetyController:
    """Caller holds one lock. A trip clears authority; new traffic cannot re-arm."""
    def __init__(self, policy=Policy(), converter=wheels_from_twist):
        self.policy, self.converter = policy, converter
        self.source = 'none'
        self.armed = self.arming = False
        self.generation = 0
        self.arm_epoch = None
        self.command = None
        self.operator_seen = self.scan_seen = self.feedback_seen = None
        self.operator_counter = None
        self.feedback_state = None
        self.feedback_signature = None
        self.reason = 'startup: explicit ARM required'
        self.stop_pending = True

    def trip(self, reason):
        self.generation += 1
        self.armed = self.arming = False
        self.command = None
        self.arm_epoch = None
        self.reason = reason
        self.stop_pending = True

    def select(self, source):
        if source not in ('none', 'nav', 'manual'):
            raise ValueError('Source must be none, nav or manual')
        self.trip('control source selected; explicit ARM required')
        self.source = source

    def heartbeat(self, counter, now):
        if type(counter) is not int or not 0 <= counter <= 0xffffffff:
            self.trip('invalid operator heartbeat')
            return False
        if self.operator_counter is not None:
            step = (counter - self.operator_counter) & 0xffffffff
            if not 0 < step <= 0x7fffffff:
                return False  # Replay/duplicate never refreshes the permit.
        self.operator_counter, self.operator_seen = counter, now
        return True

    def scan(self, now, valid=True):
        self.scan_seen = now if valid else None
        if not valid and (self.armed or self.arming):
            self.trip('invalid laser scan')

    def feedback(self, packet, now):
        # Runtime calls control_client.fresh first. Track progression separately.
        signature = (packet['sample_sequence'], packet['speed_tick_ms'])
        if signature != self.feedback_signature:
            self.feedback_seen, self.feedback_signature = now, signature
        self.feedback_state = packet['controller_state']
        if self.armed and self.feedback_state not in (5, 10):
            self.trip('MCU no longer armed')

    def receive_command(self, source, v, w, now):
        if source != self.source or not self.armed:
            return False
        if not all(math.isfinite(x) for x in (v, w)):
            self.trip('invalid velocity')
            return False
        # Commands received before/during ARM never become future movement.
        if self.arm_epoch is None or now <= self.arm_epoch:
            return False
        self.command = (v, w, now)
        return True

    def readiness_error(self, now):
        if not self.policy.enable_motion or not self.policy.site_motion_authorized:
            return 'motion disabled by explicit configuration'
        if self.source == 'none':
            return 'no control source selected'
        for seen, timeout, name in (
            (self.operator_seen, self.policy.operator_timeout, 'operator permit'),
            (self.scan_seen, self.policy.scan_timeout, 'laser scan'),
            (self.feedback_seen, self.policy.feedback_timeout, 'wheel feedback'),
        ):
            if seen is None or not 0 <= now - seen <= timeout:
                return 'stale or missing ' + name
        return None

    def begin_arm(self, now):
        error = self.readiness_error(now)
        if error or self.stop_pending or self.feedback_state != 1 or self.armed or self.arming:
            raise RuntimeError(error or 'MCU must be DISARMED; no implicit recovery')
        self.generation += 1
        self.arming = True
        self.command = None
        self.stop_pending = False
        self.reason = 'ARM in progress at zero target'
        return self.generation

    def complete_arm(self, generation, now):
        error = self.readiness_error(now)
        if error or generation != self.generation or not self.arming or self.feedback_state != 10:
            detail = error or ('ARM cancelled: ' + self.reason
                              if generation != self.generation or not self.arming
                              else 'READY feedback not verified: state=%s' % self.feedback_state)
            self.trip(detail)
            raise RuntimeError(self.reason)
        self.arming, self.armed = False, True
        self.arm_epoch = now
        self.command = None
        self.reason = 'READY; waiting for new selected-source velocity'

    def target(self, now):
        if not self.armed:
            return None
        error = self.readiness_error(now)
        if error:
            self.trip(error)
            return None
        if self.command is None:
            # One short zero-target allowance for new commands after explicit ARM.
            if now - self.arm_epoch > self.policy.cmd_timeout:
                self.trip('no new velocity after ARM')
                return None
            return (0, 0)
        v, w, received = self.command
        if not 0 <= now - received <= self.policy.cmd_timeout:
            self.trip('stale selected-source velocity')
            return None
        result = self.converter(v, w, self.policy.diameter, self.policy.separation)
        if len(result) != 2 or any(type(x) is not int or abs(x) > 5 for x in result):
            self.trip('wheel conversion outside firmware envelope')
            return None
        return result


class MeasuredOdometry:
    """Same v8 trapezoidal RPM integration and MCU-time gap policy, adapted to v9.

    Each sample is accepted only after runtime's strict v9 fresh(). No commanded
    velocity enters this class. Missing samples do not create invented movement.
    """

    # 2026-10-23: the MCU speed estimate is quantised to 0.1 RPM and can report a
    # residual on one wheel (observed: left -0.1 RPM, right 0.0 RPM) while the
    # wheels are mechanically still. That fed /odom v=-0.00052 m/s, w=+0.00349 rad/s,
    # which tripped the strict post-ARM zero window (|v|,|w| <= 1e-9) and aborted
    # three 90-degree turns. Treat sub-0.2 RPM as standstill: 0.2 RPM is 2.1 mm/s at
    # a 200 mm wheel, far below the slowest commanded turn (~1.7 RPM), so real motion
    # is never masked. Applied before the v/w conversion so pose integration stops too.
    DEADBAND_RPM = 0.2

    def __init__(self, diameter=.2, separation=.3, calibrated=False):
        self.diameter, self.separation, self.calibrated = diameter, separation, calibrated
        self.x = self.y = self.yaw = 0.
        self.previous = None
        self.discontinuities = 0

    def invalidate(self):
        self.previous = None
        self.discontinuities += 1

    def update(self, packet):
        signature = (packet['sample_sequence'], packet['speed_tick_ms'])
        if self.previous and signature == self.previous['signature']:
            return None
        left_rpm = packet['speed_left_rpm']
        right_rpm = packet['speed_right_rpm']
        if abs(left_rpm) < self.DEADBAND_RPM:
            left_rpm = 0.
        if abs(right_rpm) < self.DEADBAND_RPM:
            right_rpm = 0.
        left = left_rpm * math.pi * self.diameter / 60
        right = -right_rpm * math.pi * self.diameter / 60
        v, w = (left + right) / 2, (right - left) / self.separation
        if not all(math.isfinite(x) for x in (v, w)):
            self.invalidate()
            raise ValueError('Invalid measured speed')
        result = dict(linear_x=v, angular_z=w, integrated=False, discontinuity=False,
                      mcu_speed_tick_ms=packet['speed_tick_ms'], sample_sequence=packet['sample_sequence'],
                      geometry_calibrated=self.calibrated)
        if self.previous:
            dt = ((packet['speed_tick_ms'] - self.previous['tick']) & 0xffffffff) / 1000
            step = (packet['sample_sequence'] - self.previous['sequence']) & 0xffffffff
            if step != 1 or not 0 < dt <= .25:
                self.discontinuities += 1
                result['discontinuity'] = True
            else:
                ds = (self.previous['v'] + v) * .5 * dt
                angle = (self.previous['w'] + w) * .5 * dt
                scale = math.sin(angle / 2) / (angle / 2) if abs(angle) > 1e-10 else 1.
                self.x += ds * scale * math.cos(self.yaw + angle / 2)
                self.y += ds * scale * math.sin(self.yaw + angle / 2)
                self.yaw = math.atan2(math.sin(self.yaw + angle), math.cos(self.yaw + angle))
                result.update(integrated=True, dt_s=dt)
        self.previous = dict(signature=signature, tick=packet['speed_tick_ms'],
                             sequence=packet['sample_sequence'], v=v, w=w)
        result.update(x=self.x, y=self.y, yaw=self.yaw)
        return result
