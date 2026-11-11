"""Single-segment visual docking decisions and the V12 wire representation."""

import math
import struct
import zlib

from mcu_v11 import QueuePlan


def dock_segment(kind, value):
    if kind == 'move':
        code = 3
        if not -.05 <= value < -.005:
            raise ValueError('对接倒车距离必须在 -5 厘米至 -5 毫米之间')
    elif kind == 'turn':
        code = 4
        if not math.radians(1) < abs(value) <= math.radians(5):
            raise ValueError('对接转向必须大于 1 度且不超过 5 度')
    else:
        raise ValueError('未知对接动作')
    raw = struct.pack('<If', code, value)
    quantized = struct.unpack('<f', raw[4:])[0]
    lower, upper = ((-.05, -.005) if code == 3 else (math.radians(1), math.radians(5)))
    lower, upper = (struct.unpack('<f', struct.pack('<f', bound))[0] for bound in (lower, upper))
    if not (lower <= quantized < upper if code == 3 else lower < abs(quantized) <= upper):
        raise ValueError('对接动作在 float32 编码后超出限制')
    return QueuePlan((raw,), zlib.crc32(raw))


def next_action(observation):
    """The lateral offset is axle-left-positive; yaw is vehicle CCW-positive."""
    distance = observation['distance_m']
    lateral = observation['lateral_m']
    yaw = math.radians(observation['yaw_deg'])
    if abs(distance - .20) <= .02 and abs(lateral) <= .02 and abs(yaw) <= math.radians(3):
        return None
    if distance < .18:
        raise ValueError('相机已距标签不足 18 厘米，无法继续倒车')
    remaining = distance - .20
    if remaining <= .02 and abs(lateral) > .02:
        raise ValueError('剩余倒车距离不足以消除横向偏差，请人工重新摆位')
    desired = math.atan2(lateral, max(remaining, .01)) if abs(lateral) > .02 else 0.
    desired = max(-math.radians(20), min(math.radians(20), desired))
    heading_error = math.atan2(math.sin(desired - yaw), math.cos(desired - yaw))
    if abs(heading_error) > math.radians(2):
        return 'turn', max(-math.radians(5), min(math.radians(5), heading_error))
    if remaining <= .02:
        raise ValueError('对接误差无法通过剩余倒车距离消除，请人工重新摆位')
    return 'move', -min(.05, remaining)
