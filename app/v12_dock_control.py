"""One photograph plans one initial TURN and one continuous reverse MOVE."""
import json
import math
from pathlib import Path
import struct
import zlib

from mcu_v11 import QueuePlan

REVERSE_SPEED_M_S = math.pi * .20 / 60.


def dock_segment(kind, value):
    if not math.isfinite(value):
        raise ValueError('动作必须为有限数')
    if kind == 'move':
        code, lower, upper = 3, -1., -.005
        valid = lower <= value < upper
    elif kind == 'turn':
        code, lower, upper = 4, math.radians(1), math.radians(5)
        valid = lower < abs(value) <= upper
    elif kind == 'normal_turn':
        code, lower, upper = 2, 0., 2 * math.pi
        valid = lower < abs(value) <= upper
    else:
        raise ValueError('未知对接动作')
    if not valid:
        raise ValueError('对接动作超出距离或角度限制')
    raw = struct.pack('<If', code, value)
    quantized = struct.unpack('<f', raw[4:])[0]
    lower, upper = (struct.unpack('<f', struct.pack('<f', bound))[0] for bound in (lower, upper))
    if not (lower <= quantized < upper if code == 3 else lower < abs(quantized) <= upper):
        raise ValueError('对接动作在 float32 编码后超出限制')
    return QueuePlan((raw,), zlib.crc32(raw))


def reverse_plan(observation):
    """Camera supplies axle-frame plane position and its free-space normal."""
    px, py, nx, ny = (observation[k] for k in ('plane_x_m', 'plane_y_m', 'normal_x', 'normal_y'))
    if not all(math.isfinite(v) for v in (px, py, nx, ny)) or abs(math.hypot(nx, ny) - 1.) > .001:
        raise ValueError('标签平面几何无效')
    qx, qy = px + .25 * nx, py + .25 * ny
    distance = math.hypot(qx, qy)
    if qx >= 0 or not .005 < distance <= 1.:
        raise ValueError('轮轴目标必须位于当前后方，倒车长度须大于5毫米且不超过1米')
    return dict(target_body=dict(x=qx, y=qy), turn_rad=math.atan2(-qy, -qx),
                reverse_m=distance, budget_s=max(30, math.ceil(1.25 * distance / REVERSE_SPEED_M_S + 5)))


def dock_sites(config):
    c, s = math.cos(config['yaw']), math.sin(config['yaw'])
    return dict(station=dict(x=config['x'], y=config['y'], yaw=config['yaw']),
                waiting=dict(x=config['x'] + .70*c, y=config['y'] + .70*s),
                axle_target=dict(x=config['x'] + .25*c, y=config['y'] + .25*s))


class DockSettings:
    """One map-bound JSON setting survives service restarts."""
    def __init__(self, map_id, path=None):
        self.map_id, self.path = map_id, Path(path) if path is not None else None
        self.value = dict(map_id=map_id, x=-.5, y=0., yaw=0.)
        if self.path is not None and self.path.exists():
            data = json.loads(self.path.read_text(encoding='utf-8'))
            if data.get('map_id') == map_id:
                self.value = self.checked(data)

    def checked(self, body):
        if (not isinstance(body, dict) or set(body) != {'map_id', 'x', 'y', 'yaw'}
                or body['map_id'] != self.map_id
                or any(type(body[k]) not in (int, float) or not math.isfinite(body[k]) for k in ('x', 'y', 'yaw'))):
            raise ValueError('桩坐标、朝向或地图身份无效')
        return dict(map_id=self.map_id, x=body['x'], y=body['y'],
                    yaw=math.atan2(math.sin(body['yaw']), math.cos(body['yaw'])))

    def save(self, body):
        value = self.checked(body)
        if self.path is not None:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temporary = self.path.with_suffix('.tmp')
            temporary.write_text(json.dumps(value, ensure_ascii=False), encoding='utf-8')
            temporary.replace(self.path)
        self.value = value
