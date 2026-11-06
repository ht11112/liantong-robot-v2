"""Bounded, deterministic A* and V11 MOVE/TURN swept-volume checks.

All poses are in the saved map frame, in metres and radians. This module has
no ROS, serial or runtime side effects. The swept disk bounds the body and
the modeled firmware early-stop tolerances, not unmeasured drift or braking.
"""
import hashlib
import heapq
import json
import math
from pathlib import Path
import shlex
import struct


BODY_RADIUS_M = math.hypot(.175, .20)
MOVE_TOLERANCE_M = .02
GOAL_TOLERANCE_M = .10
TURN_TOLERANCE_RAD = math.pi / 60.
MAX_SEGMENTS = 128
MAX_DURATION_S = 1200.
_WHEEL_M_PER_RPM_S = math.pi * .20 / 60.
_TURN_RAD_S = 2. * _WHEEL_M_PER_RPM_S / .30
_EPS = 1e-10
_STEPS = ((1, 0), (0, 1), (-1, 0), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1))


class PlanningError(ValueError):
    """A map, pose, route or exact wire queue cannot be accepted."""


def _number(value):
    if type(value) not in (int, float):
        raise PlanningError('Expected a finite map coordinate or angle')
    try:
        value = float(value)
    except OverflowError as exc:
        raise PlanningError('Map coordinate outside supported range') from exc
    if not math.isfinite(value) or abs(value) > 1e6:
        raise PlanningError('Map coordinate outside supported range')
    return value


def _point(value, keys=('x', 'y')):
    try:
        return tuple(_number(value[key]) for key in keys)
    except (KeyError, TypeError) as exc:
        raise PlanningError('Expected map fields ' + ', '.join(keys)) from exc


def _wrap(angle):
    return (angle + math.pi) % (2. * math.pi) - math.pi


def _duration(value):
    value = _number(value)
    if not 0 < value <= MAX_DURATION_S or round(value * 1000) < 1:
        raise PlanningError('Task duration must be within 1 ms and 1200 s')
    return round(value * 1000) / 1000.


def _float32(value):
    return struct.unpack('<f', struct.pack('<f', value))[0]


def _queue(segments):
    # The RK service loads mcu/tools directly; repository unit tests also
    # support its namespace-package path. Use the actual shared wire codec.
    try:
        from mcu_v10 import canonical_segments
    except ModuleNotFoundError as exc:
        if exc.name != 'mcu_v10':
            raise
        from mcu.tools.mcu_v10 import canonical_segments
    try:
        packed, crc, _ = canonical_segments({'segments': segments})
    except (ValueError, TypeError, OverflowError) as exc:
        raise PlanningError('Invalid V11 queue: ' + str(exc)) from exc
    canonical = []
    for index, raw in enumerate(packed):
        kind, value = struct.unpack('<If', raw)
        if kind == 1:
            minimum = GOAL_TOLERANCE_M if index + 1 == len(segments) else MOVE_TOLERANCE_M
            if value <= _float32(minimum):
                raise PlanningError('MOVE distance is below its termination tolerance')
            canonical.append(dict(kind='move', distance_m=value))
        else:
            if abs(value) <= _float32(TURN_TOLERANCE_RAD):
                raise PlanningError('TURN angle is below its termination tolerance')
            canonical.append(dict(kind='turn', angle_rad=value))
    if canonical[-1]['kind'] != 'move':
        raise PlanningError('Navigation finishes with MOVE, without a final heading adjustment')
    return canonical, crc


def _pgm(raw):
    # Read exactly the header, then consume one separator, never strip pixels.
    position, tokens = 0, []
    while len(tokens) < 4:
        while position < len(raw) and raw[position] in b' \t\r\n':
            position += 1
        if raw[position:position + 1] == b'#':
            end = raw.find(b'\n', position)
            if end < 0:
                raise PlanningError('Unterminated PGM comment')
            position = end + 1
            continue
        start = position
        while position < len(raw) and raw[position] not in b' \t\r\n':
            position += 1
        if start == position:
            raise PlanningError('Incomplete PGM header')
        tokens.append(raw[start:position])
    try:
        width, height, depth = map(int, tokens[1:])
    except ValueError as exc:
        raise PlanningError('Invalid PGM dimensions') from exc
    if (tokens[0] != b'P5' or depth != 255 or not 1 <= width <= 4096
            or not 1 <= height <= 4096 or width * height > 1_000_000):
        raise PlanningError('Expected bounded, 8-bit P5 map')
    separator = 2 if raw[position:position + 2] == b'\r\n' else 1
    if raw[position:position + 1] not in (b' ', b'\t', b'\r', b'\n'):
        raise PlanningError('Missing PGM pixel separator')
    pixels = raw[position + separator:]
    if len(pixels) != width * height:
        raise PlanningError('PGM pixel count differs from dimensions')
    return width, height, pixels


def _yaml(raw):
    """Only map_server's flat trinary YAML fields, not arbitrary YAML."""
    values = {}
    try:
        lines = raw.decode('utf-8-sig').splitlines()
        allowed = {'image', 'resolution', 'origin', 'negate', 'occupied_thresh', 'free_thresh', 'mode'}
        for line in lines:
            line = line.strip()
            if not line or line.startswith('#') or line == '---':
                continue
            key, separator, value = line.partition(':')
            key = key.strip()
            if not separator or key not in allowed or key in values:
                raise ValueError('Unknown or duplicate map field')
            tokens = shlex.split(value, comments=True)
            if not tokens or (key != 'origin' and len(tokens) != 1):
                raise ValueError('Invalid map field')
            values[key] = ' '.join(tokens)
        origin = json.loads(values['origin'])
        resolution = _number(float(values['resolution']))
        occupied, free = _number(float(values['occupied_thresh'])), _number(float(values['free_thresh']))
        negate = int(values['negate'])
        if (not isinstance(origin, list) or len(origin) != 3 or not .01 <= resolution <= 1.
                or not 0. <= free < occupied <= 1. or negate not in (0, 1)
                or values.get('mode', 'trinary') != 'trinary'):
            raise ValueError('Map geometry or trinary thresholds are invalid')
        origin = tuple(_number(value) for value in origin)
        return values['image'], resolution, origin, negate, occupied, free
    except (UnicodeError, ValueError, KeyError) as exc:
        raise PlanningError('Invalid flat map_server YAML: ' + str(exc)) from exc


def _sha(raw, expected):
    actual = hashlib.sha256(raw).hexdigest()
    if expected is not None and (not isinstance(expected, str) or actual != expected.lower()):
        raise PlanningError('Saved map SHA-256 does not match its pinned identity')
    return actual


def load_map(pgm_path, yaml_path, expected_pgm_sha256=None, expected_yaml_sha256=None):
    pgm_path, yaml_path = Path(pgm_path), Path(yaml_path)
    if pgm_path.stat().st_size > 1_010_000 or yaml_path.stat().st_size > 8192:
        raise PlanningError('Saved map files exceed the supported size')
    pgm, yaml = pgm_path.read_bytes(), yaml_path.read_bytes()
    pgm_sha, yaml_sha = _sha(pgm, expected_pgm_sha256), _sha(yaml, expected_yaml_sha256)
    image, resolution, origin, negate, occupied, free = _yaml(yaml)
    if (yaml_path.parent / image).resolve() != pgm_path.resolve():
        raise PlanningError('YAML image must identify the supplied PGM')
    width, height, pixels = _pgm(pgm)
    rows = []
    for y in range(height - 1, -1, -1):  # PGM top-down; map grid bottom-up.
        bits = 0
        for x, gray in enumerate(pixels[y*width:(y+1)*width]):
            probability = (gray if negate else 255 - gray) / 255.
            if probability >= free:  # Unknown and occupied are both blocked.
                bits |= 1 << x
        rows.append(bits)
    return GridMap(pgm, width, height, resolution, origin, tuple(rows), pgm_sha, yaml_sha,
                   occupied, free, negate)


def _point_box_sq(x, y, i, j):
    dx, dy = max(i - x, x - i - 1., 0.), max(j - y, y - j - 1., 0.)
    return dx*dx + dy*dy


def _capsule_hits_cell(a, b, radius_sq, i, j):
    """Exact closed segment-to-closed-cell distance, with conservative epsilon."""
    ax, ay = a
    bx, by = b
    if min(_point_box_sq(ax, ay, i, j), _point_box_sq(bx, by, i, j)) <= radius_sq + _EPS:
        return True
    dx, dy = bx - ax, by - ay
    length_sq = dx*dx + dy*dy
    if not length_sq:
        return False
    low, high = 0., 1.
    for position, delta, edge in ((ax, dx, i), (ay, dy, j)):
        if delta:
            t0, t1 = (edge - position) / delta, (edge + 1. - position) / delta
            low, high = max(low, min(t0, t1)), min(high, max(t0, t1))
        elif not edge <= position <= edge + 1.:
            low, high = 1., 0.
    if low <= high + _EPS:
        return True
    for x, y in ((i, j), (i+1, j), (i, j+1), (i+1, j+1)):
        fraction = min(1., max(0., ((x-ax)*dx + (y-ay)*dy) / length_sq))
        if (x-ax-fraction*dx)**2 + (y-ay-fraction*dy)**2 <= radius_sq + _EPS:
            return True
    return False


class _Costmap:
    def __init__(self, grid, search_padding=0.):
        self.width, self.height = grid.width, grid.height
        self.rows = grid.blocked_rows
        self.radius = (BODY_RADIUS_M + search_padding) / grid.resolution
        self._walkable = bytearray(self.width * self.height)
        self._disk_spans = []
        limit = int(math.floor(self.radius + .5 + _EPS))
        for dy in range(-limit, limit+1):
            vertical = max(0., abs(dy)-.5)
            horizontal = math.sqrt(max(0., self.radius**2-vertical**2))
            span = int(math.floor(horizontal+.5+_EPS))
            self._disk_spans.append((dy, span, (1 << (2*span+1)) - 1))

    def sweep(self, a, b, radius=None):
        radius = self.radius if radius is None else radius
        low_x, high_x = min(a[0], b[0])-radius, max(a[0], b[0])+radius
        low_y, high_y = min(a[1], b[1])-radius, max(a[1], b[1])+radius
        if low_x <= _EPS or low_y <= _EPS or high_x >= self.width-_EPS or high_y >= self.height-_EPS:
            return False
        # Include cells just touching the bounding box (including integer boundaries).
        left, right = math.floor(low_x-_EPS), math.floor(high_x+_EPS)
        mask = (1 << (right-left+1)) - 1
        for j in range(math.floor(low_y-_EPS), math.floor(high_y+_EPS)+1):
            hits = (self.rows[j] >> left) & mask
            while hits:
                bit = hits & -hits
                i = left + bit.bit_length() - 1
                if _capsule_hits_cell(a, b, radius*radius, i, j):
                    return False
                hits ^= bit
        return True

    def center_clear(self, x, y):
        r = self.radius
        if not (r+_EPS < x+.5 < self.width-r-_EPS and r+_EPS < y+.5 < self.height-r-_EPS):
            return False
        index = y*self.width+x
        cached = self._walkable[index]
        if not cached:
            clear = not any(self.rows[y+dy] & (mask << (x-span)) for dy, span, mask in self._disk_spans)
            cached = self._walkable[index] = 2 if clear else 1
        return cached == 2

    @staticmethod
    def center(node):
        return node[0]+.5, node[1]+.5

    def connectors(self, point):
        i, j = math.floor(point[0]), math.floor(point[1])
        for y in range(j-1, j+2):
            for x in range(i-1, i+2):
                if self.center_clear(x, y) and self.sweep(point, self.center((x, y))):
                    yield x, y

    def astar(self, start, goal, max_distance):
        if self.sweep(start, goal):
            return [start, goal]
        targets = set(self.connectors(goal))
        if not targets:
            raise PlanningError('No traversable grid connection at the requested goal')
        open_nodes, cost, parent = [], {}, {}
        def add(node, distance, previous):
            estimate = distance + math.hypot(self.center(node)[0]-goal[0], self.center(node)[1]-goal[1])
            if estimate <= max_distance and distance < cost.get(node, math.inf):
                cost[node], parent[node] = distance, previous
                heapq.heappush(open_nodes, (estimate, distance, node))
        for node in self.connectors(start):
            x, y = self.center(node)
            add(node, math.hypot(x-start[0], y-start[1]), None)
        while open_nodes:
            _, distance, node = heapq.heappop(open_nodes)
            if distance != cost[node]:
                continue
            if node in targets:
                chain = [goal]
                while node is not None:
                    chain.append(self.center(node))
                    node = parent[node]
                chain.append(start)
                return list(reversed(chain))
            x, y = node
            for dx, dy in _STEPS:
                neighbor = x+dx, y+dy
                next_cost = distance + (math.sqrt(2.) if dx and dy else 1.)
                if next_cost >= cost.get(neighbor, math.inf) or not self.center_clear(*neighbor):
                    continue
                if dx and dy and not (self.center_clear(x+dx, y) and self.center_clear(x, y+dy)):
                    continue  # No diagonal corner cutting, independently of sweep.
                if self.sweep(self.center(node), self.center(neighbor)):
                    add(neighbor, next_cost, node)
        raise PlanningError('No path through known clear space within the task distance budget')

    def simplify(self, chain):
        result, index = [chain[0]], 0
        while index + 1 < len(chain):
            for later in range(len(chain)-1, index, -1):
                if self.sweep(chain[index], chain[later]):
                    result.append(chain[later])
                    index = later
                    break
            else:
                raise PlanningError('Grid route has no collision-free straight continuation')
            if len(result) > MAX_SEGMENTS + 1:
                raise PlanningError('Path exceeds the MCU action capacity')
        return result


class GridMap:
    def __init__(self, pgm, width, height, resolution, origin, rows, pgm_sha, yaml_sha,
                 occupied, free, negate):
        self.pgm_bytes = pgm
        self.width, self.height, self.resolution, self.origin = width, height, resolution, origin
        self.blocked_rows = rows
        self.pgm_sha256, self.yaml_sha256 = pgm_sha, yaml_sha
        self.map_id = hashlib.sha256((pgm_sha + yaml_sha).encode('ascii')).hexdigest()
        self.occupied_thresh, self.free_thresh, self.negate = occupied, free, negate
        self._cos, self._sin = math.cos(origin[2]), math.sin(origin[2])

    def metadata(self):
        return dict(width=self.width, height=self.height, resolution=self.resolution,
                    origin=list(self.origin), pgm_sha256=self.pgm_sha256, yaml_sha256=self.yaml_sha256,
                    map_id=self.map_id, occupied_thresh=self.occupied_thresh,
                    free_thresh=self.free_thresh, negate=self.negate)

    def costmap_pgm(self):
        """Draw the baseline footprint's cell-center clearances in PGM row order."""
        width, height, pixels = _pgm(self.pgm_bytes)
        costmap = _Costmap(self)
        raster = bytearray(width * height)
        for row in range(height):
            y = height - 1 - row
            blocked = self.blocked_rows[y]
            for x in range(width):
                index = row * width + x
                if blocked & (1 << x):
                    gray = pixels[index]
                    probability = (gray if self.negate else 255 - gray) / 255.
                    raster[index] = 0 if probability > self.occupied_thresh else 175
                else:
                    raster[index] = 255 if costmap.center_clear(x, y) else 100
        return ('P5\n{} {}\n255\n'.format(width, height)).encode('ascii') + raster

    def _to_grid(self, x, y):
        dx, dy = x-self.origin[0], y-self.origin[1]
        return ((self._cos*dx+self._sin*dy)/self.resolution,
                (-self._sin*dx+self._cos*dy)/self.resolution)

    def _to_map(self, x, y):
        return (self.origin[0] + self.resolution*(self._cos*x-self._sin*y),
                self.origin[1] + self.resolution*(self._sin*x+self._cos*y))

    def _endpoints(self, costmap, pose, goal):
        a, b = self._to_grid(*pose[:2]), self._to_grid(*goal)
        if not costmap.sweep(a, a):
            raise PlanningError('Start footprint intersects occupied, unknown or outside space')
        if not costmap.sweep(b, b):
            raise PlanningError('Goal footprint intersects occupied, unknown or outside space')
        return a, b

    def _verify(self, costmap, pose, goal, segments, duration):
        x, y, yaw = pose
        path, distance, estimated_time = [dict(x=x, y=y)], 0., 4.
        position_error, heading_error = 0., 0.
        for index, segment in enumerate(segments):
            start = self._to_grid(x, y)
            if segment['kind'] == 'turn':
                angle = segment['angle_rad']
                if not costmap.sweep(start, start, (BODY_RADIUS_M+position_error)/self.resolution):
                    raise PlanningError('TURN swept disk intersects a blocked cell or map boundary')
                yaw = _wrap(yaw+angle)
                heading_error += _float32(TURN_TOLERANCE_RAD)
                estimated_time += abs(angle)/_TURN_RAD_S
            else:
                length = segment['distance_m']
                x, y = x+length*math.cos(yaw), y+length*math.sin(yaw)
                end = self._to_grid(x, y)
                yaw_distance = 2.*length*math.sin(min(math.pi, heading_error)/2.)
                radius = (BODY_RADIUS_M+position_error+yaw_distance)/self.resolution
                if not costmap.sweep(start, end, radius):
                    raise PlanningError('MOVE swept capsule intersects a blocked cell or map boundary')
                path.append(dict(x=x, y=y))
                distance += length
                estimated_time += length/_WHEEL_M_PER_RPM_S
                position_error += yaw_distance + _float32(GOAL_TOLERANCE_M if index+1 == len(segments) else MOVE_TOLERANCE_M)
        estimated_time += 2.*(len(segments)-1)
        error = math.hypot(x-goal[0], y-goal[1])
        if error > GOAL_TOLERANCE_M+_EPS:
            raise PlanningError('Encoded actions do not end within the nominal map goal tolerance')
        if estimated_time > duration:
            raise PlanningError('Conservative nominal action duration exceeds the task limit')
        return dict(path=path, distance_m=distance, nominal_end_pose=dict(x=x, y=y, yaw=yaw),
                    nominal_goal_error_m=error, nominal_duration_s=estimated_time,
                    modeled_early_stop_position_error_m=position_error,
                    modeled_early_stop_heading_error_rad=heading_error,
                    actual_endpoint_verified=False)

    def _compile(self, start, chain):
        x, y, yaw = start
        segments = []
        for waypoint in chain[1:]:
            wx, wy = self._to_map(*waypoint)
            dx, dy = wx-x, wy-y
            turn = _float32(_wrap(math.atan2(dy, dx)-yaw))
            if abs(turn) > _float32(TURN_TOLERANCE_RAD):
                segments.append(dict(kind='turn', angle_rad=turn))
                yaw = _wrap(yaw+turn)
            # An omitted sub-tolerance TURN really does not turn the robot.
            length = _float32(dx*math.cos(yaw) + dy*math.sin(yaw))
            segments.append(dict(kind='move', distance_m=length))
            x, y = x+length*math.cos(yaw), y+length*math.sin(yaw)
        return _queue(segments)

    def plan(self, pose, goal, *, duration_s=MAX_DURATION_S):
        start, target = _point(pose, ('x', 'y', 'yaw')), _point(goal)
        duration = _duration(duration_s)
        costmap = _Costmap(self)
        a, b = self._endpoints(costmap, start, target)
        result = dict(map_id=self.map_id, start=dict(zip(('x', 'y', 'yaw'), start)),
                      goal=dict(zip(('x', 'y'), target)), duration_s=duration,
                      goal_tolerance_m=GOAL_TOLERANCE_M)
        if math.hypot(target[0]-start[0], target[1]-start[1]) <= GOAL_TOLERANCE_M+_EPS:
            return dict(result, no_motion=True, segments=[], queue_crc32=0,
                        path=[dict(x=start[0], y=start[1])], distance_m=0.,
                        nominal_goal_error_m=math.hypot(target[0]-start[0], target[1]-start[1]),
                        actual_endpoint_verified=False)
        maximum = duration * 2. * _WHEEL_M_PER_RPM_S / self.resolution
        last_error = None
        # Try wider candidate routes only during preview. Every candidate must
        # still pass the exact encoded sweep with its own early-stop envelope.
        for padding in (0., .10, .25, .50):
            search = costmap if not padding else _Costmap(self, padding)
            if not search.sweep(a, a) or not search.sweep(b, b):
                break  # Larger search disks cannot reconnect these endpoints.
            try:
                chain = search.simplify(search.astar(a, b, maximum))
            except PlanningError as exc:
                if last_error is None:
                    last_error = exc
                break  # A wider disk cannot open a path.
            try:
                segments, crc = self._compile(start, chain)
                checked = self._verify(costmap, start, target, segments, duration)
            except PlanningError as exc:
                last_error = exc
                continue
            return dict(result, no_motion=False, segments=segments, queue_crc32=crc,
                        search_padding_m=padding, **checked)
        raise last_error or PlanningError('No collision-checked executable action queue')

    def validate_plan(self, pose, plan):
        """Recheck the same committed bytes from a current pose and costmap."""
        if not isinstance(plan, dict) or plan.get('map_id') != self.map_id or plan.get('no_motion') is not False:
            raise PlanningError('Expected an executable preview for this exact map')
        start, target = _point(pose, ('x', 'y', 'yaw')), _point(plan.get('goal'))
        segments, crc = _queue(plan.get('segments'))
        if type(plan.get('queue_crc32')) is not int or crc != plan['queue_crc32']:
            raise PlanningError('Preview queue CRC does not match the exact action bytes')
        costmap = _Costmap(self)
        self._endpoints(costmap, start, target)
        return self._verify(costmap, start, target, segments, _duration(plan.get('duration_s')))
