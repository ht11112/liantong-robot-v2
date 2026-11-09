"""Small dynamic obstacle overlay; only observed finite rays clear old hits.

Coordinates supplied by the caller are in the map frame. The caller must
discard unusable/stale scans and transform each scan using its measured pose.
No age, missing return, or change of field of view erases an occupied cell.
"""
import math


def _coordinates(values, length):
    if len(values) != length or not all(isinstance(v, (int, float)) and
                                       math.isfinite(v) for v in values):
        raise ValueError('Obstacle observations require finite map coordinates')
    return tuple(values)


def _clip(a, b, width, height):
    """Clip before traversal so work is bounded by the map dimensions."""
    start, end = 0., 1.
    for position, delta, limit in ((a[0], b[0]-a[0], width),
                                   (a[1], b[1]-a[1], height)):
        if not delta:
            if not 0. <= position < limit:
                return None
            continue
        first, last = sorted((-position/delta, (limit-position)/delta))
        start, end = max(start, first), min(end, last)
        if start >= end:
            return None
    return tuple(max(0., min(limit, a[i]+start*(b[i]-a[i])))
                 for i, limit in enumerate((width, height))), tuple(
        max(0., min(limit, a[i]+end*(b[i]-a[i])))
        for i, limit in enumerate((width, height)))


def _ray_cells(a, b, width, height):
    """Cells crossed by the ray interior, excluding its original endpoint cells.

    The first cell can contain a filtered near-field return and the last contains
    the measured hit. Neither is clearing evidence. Corner-only touches likewise
    do not clear a cell. Boundary rays use the same floor/half-open cell
    convention as the measured hits.
    """
    excluded = {tuple(map(math.floor, a)), tuple(map(math.floor, b))}
    clipped = _clip(a, b, width, height)
    if clipped is None:
        return
    a, b = clipped
    dx, dy = b[0]-a[0], b[1]-a[1]
    x, y = math.floor(a[0]), math.floor(a[1])
    if dx < 0 and x == a[0]:
        x -= 1
    if dy < 0 and y == a[1]:
        y -= 1
    step_x, step_y = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    time_x = ((x+1 if dx > 0 else x)-a[0])/dx if dx else math.inf
    time_y = ((y+1 if dy > 0 else y)-a[1])/dy if dy else math.inf
    delta_x, delta_y = (abs(1./dx) if dx else math.inf), (abs(1./dy) if dy else math.inf)
    current = 0.
    # Pure numerical tolerance in a clipped segment's parameter, not a sensor
    # range, timeout, clearance, or other hardware-derived threshold.
    epsilon = 1e-12
    while 0 <= x < width and 0 <= y < height and current < 1.:
        following = min(time_x, time_y, 1.)
        if following-current > epsilon and (x, y) not in excluded:
            yield x, y
        if following >= 1.:
            break
        if time_x <= following+epsilon:
            x += step_x
            time_x += delta_x
        if time_y <= following+epsilon:
            y += step_y
            time_y += delta_y
        current = following


class DynamicObstacles:
    """Remember hits only in map-free cells; static occupancy stays unchanged."""
    def __init__(self, grid):
        self.grid = grid
        self.cells = frozenset()

    def clear(self):
        """Explicit session/map reset, never an interpretation of a blank scan."""
        self.cells = frozenset()

    def update(self, points, clear_rays=()):
        """Add map-frame hits and remove cells crossed by measured free segments.

        Each ray is (start_x, start_y, hit_x, hit_y), starting at the actual
        observable minimum range, ending at a valid finite measured hit.
        Infinite/no-return and invalid rays must not be supplied as clear rays.
        Hits win over clearing evidence within the same observation.
        """
        # Validate the full observation before mutating state.
        points = tuple(_coordinates(point, 2) for point in points)
        rays = tuple(_coordinates(ray, 4) for ray in clear_rays)
        hits = self.grid.obstacle_cells(points)
        remaining = set(self.cells)
        for sx, sy, ex, ey in rays:
            if not remaining:
                break
            a, b = self.grid._to_grid(sx, sy), self.grid._to_grid(ex, ey)
            remaining.difference_update(_ray_cells(a, b, self.grid.width, self.grid.height))
        self.cells = frozenset(remaining | hits)
        return self.cells
