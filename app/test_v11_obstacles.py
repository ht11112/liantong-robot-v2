"""Observed-space dynamic obstacle clearing; no devices or ROS runtime."""
import math
import unittest

from v11_map_plan import GridMap
from v11_obstacles import DynamicObstacles


def grid(*, origin=(0., 0., 0.), resolution=1., blocked=()):
    rows = [0]*10
    for x, y in blocked:
        rows[y] |= 1 << x
    return GridMap(b'', 10, 10, resolution, origin, tuple(rows), 'pgm', 'yaml',
                   .65, .196, 0)


class DynamicObstacleTests(unittest.TestCase):
    def test_farther_measured_return_clears_removed_obstacle(self):
        memory = DynamicObstacles(grid())
        memory.update(((3.5, 5.5),), ((1.5, 5.5, 3.5, 5.5),))
        self.assertEqual(memory.cells, {(3, 5)})
        self.assertEqual(memory.update(((7.5, 5.5),), ((1.5, 5.5, 7.5, 5.5),)), {(7, 5)})

    def test_blank_scan_or_off_axis_observations_do_not_erase(self):
        memory = DynamicObstacles(grid())
        memory.update(((3.5, 5.5),))
        for _ in range(100):
            self.assertEqual(memory.update(()), {(3, 5)})
        memory.update(((7.5, 8.5),), ((1.5, 8.5, 7.5, 8.5),))
        self.assertEqual(memory.cells, {(3, 5), (7, 8)})

    def test_near_field_start_and_measured_endpoint_cells_are_preserved(self):
        memory = DynamicObstacles(grid())
        memory.update(((1.5, 5.5), (2.5, 5.5), (3.5, 5.5), (7.5, 5.5)))
        memory.update((), ((2.2, 5.5, 7.8, 5.5),))
        self.assertEqual(memory.cells, {(1, 5), (2, 5), (7, 5)})

    def test_current_hits_win_and_static_cells_are_not_modified(self):
        base = grid(blocked=((4, 5),))
        rows = base.blocked_rows
        memory = DynamicObstacles(base)
        memory.update(((3.5, 5.5),))
        memory.update(((3.5, 5.5), (4.5, 5.5)), ((1.5, 5.5, 7.5, 5.5),))
        self.assertEqual(memory.cells, {(3, 5)})
        self.assertEqual(base.blocked_rows, rows)

    def test_corner_touches_do_not_clear_adjacent_cells(self):
        memory = DynamicObstacles(grid())
        memory.update(((3.5, 3.5), (3.5, 4.5), (4.5, 3.5)))
        memory.update((), ((1.5, 1.5, 7.5, 7.5),))
        self.assertEqual(memory.cells, {(3, 4), (4, 3)})

    def test_reverse_ray_and_rotated_scaled_map(self):
        base = grid(origin=(20., -5., math.pi/2), resolution=.5)
        memory = DynamicObstacles(base)
        point = lambda x, y: base._to_map(x, y)
        memory.update((point(3.5, 5.5), point(7.5, 5.5)))
        memory.update((point(1.5, 5.5),), (point(8.5, 5.5)+point(1.5, 5.5),))
        self.assertEqual(memory.cells, {(1, 5)})

    def test_axis_on_grid_boundary_uses_same_cell_convention_as_hits(self):
        memory = DynamicObstacles(grid())
        memory.update(((3.5, 5.), (3.5, 4.5)))
        memory.update((), ((1.5, 5., 7.5, 5.),))
        self.assertEqual(memory.cells, {(3, 4)})

    def test_long_outside_map_ray_is_clipped(self):
        memory = DynamicObstacles(grid())
        memory.update(((.5, 5.5), (5.5, 5.5), (9.5, 5.5)))
        memory.update((), ((-1000000., 5.5, 1000000., 5.5),))
        self.assertEqual(memory.cells, set())

    def test_invalid_observation_does_not_partly_clear_memory(self):
        memory = DynamicObstacles(grid())
        memory.update(((3.5, 5.5),))
        for rays in (((1.5, 5.5, 7.5, 5.5), (0., 0., math.inf, 0.)),
                     ((math.nan, 0., 1., 1.),)):
            with self.assertRaises(ValueError):
                memory.update((), rays)
            self.assertEqual(memory.cells, {(3, 5)})
        memory.clear()
        self.assertEqual(memory.cells, set())


if __name__ == '__main__':
    unittest.main()
