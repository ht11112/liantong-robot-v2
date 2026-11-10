"""Offline maps, occupancy and exact-wire route acceptance. No devices."""
import copy
import hashlib
import math
from pathlib import Path
import tempfile
import unittest

from v11_map_plan import BODY_RADIUS_M, GridMap, PlanningError, _pgm, _queue, load_map


def pose(x, y, yaw=0.):
    return dict(x=x, y=y, yaw=yaw)


def wire_preview(grid, goal, actions):
    # Construct exact wire bytes independently of A* to exercise the final
    # verifier against intentionally unsafe routes through this saved map.
    segments, crc = _queue(actions)
    return dict(map_id=grid.map_id, goal=goal, segments=segments, queue_crc32=crc, no_motion=False)


class MapPlanTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.folder = Path(self.temp.name)
        self.pgm, self.yaml = self.folder/'floor.pgm', self.folder/'floor.yaml'

    def tearDown(self):
        self.temp.cleanup()

    def make_map(self, changed=(), *, resolution=.1, origin=(0., 0., 0.), width=60, height=60):
        pixels = {(i,j): value for i,j,value in changed}
        raw = ('P5\r\n# keep raw image bytes\r\n{} {}\r\n255\r\n'.format(width,height)).encode('ascii')
        raw += bytes(pixels.get((x,y),254) for y in range(height-1,-1,-1) for x in range(width))
        self.pgm.write_bytes(raw)
        self.yaml.write_text('image: "floor.pgm" # same external file\nresolution: {}\norigin: {}\n'
                             'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n'.format(
                                 resolution,list(origin)),encoding='utf-8')
        return load_map(self.pgm,self.yaml)

    def test_original_map_bytes_y_flip_trinary_and_hash_identity(self):
        # First image pixel is LF (a real dark pixel, not header whitespace).
        grid = self.make_map(((0,59,10),(20,40,205)))
        self.assertEqual(grid.pgm_bytes,self.pgm.read_bytes())
        self.assertEqual(grid.metadata()['origin'],[0.,0.,0.])
        self.assertEqual(grid.metadata()['pgm_sha256'],hashlib.sha256(grid.pgm_bytes).hexdigest())
        self.assertTrue(grid.blocked_rows[59] & 1)
        self.assertTrue(grid.blocked_rows[40] & (1 << 20))
        self.assertFalse(grid.blocked_rows[19] & (1 << 20))
        with self.assertRaisesRegex(PlanningError,'SHA-256'):
            load_map(self.pgm,self.yaml,expected_yaml_sha256='0'*64)
        with self.assertRaisesRegex(PlanningError,'Goal footprint'):
            grid.plan(pose(2.05,2.),pose(2.05,4.05))

    def test_costmap_image_marks_obstacles_and_cell_clearance_in_image_order(self):
        grid = self.make_map(((7,46,0),(20,40,205)))
        width, height, raster = _pgm(grid.costmap_pgm())
        self.assertEqual((width, height), (60, 60))

        def pixel(x, y):
            return raster[(height-1-y)*width+x]

        self.assertEqual(pixel(7, 46), 0)
        self.assertEqual(pixel(20, 40), 175)
        self.assertEqual(pixel(21, 40), 100)
        self.assertEqual(pixel(1, 30), 100)
        self.assertEqual(pixel(7, 13), 255)

    def test_reject_wrong_yaml_image_and_truncated_pixels(self):
        self.make_map()
        original = self.yaml.read_text(encoding='utf-8')
        self.yaml.write_text(original.replace('floor.pgm','other.pgm'),encoding='utf-8')
        with self.assertRaisesRegex(PlanningError,'supplied PGM'):
            load_map(self.pgm,self.yaml)
        self.yaml.write_text(original,encoding='utf-8')
        self.pgm.write_bytes(self.pgm.read_bytes()[:-1])
        with self.assertRaisesRegex(PlanningError,'pixel count'):
            load_map(self.pgm,self.yaml)

    def test_rotated_map_transform_and_straight_no_blind_compensation(self):
        grid = self.make_map(origin=(10.,20.,math.pi/2.))
        start, goal = pose(8.,22.,math.pi/2.), pose(8.,22.5)
        plan = grid.plan(start,goal)
        self.assertFalse(plan['no_motion'])
        self.assertEqual(plan['segments'],[dict(kind='move',distance_m=.5)])
        self.assertAlmostEqual(plan['path'][-1]['y'],22.5)
        self.assertGreater(plan['nominal_duration_s'], 0.)
        self.assertAlmostEqual(plan['modeled_early_stop_position_error_m'],.1)
        self.assertFalse(plan['actual_endpoint_verified'])
        self.assertEqual(grid.validate_plan(start,plan)['path'],plan['path'])

    def test_nominal_endpoint_tolerance_can_avoid_unnecessary_motion(self):
        grid = self.make_map()
        plan = grid.plan(pose(2.,2.),pose(2.08,2.))
        self.assertTrue(plan['no_motion'])
        self.assertFalse(plan['actual_endpoint_verified'])
        self.assertEqual(plan['segments'],[])
        with self.assertRaisesRegex(PlanningError,'executable preview'):
            grid.validate_plan(pose(2.,2.),plan)

    def test_outside_boundary_body_overlap_rejected_even_for_free_center(self):
        grid = self.make_map()
        with self.assertRaisesRegex(PlanningError,'Start footprint'):
            grid.plan(pose(.20,2.),pose(2.,2.))
        with self.assertRaisesRegex(PlanningError,'Goal footprint'):
            grid.plan(pose(2.,2.),pose(5.8,2.))

    def test_unknown_wall_cannot_be_crossed(self):
        grid = self.make_map((30,j,205) for j in range(60))
        with self.assertRaisesRegex(PlanningError,'No path'):
            grid.plan(pose(1.5,3.05),pose(4.5,3.05))

    def test_Astar_detours_deterministically_and_queue_revalidates(self):
        grid = self.make_map(((30,30,0),))
        start, goal = pose(1.5,3.05), pose(4.5,3.05)
        plan = grid.plan(start,goal)
        self.assertGreater(len(plan['path']),2)
        self.assertGreater(plan['distance_m'],3.)
        self.assertTrue(any(action['kind']=='turn' for action in plan['segments']))
        self.assertEqual(plan['segments'][-1]['kind'],'move')
        self.assertEqual(plan,grid.plan(start,goal))
        self.assertEqual(grid.validate_plan(start,plan)['path'],plan['path'])

    def test_dynamic_cells_respect_rotated_map_and_saved_occupancy(self):
        grid = self.make_map(((1, 1, 0), (2, 2, 205)), origin=(10., 20., math.pi/2.))
        points = [(9.85, 20.15), (9.75, 20.25), (9.65, 20.35), (9.66, 20.36), (10.1, 20.1)]
        self.assertEqual(grid.obstacle_cells(points), frozenset({(3, 3)}))

    def test_dynamic_obstacle_detours_without_changing_saved_costmap(self):
        grid = self.make_map()
        start, goal = pose(1.5, 3.05), pose(4.5, 3.05)
        direct = grid.plan(start, goal)
        costmap = grid.costmap_pgm()
        cells = grid.obstacle_cells([(3.05, 3.05)])
        with self.assertRaisesRegex(PlanningError, 'MOVE swept capsule'):
            grid.validate_plan(start, direct, obstacle_cells=cells)
        detour = grid.plan(start, goal, obstacle_cells=cells)
        self.assertGreater(detour['distance_m'], direct['distance_m'])
        self.assertEqual(grid.validate_plan(start, detour, obstacle_cells=cells)['path'], detour['path'])
        self.assertEqual(grid.costmap_pgm(), costmap)
        self.assertEqual(grid.plan(start, goal), direct)

    def test_remaining_move_ignores_passed_cells_and_motion_past_goal(self):
        grid = self.make_map()
        plan = grid.plan(pose(1.5, 3.05), pose(3.5, 3.05))
        current = pose(2.5, 3.05)
        off_route = grid.obstacle_cells([(1.85, 3.05), (3.85, 3.05), (3.05, 3.65)])
        self.assertFalse(grid.route_blocked(current, plan, 0, 1., off_route))
        ahead = grid.obstacle_cells([(3.25, 3.05)])
        self.assertTrue(grid.route_blocked(current, plan, 0, 1., ahead))

    def test_remaining_turn_uses_direction_and_progress_before_later_move(self):
        grid = self.make_map()
        plan = grid.plan(pose(2.5, 3.5), pose(2.5, 2.))
        self.assertLess(plan['segments'][0]['angle_rad'], 0.)
        partway = pose(2.5, 3.5, -math.pi/4.)
        after_turn = pose(2.5, 3.5, -math.pi/2.)
        ahead = grid.obstacle_cells([(2.5, 2.5)])
        self.assertTrue(grid.route_blocked(partway, plan, 0, math.pi/4., ahead))
        self.assertTrue(grid.route_blocked(after_turn, plan, 1, 0., ahead))
        self.assertFalse(grid.route_blocked(after_turn, plan, 2, 0., ahead))

    def bent_route(self, grid):
        start = pose(1., 2.)
        plan = wire_preview(grid, dict(x=3., y=4.), [dict(kind='move', distance_m=2.),
                                                dict(kind='turn', angle_rad=math.pi/2.),
                                                dict(kind='move', distance_m=2.)])
        return dict(plan, **grid.validate_plan(start, plan))

    def test_resume_preserves_remaining_waypoints_and_respects_reported_progress(self):
        grid = self.make_map()
        original = self.bent_route(grid)
        stopped = pose(2., 2.3)
        cells = grid.obstacle_cells([(1.55, 2.), (4., 3.)])
        resumed = grid.resume_route(stopped, original, 0, 1.3, obstacle_cells=cells)
        self.assertEqual(resumed['goal'], original['goal'])
        self.assertEqual(resumed['path'], grid.validate_plan(stopped, resumed, obstacle_cells=cells)['path'])
        self.assertEqual(len(resumed['path']), 4)
        for point, expected in zip(resumed['path'][1:], [(2.3, 2.), (3., 2.), (3., 4.)]):
            self.assertAlmostEqual(point['x'], expected[0], places=5)
            self.assertAlmostEqual(point['y'], expected[1], places=5)

    def test_resume_reports_only_remaining_route_obstacle_as_blockage(self):
        grid = self.make_map()
        original = self.bent_route(grid)
        stopped = pose(2., 3.)
        ahead = grid.obstacle_cells([(3., 3.5)])
        self.assertIsNone(grid.resume_route(stopped, original, 0, 1.3, obstacle_cells=ahead))
        connector_only = grid.obstacle_cells([(2.15, 2.55)])
        with self.assertRaisesRegex(PlanningError, 'Reconnect sweep'):
            grid.resume_route(stopped, original, 0, 1.3, obstacle_cells=connector_only)
        static_grid = self.make_map(((21, 25, 0),))
        with self.assertRaisesRegex(PlanningError, 'Reconnect sweep'):
            static_grid.resume_route(stopped, self.bent_route(static_grid), 0, 1.3)

    def test_resume_turn_uses_original_pivot_and_handoff_can_skip_completed_segment(self):
        grid = self.make_map()
        original = self.bent_route(grid)
        stopped = pose(3.04, 2., math.pi/4.)
        resumed = grid.resume_route(stopped, original, 1, math.pi/4.)
        self.assertEqual(len(resumed['path']), 3)
        self.assertAlmostEqual(resumed['path'][1]['x'], 3., places=5)
        self.assertAlmostEqual(resumed['path'][1]['y'], 2., places=5)
        handoff = grid.resume_route(pose(3., 2., math.pi/2.), original, 2, 0.)
        self.assertEqual(handoff['segments'], [dict(kind='move', distance_m=2.)])

    def test_resume_overshoot_stays_stopped_and_goal_tolerance_needs_no_motion(self):
        grid = self.make_map()
        original = self.bent_route(grid)
        with self.assertRaisesRegex(PlanningError, 'Stopped beyond'):
            grid.resume_route(pose(3.08, 2.), original, 0, 1.9)
        resumed = grid.resume_route(pose(3.01, 2.), original, 0, 1.9)
        self.assertEqual(len(resumed['path']), 2)
        arrived = grid.resume_route(pose(3., 3.95), original, 2, 1.9)
        self.assertTrue(arrived['no_motion'])
        self.assertEqual(arrived['segments'], [])

    def test_straight_sweep_detects_closed_cell_edge_not_only_centers(self):
        grid = self.make_map(((20,20,0),))
        y = 2.-BODY_RADIUS_M  # Exact swept disk tangency at cell y=2 m.
        start = pose(1.,y)
        plan = wire_preview(grid,pose(3.5,y),[dict(kind='move',distance_m=2.5)])
        # The cell center is beyond the path disk. Its occupied square touches
        # the disk, so the encoded queue is unsafe.
        with self.assertRaisesRegex(PlanningError,'MOVE swept capsule'):
            grid.validate_plan(start,plan)

    def test_full_turn_disk_catches_corner_outside_both_endpoint_rectangles(self):
        grid = self.make_map(((124,100,0),),resolution=.01,width=200,height=200)
        # x offset .24 exceeds the .175/.20 extents of the two rectangles at
        # yaw 0/90, but lies inside the intermediate corner sweep (.266 m).
        with self.assertRaisesRegex(PlanningError,'Start footprint'):
            grid.plan(pose(1.,1.),pose(1.,1.5))

    def test_revalidation_accounts_for_intermediate_turn_termination_error(self):
        grid = self.make_map(((23,27,0),))
        start = pose(2.,2.)
        plan = wire_preview(grid,pose(2.,3.5),[dict(kind='turn',angle_rad=math.pi/2.),
                                             dict(kind='move',distance_m=1.5)])
        # Nominal straight centerline is 0.30m from this occupied square,
        # beyond the body radius, but the 3° TURN shortfall envelope reaches it.
        with self.assertRaisesRegex(PlanningError,'MOVE swept capsule'):
            grid.validate_plan(start,plan)
        nominal_grid = load_map(self.pgm, self.yaml, use_modeled_error_margin=False)
        nominal_grid.validate_plan(start, plan)
        crossing = pose(2.1, 2.)
        with self.assertRaisesRegex(PlanningError, 'MOVE swept capsule'):
            nominal_grid.validate_plan(crossing, wire_preview(
                nominal_grid, pose(2.1, 3.5), plan['segments']))

    def test_small_turn_omission_does_not_invent_a_heading_change(self):
        grid = self.make_map()
        start = pose(2.,2.,.02)
        plan = grid.plan(start,pose(2.5,2.))
        self.assertEqual(len(plan['segments']),1)
        self.assertEqual(plan['segments'][0]['kind'],'move')
        self.assertAlmostEqual(plan['nominal_end_pose']['yaw'],.02)
        self.assertGreater(plan['nominal_goal_error_m'],.009)
        self.assertLess(plan['nominal_goal_error_m'],.011)

    def test_crc_matches_V11_example_and_preserves_float32(self):
        actions, checksum = _queue([dict(kind='move',distance_m=.5),
                                    dict(kind='turn',angle_rad=math.pi/2.),
                                    dict(kind='move',distance_m=.5)])
        # Same three exact protocol records as the independently audited READY test.
        self.assertEqual(checksum,0xeb0c70d9)
        self.assertEqual(actions[1]['angle_rad'],1.5707963705062866)

    def test_reject_tampered_preview_map_queue_and_fresh_pose(self):
        grid = self.make_map()
        start = pose(2.,2.)
        plan = grid.plan(start,pose(2.5,2.))
        altered = copy.deepcopy(plan)
        altered['segments'][0]['distance_m'] = .51
        with self.assertRaisesRegex(PlanningError,'CRC'):
            grid.validate_plan(start,altered)
        with self.assertRaisesRegex(PlanningError,'goal tolerance'):
            grid.validate_plan(pose(2.,2.3),plan)
        other = self.make_map(((3,3,0),))
        self.assertIsInstance(other,GridMap)
        with self.assertRaisesRegex(PlanningError,'exact map'):
            other.validate_plan(start,plan)

    def test_long_route_is_not_limited_by_nominal_duration(self):
        grid = self.make_map(width=320)
        start, goal = pose(2., 2.), pose(30., 2.)
        plan = grid.plan(start, goal)
        self.assertGreater(plan['nominal_duration_s'], 1200.)
        self.assertEqual(plan['segments'], [dict(kind='move', distance_m=28.)])
        self.assertEqual(grid.validate_plan(start, plan)['path'], plan['path'])

    def test_capacity_and_short_actions_rejected(self):
        with self.assertRaisesRegex(PlanningError,'128'):
            _queue([dict(kind='move',distance_m=.5)]*129)
        with self.assertRaisesRegex(PlanningError,'below its termination tolerance'):
            _queue([dict(kind='move',distance_m=.10)])
        with self.assertRaisesRegex(PlanningError,'below its termination tolerance'):
            _queue([dict(kind='move',distance_m=.02),dict(kind='move',distance_m=.5)])


if __name__ == '__main__':
    unittest.main()
