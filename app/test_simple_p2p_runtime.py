"""Offline original-scan and body-clearance checks; no ROS/device imports."""
import math
from types import SimpleNamespace as NS
import unittest

from simple_p2p_runtime import DEFAULT_CONFIG, ScanGate, load_config


SETTINGS = dict(stop_clearance_m=.2, body_front_m=.2, body_rear_m=.2, body_half_width_m=.25,
                laser_x_m=.13, laser_y_m=0., laser_yaw_rad=0., scan_timeout_s=.75)


def scan(stamp=100, obstacle=None):
    ranges = [10.] * 181
    if obstacle is not None:
        ranges[90] = obstacle
    return NS(header=NS(stamp=NS(secs=stamp, nsecs=0)), angle_min=-math.pi/2,
              angle_max=math.pi/2, angle_increment=math.pi/180,
              range_min=.1, range_max=50., ranges=ranges)


class ScanGateTests(unittest.TestCase):
    def test_clear_front_does_not_prove_rear_rotation_clearance(self):
        gate = ScanGate(SETTINGS)
        gate.accept(scan(), 100100000000, 10.)
        self.assertTrue(gate(10., 'drive')['ok'])
        self.assertFalse(gate(10., 'turn')['ok'])
        checked = ScanGate(SETTINGS, turn_area_clear=True)
        checked.accept(scan(), 100100000000, 10.)
        self.assertTrue(checked(10., 'turn')['ok'])

    def test_collision_clearance_uses_body_front_and_laser_offset(self):
        gate = ScanGate(SETTINGS, turn_area_clear=True)
        gate.accept(scan(obstacle=.25), 100100000000, 10.)
        self.assertFalse(gate(10., 'drive')['ok'])  # x=.38 -> body clearance .18
        gate.accept(scan(stamp=101, obstacle=.31), 101100000000, 11.)
        self.assertTrue(gate(11., 'drive')['ok'])   # x=.44 -> clearance .24

    def test_source_age_is_original_not_callback_or_duplicate_receipt(self):
        gate = ScanGate(SETTINGS)
        gate.accept(scan(), 100100000000, 10.)
        self.assertAlmostEqual(gate(10., 'drive')['source_monotonic_s'], 9.9)
        gate.accept(scan(), 100100000000, 10.6)
        self.assertAlmostEqual(gate(10.6, 'drive')['source_monotonic_s'], 9.9)
        self.assertFalse(gate(10.8, 'drive')['ok'])

    def test_bad_or_regressed_original_scan_never_clears_obstacles(self):
        gate = ScanGate(SETTINGS)
        gate.accept(scan(stamp=101), 101100000000, 10.)
        gate.accept(scan(stamp=100), 100100000000, 10.1)
        self.assertFalse(gate(10.1, 'drive')['ok'])
        broken = scan(stamp=102)
        broken.ranges[90] = float('nan')
        gate.accept(broken, 102100000000, 11.)
        self.assertFalse(gate(11., 'drive')['ok'])

    def test_absent_unknown_and_stale_scans_are_not_clear(self):
        gate = ScanGate(SETTINGS)
        self.assertFalse(gate(10., 'drive')['ok'])
        unknown = scan()
        unknown.ranges = [math.inf] * 181
        gate.accept(unknown, 100100000000, 10.)
        self.assertFalse(gate(10., 'drive')['ok'])
        gate.accept(scan(), 101000000000, 10.)
        self.assertFalse(gate(10., 'drive')['ok'])

    def test_turn_checks_corner_sweep_even_with_operator_rear_check(self):
        gate = ScanGate(SETTINGS, turn_area_clear=True)
        gate.accept(scan(obstacle=.32), 100100000000, 10.)
        self.assertTrue(gate(10., 'drive')['ok'])  # Static front gap is .25m.
        self.assertFalse(gate(10., 'turn')['ok'])  # Swept corner gap is only .13m.

    def test_sparse_front_sector_cannot_claim_obstacle_coverage(self):
        gate = ScanGate(SETTINGS, turn_area_clear=True)
        sparse = scan()
        sparse.ranges = [10.] * 11
        sparse.angle_increment = math.pi/10
        gate.accept(sparse, 100100000000, 10.)
        self.assertFalse(gate(10., 'drive')['ok'])

    def test_cli_default_configuration_exists_in_repo_and_standalone_package(self):
        controller, obstacle = load_config(DEFAULT_CONFIG)
        self.assertEqual(controller.max_linear_m_s, .03)
        self.assertEqual(obstacle['stop_clearance_m'], .2)


if __name__ == '__main__':
    unittest.main()
