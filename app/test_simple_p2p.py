"""Pure controller tests: no maps, ROS, serial, or physical stop assertions."""
from dataclasses import FrozenInstanceError
import importlib.util
import math
from pathlib import Path
import sys
import unittest

from simple_p2p import Config, Pose, SimplePointController


class SimplePointTests(unittest.TestCase):
    def test_import_has_no_ROS_or_device_side_effects(self):
        before = set(sys.modules)
        spec = importlib.util.spec_from_file_location('isolated_simple_point',
            str(Path(__file__).with_name('simple_p2p.py')))
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        self.assertFalse({'rospy', 'serial', 'tf2_ros'} & (set(sys.modules)-before))

    def test_validated_configuration_is_immutable_and_enforces_user_caps(self):
        configuration = Config()
        with self.assertRaises(FrozenInstanceError):
            configuration.max_linear_m_s = 1.
        for values in (dict(max_linear_m_s=.030001), dict(max_angular_rad_s=math.radians(7)),
                       dict(duration_s=201), dict(arrival_tolerance_m=.11),
                       dict(minimum_linear_m_s=.04), dict(minimum_angular_rad_s=.2),
                       dict(align_tolerance_rad=math.radians(9)), dict(slowdown_distance_m=.1),
                       dict(wheel_diameter_m=0), dict(wheel_separation_m=float('inf'))):
            with self.subTest(values=values), self.assertRaises(ValueError):
                Config(**values)

    def test_nonfinite_boolean_and_overflow_inputs_are_rejected(self):
        for invalid in (float('nan'), float('inf'), -float('inf'), True, 10**10000):
            with self.subTest(value_type=type(invalid).__name__):
                with self.assertRaises(ValueError):
                    Config(max_linear_m_s=invalid)
                with self.assertRaises(ValueError):
                    Pose(invalid, 0., 0.)
                with self.assertRaises(ValueError):
                    SimplePointController(invalid, 0.)

    def test_diagonal_target_turns_left_then_drives_straight(self):
        controller = SimplePointController(1., 1.)
        first = controller.tick(Pose(0., 0., 0.), 0.)
        self.assertEqual(first.phase, 'turn')
        self.assertEqual(first.linear_m_s, 0.)
        self.assertGreater(first.angular_rad_s, 0.)
        self.assertAlmostEqual(first.heading_error_rad, math.pi/4)
        drive = controller.tick(Pose(0., 0., math.pi/4), 1.)
        self.assertEqual(drive.phase, 'drive')
        self.assertEqual(drive.angular_rad_s, 0.)
        self.assertEqual(drive.linear_m_s, .03)

    def test_goal_behind_never_commands_reverse(self):
        controller = SimplePointController(-1., 0.)
        turn = controller.tick(Pose(0., 0., 0.), 0.)
        self.assertEqual(turn.phase, 'turn')
        self.assertEqual(turn.linear_m_s, 0.)
        self.assertAlmostEqual(abs(turn.heading_error_rad), math.pi)
        drive = controller.tick(Pose(0., 0., math.pi), 1.)
        self.assertEqual(drive.phase, 'drive')
        self.assertGreater(drive.linear_m_s, 0.)
        self.assertEqual(drive.angular_rad_s, 0.)

    def test_heading_wrap_at_both_pi_boundaries_chooses_short_rotation(self):
        for sign in (-1, 1):
            goal_angle = sign*(math.pi-math.radians(2))
            pose_angle = -sign*(math.pi-math.radians(2))
            controller = SimplePointController(math.cos(goal_angle), math.sin(goal_angle))
            command = controller.tick(Pose(0., 0., pose_angle), 0.)
            self.assertAlmostEqual(command.heading_error_rad, -sign*math.radians(4))
            self.assertEqual(command.phase, 'turn')
            self.assertEqual(command.linear_m_s, 0.)
            self.assertLess(command.angular_rad_s*sign, 0.)

    def test_drive_error_requires_zero_tick_then_realigns(self):
        controller = SimplePointController(1., 0.)
        self.assertEqual(controller.tick(Pose(0., 0., 0.), 0.).phase, 'drive')
        paused = controller.tick(Pose(.2, 0., math.radians(9)), 1.)
        self.assertEqual((paused.phase, paused.reason), ('turn', 'realigning'))
        self.assertEqual((paused.linear_m_s, paused.angular_rad_s), (0., 0.))
        turning = controller.tick(Pose(.2, 0., math.radians(9)), 1.05)
        self.assertEqual(turning.phase, 'turn')
        self.assertEqual(turning.linear_m_s, 0.)
        self.assertLess(turning.angular_rad_s, 0.)
        resumed = controller.tick(Pose(.2, 0., math.radians(2)), 2.)
        self.assertEqual(resumed.phase, 'drive')
        self.assertEqual(resumed.angular_rad_s, 0.)

    def test_hysteresis_does_not_steer_or_realign_below_eight_degrees(self):
        controller = SimplePointController(1., 0.)
        controller.tick(Pose(0., 0., 0.), 0.)
        command = controller.tick(Pose(.1, 0., math.radians(6)), 1.)
        self.assertEqual(command.phase, 'drive')
        self.assertEqual(command.angular_rad_s, 0.)

    def test_near_goal_and_near_alignment_slow_down_above_nominal_minima(self):
        configuration = Config()
        near = SimplePointController(.15, 0.).tick(Pose(0., 0., 0.), 0.)
        far = SimplePointController(1., 0.).tick(Pose(0., 0., 0.), 0.)
        self.assertGreaterEqual(near.linear_m_s, configuration.minimum_linear_m_s)
        self.assertLess(near.linear_m_s, far.linear_m_s)
        near_turn = SimplePointController(1., 0.).tick(Pose(0., 0., math.radians(5)), 0.)
        far_turn = SimplePointController(1., 0.).tick(Pose(0., 0., math.radians(60)), 0.)
        self.assertGreaterEqual(abs(near_turn.angular_rad_s), configuration.minimum_angular_rad_s)
        self.assertLess(abs(near_turn.angular_rad_s), abs(far_turn.angular_rad_s))

    def test_arrival_includes_ten_centimeters_and_ignores_final_yaw(self):
        controller = SimplePointController(.1, 0.)
        stop = controller.tick(Pose(0., 0., math.pi), 0.)
        self.assertEqual((stop.phase, stop.reason), ('stop', 'arrived'))
        self.assertEqual((stop.linear_m_s, stop.angular_rad_s), (0., 0.))
        self.assertEqual(controller.tick(Pose(-10., 0., 0.), 1.), stop)

    def test_timeout_boundary_faults_and_never_revives(self):
        controller = SimplePointController(1., 0., started_at=10.)
        self.assertEqual(controller.tick(Pose(0., 0., 0.), 209.99).phase, 'drive')
        fault = controller.tick(Pose(0., 0., 0.), 210.)
        self.assertEqual((fault.phase, fault.reason), ('fault', 'timeout'))
        self.assertEqual((fault.linear_m_s, fault.angular_rad_s), (0., 0.))
        self.assertEqual(controller.tick(Pose(1., 0., 0.), 211.), fault)

    def test_bad_or_backward_clock_and_bad_pose_latch_zero_fault(self):
        for invalid in (float('nan'), float('inf'), True, -.1):
            controller = SimplePointController(1., 0.)
            command = controller.tick(Pose(0., 0., 0.), invalid)
            self.assertEqual(command.phase, 'fault')
            self.assertEqual((command.linear_m_s, command.angular_rad_s), (0., 0.))
        controller = SimplePointController(1., 0.)
        controller.tick(Pose(0., 0., 0.), 2.)
        self.assertEqual(controller.tick(Pose(0., 0., 0.), 1.).reason, 'time_went_backwards')
        self.assertEqual(SimplePointController(1., 0.).tick(None, 0.).reason, 'invalid_pose')

    def test_cancellation_is_terminal_zero_intent(self):
        controller = SimplePointController(1., 0.)
        controller.tick(Pose(0., 0., 0.), 0.)
        controller.cancel()
        stop = controller.tick(Pose(0., 0., 0.), .1)
        self.assertEqual((stop.phase, stop.reason), ('stop', 'cancelled'))
        self.assertEqual((stop.linear_m_s, stop.angular_rad_s), (0., 0.))

    def test_sampled_diagonal_kinematics_arrive_without_blended_turn_or_reverse(self):
        controller = SimplePointController(1., 1.)
        x = y = yaw = 0.
        for step in range(4000):
            command = controller.tick(Pose(x, y, yaw), step*.05)
            self.assertGreaterEqual(command.linear_m_s, 0.)
            self.assertLessEqual(command.linear_m_s, .03)
            self.assertLessEqual(abs(command.angular_rad_s), math.radians(6.9))
            self.assertFalse(command.linear_m_s and command.angular_rad_s)
            if command.phase in ('stop', 'fault'):
                self.assertEqual((command.phase, command.reason), ('stop', 'arrived'))
                self.assertLessEqual(math.hypot(1.-x, 1.-y), .1)
                break
            yaw += command.angular_rad_s*.05
            x += command.linear_m_s*math.cos(yaw)*.05
            y += command.linear_m_s*math.sin(yaw)*.05
        else:
            self.fail('Simple noise-free intent simulation did not reach its tolerance')


if __name__ == '__main__':
    unittest.main()
