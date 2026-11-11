"""Exercise the real line controller with independent differential kinematics."""

import math
import unittest

from v13_move_control import LineReference, MoveController


def simulate(reference, cross, heading, rpm):
    pose = dict(x=reference.x - math.sin(reference.yaw) * cross,
                y=reference.y + math.cos(reference.yaw) * cross,
                yaw=reference.yaw + heading)
    controller = MoveController(reference)
    rate, distance, dt = 0., 0., .1
    samples = []
    while distance < reference.length_m:
        command = controller.command(pose, rate)
        difference = command['differential_millirpm'] / 1000.
        left, right = rpm - max(0., difference), rpm + min(0., difference)
        # Integrate the exact differential-drive arc for each held command.
        velocity = (left + right) / 2. * math.pi * .20 / 60.
        rate = (right - left) * math.pi * .20 / (60. * .30)
        end_yaw = pose['yaw'] + rate * dt
        if abs(rate) < 1e-12:
            pose['x'] += velocity * dt * math.cos(pose['yaw'])
            pose['y'] += velocity * dt * math.sin(pose['yaw'])
        else:
            pose['x'] += velocity / rate * (math.sin(end_yaw) - math.sin(pose['yaw']))
            pose['y'] += velocity / rate * (math.cos(pose['yaw']) - math.cos(end_yaw))
        pose['yaw'] = math.atan2(math.sin(end_yaw), math.cos(end_yaw))
        distance += velocity * dt
        samples.append((controller.command(pose, rate), left, right))
    return pose, samples


class MoveControlTests(unittest.TestCase):
    def test_reference_accumulates_exact_preceding_moves_and_turns(self):
        plan = dict(start=dict(x=1., y=2., yaw=math.pi / 2.), segments=[
            dict(kind='move', distance_m=.3), dict(kind='turn', angle_rad=math.pi / 2.),
            dict(kind='move', distance_m=.4), dict(kind='turn', angle_rad=-math.pi / 2.),
            dict(kind='move', distance_m=.8)])
        second = LineReference.from_plan(plan, 2)
        self.assertAlmostEqual(second.x, 1.)
        self.assertAlmostEqual(second.y, 2.3)
        self.assertAlmostEqual(second.yaw, math.pi)
        final = LineReference.from_plan(plan, 4)
        self.assertAlmostEqual(final.x, .6)
        self.assertAlmostEqual(final.y, 2.3)
        self.assertAlmostEqual(final.yaw, math.pi / 2.)
        self.assertEqual(final.length_m, .8)
        with self.assertRaises(ValueError):
            LineReference.from_plan(plan, 1)

    def test_turn_residual_is_corrected_against_planned_heading(self):
        plan = dict(start=dict(x=0., y=0., yaw=0.), segments=[
            dict(kind='turn', angle_rad=math.pi / 2.), dict(kind='move', distance_m=1.)])
        controller = MoveController(LineReference.from_plan(plan, 1))
        for residual in (-2., 2.):
            with self.subTest(residual=residual):
                result = controller.command(dict(x=0., y=0.,
                    yaw=math.radians(90. + residual)), 0.)
                self.assertLess(result['differential_millirpm'] * residual, 0.)

    def test_cross_track_and_left_right_signs_in_rotated_frames(self):
        for yaw in (0., math.pi / 2., math.radians(179.), math.radians(-179.)):
            controller = MoveController(LineReference(2., 3., yaw, 1.))
            for cross in (-.05, .05):
                with self.subTest(yaw=yaw, cross=cross):
                    pose = dict(x=2.-math.sin(yaw)*cross, y=3.+math.cos(yaw)*cross, yaw=yaw)
                    result = controller.command(pose, 0.)
                    self.assertAlmostEqual(result['cross_track_m'], cross)
                    self.assertLess(result['differential_millirpm'] * cross, 0.)

    def test_heading_wrap_uses_short_arc(self):
        for desired, actual, sign in ((179., -179., -1), (-179., 179., 1)):
            result = MoveController(LineReference(0., 0., math.radians(desired), 1.)).command(
                dict(x=0., y=0., yaw=math.radians(actual)), 0.)
            self.assertAlmostEqual(abs(result['heading_error_rad']), math.radians(2.))
            self.assertGreater(result['differential_millirpm'] * sign, 0.)

    def test_joint_line_deadband_suppresses_small_alternating_noise(self):
        controller = MoveController(LineReference(0., 0., 0., 1.))
        for sign in (1, -1, 1, -1):
            result = controller.command(dict(x=.2, y=sign*.004,
                                             yaw=sign*math.radians(.15)), sign*.002)
            self.assertEqual(result['differential_millirpm'], 0)

    def test_smaller_heading_error_requests_smaller_correction(self):
        controller = MoveController(LineReference(0., 0., 0., 1.))
        corrections = [abs(controller.command(dict(x=0., y=0., yaw=math.radians(error)),
                                               0.)['differential_millirpm'])
                       for error in (2., 1., .5, .25, .15)]
        self.assertTrue(all(a > b for a, b in zip(corrections, corrections[1:])))
        self.assertEqual(corrections[-1], 0)

    def test_yaw_rate_damps_rotation_toward_desired_heading(self):
        controller = MoveController(LineReference(0., 0., 0., 1.))
        pose = dict(x=0., y=0., yaw=-math.radians(1.))
        static = controller.command(pose, 0.)['differential_millirpm']
        toward = controller.command(pose, .01)['differential_millirpm']
        away = controller.command(pose, -.01)['differential_millirpm']
        self.assertGreater(away, static)
        self.assertGreater(static, toward)
        self.assertGreater(toward, 0)

    def test_return_angle_and_wheel_difference_are_bounded(self):
        for limit in (1000, 2000):
            controller = MoveController(LineReference(0., 0., 0., 1.),
                                        max_differential_millirpm=limit)
            for sign in (-1, 1):
                result = controller.command(dict(x=0., y=sign*10., yaw=0.), 0.)
                self.assertAlmostEqual(result['desired_yaw_rad'], -sign*math.radians(8.))
                self.assertEqual(result['differential_millirpm'], -sign*limit)

    def test_along_track_is_relative_to_planned_origin(self):
        controller = MoveController(LineReference(2., 3., math.pi / 2., 1.))
        result = controller.command(dict(x=1.95, y=3.4, yaw=math.pi / 2.), 0.)
        self.assertAlmostEqual(result['along_track_m'], .4)
        self.assertAlmostEqual(result['cross_track_m'], .05)

    def test_real_kinematics_return_to_line_without_repeated_divergence(self):
        for rpm in (3., 5.):
            for yaw in (0., math.pi / 2., math.radians(179.), math.radians(-179.)):
                for cross, heading in ((.05, 2.), (.05, -2.), (-.05, 2.), (-.05, -2.)):
                    with self.subTest(rpm=rpm, yaw=yaw, cross=cross, heading=heading):
                        line = LineReference(1., 2., yaw, 4.)
                        _, samples = simulate(line, cross, math.radians(heading), rpm)
                        errors = [sample[0]['cross_track_m'] for sample in samples]
                        self.assertLess(max(map(abs, errors)), .06)
                        self.assertLess(max(map(abs, errors[len(errors)*3//4:])), .007)
                        self.assertTrue(all(left > 0. and right > 0. for _, left, right in samples))
                        excursions = [error for error in errors if abs(error) > .01]
                        self.assertTrue(all(error * cross > 0. for error in excursions))

    def test_initial_two_degree_error_curves_back_instead_of_new_line(self):
        line = LineReference(0., 0., 0., 3.)
        for heading in (-2., 2.):
            with self.subTest(heading=heading):
                pose, samples = simulate(line, 0., math.radians(heading), 5.)
                self.assertLess(abs(pose['y']), .006)
                self.assertLess(abs(pose['yaw']), math.radians(.3))
                self.assertGreater(samples[-1][0]['along_track_m'], 2.99)


if __name__ == '__main__':
    unittest.main()
