import math
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

from v11_map_runtime import Navigation, load_map
from mcu_v11 import ACK_VALID, ODOM_INITIALIZED, ODOM_VALID, Exchange, QueuePlan, Telemetry


class Localization:
    def __init__(self, result):
        self.result = result
        self.calls = 0

    def publish(self, exchange):
        pass

    def localize(self, seed):
        self.calls += 1
        if isinstance(self.result, Exception):
            raise self.result
        return self.result


class NavigationTest(unittest.TestCase):
    def setUp(self):
        tmp = TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        folder = Path(tmp.name)
        pgm, yaml = folder / 'map.pgm', folder / 'map.yaml'
        pgm.write_bytes(b'P5\n80 80\n255\n' + bytes([254]) * 6400)
        yaml.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
                        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
                        encoding='utf-8')
        self.grid = load_map(pgm, yaml)
        self.now = 5.
        self.sample = Telemetry(
            boot=42, epoch=1, publication_sequence=1, publication_tick_ms=100,
            sample_sequence=1, measurement_tick_ms=95, task=0, ack_request=0,
            ack_result=0, state=0, flags=ACK_VALID | ODOM_INITIALIZED | ODOM_VALID,
            index=0, count=0, progress=0., x=0., y=0., yaw=0., v=0., w=0.,
            position_left=0, position_right=0, left_tenths_rpm=0, right_tenths_rpm=0,
            first_error=0, first_drive_error=0, first_error_tick_ms=0,
            reserved_100=0, reserved_102=0, accepted_request_watermark=0)
        self.nav = Navigation(self.grid, clock=lambda: self.now)
        self.observe()

    def observe(self, **changes):
        self.sample = replace(self.sample, **changes)
        self.nav.observe(Exchange(7, 1, self.now - .02, self.now, self.sample, '', ''))

    def test_single_amcl_anchor_reused_by_two_goals(self):
        pose = dict(x=1., y=1., yaw=0.)
        localizer = Localization((pose, dict(x=0., y=0., yaw=0.)))
        self.nav.localizer = localizer
        self.nav.align(pose, 'amcl')
        for goal in (dict(x=1.5, y=1.), dict(x=2., y=1.)):
            self.nav.preview(goal)
            result = self.nav.snapshot()
            self.assertEqual(result['localization']['mode'], 'amcl')
            plan = result['preview']
            self.assertEqual(plan['queue_crc32'], QueuePlan.from_document(plan).checksum)
            self.assertEqual(plan['path'][0], dict(x=1., y=1.))
            self.assertTrue(result['can_start'])
        self.assertEqual(localizer.calls, 1)

    def test_failed_amcl_manual_anchor_projects_mcu_delta_into_map(self):
        self.nav.localizer = Localization(TimeoutError('AMCL timed out'))
        seed = dict(x=1., y=1., yaw=math.pi / 2)
        self.observe(x=.75, y=.2)
        with self.assertRaises(TimeoutError):
            self.nav.align(seed, 'amcl')
        self.assertIsNone(self.nav.snapshot()['pose'])
        self.nav.align(seed, 'manual')
        state = self.nav.snapshot()
        self.assertEqual(state['localization']['mode'], 'manual')
        self.assertAlmostEqual(state['local_pose']['x'], 0.)
        self.assertAlmostEqual(state['local_pose']['y'], 0.)
        self.observe(x=1.25, sample_sequence=2, measurement_tick_ms=145, publication_tick_ms=150)
        state = self.nav.snapshot()
        self.assertAlmostEqual(state['pose']['x'], 1.)
        self.assertAlmostEqual(state['pose']['y'], 1.5)
        self.assertAlmostEqual(state['local_pose']['x'], .5)
        self.nav.preview(dict(x=1., y=2.))
        self.assertEqual(len(self.nav.snapshot()['preview']['segments']), 1)

    def test_duplicate_source_does_not_extend_map_anchor_lifetime(self):
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        self.now += .260
        self.observe(publication_sequence=2, publication_tick_ms=101)
        state = self.nav.snapshot()
        self.assertFalse(state['source_fresh'])
        self.assertEqual(state['localization']['mode'], 'unlocalized')
        self.assertIsNone(state['pose'])

    def test_manual_zero_accepts_wrapped_yaw_roundoff_but_not_motion(self):
        seed = dict(x=1., y=1., yaw=0.)
        require_stationary = self.nav._require_stationary
        for displacement in (0., .002):
            with self.subTest(displacement=displacement):
                self.observe(x=0., yaw=math.pi - 1e-7)

                def new_sample():
                    if self.nav.work == 'aligning':
                        self.observe(x=displacement, yaw=-math.pi + 1e-7)
                    return require_stationary()

                with patch.object(self.nav, '_require_stationary', side_effect=new_sample):
                    if displacement:
                        with self.assertRaisesRegex(ValueError, '置零期间车体位置发生变化'):
                            self.nav.align(seed, 'manual')
                    else:
                        self.nav.align(seed, 'manual')
                        self.assertEqual(self.nav.snapshot()['local_pose'], dict(x=0., y=0., yaw=0.))

    def test_preview_accepts_roundoff_but_rejects_heading_change_during_planning(self):
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        goal = dict(x=1.5, y=1.)
        plan = self.grid.plan

        for yaw, success in ((2e-7, True), (.002, False)):
            with self.subTest(yaw=yaw):
                def plan_after_sample(pose, target):
                    self.observe(x=4e-7, y=4e-7, yaw=yaw)
                    return plan(pose, target)

                with patch.object(self.grid, 'plan', side_effect=plan_after_sample):
                    if success:
                        self.nav.preview(goal)
                    else:
                        with self.assertRaisesRegex(ValueError, '规划期间起点已变化'):
                            self.nav.preview(goal)
                self.assertEqual(self.nav.snapshot()['can_start'], success)

    def test_upload_validates_current_pose_and_checks_motion_before_and_after_upload(self):
        for before, after, error in ((4e-7, 7e-7, None),
                                     (.002, .002, '^起点已变化'),
                                     (4e-7, .002, '^上传期间起点已变化')):
            with self.subTest(before=before, after=after):
                self.nav = Navigation(self.grid, mode='drive', clock=lambda: self.now)
                self.observe(x=0., yaw=0., task=0, state=0)
                self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
                self.nav.preview(dict(x=1.5, y=1.))
                plan = self.nav.snapshot()['preview']
                self.observe(x=before, yaw=2e-7)
                current_pose = self.nav.snapshot()['pose']
                master = Mock()

                def upload(plan, on_ack):
                    sample = replace(self.sample, task=1, state=2, x=after, yaw=3e-7)
                    reply = Exchange(3, 3, self.now - .02, self.now, sample, '', '')
                    on_ack(reply)
                    return [reply]

                master.upload.side_effect = upload
                master.start.return_value = Exchange(4, 4, self.now - .02, self.now,
                                                     replace(self.sample, task=1, state=3), '', '')
                with patch.object(self.grid, 'validate_plan', wraps=self.grid.validate_plan) as validate:
                    if error:
                        with self.assertRaisesRegex(ValueError, error):
                            self.nav._upload(master, plan)
                    else:
                        self.nav._upload(master, plan)
                    if before == .002:
                        validate.assert_not_called()
                        master.upload.assert_not_called()
                    else:
                        validate.assert_called_once_with(current_pose, plan)
                        master.upload.assert_called_once()
                if error:
                    master.start.assert_not_called()
                else:
                    master.start.assert_called_once_with(1)

    def test_boot_change_invalidates_prepared_route(self):
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        self.nav.preview(dict(x=1.5, y=1.))
        old_id = self.nav.snapshot()['preview']['id']
        self.observe(boot=43)
        self.assertIsNone(self.nav.snapshot()['preview'])
        with self.assertRaises(ValueError):
            self.nav.request_start(old_id)

    def test_start_consumes_preview_but_keeps_route_visible(self):
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        self.nav.preview(dict(x=1.5, y=1.))
        plan = self.nav.snapshot()['preview']
        self.nav.request_start(plan['id'])
        state = self.nav.snapshot()
        self.assertIsNone(state['preview'])
        self.assertEqual(state['route']['path'], plan['path'])
        self.assertFalse(state['can_start'])
        self.assertFalse(state['can_stop'])
        with self.assertRaises(ValueError):
            self.nav.request_stop()
        with self.assertRaises(ValueError):
            self.nav.request_start(plan['id'])


if __name__ == '__main__':
    unittest.main()
