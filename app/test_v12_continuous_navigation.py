"""Continuous localization drives the real queue/stop/replan state machine."""

from copy import deepcopy
from dataclasses import replace
import math
import unittest
from unittest.mock import patch

from v11_map_runtime import ARRIVAL_REACHED, Navigation, compose, inverse
from mcu_v11 import EXECUTING, STOP_CONFIRMED
import test_v11_map_runtime as fixtures


class TrackingInput:
    def __init__(self, clock):
        self.clock = clock
        self.calls = self.stops = 0
        self.data = dict(enabled=False, status='inactive', fresh=False, reason=None, session_id=0,
                         map_pose=None, odom_pose=None, alignment=None, age_ms=None,
                         sigma_x_m=None, sigma_y_m=None, sigma_yaw_rad=None,
                         source_monotonic_s=None)

    def publish(self, exchange):
        pass

    def localize(self, seed, *, cancelled=None):
        self.calls += 1
        self.data.update(enabled=True, session_id=self.data['session_id'] + 1)
        self.accept(seed)
        return self.data['map_pose'], self.data['odom_pose']

    def accept(self, pose, *, odom=None, source=None, fresh=True):
        source = self.clock() - .01 if source is None else source
        odom = dict(x=0., y=0., yaw=0., boot=42, epoch=1) if odom is None else dict(odom)
        stamp = int(source * 1e9)
        odom.update(stamp_ns=stamp, source_monotonic_s=source)
        pose = dict(pose, stamp_ns=stamp)
        self.data.update(enabled=True, status='tracking' if fresh else 'odom_fallback', fresh=fresh,
                         map_pose=pose, odom_pose=odom, alignment=compose(pose, inverse(odom)),
                         age_ms=(self.clock() - source) * 1000, sigma_x_m=.02, sigma_y_m=.02,
                         sigma_yaw_rad=math.radians(1.), source_monotonic_s=source)

    def tracking_snapshot(self):
        return deepcopy(self.data)

    def stop_tracking(self):
        self.stops += 1
        self.data.update(enabled=False, fresh=False, status='inactive')


class ScanInput:
    def __init__(self):
        self.frame = None

    def publish(self, exchange):
        pass

    def latest_observation(self):
        return self.frame


class QueueInput:
    def __init__(self, nav):
        self.nav = nav
        self.started = []
        self.renewed = []
        self.queries = 0
        self.uploaded = []
        self.verified = True

    def query(self):
        self.queries += 1
        return self.nav.latest

    def stop(self, on_reply):
        on_reply(self.nav.latest)
        return dict(stop_verified=self.verified,
                    error={} if self.verified else dict(code='stop_proof_deadline'))

    def upload(self, plan, on_ack):
        self.uploaded.append(plan)
        ready = replace(self.nav.latest, status=replace(self.nav.latest.status, task=2, state=2,
                                                        first_error=0, first_drive_error=0))
        on_ack(ready)
        return [ready]

    def start(self, task):
        self.started.append(task)
        return replace(self.nav.latest, status=replace(self.nav.latest.status, task=task, state=4,
                       flags=self.nav.latest.status.flags & ~STOP_CONFIRMED | EXECUTING,
                       first_error=0, lease_remaining_ms=30000))

    def renew(self, task):
        self.renewed.append(task)
        return replace(self.nav.latest, status=replace(self.nav.latest.status, lease_remaining_ms=30000))


class ContinuousNavigationTest(unittest.TestCase):
    observe = fixtures.NavigationTest.observe

    def setUp(self):
        fixtures.NavigationTest.setUp(self)
        self.localizer = TrackingInput(lambda: self.now)
        self.nav = Navigation(self.grid, self.localizer, mode='drive', clock=lambda: self.now,
                              continuous_amcl=True)
        self.observe()
        self.seed = dict(x=1., y=1., yaw=0.)
        self.nav.align(self.seed, 'amcl')
        self.master = QueueInput(self.nav)

    def tick(self, dt=.05, **changes):
        self.now += dt
        tick = self.sample.measurement_tick_ms + round(dt * 1000)
        self.observe(sample_sequence=self.sample.sample_sequence + 1, measurement_tick_ms=tick,
                     publication_tick_ms=tick + 5, **changes)

    def executing(self, target=None):
        target = dict(x=2.5, y=1.) if target is None else target
        self.nav.preview(target)
        self.nav.request_start(self.nav.plan['id'])
        self.nav.pending = self.nav.work = None
        self.tick(state=4, task=1, count=2, flags=self.sample.flags | EXECUTING, lease_remaining_ms=30000)
        self.nav.lease_task = (42, 1, 1)
        self.nav.goal_task = (42, 1, 1)
        return self.nav.alignment

    def park(self, *, verified=True):
        self.tick(state=9, flags=self.sample.flags & ~EXECUTING | STOP_CONFIRMED,
                  first_error=1, lease_remaining_ms=0)
        self.master.verified = verified
        self.nav._stop(self.master)
        self.nav.stop_requested = False
        self.nav._amcl_cycle()

    def scan(self, *, source=None):
        scanner = self.nav.scanner
        source = self.now if source is None else source
        scanner.frame = dict(pose=dict(x=self.sample.x, y=self.sample.y, yaw=self.sample.yaw,
                                       boot=42, epoch=1), stamp_ns=int(source * 1e9), usable=True,
                             source_mono=source, expires_mono=source + .5,
                             points=((0., 2.),), clear_rays=())
        self.nav.update_scan()

    def complete(self, *, raw_x=1.5):
        self.tick(state=8, flags=self.sample.flags & ~EXECUTING | STOP_CONFIRMED | ARRIVAL_REACHED,
                  x=raw_x, lease_remaining_ms=0)
        self.nav._amcl_cycle()

    def test_tracking_fields_and_same_session_two_previews(self):
        first = self.nav.snapshot()
        self.assertEqual(first['localization']['tracking_state'], 'tracking')
        self.assertTrue(first['can_localize'])
        for x in (1.8, 2.5):
            self.nav.preview(dict(x=x, y=1.))
            self.assertTrue(self.nav.snapshot()['can_start'])
        self.assertEqual(self.localizer.calls, 1)
        self.assertEqual(self.nav.snapshot()['goal']['phase'], 'idle')

    def test_completed_old_queue_does_not_complete_the_next_goal_before_upload(self):
        self.executing(dict(x=1.5, y=1.))
        self.complete(raw_x=.5)
        self.tick()
        self.localizer.accept(dict(x=1.5, y=1., yaw=0.),
                              odom=dict(x=.5, y=0., yaw=0., boot=42, epoch=1))
        self.nav._amcl_cycle()
        self.assertTrue(self.nav.goal['map_arrived'])
        self.nav.preview(dict(x=2., y=1.))
        self.nav.request_start(self.nav.plan['id'])
        plan = self.nav.pending
        self.nav._amcl_cycle()
        self.assertIs(self.nav.pending, plan)
        self.assertEqual(self.nav.goal['phase'], 'executing')
        self.assertIsNone(self.nav.goal['map_arrived'])
        self.assertIsNone(self.nav.detour)
        self.nav.pending = None
        self.nav._upload(self.master, plan)
        self.assertEqual(self.localizer.calls, 1)
        self.assertEqual(self.nav.goal_task, (42, 1, 2))

    def test_ekf_trial_is_not_reclassified_as_a_map_goal(self):
        self.executing()
        self.nav.ekf_turn = dict(phase='turning')
        self.localizer.accept(dict(x=1.4, y=1., yaw=0.))
        self.nav._amcl_cycle()
        self.assertFalse(self.nav.stop_requested)
        self.assertEqual(self.nav.goal['phase'], 'executing')

    def test_start_upload_can_be_cancelled_by_large_amcl_correction(self):
        self.nav.preview(dict(x=2., y=1.))
        self.nav.request_start(self.nav.plan['id'])
        self.localizer.accept(dict(x=1.3, y=1., yaw=0.))
        plan = self.nav.pending
        with self.assertRaisesRegex(ValueError, '上传已取消'):
            self.nav._upload(self.master, plan)
        self.assertTrue(self.nav.stop_requested)
        self.assertEqual(self.master.started, [])

    def test_cancelled_initialization_stops_the_background_session(self):
        def cancelled_localize(seed, *, cancelled):
            self.nav.request_stop()
            self.assertTrue(cancelled())
            return self.localizer.data['map_pose'], self.localizer.data['odom_pose']
        with patch.object(self.localizer, 'localize', side_effect=cancelled_localize):
            with self.assertRaisesRegex(ValueError, '定位已取消'):
                self.nav.align(self.seed, 'amcl')
        self.assertFalse(self.localizer.data['enabled'])
        self.assertIsNone(self.nav.goal['target'])

    def test_small_amcl_jitter_cannot_move_preview_reference(self):
        self.nav.preview(dict(x=2., y=1.))
        anchor, plan = self.nav.alignment, self.nav.plan
        self.localizer.accept(dict(x=1.04, y=1.02, yaw=math.radians(2.)))
        self.nav._amcl_cycle()
        state = self.nav.snapshot()
        self.assertIs(self.nav.alignment, anchor)
        self.assertIs(self.nav.plan, plan)
        self.assertEqual(state['pose'], self.seed)
        self.assertAlmostEqual(state['localization']['amcl_pose']['x'], 1.04)
        self.assertAlmostEqual(state['localization']['correction_distance_m'], math.hypot(.04, .02))
        self.assertTrue(state['can_start'])

    def test_new_preview_adopts_latest_trusted_transform(self):
        self.nav.preview(dict(x=2., y=1.))
        anchor = self.nav.alignment
        self.localizer.accept(dict(x=1.04, y=1., yaw=0.))
        self.nav.preview(dict(x=2.5, y=1.))
        self.assertIsNot(self.nav.alignment, anchor)
        self.assertAlmostEqual(self.nav.plan['start']['x'], 1.04)

    def test_cumulative_small_position_changes_trigger_stop(self):
        anchor = self.executing()
        for offset in (.05, .10, .15, .19):
            self.localizer.accept(dict(x=1. + offset, y=1., yaw=0.))
            self.nav._amcl_cycle()
            self.assertFalse(self.nav.stop_requested)
        self.localizer.accept(dict(x=1.21, y=1., yaw=0.))
        self.nav._amcl_cycle()
        self.assertTrue(self.nav.stop_requested)
        self.assertIs(self.nav.alignment, anchor)
        self.assertEqual(self.nav.goal['phase'], 'replanning')
        self.assertNotIn('remaining', self.nav.detour)

    def test_heading_threshold_uses_same_stamp_odom_not_current_odom(self):
        self.executing()
        self.tick(yaw=math.radians(30.))
        self.localizer.accept(dict(x=1., y=1., yaw=math.radians(9.)),
                              odom=dict(x=0., y=0., yaw=0., boot=42, epoch=1))
        self.nav._amcl_cycle()
        self.assertFalse(self.nav.stop_requested)
        self.localizer.accept(dict(x=1., y=1., yaw=math.radians(11.)))
        self.nav._amcl_cycle()
        self.assertTrue(self.nav.stop_requested)

    def test_missing_parking_proof_never_changes_transform_or_restarts(self):
        anchor = self.executing()
        self.localizer.accept(dict(x=1.3, y=1., yaw=0.))
        self.nav._amcl_cycle()
        self.park(verified=False)
        self.assertIs(self.nav.alignment, anchor)
        self.assertIsNone(self.nav.detour)
        self.assertEqual(self.nav.goal['phase'], 'cancelled')
        self.assertEqual(self.master.started, [])

    def test_stopped_correction_flushes_old_obstacles_and_waits_for_new_scan(self):
        self.nav.scanner = ScanInput()
        self.executing()
        self.scan()
        old_frame = deepcopy(self.nav.scanner.frame)
        self.assertTrue(self.nav.obstacles.cells)
        self.localizer.accept(dict(x=1.3, y=1., yaw=0.))
        self.nav._amcl_cycle()
        self.park()
        self.assertAlmostEqual(self.nav.alignment['x'], 1.3)
        self.assertFalse(self.nav.obstacles.cells)
        self.assertIsNone(self.nav.scan_points)
        self.assertEqual(len(self.nav.trail), 1)
        self.nav.scanner.frame = old_frame
        self.nav.update_scan()
        self.nav._avoidance_cycle(self.master, self.nav.latest)
        self.assertIsNone(self.nav.pending)
        self.tick(.02)
        self.scan()
        with patch.object(self.grid, 'resume_route', wraps=self.grid.resume_route) as resume:
            self.nav._avoidance_cycle(self.master, self.nav.latest)
            resume.assert_not_called()
        self.assertAlmostEqual(self.nav.pending['start']['x'], 1.3)
        self.assertEqual(self.nav.pending['goal'], dict(x=2.5, y=1.))

    def test_expired_amcl_uses_last_trusted_reference_after_verified_stop(self):
        self.executing()
        self.localizer.accept(dict(x=1.3, y=1., yaw=0.))
        self.nav._amcl_cycle()
        self.localizer.data.update(fresh=False, status='odom_fallback')
        self.park()
        self.assertAlmostEqual(self.nav.alignment['x'], 1.3)
        self.assertEqual(self.nav.snapshot()['localization']['tracking_state'], 'odom_fallback')

    def test_amcl_loss_keeps_motion_and_lease_renewal(self):
        anchor = self.executing()
        self.localizer.data.update(fresh=False, status='odom_fallback', reason='AMCL exited')
        self.tick(x=.1, lease_remaining_ms=4000)
        self.nav._amcl_cycle()
        self.nav._renew_if_due(self.master, self.nav.latest)
        self.assertIs(self.nav.alignment, anchor)
        self.assertAlmostEqual(self.nav.snapshot()['pose']['x'], 1.1)
        self.assertFalse(self.nav.stop_requested)
        self.assertEqual(self.master.renewed, [1])

    def test_recovered_big_amcl_jump_stops_fallback_motion(self):
        self.executing()
        self.localizer.data.update(fresh=False, status='odom_fallback')
        self.nav._amcl_cycle()
        self.localizer.accept(dict(x=1.4, y=1., yaw=0.))
        self.nav._amcl_cycle()
        self.assertTrue(self.nav.stop_requested)

    def test_manual_stop_cancels_goal_even_if_amcl_recovers(self):
        self.executing()
        self.nav.request_stop()
        self.park()
        self.localizer.accept(dict(x=1.4, y=1., yaw=0.))
        self.nav._amcl_cycle()
        self.assertIsNone(self.nav.goal['target'])
        self.assertEqual(self.nav.goal['phase'], 'cancelled')
        self.assertIsNone(self.nav.detour)
        self.assertEqual(self.master.started, [])

    def test_manual_zero_stops_tracking_and_old_pose_cannot_overwrite_it(self):
        self.nav.preview(dict(x=2., y=1.))
        manual = dict(x=2., y=2., yaw=math.pi / 2)
        self.nav.align(manual, 'manual')
        self.assertEqual(self.localizer.stops, 1)
        self.localizer.accept(dict(x=1.4, y=1., yaw=0.))
        self.nav._amcl_cycle()
        state = self.nav.snapshot()
        self.assertEqual(state['pose'], manual)
        self.assertEqual(state['localization']['tracking_state'], 'manual')
        self.assertIsNone(state['localization']['amcl_pose'])
        self.assertIsNone(state['goal']['target'])
        self.assertIsNone(self.nav.plan)

    def test_mcu_restart_or_invalid_odom_invalidates_tracking_and_goal(self):
        for change in (dict(boot=43), dict(flags=0)):
            with self.subTest(change=change):
                self.setUp()
                self.executing()
                self.tick(**change)
                self.nav._amcl_cycle()
                self.assertIsNone(self.nav.alignment)
                self.assertIsNone(self.nav.goal['target'])
                self.assertFalse(self.localizer.data['enabled'])

    def test_complete_requires_a_new_amcl_scan_after_measured_stop(self):
        self.executing()
        self.localizer.accept(dict(x=2.5, y=1., yaw=0.), source=self.now - .01)
        self.complete()
        self.assertTrue(self.nav.snapshot()['task']['arrival'])
        self.assertIsNone(self.nav.goal['map_arrived'])
        self.assertEqual(self.nav.goal['phase'], 'unverified')
        # A late callback for the old scan still cannot prove map arrival.
        self.localizer.accept(dict(x=2.5, y=1., yaw=0.), source=self.nav.goal_stop_time - .001)
        self.nav._amcl_cycle()
        self.assertIsNone(self.nav.goal['map_arrived'])
        self.tick()
        self.localizer.accept(dict(x=2.46, y=1., yaw=0.),
                              odom=dict(x=1.5, y=0., yaw=0., boot=42, epoch=1))
        self.nav._amcl_cycle()
        self.assertTrue(self.nav.goal['map_arrived'])
        self.assertAlmostEqual(self.nav.goal['distance_m'], .04)

    def test_mcu_arrival_without_amcl_stays_unverified_then_recovery_replans(self):
        self.executing()
        self.localizer.data.update(fresh=False, status='odom_fallback')
        self.complete()
        self.assertEqual(self.nav.goal['phase'], 'unverified')
        self.assertIsNone(self.nav.goal['map_arrived'])
        self.tick()
        self.localizer.accept(dict(x=2.1, y=1., yaw=0.),
                              odom=dict(x=1.5, y=0., yaw=0., boot=42, epoch=1))
        self.nav._amcl_cycle()
        self.assertFalse(self.nav.goal['map_arrived'])
        self.assertEqual(self.nav.goal['phase'], 'replanning')
        self.nav._avoidance_cycle(self.master, self.nav.latest)
        self.assertIsNotNone(self.nav.pending)
        self.assertAlmostEqual(self.nav.pending['start']['x'], 2.1)
        self.assertEqual(self.nav.pending['goal'], dict(x=2.5, y=1.))
        self.assertTrue(self.nav.latest.status.flags & ARRIVAL_REACHED)

    def test_manual_mode_can_execute_but_cannot_claim_map_arrival(self):
        self.nav.align(self.seed, 'manual')
        self.executing()
        self.complete()
        self.assertEqual(self.nav.goal['phase'], 'unverified')
        self.assertIsNone(self.nav.goal['map_arrived'])

    def test_corrected_queue_uses_existing_upload_and_start(self):
        self.executing()
        self.localizer.accept(dict(x=1.3, y=1., yaw=0.))
        self.nav._amcl_cycle()
        self.park()
        self.nav._avoidance_cycle(self.master, self.nav.latest)
        plan = self.nav.pending
        self.nav.pending = None
        self.nav._upload(self.master, plan)
        self.assertEqual(self.master.started, [2])
        self.assertEqual(self.nav.goal['target'], dict(x=2.5, y=1.))
        self.assertEqual(self.nav.goal['phase'], 'executing')
        self.assertEqual(self.nav.lease_task, (42, 1, 2))


if __name__ == '__main__':
    unittest.main()
