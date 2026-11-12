import math
import struct
import unittest
from dataclasses import replace
from threading import Event, Thread
from unittest.mock import Mock, patch

import test_v11_map_runtime as base
from mcu_v11 import EXECUTING, STOP_CONFIRMED, Rejected
from v12_dock_control import dock_segment, next_action


class Camera:
    def __init__(self):
        self.value = dict(fresh=True, ready=True, frame_seq=1, distance_m=.5,
                          lateral_m=0., yaw_deg=0., reason=None)

    def snapshot(self):
        return dict(self.value)


class DockTest(unittest.TestCase):
    setUp = base.NavigationTest.setUp
    observe = base.NavigationTest.observe

    def enable(self):
        self.nav.mode = 'drive'
        self.nav.camera = Camera()
        self.observe(flags=self.sample.flags | STOP_CONFIRMED)

    def queue_step(self):
        self.nav.request_dock()
        self.nav.camera.value['frame_seq'] += 1
        self.nav._dock_cycle()
        return self.nav.pending

    def test_segment_wire_limits_and_geometry(self):
        self.assertEqual(struct.unpack('<If', dock_segment('move', -.05).segments[0])[0], 3)
        self.assertEqual(struct.unpack('<If', dock_segment('turn', math.radians(5)).segments[0])[0], 4)
        for kind, value in [('move', -.005), ('move', .01), ('move', -.051),
                            ('turn', math.radians(1)), ('turn', math.radians(5.1))]:
            with self.assertRaises(ValueError):
                dock_segment(kind, value)
        observation = dict(distance_m=.5, lateral_m=.04, yaw_deg=0.)
        self.assertGreater(next_action(observation)[1], 0.)
        observation['lateral_m'] = -.04
        self.assertLess(next_action(observation)[1], 0.)
        self.assertEqual(next_action(dict(distance_m=.5, lateral_m=0., yaw_deg=0.)), ('move', -.05))
        self.assertIsNone(next_action(dict(distance_m=.20, lateral_m=.01, yaw_deg=2.)))
        with self.assertRaisesRegex(ValueError, '人工重新摆位'):
            next_action(dict(distance_m=.21, lateral_m=.04, yaw_deg=0.))

    def test_readiness_exclusion_and_fresh_frame(self):
        self.enable()
        self.nav.camera.value['ready'] = False
        with self.assertRaises(ValueError):
            self.nav.request_dock()
        self.nav.camera.value['ready'] = True
        self.nav.request_dock()
        self.nav._dock_cycle()
        self.assertIsNone(self.nav.pending)
        with self.assertRaises(ValueError):
            self.nav.align(dict(x=0., y=0., yaw=0.), 'manual')
        self.nav.camera.value['frame_seq'] += 1
        self.nav._dock_cycle()
        self.assertIsNotNone(self.nav.pending)
        self.nav.camera.value['fresh'] = False
        self.nav._dock_cycle()
        self.assertTrue(self.nav.stop_requested)
        self.assertEqual(self.nav.dock_stop_outcome, 'failed')

    def test_invalidating_running_session_requests_stop(self):
        self.enable()
        self.queue_step()
        self.nav._invalidate('反馈缓存遗漏')
        self.assertTrue(self.nav.stop_requested)
        self.assertEqual(self.nav.dock['phase'], 'stopping')

    def test_rejected_old_firmware_never_starts(self):
        self.enable()
        plan = self.queue_step()
        master = Mock()
        master.upload.side_effect = Rejected('MCU rejected APPEND')
        with self.assertRaises(Rejected):
            self.nav._upload_dock(master, plan)
        master.start.assert_not_called()

    def test_dock_stall_does_not_renew_original_lease(self):
        for single_step, kind in ((False, 'move'), (False, 'turn'), (True, 'move'), (True, 'turn')):
            with self.subTest(single_step=single_step, kind=kind):
                self.setUp()
                self.enable()
                if single_step:
                    self.nav.dock_single_step_test = True
                    value = -.03 if kind == 'move' else math.radians(3)
                    self.nav.request_dock_step(kind, value)
                else:
                    self.nav.camera.value['lateral_m'] = .04 if kind == 'turn' else 0.
                    self.nav.request_dock()
                self.nav.camera.value['frame_seq'] += 1
                self.nav._dock_cycle()
                plan, self.nav.pending = self.nav.pending, None
                nav = self.nav

                class Peer:
                    def __init__(self):
                        self.renewals = []

                    def upload(self, document, *, on_ack):
                        ready = replace(nav.latest, status=replace(nav.latest.status, task=7, state=2, count=1))
                        on_ack(ready)
                        return [ready]

                    def start(self, task):
                        started = replace(nav.latest, status=replace(nav.latest.status, state=4 if kind == 'move' else 5,
                                          flags=nav.latest.status.flags | EXECUTING, lease_remaining_ms=30000))
                        return started

                    def renew(self, task):
                        self.renewals.append(task)
                        return nav.latest

                peer = Peer()
                nav._upload_dock(peer, plan)
                nav.work = None
                self.assertEqual(nav.dock['phase'], 'moving')
                for index, (elapsed, remaining) in enumerate(((25., 5000), (26., 4000), (28., 2000), (29., 1000)), start=2):
                    self.now = 5. + elapsed
                    self.observe(task=7, state=4 if kind == 'move' else 5,
                                 flags=self.sample.flags | EXECUTING, sample_sequence=index,
                                 measurement_tick_ms=95 + index, lease_remaining_ms=remaining, progress=0.)
                    nav.camera.value['frame_seq'] += 1
                    nav._dock_cycle()
                    nav._renew_if_due(peer, nav.latest)
                    self.assertEqual(nav.dock['phase'], 'moving')
                self.assertEqual(peer.renewals, [])
                self.now = 35.
                self.observe(task=7, state=7, flags=self.sample.flags & ~EXECUTING,
                             sample_sequence=7, measurement_tick_ms=102, lease_remaining_ms=0, first_error=3)
                nav._dock_cycle()
                self.assertTrue(nav.stop_requested)
                self.assertEqual(nav.dock_stop_outcome, 'failed')
                self.assertIsNone(nav.lease_task)

    def test_loss_during_upload_fences_start(self):
        self.enable()
        plan = self.queue_step()
        entered, release = Event(), Event()
        master = Mock()
        master.stop_revision = 0
        master.request_stop.side_effect = lambda: setattr(master, 'stop_revision', master.stop_revision + 1)
        self.nav.feedback = master
        def upload(plan, *, on_ack, revision):
            entered.set()
            self.assertTrue(release.wait(2))
            return [self.nav.latest]
        master.upload.side_effect = upload
        errors = []
        def worker():
            try:
                self.nav._upload_dock(master, plan)
            except ValueError as exc:
                errors.append(str(exc))
        thread = Thread(target=worker)
        thread.start()
        self.assertTrue(entered.wait(2))
        self.nav.camera.value['ready'] = False
        self.nav._dock_cycle()
        release.set()
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(master.stop_revision, 1)
        master.start.assert_not_called()

    def test_manual_stop_during_start_requests_worker_fence(self):
        self.enable()
        plan = self.queue_step()
        entered, release = Event(), Event()
        master = Mock()
        master.stop_revision = 0
        master.request_stop.side_effect = lambda: setattr(master, 'stop_revision', master.stop_revision + 1)
        master.history_since.return_value = dict(cursor=0, exchanges=[], missed=0, connected=True, error=None)
        master.upload.return_value = [self.nav.latest]
        self.nav.feedback = master
        def start(task, *, revision):
            self.assertEqual(revision, 0)
            entered.set()
            self.assertTrue(release.wait(2))
            return self.nav.latest
        master.start.side_effect = start
        thread = Thread(target=self.nav._upload_dock, args=(master, plan))
        thread.start()
        self.assertTrue(entered.wait(2))
        self.nav.request_stop()
        release.set()
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(master.stop_revision, 1)
        self.assertTrue(self.nav.stop_requested)
        self.assertEqual(self.nav.dock_stop_outcome, 'cancelled')
        self.assertIsNone(self.nav.lease_task)
        self.nav._renew_if_due(master, self.nav.latest)
        master.renew.assert_not_called()

    def test_step_requires_stop_proof_then_later_frame(self):
        self.enable()
        self.queue_step()
        self.nav.work = None
        self.nav.dock['phase'] = 'moving'
        self.observe(state=7, flags=self.sample.flags | 8)
        self.nav._dock_cycle()
        self.assertTrue(self.nav.stop_requested)
        master = Mock()
        master.stop.return_value = dict(stop_verified=True)
        self.nav._stop(master)
        self.nav.stop_requested = False
        self.nav.camera.value.update(distance_m=.20, lateral_m=0., yaw_deg=0.)
        self.nav._dock_cycle()
        self.assertEqual(self.nav.dock['phase'], 'observing')
        self.nav.camera.value['frame_seq'] += 1
        self.nav._dock_cycle()
        self.assertEqual(self.nav.dock['phase'], 'complete')

    def test_complete_state_and_loss_during_stop_are_latched(self):
        self.enable()
        self.queue_step()
        self.nav.work = None
        self.nav.dock['phase'] = 'moving'
        self.observe(state=8, flags=self.sample.flags | 8)
        self.nav._dock_cycle()
        self.assertEqual(self.nav.dock_stop_outcome, 'continue')
        self.nav.camera.value['ready'] = False
        self.nav._dock_cycle()
        self.nav.camera.value['ready'] = True
        master = Mock()
        master.stop.return_value = dict(stop_verified=True)
        self.nav._stop(master)
        self.assertEqual(self.nav.dock['phase'], 'failed')

    def test_stop_error_fails_dock_without_claiming_proof(self):
        self.enable()
        self.queue_step()
        self.nav.request_stop()
        closed, master = Event(), Mock(failed=False)
        master.discover.return_value = self.nav.latest
        master.history_since.return_value = dict(cursor=0, missed=0, exchanges=[], connected=True, error=None)
        def failed_stop(**unused):
            closed.set()
            raise ValueError('stop result unavailable')
        master.stop.side_effect = failed_stop
        with patch('v11_map_runtime.FeedbackWorker', return_value=master):
            self.nav.run('/dev/offline', closed)
        self.assertEqual(self.nav.dock['phase'], 'failed')
        self.assertIn('停车未确认', self.nav.dock['message'])
        self.assertFalse(self.nav.stop_verified)

    def test_single_step_is_opt_in_bounded_and_never_continues(self):
        self.enable()
        with self.assertRaisesRegex(ValueError, '未启用'):
            self.nav.request_dock_step('move', -.03)
        self.nav.dock_single_step_test = True
        for kind, value in [('move', -.019), ('move', -.051), ('turn', math.radians(1))]:
            with self.assertRaises(ValueError):
                self.nav.request_dock_step(kind, value)
        self.nav.camera.value['ready'] = False
        with self.assertRaises(ValueError):
            self.nav.request_dock_step('move', -.03)
        self.nav.camera.value['ready'] = True
        self.nav.request_dock_step('move', -.03)
        self.nav._dock_cycle()
        self.assertIsNone(self.nav.pending)
        self.nav.camera.value['frame_seq'] += 1
        self.nav._dock_cycle()
        code, value = struct.unpack('<If', self.nav.pending.segments[0])
        self.assertEqual(code, 3)
        self.assertAlmostEqual(value, -.03)
        self.nav.pending = None
        self.nav.work = None
        self.nav.dock['phase'] = 'moving'
        self.nav.lease_task = (42, 1, self.sample.task)
        self.observe(state=8, flags=self.sample.flags | 8)
        self.nav._dock_cycle()
        self.assertEqual(self.nav.dock_stop_outcome, 'step_complete')
        self.assertIsNone(self.nav.lease_task)
        master = Mock()
        self.nav._renew_if_due(master, self.nav.latest)
        master.renew.assert_not_called()
        master.stop.return_value = dict(stop_verified=True)
        self.nav._stop(master)
        self.nav.stop_requested = False
        self.nav.camera.value['frame_seq'] += 1
        self.nav._dock_cycle()
        self.assertEqual(self.nav.dock['phase'], 'step_complete')
        self.assertIsNone(self.nav.pending)

    def test_single_step_stale_during_stop_fails(self):
        self.enable()
        self.nav.dock_single_step_test = True
        self.nav.request_dock_step('turn', math.radians(3))
        self.nav.camera.value['frame_seq'] += 1
        self.nav._dock_cycle()
        self.nav.work = None
        self.nav.dock['phase'] = 'moving'
        self.observe(state=8, flags=self.sample.flags | 8)
        self.nav._dock_cycle()
        self.nav.camera.value['fresh'] = False
        self.nav._dock_cycle()
        self.nav.camera.value['fresh'] = True
        master = Mock()
        master.stop.return_value = dict(stop_verified=True)
        self.nav._stop(master)
        self.assertEqual(self.nav.dock['phase'], 'failed')


if __name__ == '__main__':
    unittest.main()
