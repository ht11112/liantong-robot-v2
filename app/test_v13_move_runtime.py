"""Exercise guided navigation with production planning, tracking and frames."""

from dataclasses import replace
from contextlib import redirect_stdout
import io
import json
import math
import struct
from threading import Event, Thread
import time
import unittest
from unittest.mock import Mock, patch

import test_v11_map_runtime as fixtures
from mcu_v11 import EXECUTING, STEER, STOP, QueuePlan
from v11_map_runtime import Navigation, compose
from v12_feedback_worker import FeedbackWorker
from test_v12_feedback_process import guided_simulated_port


class EstimateSource:
    def __init__(self):
        self.pose = dict(x=2., y=-1., yaw=math.radians(70.))
        self.rate = self.speed = 0.
        self.fresh = True
        self.scan_pose = None

    def ekf_snapshot(self):
        return dict(fresh=self.fresh, pose=dict(self.pose), yaw_rad=self.pose['yaw'],
                    angular_z_rad_s=self.rate, linear_x_m_s=self.speed, age_ms=10.)

    def imu_snapshot(self):
        return None

    def pose_at(self, stamp):
        return self.scan_pose

    def publish(self, exchange):
        pass

    def invalidate_history(self):
        self.fresh = False


class GuidedNavigationTests(unittest.TestCase):
    setUp = fixtures.NavigationTest.setUp
    observe = fixtures.NavigationTest.observe
    scan_observation = fixtures.NavigationTest.scan_observation

    def prepare(self, yaw=0.):
        self.source = EstimateSource()
        self.nav = Navigation(self.grid, mode='drive', clock=lambda: self.now,
                              ekf=self.source, move_correction=True)
        self.observe()
        self.nav.align(dict(x=1., y=1., yaw=yaw), 'manual')
        self.nav.preview(dict(x=1.+math.cos(yaw), y=1.+math.sin(yaw)))
        return self.nav.plan

    def start(self, yaw=0.):
        plan = self.prepare(yaw)
        self.nav.request_start(plan['id'])
        self.master = Mock()

        def upload(document, on_ack):
            ready = replace(self.nav.latest, status=replace(self.sample, task=1, state=2,
                                                           count=len(plan['segments'])))
            on_ack(ready)
            return [ready]

        self.master.upload.side_effect = upload
        active = replace(self.sample, task=1, state=4, count=len(plan['segments']),
                         flags=self.sample.flags | EXECUTING, lease_remaining_ms=30000)
        self.master.start.return_value = replace(self.nav.latest, status=active)
        self.nav._upload(self.master, plan)
        self.nav.work = self.nav.pending = None
        self.master.steer.side_effect = lambda *args: self.nav.latest
        return plan

    def test_alignment_uses_ekf_pose_and_guided_preview_crc(self):
        plan = self.prepare(math.pi/2)
        pose = self.nav.snapshot()['pose']
        self.assertAlmostEqual(pose['x'], 1.)
        self.assertAlmostEqual(pose['y'], 1.)
        self.assertAlmostEqual(pose['yaw'], math.pi/2)
        self.source.pose['x'] += math.cos(math.radians(70.)) * .2
        self.source.pose['y'] += math.sin(math.radians(70.)) * .2
        pose = self.nav.snapshot()['pose']
        self.assertAlmostEqual(pose['x'], 1.)
        self.assertAlmostEqual(pose['y'], 1.2)
        encoded = QueuePlan.from_document(plan)
        self.assertEqual(struct.unpack('<I', encoded.segments[0][:4])[0], 3)
        self.assertEqual(encoded.checksum, plan['queue_crc32'])
        self.grid.validate_plan(dict(x=1., y=1., yaw=math.pi/2), plan)

    def test_heading_residual_steers_toward_planned_line_at_ten_hz(self):
        self.start()
        self.source.pose['yaw'] += math.radians(2.)
        self.nav._move_correction_cycle(self.master)
        args = self.master.steer.call_args.args
        self.assertEqual(args[:2], (1, 0))
        self.assertLess(args[2], 0)
        self.assertEqual(args[3], self.nav.latest.status.measurement_tick_ms)
        self.now += .05
        self.nav._move_correction_cycle(self.master)
        self.assertEqual(self.master.steer.call_count, 1)
        self.now += .051
        self.source.pose['yaw'] -= math.radians(1.5)
        self.nav._move_correction_cycle(self.master)
        self.assertEqual(self.master.steer.call_count, 2)
        self.assertLess(abs(self.master.steer.call_args.args[2]), abs(args[2]))

    def test_stale_ekf_or_wheel_stops_and_manual_stop_cancels_correction(self):
        for failure in ('ekf', 'wheel', 'manual'):
            with self.subTest(failure=failure):
                self.start()
                if failure == 'ekf':
                    self.source.fresh = False
                elif failure == 'wheel':
                    self.now += .501
                else:
                    self.nav.request_stop()
                self.nav._move_correction_cycle(self.master)
                self.assertTrue(self.nav.stop_requested)
                self.master.steer.assert_not_called()

    def test_finished_segment_before_actual_send_keeps_session_connected(self):
        self.start()

        def changed(*args):
            self.nav.observe(replace(self.nav.latest, status=replace(self.nav.latest.status,
                state=6, flags=self.nav.latest.status.flags & ~EXECUTING)))
            raise ValueError('STEER requires the exact active MOVE')

        self.master.steer.side_effect = changed
        self.nav._move_correction_cycle(self.master)
        self.assertTrue(self.nav.connected)
        self.assertFalse(self.nav.stop_requested)
        self.assertIsNone(self.nav.move_controller_key)

    def test_motion_ekf_frame_cannot_be_used_for_zero_alignment(self):
        self.prepare()
        for speed, rate in ((.01, 0.), (0., .03)):
            self.source.speed, self.source.rate = speed, rate
            with self.assertRaisesRegex(ValueError, '停稳'):
                self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')

    def test_epoch_change_invalidates_fused_map_alignment(self):
        self.start()
        self.nav.observe(replace(self.nav.latest, status=replace(self.nav.latest.status,
            epoch=2, state=0, task=0, flags=self.sample.flags, lease_remaining_ms=0)))
        self.assertIsNone(self.nav.ekf_alignment)
        self.assertIsNone(self.nav.snapshot()['pose'])
        self.nav._move_correction_cycle(self.master)
        self.master.steer.assert_not_called()
        self.assertIsNone(self.nav.route)

    def test_scan_projection_uses_fused_pose_at_scan_time(self):
        self.prepare()
        self.nav.scanner = Mock()
        local = dict(self.source.pose)
        local['yaw'] += math.pi/2
        self.source.scan_pose = local
        frame = dict(pose=dict(boot=42, epoch=1), stamp_ns=9, source_mono=self.now-.1,
                     expires_mono=self.now+.4, usable=True, points=((1., 0.),), clear_rays=())
        self.nav.scanner.latest_observation.return_value = frame
        self.nav.update_scan()
        expected = compose(self.nav.ekf_alignment, local)
        point = self.nav.scan_points[0]
        self.assertAlmostEqual(point[0], expected['x'] + math.cos(expected['yaw']))
        self.assertAlmostEqual(point[1], expected['y'] + math.sin(expected['yaw']))
        self.source.scan_pose = None
        self.nav.scanner.latest_observation.return_value = dict(frame, stamp_ns=10)
        self.nav.update_scan()
        self.assertEqual(self.nav.scan_stamp, 9)

    def wait_until(self, condition, timeout=3.):
        deadline = time.monotonic()+timeout
        while not condition():
            if time.monotonic() >= deadline:
                self.fail('Timed out waiting for navigation worker')
            time.sleep(.005)

    def test_real_consumer_keeps_steering_during_blocked_route_check_and_stop(self):
        source = EstimateSource()
        nav = Navigation(self.grid, mode='drive', scanner=Mock(), ekf=source, move_correction=True)
        master = FeedbackWorker('normal', port_factory=guided_simulated_port)
        closed, blocked, release = Event(), Event(), Event()
        check_route = self.grid.route_blocked
        log = io.StringIO()

        def slow_check(*args):
            if not blocked.is_set():
                blocked.set()
                release.wait(2.)
            return check_route(*args)

        with patch('v11_map_runtime.FeedbackWorker', return_value=master), \
                patch.object(self.grid, 'route_blocked', side_effect=slow_check), redirect_stdout(log):
            navigation = Thread(target=nav.run, args=('offline', closed, True))
            navigation.start()
            try:
                self.wait_until(lambda: nav.connected and nav._fresh())
                nav.align(dict(x=1., y=1., yaw=0.), 'manual')
                nav.scan_points, nav.scan_expires = (), time.monotonic()+5.
                nav.preview(dict(x=2., y=1.))
                nav.request_start(nav.plan['id'])
                self.assertTrue(blocked.wait(3.))
                cursor = master.transmissions_since(0)['cursor']
                history_cursor = master.snapshot()['cursor']
                source.pose['yaw'] += .03
                started = time.monotonic()
                time.sleep(.72)
                self.assertFalse(release.is_set())
                replies = [r for r in master.history_since(history_cursor)['exchanges'] if r.opcode == STEER]
                self.assertGreaterEqual(len(replies), 4)
                times = [r.sent_at for r in replies]
                self.assertLess(max(b-a for a, b in zip(times, times[1:])), .3)
                bases = [struct.unpack_from('<I', bytes.fromhex(r.request_hex), 38)[0] for r in replies]
                self.assertEqual(bases, sorted(set(bases)))
                self.assertLess(times[0]-started, .3)
                self.assertTrue(all(struct.unpack_from('<h', bytes.fromhex(r.request_hex), 36)[0] < 0
                                    for r in replies[-3:]))

                nav.request_stop()
                self.wait_until(lambda: any(e['opcode'] == STOP
                    for e in master.transmissions_since(cursor)['entries']))
                time.sleep(.15)
                entries = master.transmissions_since(cursor)['entries']
                stop_index = next(i for i, e in enumerate(entries) if e['opcode'] == STOP)
                self.assertNotIn(STEER, [e['opcode'] for e in entries[stop_index+1:]])
                release.set()
                self.wait_until(lambda: nav.stop_verified is True)
                self.assertIsNone(nav.feedback_error)
            finally:
                release.set()
                closed.set()
                navigation.join(4.)
                if navigation.is_alive():
                    self.fail('Navigation did not release its consumer and serial owner')
        self.assertFalse(master.process.is_alive())
        traces = [json.loads(line) for line in log.getvalue().splitlines()
                  if line.startswith('{') and json.loads(line).get('event') == 'v13_steer']
        self.assertGreaterEqual(len(traces), 4)
        for trace in traces:
            if trace['result'] == 'acknowledged':
                self.assertLessEqual(trace['submitted_monotonic_s'], trace['sent_monotonic_s'])
                self.assertLessEqual(trace['sent_monotonic_s'], trace['received_monotonic_s'])
                self.assertLessEqual(trace['received_monotonic_s'], trace['finished_monotonic_s'])
                self.assertIn('publication_tick_ms', trace)
                self.assertIn('flags', trace)

    def test_correction_exception_reaches_existing_owner_error_path(self):
        self.start()
        self.nav.feedback = self.master
        failure = RuntimeError('correction calculation failed')
        self.master.history_since.return_value = dict(exchanges=[], cursor=0, missed=0, connected=True)
        self.nav._move_correction_cycle = Mock(side_effect=failure)
        self.nav._run_feedback_consumer(Event())
        self.assertIs(self.nav.feedback_error, failure)
        self.assertFalse(self.nav.connected)
        self.assertIsNone(self.nav.route)
        self.assertIsNone(self.nav.lease_task)
        self.assertEqual(self.nav.error, str(failure))


if __name__ == '__main__':
    unittest.main()
