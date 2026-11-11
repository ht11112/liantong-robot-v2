import math
import struct
from contextlib import redirect_stdout
import io
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event, Thread
import unittest
from dataclasses import astuple, replace
from unittest.mock import Mock, patch

from v11_map_runtime import Navigation, load_map, same_pose
from mcu_v11 import (ACK_VALID, EXECUTING, ODOM_INITIALIZED, ODOM_VALID, RENEW, STOP_CONFIRMED,
                     HEADER, STATUS, Exchange, LinkError, QueuePlan, RequestCancelled, Telemetry, command, crc16)
from v12_feedback_worker import FeedbackWorker, SharedFeedbackCache
from v11_ros import OdomHistory
from v11_ros import ObstacleScans, RosObstacleScanner
from test_v11_ros import OFFSET, obstacle_scan


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


class CachedOdom:
    def __init__(self):
        self.odom = OdomHistory(0)
        self.seen = []

    def publish(self, exchange):
        self.seen.append(exchange)
        self.odom.add(exchange)

    def invalidate_history(self):
        self.odom.invalidate()


def cache_exchange(sample, request, sent):
    raw = HEADER + STATUS.pack(*astuple(sample))
    raw += struct.pack('<H', crc16(raw[2:]))
    packet = command(7, sample.boot, sample.epoch, request)
    return Exchange(7, request, sent, sent + .01, sample, packet.hex(), raw.hex())


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
            lease_remaining_ms=0, accepted_request_watermark=0)
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

    def test_duplicate_source_does_not_refresh_pose_or_invalidate_alignment(self):
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        original_source_time = self.nav.source_time
        self.now += .260
        self.observe(publication_sequence=2, publication_tick_ms=101)
        self.assertTrue(self.nav.snapshot()['source_fresh'])
        self.assertEqual(self.nav.source_time, original_source_time)
        self.now = original_source_time + .500
        state = self.nav.snapshot()
        self.assertFalse(state['source_fresh'])
        self.assertEqual(state['localization']['mode'], 'manual')
        self.assertIsNone(state['pose'])
        self.observe(flags=ACK_VALID | ODOM_INITIALIZED)
        self.assertEqual(self.nav.snapshot()['localization']['mode'], 'unlocalized')

    def test_query_misses_keep_owner_and_age_real_feedback_until_next_reply(self):
        self.nav.mode = 'drive'
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        self.nav.localizer = Mock()
        initial = self.nav.latest
        initial_alignment = self.nav.alignment
        initial_source_time = self.nav.source_time
        closed, master = Event(), Mock(failed=False)
        master.discover.return_value = initial
        master.latest = initial
        master.snapshot.side_effect = lambda: dict(latest=master.latest, connected=True, error=None)
        master.cached_diagnostics.return_value = {}
        history = [initial]
        master.history_since.side_effect = lambda cursor: dict(
            cursor=len(history), exchanges=history[cursor:], missed=0, connected=True, error=None)
        steps = iter(('drop', 'age', 'receive', 'verify'))

        def query():
            step = next(steps)
            if step == 'drop':
                self.now += .1
                return None
            if step == 'age':
                state = self.nav.snapshot()
                self.assertIs(self.nav.latest, initial)
                self.assertEqual(self.nav.source_time, initial_source_time)
                self.assertTrue(state['connected'])
                self.assertTrue(state['source_fresh'])
                self.assertIsNone(state['error'])
                self.now += .41
                return None
            if step == 'receive':
                state = self.nav.snapshot()
                self.assertTrue(state['connected'])
                self.assertTrue(state['can_stop'])
                self.assertFalse(state['source_fresh'])
                self.assertFalse(state['can_prepare'])
                self.assertEqual(state['task']['phase'], 'unknown')
                self.assertEqual(state['localization']['mode'], 'manual')
                self.assertIs(self.nav.alignment, initial_alignment)
                self.assertIsNone(state['pose'])
                self.assertIsNone(state['error'])
                self.now += .02
                sample = replace(self.sample, publication_sequence=2, sample_sequence=2,
                                 publication_tick_ms=150, measurement_tick_ms=145, x=.25)
                master.latest = Exchange(7, 2, self.now - .01, self.now, sample, '', '')
                history.append(master.latest)
                return master.latest
            state = self.nav.snapshot()
            self.assertTrue(state['connected'])
            self.assertTrue(state['source_fresh'])
            self.assertTrue(state['can_prepare'])
            self.assertEqual(state['task']['phase'], 'idle')
            self.assertEqual(state['localization']['mode'], 'manual')
            self.assertIs(self.nav.alignment, initial_alignment)
            self.assertEqual(state['pose'], dict(x=1.25, y=1., yaw=0.))
            self.assertEqual(self.nav.latest.status.sample_sequence, 2)
            self.assertGreater(self.nav.source_time, initial_source_time)
            self.assertIsNone(state['error'])
            closed.set()
            return None

        master.query.side_effect = query
        with patch('v11_map_runtime.FeedbackWorker', return_value=master) as owner, \
                patch('builtins.print') as output:
            self.nav.run('/dev/ttyUSB2', closed)
        self.assertTrue(closed.is_set())
        owner.assert_called_once_with('/dev/ttyUSB2', trace_timing=False)
        master.launch.assert_called_once_with()
        master.close.assert_called_once_with()
        master.start.assert_not_called()
        master.renew.assert_not_called()
        master.upload.assert_not_called()
        master.stop.assert_not_called()
        master.diagnostics.assert_not_called()
        output.assert_not_called()
        self.assertEqual(self.nav.localizer.publish.call_count, 2)

    def test_actual_link_failure_still_closes_owner_and_reports_error(self):
        master = Mock(failed=True)
        master.discover.return_value = self.nav.latest
        master.latest = self.nav.latest
        master.snapshot.side_effect = lambda: dict(latest=master.latest, connected=False, error=None)
        master.cached_diagnostics.return_value = {}
        master.history_since.side_effect = lambda cursor: dict(cursor=1,
            exchanges=[master.latest] if not cursor else [], missed=0, connected=True, error=None)
        master.query.side_effect = LinkError('Serial exchange failed')
        with patch('v11_map_runtime.FeedbackWorker', return_value=master), patch('builtins.print'):
            self.nav.run('/dev/ttyUSB2', Event())
        state = self.nav.snapshot()
        self.assertFalse(state['connected'])
        self.assertFalse(state['source_fresh'])
        self.assertFalse(state['can_stop'])
        self.assertEqual(state['task']['phase'], 'unknown')
        self.assertEqual(state['error'], 'Serial exchange failed')

    def test_cached_history_preserves_invalid_sample_before_newer_command_result(self):
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        odom = CachedOdom()
        self.nav.localizer = odom
        worker = FeedbackWorker('unused')
        self.addCleanup(worker.close)
        self.nav.feedback = worker
        samples = [self.sample, replace(self.sample, flags=ACK_VALID | ODOM_INITIALIZED),
                   replace(self.sample, sample_sequence=2, publication_sequence=3,
                           measurement_tick_ms=145, publication_tick_ms=150)]
        frames = [cache_exchange(sample, index+1, 4.88+index*.05)
                  for index, sample in enumerate(samples)]
        for frame in frames:
            worker.cache.publish(frame)
        self.nav.observe(frames[-1])
        self.assertEqual(odom.seen, frames)
        self.assertEqual(self.nav.localization['mode'], 'unlocalized')
        self.assertEqual(odom.odom.history.summary()['count'], 1)
        source_time = self.nav.source_time
        self.nav.observe(frames[-1])
        self.assertEqual(odom.seen, frames)
        self.assertEqual(self.nav.source_time, source_time)
        self.assertEqual(self.nav.snapshot()['communication']['consumer_lag_ms'], 0.)
        self.assertEqual(worker.cache.tx_sequence.value, 0)

    def test_cache_overrun_invalidates_alignment_and_history_without_restamping(self):
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        odom = CachedOdom()
        self.nav.localizer = odom
        worker = FeedbackWorker('unused')
        worker.cache = SharedFeedbackCache(capacity=2)
        self.addCleanup(worker.close)
        self.nav.feedback = worker
        first = cache_exchange(self.sample, 1, 4.80)
        worker.cache.publish(first)
        self.nav._consume_feedback()
        for index in range(1, 4):
            sample = replace(self.sample, sample_sequence=index+1, publication_sequence=index+1,
                             measurement_tick_ms=95+index*50, publication_tick_ms=100+index*50)
            worker.cache.publish(cache_exchange(sample, index+1, 4.80+index*.05))
        self.nav.pending = {'old': 'route'}
        with redirect_stdout(io.StringIO()) as logged:
            self.nav._consume_feedback()
        self.assertEqual(self.nav.feedback_missed, 1)
        self.assertIn('v12_feedback_overrun', logged.getvalue())
        self.assertEqual(self.nav.localization['mode'], 'unlocalized')
        self.assertIsNone(self.nav.pending)
        self.assertIsNone(self.nav.route)
        self.assertIn('cache_overrun', self.nav.error)
        self.assertEqual([e.status.sample_sequence for e in odom.seen], [1, 3, 4])
        self.assertEqual(odom.odom.history.summary()['count'], 2)
        self.assertAlmostEqual(self.nav.source_time, 4.944)
        self.assertEqual(worker.stop_revision, 0)

    def test_feedback_consumer_failure_invalidates_state_without_navigation_cycle(self):
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        worker = FeedbackWorker('unused')
        self.addCleanup(worker.close)
        worker.cache.publish(cache_exchange(self.sample, 1, 4.98))
        self.nav.feedback = worker
        self.nav._consume_feedback()
        self.nav.pending = {'old': 'route'}
        worker.cache.disconnect(dict(type='LinkError', message='serial owner ended'))
        self.nav._run_feedback_consumer(Event())
        self.assertFalse(self.nav.connected)
        self.assertIsNone(self.nav.pending)
        self.assertIsNone(self.nav.lease_task)
        self.assertEqual(self.nav.localization['mode'], 'unlocalized')
        self.assertEqual(self.nav.error, 'serial owner ended')

    def test_cache_overrun_during_execution_requests_stop_once(self):
        self.nav.mode = 'drive'
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        worker = FeedbackWorker('unused')
        worker.cache = SharedFeedbackCache(capacity=2)
        self.addCleanup(worker.close)
        self.nav.feedback = worker
        for index in range(3):
            sample = replace(self.sample, flags=self.sample.flags | EXECUTING, task=7, state=4,
                             lease_remaining_ms=20000, publication_sequence=index+1,
                             sample_sequence=index+1, measurement_tick_ms=95+index*50,
                             publication_tick_ms=100+index*50)
            worker.cache.publish(cache_exchange(sample, index+1, 4.88+index*.05))
        self.nav.lease_task = 42, 1, 7
        with patch.object(worker, 'request_stop') as stop, redirect_stdout(io.StringIO()):
            self.nav._consume_feedback()
            self.nav._consume_feedback()
        stop.assert_called_once_with()
        self.assertTrue(self.nav.stop_requested)
        self.assertIsNone(self.nav.lease_task)
        self.assertEqual(self.nav.localization['mode'], 'unlocalized')

    def test_stop_cancels_pending_renew_without_closing_feedback_worker(self):
        self.nav.mode = 'drive'
        self.observe(task=7, state=4, flags=self.sample.flags | EXECUTING, lease_remaining_ms=4000)
        self.nav.lease_task = 42, 1, 7
        master = Mock(failed=False)

        def renew(task):
            self.nav.request_stop()
            raise RequestCancelled('Control request cancelled by STOP')

        master.renew.side_effect = renew
        self.nav._renew_if_due(master, self.nav.latest)
        self.assertTrue(self.nav.connected)
        self.assertTrue(self.nav.stop_requested)
        self.assertIsNone(self.nav.lease_task)
        self.assertIsNone(self.nav.error)
        master.close.assert_not_called()

    def test_renew_reads_completed_feedback_before_submitting_rpc(self):
        self.nav.mode = 'drive'
        self.observe(task=7, state=4, flags=self.sample.flags | EXECUTING, lease_remaining_ms=4000)
        self.nav.lease_task = 42, 1, 7
        stale = self.nav.latest
        worker = FeedbackWorker('unused')
        self.addCleanup(worker.close)
        self.nav.feedback = worker
        completed = replace(self.sample, state=8, flags=self.sample.flags & ~EXECUTING | STOP_CONFIRMED,
                            lease_remaining_ms=0, sample_sequence=2, publication_sequence=2,
                            measurement_tick_ms=145, publication_tick_ms=150)
        worker.cache.publish(cache_exchange(completed, 2, 4.98))
        with patch.object(worker, 'renew') as renew:
            self.nav._renew_if_due(worker, stale)
        renew.assert_not_called()
        self.assertIsNone(self.nav.lease_task)
        self.assertEqual(self.nav.latest.status.state, 8)
        self.assertTrue(self.nav.connected)

    def test_task_ending_during_renew_rpc_does_not_close_feedback_worker(self):
        self.nav.mode = 'drive'
        self.observe(task=7, state=4, flags=self.sample.flags | EXECUTING, lease_remaining_ms=4000)
        self.nav.lease_task = 42, 1, 7
        stale = self.nav.latest
        worker = FeedbackWorker('unused')
        self.addCleanup(worker.close)
        self.nav.feedback = worker
        worker.cache.publish(cache_exchange(self.sample, 1, 4.97))
        completed = replace(self.sample, state=8, flags=self.sample.flags & ~EXECUTING | STOP_CONFIRMED,
                            lease_remaining_ms=0, sample_sequence=2, publication_sequence=2,
                            measurement_tick_ms=145, publication_tick_ms=150)

        def completed_before_renew(task, *, revision):
            worker.cache.publish(cache_exchange(completed, 2, 4.98))
            raise ValueError('RENEW requires the exact active task with an unexpired lease')

        with patch.object(worker, 'renew', side_effect=completed_before_renew) as renew, \
                patch.object(worker, 'close') as close:
            self.nav._renew_if_due(worker, stale)
            renew.assert_called_once_with(7, revision=0)
            close.assert_not_called()
        self.assertIsNone(self.nav.lease_task)
        self.assertTrue(self.nav.connected)
        self.assertIsNone(self.nav.error)
        self.assertEqual(self.nav.latest.status.state, 8)
        worker.cache.publish(cache_exchange(replace(completed, publication_sequence=3), 3, 4.99))
        self.nav._consume_feedback()
        self.assertEqual(self.nav.latest.status.publication_sequence, 3)

    def test_same_pose_checks_distance_and_wrapped_heading_at_the_boundary(self):
        reference = dict(x=0., y=0., yaw=math.radians(179.))
        self.assertTrue(same_pose(dict(x=.02, y=0., yaw=math.radians(-179.)), reference))
        self.assertFalse(same_pose(dict(x=.015, y=.015, yaw=reference['yaw']), reference))
        self.assertFalse(same_pose(dict(x=0., y=0., yaw=math.radians(-178.9)), reference))

    def test_manual_zero_accepts_small_pose_changes_but_rejects_displacement_over_limit(self):
        seed = dict(x=1., y=1., yaw=0.)
        require_stationary = self.nav._require_stationary
        for displacement, success in ((.019, True), (.021, False)):
            with self.subTest(displacement=displacement):
                self.observe(x=0., yaw=math.pi - 1e-7)

                def new_sample():
                    if self.nav.work == 'aligning':
                        self.observe(x=displacement, yaw=-math.pi + 1e-7)
                    return require_stationary()

                with patch.object(self.nav, '_require_stationary', side_effect=new_sample):
                    if not success:
                        with self.assertRaisesRegex(ValueError, '置零期间车体位置发生变化'):
                            self.nav.align(seed, 'manual')
                    else:
                        self.nav.align(seed, 'manual')
                        self.assertEqual(self.nav.snapshot()['local_pose'], dict(x=0., y=0., yaw=0.))

    def test_preview_accepts_small_heading_change_but_rejects_over_limit(self):
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        goal = dict(x=1.5, y=1.)
        plan = self.grid.plan

        for yaw, success in ((math.radians(1.9), True), (math.radians(2.1), False)):
            with self.subTest(yaw=yaw):
                self.observe(x=0., yaw=0.)

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
        for before, after, error in ((.010, .025, None),
                                     (.021, .021, '^起点已变化'),
                                     (.010, .031, '^上传期间起点已变化')):
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
                active = replace(self.sample, task=1, state=3, flags=self.sample.flags | EXECUTING,
                                 lease_remaining_ms=30000)
                master.start.return_value = Exchange(4, 4, self.now - .02, self.now, active, '', '')
                with patch.object(self.grid, 'validate_plan', wraps=self.grid.validate_plan) as validate:
                    if error:
                        with self.assertRaisesRegex(ValueError, error):
                            self.nav._upload(master, plan)
                    else:
                        self.nav._upload(master, plan)
                    if before > .02:
                        validate.assert_not_called()
                        master.upload.assert_not_called()
                    else:
                        self.assertEqual(validate.call_args_list[0].args, (current_pose, plan))
                        if error:
                            self.assertEqual(validate.call_count, 1)
                        else:
                            self.assertEqual(validate.call_count, 2)
                            self.assertAlmostEqual(validate.call_args.args[0]['x'], 1. + after)
                        master.upload.assert_called_once()
                if error:
                    master.start.assert_not_called()
                    self.assertIsNone(self.nav.lease_task)
                else:
                    master.start.assert_called_once_with(1)
                    self.assertEqual(self.nav.lease_task, (42, 1, 1))

    def test_renew_uses_fresh_owned_task_status_and_stops_on_cancel(self):
        self.nav.mode = 'drive'
        self.nav.lease_task = (42, 1, 7)
        master = Mock()
        active = self.sample.flags | EXECUTING
        for index, remaining in enumerate((30000, 5010, 30000, 5010), start=2):
            self.now += 25.
            self.observe(task=7, state=4, flags=active, sample_sequence=index,
                         lease_remaining_ms=remaining)
            renewed = replace(self.sample, lease_remaining_ms=30000)
            master.renew.return_value = Exchange(RENEW, index, self.now - .01, self.now, renewed, '', '')
            self.nav._renew_if_due(master, self.nav.latest)
            self.assertAlmostEqual(self.nav.snapshot()['task']['lease_remaining_s'],
                                   29.98 if remaining == 30000 else 29.99)
        self.assertEqual(master.renew.call_count, 2)
        master.renew.assert_called_with(7)

        self.nav.request_stop()
        self.observe(task=7, state=4, flags=active, sample_sequence=6, lease_remaining_ms=4000)
        self.nav._renew_if_due(master, self.nav.latest)
        self.assertEqual(master.renew.call_count, 2)
        self.assertIsNone(self.nav.lease_task)

    def test_never_adopts_a_discovered_or_replaced_active_task(self):
        active = self.sample.flags | EXECUTING
        self.observe(task=7, state=4, flags=active, lease_remaining_ms=4000)
        master = Mock()
        self.nav._renew_if_due(master, self.nav.latest)
        self.nav.lease_task = (42, 1, 7)
        self.observe(task=8, state=4, flags=active, lease_remaining_ms=4000)
        self.nav._renew_if_due(master, self.nav.latest)
        self.assertIsNone(self.nav.lease_task)
        master.renew.assert_not_called()

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

    def obstacle_test_start(self):
        self.grid.use_modeled_error_margin = False
        scanner = Mock()
        scanner.diagnostics.return_value = dict(failure_reason=None, paired=True)
        self.scan_id = 0
        self.nav = Navigation(self.grid, mode='drive', clock=lambda: self.now, scanner=scanner)
        self.observe()
        self.nav.align(dict(x=.6, y=2., yaw=0.), 'manual')
        self.scan_observation(scanner)
        self.nav.preview(dict(x=3.45, y=2.))
        self.nav.request_start(self.nav.plan['id'])
        self.nav.pending = self.nav.work = None
        self.nav.lease_task = (42, 1, 1)
        self.observe(task=1, state=4, flags=self.sample.flags | EXECUTING, lease_remaining_ms=25000)
        return scanner

    def scan_observation(self, scanner, points=None, rays=(), *, usable=True, x=0., clear=True):
        self.scan_id += 1
        if points is None:
            # A real usable frame has a finite return, here outside this map
            # and away from the planned route. Blank scans are not clear space.
            points = ((0., 3.),) if usable else ()
            rays = ((0., 1., 0., 3.),) if usable else ()
        scanner.latest_observation.return_value = dict(
            pose=dict(boot=42, epoch=1, x=x, y=0., yaw=0.), points=points,
            clear_rays=rays, stamp_ns=self.scan_id, usable=usable,
            source_mono=self.now, expires_mono=self.now + .25)
        self.nav.update_scan()
        if clear:
            self.nav.clear_obstacle_rays()

    def test_blocked_route_stops_then_replans_from_stopped_pose_and_starts_new_task(self):
        scanner = self.obstacle_test_start()
        master = Mock()
        self.scan_observation(scanner, ((1.20, 0.),), x=.10)
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertTrue(self.nav.stop_requested)
        self.assertIsNone(self.nav.lease_task)
        self.assertEqual(self.nav.snapshot()['avoidance']['phase'], 'stopping')
        self.assertAlmostEqual(self.nav.snapshot()['avoidance']['points'][0]['x'], 1.90)
        master.upload.assert_not_called()

        self.observe(state=9, flags=self.sample.flags & ~EXECUTING | STOP_CONFIRMED,
                     first_error=1, lease_remaining_ms=0, x=.20)
        master.stop.return_value = dict(stop_verified=True)
        self.nav._stop(master)
        self.nav.stop_requested = False
        master.stop.assert_called_once()
        # The triggering return can be absent from the next front scan.
        self.scan_observation(scanner, x=.10)
        master.query.return_value = self.nav.latest
        self.nav._avoidance_cycle(master, self.nav.latest)
        route = self.nav.pending
        self.assertEqual(route['goal'], dict(x=3.45, y=2.))
        self.assertAlmostEqual(route['start']['x'], .8)
        self.assertGreater(len(route['path']), 2)
        self.assertEqual(self.nav.snapshot()['avoidance']['phase'], 'resuming')

        def upload(plan, on_ack):
            self.assertIs(plan, route)
            ready = replace(self.nav.latest, status=replace(self.sample, task=2, state=2, first_error=0))
            on_ack(ready)
            return [ready]

        master.upload.side_effect = upload
        master.start.return_value = replace(self.nav.latest, status=replace(
            self.sample, task=2, state=3, first_error=0, lease_remaining_ms=30000,
            flags=self.sample.flags & ~STOP_CONFIRMED | EXECUTING))
        self.nav._upload(master, route)
        master.start.assert_called_once_with(2)
        self.assertEqual(self.nav.lease_task, (42, 1, 2))
        self.assertEqual(self.nav.snapshot()['avoidance']['replans'], 1)

    def test_blocked_goal_waits_and_manual_stop_discards_auto_restart(self):
        scanner = self.obstacle_test_start()
        master = Mock()
        self.scan_observation(scanner, ((2.85, 0.),))
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertTrue(self.nav.stop_requested)
        self.observe(state=9, flags=self.sample.flags & ~EXECUTING | STOP_CONFIRMED,
                     first_error=1, lease_remaining_ms=0)
        master.stop.return_value = dict(stop_verified=True)
        self.nav._stop(master)
        self.nav.stop_requested = False
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertEqual(self.nav.snapshot()['avoidance']['phase'], 'waiting')
        self.assertIsNone(self.nav.pending)
        self.nav.request_stop()
        self.assertIsNone(self.nav.detour)
        self.assertEqual(self.nav.snapshot()['avoidance']['phase'], 'cancelled')
        master.query.assert_not_called()
        master.upload.assert_not_called()
        master.start.assert_not_called()

    def test_obstacle_stop_without_measured_proof_does_not_plan_a_restart(self):
        scanner = self.obstacle_test_start()
        master = Mock()
        self.scan_observation(scanner, ((1.5, 0.),))
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertTrue(self.nav.stop_requested)
        master.stop.return_value = dict(stop_verified=False, error=dict(code='stop_proof_deadline'))
        self.nav._stop(master)
        self.assertIsNone(self.nav.detour)
        self.assertEqual(self.nav.snapshot()['avoidance']['phase'], 'cancelled')
        master.query.assert_not_called()
        master.start.assert_not_called()

    def stopped_detour(self, master):
        self.observe(state=9, flags=self.sample.flags & ~EXECUTING | STOP_CONFIRMED,
                     first_error=1, lease_remaining_ms=0)
        master.stop.return_value = dict(stop_verified=True)
        self.nav._stop(master)
        self.nav.stop_requested = False
        master.query.return_value = self.nav.latest

    def fresh_tick(self, dt):
        self.now += dt
        tick = self.sample.measurement_tick_ms + round(dt * 1000)
        self.observe(sample_sequence=self.sample.sample_sequence + 1,
                     measurement_tick_ms=tick, publication_tick_ms=tick + 5)

    def test_removed_obstacle_reopens_goal_only_after_measured_clear_ray(self):
        scanner = self.obstacle_test_start()
        master = Mock()
        self.scan_observation(scanner, ((2.85, 0.),))
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.stopped_detour(master)
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertEqual(self.nav.avoidance_phase, 'waiting')
        blocked = self.nav.obstacles.cells
        self.assertTrue(blocked)
        # Bad and out-of-view observations never remove the remembered blocker.
        self.scan_observation(scanner, usable=False)
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertEqual(self.nav.obstacles.cells, blocked)
        self.fresh_tick(1.01)
        self.scan_observation(scanner)
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertIsNone(self.nav.pending)
        self.assertEqual(self.nav.obstacles.cells, blocked)
        self.scan_observation(scanner, ((3.4, 0.),), ((1., 0., 3.4, 0.),))
        master.query.return_value = self.nav.latest
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertIsNotNone(self.nav.pending)
        self.assertEqual(self.nav.pending['goal'], dict(x=3.45, y=2.))
        self.assertTrue(blocked.isdisjoint(self.nav.obstacles.cells))

    def test_sustained_missing_scan_stops_once_then_resumes_when_scan_returns(self):
        scanner = self.obstacle_test_start()
        master = Mock()
        scanner.latest_observation.return_value = None
        self.observe(x=.8, progress=.8)
        self.fresh_tick(.3)
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertFalse(self.nav.stop_requested)
        self.fresh_tick(.5)
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertTrue(self.nav.stop_requested)
        self.assertIsNone(self.nav.lease_task)
        self.observe(x=.83)
        self.stopped_detour(master)
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertIsNone(self.nav.pending)
        self.scan_observation(scanner)
        with patch.object(self.grid, 'plan', wraps=self.grid.plan) as search:
            self.nav._avoidance_cycle(master, self.nav.latest)
            search.assert_not_called()
        route = self.nav.pending
        self.assertAlmostEqual(route['start']['x'], 1.43)
        self.assertEqual(len(route['segments']), 1)
        self.assertAlmostEqual(route['segments'][0]['distance_m'], 2.02, places=5)
        self.assertEqual(master.stop.call_count, 1)

        def upload(plan, on_ack):
            ready = replace(self.nav.latest, status=replace(self.sample, task=2, state=2, first_error=0))
            on_ack(ready)
            return [ready]

        master.upload.side_effect = upload
        master.start.return_value = replace(self.nav.latest, status=replace(
            self.sample, task=2, state=3, first_error=0, lease_remaining_ms=30000,
            flags=self.sample.flags & ~STOP_CONFIRMED | EXECUTING))
        self.nav._upload(master, route)
        master.start.assert_called_once_with(2)
        self.assertEqual(self.nav.replans, 0)
        self.assertEqual(self.nav.avoidance_message, '沿原路线继续前往目标')

    def test_recovered_scan_searches_only_for_an_obstacle_on_original_remainder(self):
        initial = self.sample
        for obstruction in ('remaining', 'connector'):
            with self.subTest(obstruction=obstruction):
                self.sample = initial
                scanner = self.obstacle_test_start()
                master = Mock()
                lateral = 1.0 if obstruction == 'connector' else 0.
                self.observe(x=.8, y=lateral, progress=.8)
                self.fresh_tick(.8)
                self.nav._avoidance_cycle(master, self.nav.latest)
                self.assertTrue(self.nav.stop_requested)
                self.stopped_detour(master)
                obstacle = (1.35, 0.) if obstruction == 'remaining' else (.8, .5)
                self.scan_observation(scanner, (obstacle,))
                with patch.object(self.grid, 'plan', wraps=self.grid.plan) as search:
                    self.nav._avoidance_cycle(master, self.nav.latest)
                if obstruction == 'remaining':
                    search.assert_called_once()
                    self.assertIsNotNone(self.nav.pending)
                    self.assertFalse(self.nav.detour['rejoining'])
                else:
                    search.assert_not_called()
                    self.assertIsNone(self.nav.pending)
                    self.assertEqual(self.nav.avoidance_message, '无法安全接回原路线，保持停车')
                    master.start.assert_not_called()

    def test_duplicate_scan_does_not_refresh_scan_deadline(self):
        scanner = self.obstacle_test_start()
        master = Mock()
        seen = self.nav.last_usable_scan
        self.fresh_tick(.2)
        self.nav.update_scan()
        self.nav.snapshot()
        self.assertEqual(self.nav.last_usable_scan, seen)
        self.scan_observation(scanner, usable=False)
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertFalse(self.nav.stop_requested)
        self.fresh_tick(.6)
        self.nav._avoidance_cycle(master, self.nav.latest)
        self.assertTrue(self.nav.stop_requested)

    def test_alignment_change_clears_dynamic_map_memory(self):
        scanner = self.obstacle_test_start()
        self.scan_observation(scanner, ((1.2, 0.),))
        self.nav.snapshot()
        self.assertTrue(self.nav.obstacles.cells)
        self.observe(state=0, flags=self.sample.flags & ~EXECUTING, lease_remaining_ms=0)
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        self.assertFalse(self.nav.obstacles.cells)
        self.assertIsNone(self.nav.scan_stamp)

    def test_slow_clearance_leaves_serial_and_web_free_and_preserves_newer_hit(self):
        scanner = self.obstacle_test_start()
        hit = ((1.20, 0.),)
        self.scan_observation(scanner, hit)
        remembered = self.nav.obstacles.cells
        self.assertTrue(remembered)
        self.scan_observation(scanner, ((3.4, 0.),), ((1., 0., 3.4, 0.),), clear=False)
        entered, release, done = Event(), Event(), Event()
        calculate = self.nav.obstacles.cleared_cells
        cleared = []

        def slow_clear(cells, rays):
            cleared.append(calculate(cells, rays))
            entered.set()
            release.wait(2.)
            return cleared[0]

        def accept_feedback_and_new_hit():
            self.fresh_tick(.1)
            self.nav.snapshot()
            self.scan_observation(scanner, hit, clear=False)
            done.set()

        with patch.object(self.nav.obstacles, 'cleared_cells', side_effect=slow_clear):
            worker = Thread(target=self.nav.clear_obstacle_rays)
            reader = Thread(target=accept_feedback_and_new_hit)
            worker.start()
            try:
                self.assertTrue(entered.wait(1.))
                reader.start()
                self.assertTrue(done.wait(1.))
            finally:
                release.set()
                worker.join(2.)
                if reader.ident is not None:
                    reader.join(2.)
        self.assertTrue(remembered.issubset(cleared[0]))
        self.assertTrue(remembered.issubset(self.nav.obstacles.cells))

    def test_start_waits_for_scan_and_respects_timeout_and_stop(self):
        initial = self.sample
        for outcome in ('paired', 'late_pair', 'timeout', 'stop', 'late_stop', 'expires_after_upload'):
            with self.subTest(outcome=outcome):
                self.sample = initial
                scanner = self.obstacle_test_start()
                self.observe(task=0, state=0, flags=self.sample.flags & ~EXECUTING, lease_remaining_ms=0)
                self.nav.lease_task = None
                self.nav.preview(dict(x=3.45, y=2.))
                self.fresh_tick(.26)
                state = self.nav.snapshot()
                self.assertEqual(state['avoidance']['scan_status'], 'unavailable')
                self.assertTrue(state['can_start'])
                self.nav.request_start(self.nav.plan['id'])
                self.assertIn('启动前等待', self.nav.snapshot()['avoidance']['message'])
                master = Mock(reply_timeout=.1)
                master.sleep.side_effect = lambda seconds: setattr(self, 'now', self.now + seconds)
                began = self.now

                def query(*, start_before):
                    self.assertLess(self.now, start_before)
                    self.fresh_tick(.05)
                    if outcome == 'stop' or (outcome == 'late_stop' and self.now-began >= 1.2):
                        self.nav.request_stop()
                    elif outcome not in ('timeout', 'late_stop') and (outcome != 'late_pair' or self.now-began >= 1.2):
                        self.scan_observation(scanner)
                    return self.nav.latest

                def upload(plan, *, on_ack):
                    self.assertIsNotNone(self.nav._scan_points())
                    if outcome == 'expires_after_upload':
                        self.fresh_tick(.26)
                        self.assertIsNone(self.nav._scan_points())
                    on_ack(self.nav.latest)
                    return [self.nav.latest]

                def start(task):
                    self.assertIsNotNone(self.nav._scan_points())
                    return self.nav.latest

                master.query.side_effect = query
                master.upload.side_effect = upload
                master.start.side_effect = start
                if outcome == 'timeout':
                    with self.assertRaisesRegex(ValueError, '配对超时（2 s）'):
                        self.nav._upload(master, self.nav.pending)
                    self.assertAlmostEqual(self.now - began, 2.)
                else:
                    self.nav._upload(master, self.nav.pending)
                if outcome in ('timeout', 'stop', 'late_stop'):
                    master.upload.assert_not_called()
                    master.start.assert_not_called()
                    self.assertEqual(self.nav.stop_requested, outcome in ('stop', 'late_stop'))
                else:
                    master.upload.assert_called_once()
                    master.start.assert_called_once()
                    if outcome != 'late_pair':
                        self.assertEqual(master.query.call_count, 2 if outcome == 'expires_after_upload' else 1)
                if outcome in ('late_pair', 'late_stop'):
                    self.assertGreaterEqual(self.now-began, 1.2)
                    self.assertLess(self.now-began, 2.)

    def test_scan_wait_accepts_current_pair_at_deadline_but_stop_and_stale_still_block(self):
        initial = self.sample
        for lateness, condition in ((0., 'fresh'), (.01, 'fresh'), (.01, 'stale'), (0., 'stop')):
            with self.subTest(lateness=lateness, condition=condition):
                self.sample = initial
                self.obstacle_test_start()
                self.observe(task=0, state=0, flags=self.sample.flags & ~EXECUTING, lease_remaining_ms=0)
                if condition == 'stale':
                    self.fresh_tick(.25)
                    self.assertIsNone(self.nav._scan_points())
                else:
                    self.assertIsNotNone(self.nav._scan_points())
                if condition == 'stop':
                    self.nav.request_stop()
                deadline = self.now - lateness
                checked_at = self.now
                master = Mock(reply_timeout=.1)
                with redirect_stdout(io.StringIO()):
                    if condition == 'stale':
                        with self.assertRaisesRegex(ValueError, '配对超时'):
                            self.nav._wait_start_scan(master, deadline, 'before_upload')
                    else:
                        self.assertEqual(self.nav._wait_start_scan(master, deadline, 'before_upload'),
                                         condition == 'fresh')
                self.assertEqual(self.now, checked_at)
                master.query.assert_not_called()
                master.sleep.assert_not_called()
                master.upload.assert_not_called()
                master.start.assert_not_called()

    def test_cancel_reaches_worker_while_start_ack_is_pending(self):
        self.nav.mode = 'drive'
        self.nav.align(dict(x=.6, y=2., yaw=0.), 'manual')
        self.nav.preview(dict(x=2., y=2.))
        plan = self.nav.plan
        master = Mock()
        self.nav.feedback = master
        master.stop_revision = 0
        master.history_since.return_value = dict(cursor=0, exchanges=[], missed=0, connected=True, error=None)
        master.upload.return_value = [self.nav.latest]
        entering, accepted = Event(), Event()
        order = []

        def cancel():
            entering.set()
            self.nav.request_stop()
            order.append('cancel accepted')
            accepted.set()

        worker = Thread(target=cancel)
        def signal_stop():
            master.stop_revision += 1
        master.request_stop.side_effect = signal_stop

        def start(task, *, revision):
            self.assertEqual(revision, 0)
            worker.start()
            self.assertTrue(entering.wait(1.))
            self.assertTrue(accepted.wait(1.))
            order.append('start completed')
            return self.nav.latest
        master.start.side_effect = start
        try:
            self.nav._upload(master, plan)
        finally:
            worker.join(1.)
        self.assertTrue(accepted.is_set())
        self.assertEqual(order, ['cancel accepted', 'start completed'])
        master.request_stop.assert_called_once_with()
        self.assertEqual(master.stop_revision, 1)
        self.assertTrue(self.nav.stop_requested)

    def test_detour_resume_waits_for_pairing_and_keeps_stopped_on_timeout(self):
        initial = self.sample
        for outcome in ('paired', 'timeout', 'stop', 'expires_after_upload', 'timeout_after_upload'):
            with self.subTest(outcome=outcome):
                self.sample = initial
                scanner = self.obstacle_test_start()
                master = Mock(reply_timeout=.1)
                self.nav.detour = dict(goal=self.nav.route['goal'], attempt=None, retry_at=0.)
                self.stopped_detour(master)
                self.scan_observation(scanner)
                self.nav._avoidance_cycle(master, self.nav.latest)
                route = self.nav.pending
                self.assertIsNotNone(route)
                self.fresh_tick(.26)
                self.assertIsNone(self.nav._scan_points())
                master.reset_mock()
                master.sleep.side_effect = lambda seconds: setattr(self, 'now', self.now + seconds)
                began = self.now
                uploaded = False

                def query(*, start_before):
                    self.assertLess(self.now, start_before)
                    self.fresh_tick(.05)
                    if outcome == 'stop':
                        self.nav.request_stop()
                    elif outcome != 'timeout' and not (uploaded and outcome == 'timeout_after_upload'):
                        self.scan_observation(scanner)
                    return self.nav.latest

                def upload(plan, *, on_ack):
                    nonlocal uploaded
                    uploaded = True
                    self.assertIsNotNone(self.nav._scan_points())
                    if outcome in ('expires_after_upload', 'timeout_after_upload'):
                        self.fresh_tick(.26)
                        self.assertIsNone(self.nav._scan_points())
                    self.observe(task=2, state=2, first_error=0)
                    on_ack(self.nav.latest)
                    return [self.nav.latest]

                def start(task):
                    self.assertEqual(task, 2)
                    self.assertIsNotNone(self.nav._scan_points())
                    return replace(self.nav.latest, status=replace(self.sample, state=3,
                                   flags=self.sample.flags & ~STOP_CONFIRMED | EXECUTING,
                                   lease_remaining_ms=30000))

                master.query.side_effect = query
                master.upload.side_effect = upload
                master.start.side_effect = start
                captured = io.StringIO()
                with redirect_stdout(captured):
                    self.nav._upload(master, route)
                if outcome in ('timeout', 'timeout_after_upload'):
                    self.assertAlmostEqual(self.now-began, 2.)
                    self.assertEqual(self.nav.avoidance_phase, 'waiting')
                    self.assertEqual(self.nav.detour['goal'], route['goal'])
                    self.assertIsNone(self.nav.detour['attempt'])
                    self.assertAlmostEqual(self.nav.detour['retry_at'], self.now+1.)
                    self.assertIsNone(self.nav.pending)
                    self.assertIsNone(self.nav.work)
                    self.assertTrue(self.nav.stop_verified)
                    master.start.assert_not_called()
                    event = json.loads(captured.getvalue())
                    self.assertEqual(event['event'], 'v12_scan_wait_expired')
                    self.assertTrue(event['automatic_resume'])
                    self.assertEqual(event['stage'], 'before_start' if uploaded else 'before_upload')
                    self.assertEqual(self.nav.snapshot()['avoidance']['last_failure'], event)
                    master.query.side_effect = None
                    master.query.return_value = self.nav.latest
                    self.nav._avoidance_cycle(master, self.nav.latest)
                    self.assertIsNone(self.nav.pending)
                    self.fresh_tick(1.01)
                    self.scan_observation(scanner)
                    master.query.return_value = self.nav.latest
                    self.nav._avoidance_cycle(master, self.nav.latest)
                    self.assertIsNotNone(self.nav.pending)
                    self.assertEqual(self.nav.pending['goal'], route['goal'])
                elif outcome == 'stop':
                    self.assertIsNone(self.nav.detour)
                    self.assertIsNone(self.nav.pending)
                    self.assertTrue(self.nav.stop_requested)
                    master.start.assert_not_called()
                else:
                    master.start.assert_called_once_with(2)
                    self.assertIsNone(self.nav.detour)
                    self.assertEqual(self.nav.replans, 1)
                    self.assertEqual(master.query.call_count, 2 if outcome == 'expires_after_upload' else 1)
                self.assertEqual(master.upload.call_count, int(outcome not in ('timeout', 'stop')))

    def test_detour_collision_recheck_waits_replans_and_manual_stop_cancels(self):
        initial = self.sample
        for stage in ('before_upload', 'before_start'):
            with self.subTest(stage=stage):
                self.sample = initial
                scanner = self.obstacle_test_start()
                master = Mock(reply_timeout=.1)
                self.nav.detour = dict(goal=self.nav.route['goal'], attempt=None, retry_at=0.)
                self.stopped_detour(master)
                self.nav._avoidance_cycle(master, self.nav.latest)
                route = self.nav.pending

                def upload(plan, *, on_ack):
                    self.observe(task=2, state=2, first_error=0)
                    on_ack(self.nav.latest)
                    self.scan_observation(scanner, ((1.5, 0.),))
                    return [self.nav.latest]

                master.upload.side_effect = upload
                if stage == 'before_upload':
                    self.scan_observation(scanner, ((1.5, 0.),))
                captured = io.StringIO()
                with redirect_stdout(captured):
                    self.nav._upload(master, route)
                self.assertEqual(master.upload.call_count, int(stage == 'before_start'))
                master.start.assert_not_called()
                self.assertEqual(self.nav.avoidance_phase, 'waiting')
                self.assertEqual(self.nav.detour['goal'], route['goal'])
                self.assertIsNone(self.nav.detour['attempt'])
                self.assertIsNone(self.nav.pending)
                self.assertIsNone(self.nav.work)
                self.assertTrue(self.nav.stop_verified)
                self.assertEqual(json.loads(captured.getvalue())['stage'], stage)

                self.fresh_tick(1.01)
                self.scan_observation(scanner)
                master.query.return_value = self.nav.latest
                self.nav._avoidance_cycle(master, self.nav.latest)
                replacement = self.nav.pending
                self.assertIsNotNone(replacement)
                self.assertEqual(replacement['goal'], route['goal'])
                self.assertNotEqual(replacement['path'], route['path'])
                self.grid.validate_plan(self.nav._map_pose(), replacement,
                                        obstacle_cells=self.nav.obstacles.cells)
                self.assertEqual(self.nav.replans, 0)
                self.nav.request_stop()
                self.assertIsNone(self.nav.detour)
                self.assertIsNone(self.nav.pending)
                self.assertTrue(self.nav.stop_requested)
                master.start.assert_not_called()

    def test_detour_queue_crc_failure_is_not_a_collision_retry(self):
        self.obstacle_test_start()
        master = Mock(reply_timeout=.1)
        self.nav.detour = dict(goal=self.nav.route['goal'], attempt=None, retry_at=0.)
        self.stopped_detour(master)
        self.nav._avoidance_cycle(master, self.nav.latest)
        route = dict(self.nav.pending, queue_crc32=self.nav.pending['queue_crc32'] ^ 1)
        with self.assertRaisesRegex(ValueError, 'CRC'):
            self.nav._upload(master, route)
        master.upload.assert_not_called()
        master.start.assert_not_called()

    def test_resume_rechecks_committed_queue_when_scan_expires_during_validation(self):
        initial = self.sample
        for outcome in ('clear', 'blocked', 'timeout', 'stop'):
            with self.subTest(outcome=outcome):
                self.sample = initial
                scanner = self.obstacle_test_start()
                master = Mock(reply_timeout=.1)
                self.nav.detour = dict(goal=self.nav.route['goal'], attempt=None, retry_at=0.)
                self.stopped_detour(master)
                self.nav._avoidance_cycle(master, self.nav.latest)
                route = self.nav.pending
                began = self.now
                validations = []
                real_validate = self.grid.validate_plan

                def upload(plan, *, on_ack):
                    self.observe(task=2, state=2, first_error=0)
                    on_ack(self.nav.latest)
                    return [self.nav.latest]

                def validate(pose, plan, *, obstacle_cells=()):
                    validations.append((pose, obstacle_cells))
                    result = real_validate(pose, plan, obstacle_cells=obstacle_cells)
                    if len(validations) == 2:
                        self.fresh_tick(.26)
                        self.assertIsNone(self.nav._scan_points())
                    return result

                def query(*, start_before):
                    self.assertLess(self.now, start_before)
                    self.fresh_tick(.05)
                    if outcome == 'stop':
                        self.nav.request_stop()
                    elif outcome != 'timeout':
                        self.observe(x=.01)
                        point = (1.5, 0. if outcome == 'blocked' else 1.2)
                        self.scan_observation(scanner, (point,))
                    return self.nav.latest

                def start(task):
                    self.assertIsNotNone(self.nav._scan_points())
                    return replace(self.nav.latest, status=replace(self.sample, state=3,
                                   flags=self.sample.flags & ~STOP_CONFIRMED | EXECUTING,
                                   lease_remaining_ms=30000))

                master.upload.side_effect = upload
                master.query.side_effect = query
                master.start.side_effect = start
                master.sleep.side_effect = lambda seconds: setattr(self, 'now', self.now + seconds)
                with patch.object(self.grid, 'validate_plan', side_effect=validate), redirect_stdout(io.StringIO()):
                    self.nav._upload(master, route)
                master.upload.assert_called_once()
                if outcome in ('clear', 'blocked'):
                    self.assertEqual(len(validations), 3)
                    self.assertAlmostEqual(validations[-1][0]['x'], .61)
                    self.assertEqual(validations[-1][1], self.nav.obstacles.cells)
                    self.assertTrue(validations[-1][1])
                if outcome == 'clear':
                    master.start.assert_called_once_with(2)
                    self.assertIsNone(self.nav.detour)
                else:
                    master.start.assert_not_called()
                if outcome == 'blocked':
                    self.assertEqual(self.nav.avoidance_phase, 'waiting')
                    self.assertEqual(self.nav.detour['goal'], route['goal'])
                    self.assertIsNone(self.nav.pending)
                    self.assertTrue(self.nav.stop_verified)
                    self.assertIn('swept rectangle', self.nav.error)
                if outcome == 'timeout':
                    self.assertAlmostEqual(self.now-began, 2.)
                    self.assertEqual(self.nav.avoidance_phase, 'waiting')
                    self.assertEqual(self.nav.detour['goal'], route['goal'])
                    self.assertTrue(self.nav.stop_verified)
                if outcome == 'stop':
                    self.assertTrue(self.nav.stop_requested)
                    self.assertIsNone(self.nav.detour)


class EkfTurnTest(unittest.TestCase):
    setUp = NavigationTest.setUp
    observe = NavigationTest.observe
    scan_observation = NavigationTest.scan_observation

    def prepare_turn(self, direction=1, initial_yaw=0.):
        self.grid.use_modeled_error_margin = False
        self.scanner = Mock()
        self.scanner.diagnostics.return_value = dict(failure_reason=None, paired=True)
        self.estimate = dict(fresh=True, yaw_rad=initial_yaw, age_ms=0.)
        self.ekf = Mock()
        self.ekf.ekf_snapshot.side_effect = lambda: dict(self.estimate)
        self.nav = Navigation(self.grid, clock=lambda: self.now, mode='drive',
                              scanner=self.scanner, ekf=self.ekf, ekf_turn_test=True)
        self.observe()
        self.nav.align(dict(x=1., y=1., yaw=0.), 'manual')
        self.scan_id = 0
        self.scan_observation(self.scanner)
        self.assertTrue(self.nav.snapshot()['can_ekf_turn'])
        self.nav.request_ekf_turn(direction)
        return self.nav.pending

    def start_turn(self, direction=1, initial_yaw=0.):
        plan = self.prepare_turn(direction, initial_yaw)
        master = Mock()

        def upload(document, on_ack):
            self.assertIs(document, plan)
            ready = replace(self.nav.latest, status=replace(self.sample, task=1, state=2, count=1))
            on_ack(ready)
            return [ready]

        master.upload.side_effect = upload
        active = replace(self.sample, task=1, state=5, count=1, flags=self.sample.flags | EXECUTING,
                         lease_remaining_ms=30000)
        master.start.return_value = replace(self.nav.latest, status=active)
        self.nav._upload(master, plan)
        self.nav.pending = self.nav.work = None
        return master, plan

    def test_turn_only_queue_and_signed_ekf_stop_across_wrap(self):
        for direction in (1, -1):
            with self.subTest(direction=direction):
                initial = direction * math.radians(170.)
                master, plan = self.start_turn(direction, initial)
                self.assertEqual([s['kind'] for s in plan['segments']], ['turn'])
                self.assertAlmostEqual(plan['segments'][0]['angle_rad'], direction * math.radians(102.))
                self.assertEqual(QueuePlan.from_document(plan).checksum, plan['queue_crc32'])
                self.assertEqual(self.nav.snapshot()['ekf_turn']['phase'], 'turning')
                self.estimate['yaw_rad'] = math.atan2(math.sin(initial + direction * math.radians(86.)),
                                                     math.cos(initial + direction * math.radians(86.)))
                self.nav._ekf_turn_cycle()
                self.assertFalse(self.nav.stop_requested)
                self.estimate['yaw_rad'] = math.atan2(math.sin(initial + direction * math.radians(88.)),
                                                     math.cos(initial + direction * math.radians(88.)))
                self.nav._ekf_turn_cycle()
                self.assertTrue(self.nav.stop_requested)
                self.assertIsNone(self.nav.detour)
                self.assertIsNone(self.nav.pending)
                self.estimate['yaw_rad'] = math.atan2(math.sin(initial + direction * math.radians(90.)),
                                                     math.cos(initial + direction * math.radians(90.)))
                master.stop.return_value = dict(stop_verified=True)
                self.nav._stop(master)
                self.assertEqual(self.nav.ekf_turn['phase'], 'complete')
                self.assertAlmostEqual(self.nav.ekf_turn['turned_deg'], direction * 90.)
                self.nav._avoidance_cycle(master, self.nav.latest)
                self.assertIsNone(self.nav.pending)
                self.assertEqual(master.upload.call_count, 1)
                self.assertEqual(master.start.call_count, 1)

    def test_turn_stale_feedback_obstacle_and_mcu_bound_abort_without_resume(self):
        for reason in ('ekf', 'scan', 'obstacle', 'mcu_bound', 'mcu_fault'):
            with self.subTest(reason=reason):
                master, _ = self.start_turn()
                if reason == 'ekf':
                    self.estimate['fresh'] = False
                elif reason == 'scan':
                    self.nav.scan_expires = self.now
                elif reason == 'obstacle':
                    self.nav.obstacles.cells = frozenset({(20, 20)})
                else:
                    self.nav.latest = replace(self.nav.latest, status=replace(self.nav.latest.status,
                        state=8 if reason == 'mcu_bound' else 9, flags=self.sample.flags,
                        first_error=0 if reason == 'mcu_bound' else 4))
                self.nav._ekf_turn_cycle()
                self.assertTrue(self.nav.stop_requested)
                self.assertEqual(self.nav.ekf_turn_outcome, 'failed')
                self.assertIsNone(self.nav.detour)
                master.stop.return_value = dict(stop_verified=True)
                self.nav._stop(master)
                self.assertEqual(self.nav.ekf_turn['phase'], 'failed')
                self.nav._avoidance_cycle(master, self.nav.latest)
                self.assertIsNone(self.nav.pending)

    def test_turn_refreshes_new_paired_scan_without_extending_its_deadline(self):
        self.start_turn()
        old_stamp = self.nav.scan_stamp
        self.nav.scan_expires = self.now
        fresh = dict(self.scanner.latest_observation.return_value, stamp_ns=old_stamp+1,
                     expires_mono=self.now+.12)
        self.scanner.latest_observation.return_value = fresh
        self.nav._ekf_turn_cycle()
        self.assertFalse(self.nav.stop_requested)
        self.assertEqual(self.nav.ekf_turn['phase'], 'turning')
        self.assertEqual(self.nav.scan_stamp, fresh['stamp_ns'])
        self.assertEqual(self.nav.scan_expires, fresh['expires_mono'])
        self.assertIsNone(self.nav.last_scan_failure)

    def test_turn_fresh_fast_path_does_not_release_lock_before_its_check(self):
        self.start_turn()
        self.nav.scan_expires = self.now+.01
        real_lock = self.nav.lock
        case = self

        class PauseAfterRelease:
            first = True

            def __enter__(self):
                real_lock.acquire()

            def __exit__(self, *unused):
                real_lock.release()
                if self.first:
                    self.first = False
                    case.now += .24

        self.nav.lock = PauseAfterRelease()
        with redirect_stdout(io.StringIO()):
            self.nav._ekf_turn_cycle()
        self.assertFalse(self.nav.stop_requested)
        self.assertEqual(self.nav.ekf_turn['phase'], 'turning')

    def test_turn_refresh_consumes_real_pending_cache_to_supply_scan_upper_bracket(self):
        self.start_turn()
        scanner = RosObstacleScanner.__new__(RosObstacleScanner)
        scanner.scans = ObstacleScans(OdomHistory(OFFSET))
        self.nav.scanner = scanner
        old_status = replace(self.nav.latest.status, sample_sequence=1,
                             measurement_tick_ms=100, publication_tick_ms=100)
        self.nav._observe_exchange(cache_exchange(old_status, 1, 5.001))
        self.now = 5.35
        self.nav.scan_expires = self.now-.1
        scanner.scans.add(obstacle_scan(5.21, [2.]*9), self.now)
        feedback = FeedbackWorker('/dev/offline-not-opened')
        self.addCleanup(feedback.close)
        feedback.request_stop = Mock()
        self.nav.feedback = feedback
        for number in range(2, 8):
            status = replace(old_status, sample_sequence=number,
                             measurement_tick_ms=50+number*50, publication_tick_ms=50+number*50)
            feedback.cache.publish(cache_exchange(status, number, 5.001+(number-1)*.05))
        with patch('v11_ros.time.monotonic', side_effect=lambda: self.now), redirect_stdout(io.StringIO()):
            self.assertIsNone(scanner.latest_observation())
            self.nav._ekf_turn_cycle()
            self.assertTrue(scanner.diagnostics()['paired'])
        self.assertFalse(self.nav.stop_requested)
        feedback.request_stop.assert_not_called()
        self.assertEqual(self.nav.latest.status.sample_sequence, 7)
        self.assertEqual(self.nav.feedback_cursor, 6)
        self.assertAlmostEqual(self.nav.scan_expires, 5.72)
        self.assertEqual(self.nav.ekf_turn['phase'], 'turning')

    def test_turn_refresh_without_new_or_usable_scan_stops_with_frozen_diagnostic(self):
        for observation in ('missing', 'same', 'expired', 'invalid'):
            with self.subTest(observation=observation), redirect_stdout(io.StringIO()):
                self.start_turn()
                self.nav.scan_expires = self.now
                current = self.scanner.latest_observation.return_value
                if observation == 'missing':
                    self.scanner.latest_observation.return_value = None
                elif observation == 'expired':
                    self.scanner.latest_observation.return_value = dict(current,
                        stamp_ns=current['stamp_ns']+1, expires_mono=self.now)
                elif observation == 'invalid':
                    self.scanner.latest_observation.return_value = dict(current,
                        stamp_ns=current['stamp_ns']+1, usable=False, points=(), clear_rays=())
                self.nav._ekf_turn_cycle()
                self.assertTrue(self.nav.stop_requested)
                self.assertEqual(self.nav.ekf_turn_outcome, 'failed')
                failure = self.nav.last_scan_failure
                self.assertEqual(failure['event'], 'v12_ekf_turn_failed')
                self.assertEqual(failure['stage'], 'turning')
                self.assertEqual(failure['task'], 1)
                self.assertEqual(failure['monotonic_s'], self.now)
                self.scanner.diagnostics.return_value = dict(paired=True, failure_reason=None)
                self.nav._ekf_turn_cycle()
                self.assertIs(self.nav.last_scan_failure, failure)
                self.assertIsNone(self.nav.detour)

    def test_manual_stop_during_scan_refresh_cancels_trial_without_overwriting_reason(self):
        self.start_turn()
        self.nav.scan_expires = self.now
        newer = dict(self.scanner.latest_observation.return_value,
                     stamp_ns=self.nav.scan_stamp+1, expires_mono=self.now+.12)

        def refresh():
            completed = Event()

            def cancel():
                self.nav.request_stop()
                completed.set()

            thread = Thread(target=cancel)
            thread.start()
            self.assertTrue(completed.wait(1.), 'scan refresh held the navigation lock')
            thread.join()
            return newer

        self.scanner.latest_observation.side_effect = refresh
        self.nav._ekf_turn_cycle()
        self.assertTrue(self.nav.stop_requested)
        self.assertEqual(self.nav.ekf_turn_outcome, 'cancelled')
        self.assertIsNone(self.nav.last_scan_failure)

    def test_overlapping_scan_refresh_cannot_replace_newer_committed_frame(self):
        self.start_turn()
        original = self.scanner.latest_observation.return_value
        older = dict(original, stamp_ns=self.nav.scan_stamp+1, expires_mono=self.now+.10)
        newer = dict(original, stamp_ns=self.nav.scan_stamp+2, expires_mono=self.now+.20)
        self.scanner.latest_observation.side_effect = [older, newer]
        real_cells = self.grid.obstacle_cells
        overlapped = False

        def project(points):
            nonlocal overlapped
            if not overlapped:
                overlapped = True
                self.nav.update_scan()
            return real_cells(points)

        with patch.object(self.grid, 'obstacle_cells', side_effect=project):
            self.nav.update_scan()
        self.assertEqual(self.nav.scan_stamp, newer['stamp_ns'])
        self.assertEqual(self.nav.scan_expires, newer['expires_mono'])

    def test_turn_manual_stop_and_normal_commands_are_guarded(self):
        master, _ = self.start_turn()
        for action in (lambda: self.nav.align(dict(x=1., y=1., yaw=0.), 'manual'),
                       lambda: self.nav.preview(dict(x=2., y=1.)),
                       lambda: self.nav.request_start('old'), lambda: self.nav.request_ekf_turn(-1)):
            with self.assertRaises(ValueError):
                action()
        self.nav.request_stop()
        master.stop.return_value = dict(stop_verified=True)
        self.nav._stop(master)
        self.assertEqual(self.nav.ekf_turn['phase'], 'cancelled')
        self.assertIsNone(self.nav.pending)

    def test_turn_stopping_overshoot_or_stale_final_angle_is_not_success(self):
        for final in (94., None):
            with self.subTest(final=final):
                master, _ = self.start_turn()
                self.estimate['yaw_rad'] = math.radians(88.)
                self.nav._ekf_turn_cycle()
                if final is None:
                    self.estimate['fresh'] = False
                else:
                    self.estimate['yaw_rad'] = math.radians(final)
                master.stop.return_value = dict(stop_verified=True)
                self.nav._stop(master)
                self.assertEqual(self.nav.ekf_turn['phase'], 'failed')

    def test_turn_disabled_invalid_direction_and_no_fresh_scan_reject(self):
        with self.assertRaises(ValueError):
            self.nav.request_ekf_turn(1)
        self.prepare_turn()
        for value in (True, 1., 0, 2, '1'):
            with self.assertRaises(ValueError):
                self.nav.request_ekf_turn(value)
        self.nav.ekf_turn.update(phase='idle')
        self.nav.work = self.nav.pending = None
        self.nav.scan_expires = self.now
        with self.assertRaisesRegex(ValueError, '雷达'):
            self.nav.request_ekf_turn(1)

    def test_turn_never_renews_its_single_mcu_lease(self):
        master, _ = self.start_turn()
        reply = replace(self.nav.latest, status=replace(self.nav.latest.status, lease_remaining_ms=1000))
        self.nav._renew_if_due(master, reply)
        master.renew.assert_not_called()

    def test_turn_requires_fresh_ekf_again_before_start(self):
        plan = self.prepare_turn()
        master = Mock()

        def upload(document, on_ack):
            ready = replace(self.nav.latest, status=replace(self.sample, task=1, state=2, count=1))
            on_ack(ready)
            self.estimate['fresh'] = False
            return [ready]

        master.upload.side_effect = upload
        with self.assertRaisesRegex(ValueError, 'EKF'):
            self.nav._upload(master, plan)
        master.start.assert_not_called()

    def test_turn_stop_exception_does_not_leave_stopping_phase(self):
        self.start_turn()
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
        self.assertEqual(self.nav.ekf_turn['phase'], 'failed')
        self.assertFalse(self.nav.stop_requested)


if __name__ == '__main__':
    unittest.main()
