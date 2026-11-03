"""Reviewed real Client with a memory MCU; no ROS, device, or remote access."""
from copy import deepcopy
import math
import struct
import threading
import unittest
from unittest.mock import patch

import base_trial_adapter as trial
from test_base_static_adapter import api, stop_api, core, Clock


class MotionMCU:
    """Acceptance precedes application, as in serial_poll then bench_poll."""
    def __init__(self, clock, *, mode="healthy", delay=0.):
        self.clock, self.mode, self.delay = clock, mode, delay
        self.timeout, self.write_timeout = .01, .1
        self.buffer, self.queued, self.writes = bytearray(), [], []
        self.state, self.session, self.challenge = 1, 0, 12345
        self.last_rx = self.last_applied = self.pack = 0
        self.lease_at, self.ready_at, self.arm_at = None, None, None
        self.pending_apply = None
        self.sample_sequence, self.sample_tick, self.sample_speed, self.sample_status = 500, 100000, 0, 0
        self.stop_count = 0
        self.warm_queries = 0
        self.injected_parking_sample = False
        self.parking_status_tick = None
        self.modeled_stop_collisions = []

    def sample(self, forced=False):
        tick = int(self.clock.value * 1000) if forced else int(self.clock.value / .05) * 50
        if forced or tick > self.sample_tick:
            self.sample_sequence += 1
            self.sample_tick = tick
            self.sample_speed = self.pack
            self.sample_status = 0x70007 if self.state in (5, 10) else 0

    def update(self):
        if self.ready_at is not None and self.clock.value >= self.ready_at:
            self.state, self.last_applied = 10, self.last_rx
            self.lease_at = self.ready_at
            self.ready_at = None
            self.sample(forced=True)
        if self.pending_apply is not None and self.clock.value >= self.pending_apply[0] and self.mode != "never_apply":
            _, self.last_applied, self.pack = self.pending_apply
            self.pending_apply, self.state = None, 5
        self.sample()

    def response(self, kind, seq):
        tick = int(self.clock.value * 1000)
        flags = 1 | (4 if self.state == 1 else 0) | (self.state << 8)
        if kind == 5:
            left, right = struct.unpack('<hh', struct.pack('<I', self.sample_speed))
            speed_pack = ((left * 10) & 0xffff) | (((right * 10) & 0xffff) << 16)
            if self.state == 5 and self.mode == 'actual_reverse':
                speed_pack = ((-10) & 0xffff) | ((-10 & 0xffff) << 16)
            if self.state == 5 and self.mode == 'actual_overspeed':
                speed_pack = 40 | ((-40 & 0xffff) << 16)
            words = [tick, self.sample_sequence, self.sample_tick, 31, 0, 0,
                     speed_pack, self.sample_tick, self.sample_tick, self.sample_tick,
                     self.sample_status, 0, flags]
            if self.mode == "feedback_fault" and self.state in (5, 10) and self.arm_at is not None and self.clock.value - self.arm_at > .35:
                words[11] = 1
        else:
            age = int((self.clock.value - self.lease_at) * 1000) if self.lease_at is not None else 0xffffffff
            pack = self.pack
            if self.mode == "bad_applied_target" and kind == 7:
                pack = 0x00010001
            words = [tick, self.state, 0, self.session, self.last_rx, self.last_applied,
                     pack, int(self.state == 1), flags, api.IDENTITY, age, self.challenge, 0]
        if kind == 2 and self.stop_count == 2 and self.state == 1:
            self.parking_status_tick = tick
        if (kind == 5 and self.stop_count == 2 and not self.injected_parking_sample
                and self.mode in ('parking_old_nonzero', 'parking_new_nonzero')):
            self.injected_parking_sample = True
            boundary = self.parking_status_tick
            # Exact four relative source offsets from the r6 STATUS2561 /
            # FB2562 sample7704, keeping this fixture's actual MCU clock.
            deltas = (-14, -38, -34, -22) if self.mode == 'parking_old_nonzero' else (1, 1, 1, 1)
            for index, delta in zip((2, 7, 8, 9), deltas):
                words[index] = (boundary + delta) & 0xffffffff
            words[6] = 65531  # Actual r6 signed left -5 raw = -0.5RPM, right 0.
        body = struct.pack('<BBBBHBB13IH', 0xaa, 0x55, 9, kind | 0x80, seq, 0, 0, *words, 0)
        return body + struct.pack('<H', api.crc16(body))

    def write(self, raw):
        self.update()
        raw = bytes(raw)
        self.writes.append((self.clock.value, raw))
        kind, seq = raw[3], struct.unpack_from('<H', raw, 4)[0]
        if kind in (2, 5) and self.stop_count == 1 and getattr(self, 'host_phase', None) == 'waiting_ready':
            self.warm_queries += 1
        left, right, session, challenge = struct.unpack_from('<hhII', raw, 6)
        if kind == 3:
            self.stop_count += 1
            if self.stop_count == 2 and self.mode in ('half_duplex_stop_collision','half_duplex_recovery_also_lost'):
                old_queries=[struct.unpack_from('<H',reply,4)[0] for due,reply in self.queued
                             if due>self.clock.value and reply[3] in (0x82,0x85)]
                if old_queries:self.modeled_stop_collisions.append(dict(stop_sequence=seq,old_query_sequences=old_queries))
            self.state, self.session, self.pack, self.pending_apply, self.ready_at = 1, 0, 0, None, None
            self.lease_at = None
            self.challenge += 17
            self.sample(forced=True)
        elif kind == 6:
            if challenge != self.challenge or self.state != 1 or not session:
                raise AssertionError('Host guessed ARM challenge/state')
            self.state, self.session, self.last_rx = 2, session, seq
            self.last_applied = 0
            self.challenge += 1
            self.ready_at = self.clock.value + (.8 if self.mode == 'slow_arm' else .390 if self.mode == 'r1_tf_lag' else .015)
            self.arm_at = self.clock.value
        elif kind == 7:
            if session != self.session or self.state not in (5, 10):
                raise AssertionError('Host SETPOINT before READY/session')
            self.last_rx, self.lease_at = seq, self.clock.value
            self.pending_apply = (self.clock.value + .004, seq, (left & 0xffff) | ((right & 0xffff) << 16))
        reply = self.response(kind, seq)
        if self.mode == 'prearm_query_timeout' and kind in (2, 5) and self.stop_count == 1 and getattr(self, 'host_phase', None) == 'waiting_ready':
            return 20
        # Software half-duplex model only, not a claim about the real RK bus.
        # A locally cancelled query is still in flight; the first immediate
        # STOP loses its reply. After its original 300ms timeout, a new frame
        # can receive a new independent reply. The second mode also loses that
        # one recovery reply, verifying that the owner never loops indefinitely.
        if kind==3 and self.stop_count>=2 and self.mode in ('half_duplex_stop_collision','half_duplex_recovery_also_lost'):
            if self.stop_count==2 or self.mode=='half_duplex_recovery_also_lost':
                return 20
        if self.mode == "short_setpoint" and kind == 7:
            return 10
        if self.mode == "corrupt_setpoint_reply" and kind == 7:
            reply = reply[:-1] + bytes([reply[-1] ^ 1])
        self.queued.append((self.clock.value + self.delay, reply))
        return 20

    def read(self, count):
        self.update()
        ready = [item for item in self.queued if item[0] <= self.clock.value]
        self.queued = [item for item in self.queued if item[0] > self.clock.value]
        for _, raw in ready:
            self.buffer.extend(raw)
        raw = bytes(self.buffer[:count])
        del self.buffer[:count]
        return raw


class TrialTests(unittest.TestCase):
    def run_trial(self, mode="healthy", *, duration=.7, command_mode="healthy", delay=0., cancel_at=None,
                  health_failure_at=None, provider=None, sink=None, post=None, cancel_on_inflight_query=False,
                  cancel_before_arm_query=False):
        clock = Clock()
        port = MotionMCU(clock, mode=mode, delay=delay)
        phases = []
        armed = [None]
        odom_history=[]
        last_tf=[None]
        def measured_sink(odom,packet,start,end):
            odom_history.append((end,odom['source_monotonic_s']))
            if sink is not None:sink(odom,packet,start,end)
        def phase(name, at):
            phases.append((name, at))
            port.host_phase = name
            if name == 'armed':
                armed[0] = at
        def supervisor(now, stage):
            failure = 'current scan expired' if (health_failure_at is not None and armed[0] is not None
                and now - armed[0] >= health_failure_at) else None
            for completed,source in odom_history:
                if completed+(.12 if mode=='r1_tf_lag' else 0.)<=now:last_tf[0]=source
            if mode=='never_new_tf' and stage=='waiting_motion_sources':
                tf_source=0.
            else:tf_source=last_tf[0]
            ready=tf_source is not None and now-tf_source<=.5
            if stage=='waiting_ready':ready=not cancel_before_arm_query
            return dict(ready=ready, complete=armed[0] is not None and now - armed[0] >= duration, failure=failure,
                        readiness_reason='awaiting_post_arm_tf' if not ready else None,
                        tf_source_monotonic_s=tf_source,
                        odom_source_monotonic_s=odom_history[-1][1] if odom_history else None)
        def command(now):
            if provider is not None:
                return provider(now, armed[0])
            age = now - armed[0]
            seq = int(age / .1) + 1
            stamp = armed[0] + (seq - 1) * .1
            if command_mode == 'duplicate':seq, stamp = 1, armed[0]
            if command_mode == 'prearm':stamp = armed[0] - .01
            return dict(task_id='trial-1', sequence=seq, received_monotonic_s=stamp,
                        linear_mps=-.01 if command_mode == 'reverse' else .03, angular_rad_s=0.)
        def cancelled():
            return bool(cancel_at is not None and armed[0] is not None and clock.value-armed[0]>=cancel_at
                or cancel_on_inflight_query and armed[0] is not None and port.stop_count==1
                and any(due>clock.value and reply[3] in (0x82,0x85) for due,reply in port.queued)
                or cancel_before_arm_query and port.host_phase == 'waiting_ready' and port.warm_queries > 0)
        def observer(now, proof):
            return post(now, proof, port) if post is not None else None
        with patch.object(api.time, 'monotonic', clock.monotonic), patch.object(api.time, 'sleep', clock.sleep):
            result = trial.run_trial_session(port, api, stop_api, core.MeasuredOdometry,
                task_id='trial-1', supervisor=supervisor, command_provider=command,
                on_phase=phase, cancelled=cancelled, odom_sink=measured_sink,
                post_stop_observer=observer if post is not None else None,
                duration_s=2., clock=clock.monotonic, sleep=clock.sleep)
        return result, port, phases

    def test_real_client_one_arm_accepted_then_applied_and_both_stop_chains(self):
        result, port, phases = self.run_trial()
        self.assertTrue(result['trial_complete'], result['first_failure'])
        self.assertEqual(result['counts']['6'], 1)
        self.assertEqual(result['counts']['3'], 2)
        self.assertGreater(result['metrics']['nonzero_setpoints'], 0)
        self.assertTrue(result['initial_stop_chain']['complete'])
        self.assertTrue(result['final_stop_chain']['complete'])
        self.assertEqual(len(result['final_stop_chain']['zero_samples']), 5)
        self.assertTrue(any(row.get('accepted_not_yet_applied') for row in result['rows'] if row['kind'] == 7))
        self.assertTrue(any(row.get('applied_command', {}).get('origin') == 'selected_command' for row in result['rows']))
        first_arm = next(at for at, raw in port.writes if raw[3] == 6)
        self.assertLessEqual(dict(phases)['arming'], first_arm)
        for _, raw in port.writes:
            self.assertEqual(len(raw), 20)
            self.assertEqual(api.crc16(raw[:18]), struct.unpack_from('<H', raw, 18)[0])
            if raw[3] == 7:
                self.assertEqual(struct.unpack_from('<hh', raw, 6), (2, -2))

    def test_every_original_tx_rx_and_quantization_are_retained(self):
        result, port, _ = self.run_trial()
        tx = [row for row in result['wire_events'] if row['direction'] == 'tx']
        rx = [row for row in result['wire_events'] if row['direction'] == 'rx']
        self.assertEqual([row['raw_hex'] for row in tx], [raw.hex() for _, raw in port.writes])
        self.assertEqual(len(rx), len(tx))
        for row in rx:
            self.assertEqual(len(bytes.fromhex(row['raw_hex'])), 64)
            api.decode(bytes.fromhex(row['raw_hex']))
        command = next(row['command'] for row in tx if row['kind'] == 7)
        self.assertEqual(command['original_command']['linear_mps'], .03)
        self.assertLess(command['quantization']['estimated']['linear_mps'], .03)
        self.assertEqual(result['dropped_wire_events'], 0)

    def test_duplicate_command_does_not_refresh_250ms_source(self):
        result, port, _ = self.run_trial(command_mode='duplicate')
        self.assertFalse(result['trial_complete'])
        self.assertIn('stale', result['first_failure']['message'])
        self.assertTrue(result['final_stop_chain']['complete'])
        self.assertEqual(result['counts']['6'], 1)
        self.assertLess(result['first_failure']['monotonic_s'] - result['armed_at_monotonic_s'], .252)

    def test_prearm_or_reverse_commands_never_become_motor_targets(self):
        for command_mode in ('prearm', 'reverse'):
            with self.subTest(command_mode=command_mode):
                result, _, _ = self.run_trial(command_mode=command_mode)
                self.assertFalse(result['trial_complete'])
                self.assertEqual(result['counts']['7'], 0)
                self.assertTrue(result['final_stop_chain']['complete'])

    def test_fault_and_cancel_stop_before_any_more_target_or_query(self):
        for kind, kwargs in (('health', {'health_failure_at': .123}), ('cancel', {'cancel_at': .123})):
            with self.subTest(kind=kind):
                result, port, _ = self.run_trial(delay=.007, **kwargs)
                failed_at = result['first_failure']['monotonic_s']
                following = [(at, raw[3]) for at, raw in port.writes if at >= failed_at]
                self.assertEqual(following[0][1], 3)
                self.assertLessEqual(following[0][0] - failed_at, .000001)
                self.assertNotIn('normal_quiesce', result)
                self.assertTrue(result['final_stop_chain']['complete'])

    def test_driver_fault_and_wrong_applied_target_are_not_accepted(self):
        for mode, message in (('feedback_fault', 'fault'), ('bad_applied_target', 'Applied sequence/target')):
            with self.subTest(mode=mode):
                result, _, _ = self.run_trial(mode)
                self.assertFalse(result['trial_complete'])
                self.assertIn(message.lower(), result['first_failure']['message'].lower())
                self.assertEqual(result['counts']['6'], 1)
                self.assertTrue(result['final_stop_chain']['complete'])

    def test_actual_reverse_or_estimated_overspeed_stops_even_with_legal_targets(self):
        for mode, text in (('actual_reverse','reverses'),('actual_overspeed','estimated chassis speed')):
            with self.subTest(mode=mode):
                result, _, _ = self.run_trial(mode)
                self.assertFalse(result['trial_complete'])
                self.assertIn(text,result['first_failure']['message'])
                self.assertTrue(result['final_stop_chain']['complete'])

    def test_acceptance_without_application_cannot_sustain_motion(self):
        result, _, _ = self.run_trial('never_apply')
        self.assertFalse(result['trial_complete'])
        self.assertIn('did not become applied', result['first_failure']['message'])
        self.assertTrue(result['final_stop_chain']['complete'])

    def test_750ms_old_feedback_deadline_remains_active_during_slow_arm(self):
        result, port, _ = self.run_trial('slow_arm')
        self.assertFalse(result['trial_complete'])
        self.assertEqual(result['first_failure']['phase'], 'arming')
        self.assertIn('Host feedback source age', result['first_failure']['message'])
        self.assertEqual(result['counts']['6'], 1)
        self.assertEqual(result['counts']['7'], 0)
        arm_at = next(at for at, raw in port.writes if raw[3] == 6)
        self.assertLess(result['first_failure']['monotonic_s'] - arm_at, .8)
        self.assertTrue(result['final_stop_chain']['complete'])

    def test_real_ready_feedback_then_true_tf_handoff_without_relaxing_500ms_gate(self):
        result,port,phases=self.run_trial('r1_tf_lag',delay=.009)
        self.assertTrue(result['trial_complete'],result['first_failure'])
        handoff=result['ready_source_handoff']
        self.assertTrue(handoff['complete'])
        first=handoff['first_feedback']['source_monotonic_s']
        self.assertGreaterEqual(handoff['tf_source_monotonic_s']+1e-6,first)
        start=dict(phases)['waiting_motion_sources'];end=dict(phases)['armed']
        self.assertTrue(all(raw[3] in (2,5) for at,raw in port.writes if start<=at<end))
        first_target=next(at for at,raw in port.writes if raw[3]==7)
        self.assertLess(first_target,handoff['ready_lease_deadline_monotonic_s'])
        self.assertLess(end-start,.2)

    def test_missing_new_tf_never_grants_setpoint_and_stops_inside_lease(self):
        result,port,_=self.run_trial('never_new_tf')
        self.assertFalse(result['trial_complete'])
        self.assertEqual(result['first_failure']['phase'],'waiting_motion_sources')
        self.assertEqual(result['counts']['7'],0)
        self.assertTrue(result['final_stop_chain']['complete'])
        self.assertLess(result['first_failure']['monotonic_s'],result['ready_source_handoff']['ready_lease_deadline_monotonic_s'])

    def test_first_new_DWA_command_must_arrive_before_250ms_no_bridge_zero(self):
        result, _, _ = self.run_trial(provider=lambda _now, _armed: None)
        self.assertFalse(result['trial_complete'])
        self.assertIn('No new selected-source velocity', result['first_failure']['message'])
        self.assertEqual(result['counts']['7'], 0)
        self.assertTrue(result['final_stop_chain']['complete'])
        self.assertLess(result['first_failure']['monotonic_s'] - result['armed_at_monotonic_s'], .252)

    def test_short_write_no_retry_and_raw_rejected_frame_remains(self):
        result, port, _ = self.run_trial('short_setpoint')
        self.assertFalse(result['trial_complete'])
        self.assertEqual(sum(raw[3] == 7 for _, raw in port.writes), 1)
        event = next(row for row in result['wire_events'] if row.get('kind') == 7)
        self.assertEqual(event['bytes_accepted'], 10)
        self.assertTrue(result['final_stop_chain']['complete'])

    def test_cancelled_inflight_query_first_stop_timeout_then_one_fresh_stop_confirmation(self):
        result,port,_=self.run_trial('half_duplex_stop_collision',delay=.009,cancel_on_inflight_query=True)
        self.assertFalse(result['trial_complete'])
        self.assertIn('cancellation',result['first_failure']['message'])
        self.assertIsNone(result['final_stop_chain'])
        self.assertEqual(result['final_stop_error']['error_type'],'TimeoutError')
        self.assertTrue(result['final_stop_recovery_attempted'])
        self.assertTrue(result['final_stop_recovery_chain']['complete'])
        self.assertEqual(result['counts']['3'],3)  # Initial, failed final, one new recovery.
        self.assertEqual(len(port.modeled_stop_collisions),1)
        failed_seq=port.modeled_stop_collisions[0]['stop_sequence']
        recovery_seq=result['final_stop_recovery_chain']['stop_ack']['packet']['sequence']
        self.assertNotEqual(recovery_seq,failed_seq)
        stops=[(at,raw) for at,raw in port.writes if raw[3]==3]
        self.assertLess(stops[1][0]-result['first_failure']['monotonic_s'],.000001)
        self.assertTrue(all(raw[3] in (2,3,5) for at,raw in port.writes if at>=stops[1][0]))
        self.assertEqual(len(result['final_stop_recovery_chain']['zero_samples']),5)

    def test_second_stop_failure_keeps_both_errors_without_a_third_recovery_attempt(self):
        result,port,_=self.run_trial('half_duplex_recovery_also_lost',delay=.009,cancel_on_inflight_query=True)
        self.assertFalse(result['trial_complete'])
        self.assertEqual(result['final_stop_error']['error_type'],'TimeoutError')
        self.assertEqual(result['final_stop_recovery_error']['error_type'],'TimeoutError')
        self.assertEqual(result['counts']['3'],3)
        self.assertNotIn('final_stop_recovery_chain',result)
        self.assertIn('cancellation',result['first_failure']['message'])

    def test_never_armed_cancel_drains_only_original_full_query_then_new_stop(self):
        result,port,_=self.run_trial(delay=.009,cancel_before_arm_query=True)
        self.assertFalse(result['trial_complete'])
        self.assertEqual(result['first_failure']['phase'],'waiting_ready')
        drain=result['pre_arm_stop_quiesce'];ticket=drain['pending']
        self.assertTrue(drain['complete'])
        self.assertTrue(drain['motion_authority_revoked'])
        self.assertFalse(drain['odometry_callback_delivered'])
        self.assertEqual(drain['queries_started'],0)
        self.assertLess(drain['started_monotonic_s']+drain['duration_s'],ticket['deadline_monotonic_s'])
        after=[(at,raw) for at,raw in port.writes if at>=result['first_failure']['monotonic_s']]
        self.assertEqual(after[0][1][3],3)
        old_response=next(row for row in result['rows'] if row['packet']['sequence']==ticket['sequence'])
        self.assertLessEqual(old_response['completed_monotonic_s'],after[0][0])
        self.assertEqual(result['counts']['6'],0)
        self.assertEqual(result['counts']['7'],0)
        self.assertEqual(result['counts']['3'],2)
        self.assertTrue(result['final_stop_chain']['complete'])

    def test_never_armed_missing_reply_keeps_original_deadline_and_first_failure(self):
        result,port,_=self.run_trial('prearm_query_timeout',cancel_before_arm_query=True)
        self.assertFalse(result['trial_complete'])
        self.assertIn('cancellation',result['first_failure']['message'])
        drain=result['pre_arm_stop_quiesce'];ticket=drain['pending']
        self.assertFalse(drain['complete'])
        self.assertEqual(drain['error']['error_type'],'TimeoutError')
        self.assertEqual(drain['error']['client_failure']['id'],ticket['id'])
        stop_at=next(at for at,raw in port.writes if raw[3]==3 and at>=result['first_failure']['monotonic_s'])
        self.assertGreaterEqual(stop_at,ticket['deadline_monotonic_s'])
        self.assertLess(stop_at-ticket['deadline_monotonic_s'],.00101)
        self.assertEqual(result['counts']['6'],0)
        self.assertEqual(result['counts']['7'],0)
        self.assertEqual(result['counts']['3'],2)
        self.assertTrue(result['final_stop_chain']['complete'])

    def test_r6_old_nonzero_can_sample_is_retained_but_not_post_status_motion(self):
        result,port,_=self.run_trial('parking_old_nonzero',delay=.009)
        self.assertTrue(result['trial_complete'],result['first_failure'])
        proof=result['final_stop_chain']
        boundary=proof['stopped_status']['packet']['tick_ms']
        rejected=[row for row in result['rows'] if row.get('parking_source_rejection')
                  and row['packet']['speed_left_rpm']==-.5]
        self.assertEqual(len(rejected),1)
        row=rejected[0];packet=row['packet']
        self.assertGreater(packet['tick_ms'],boundary)
        self.assertEqual([packet[key]-boundary for key in
            ('sample_tick_ms','left_tick_ms','right_tick_ms','speed_tick_ms')],[-14,-38,-34,-22])
        self.assertEqual(packet['controller_state'],1)
        self.assertTrue(row['excluded_from_feedback_tracker'])
        self.assertTrue(row['excluded_from_odometry'])
        self.assertNotIn('source_monotonic_s',row)
        self.assertNotIn('wheel_odometry',row)
        self.assertEqual(api.decode(bytes.fromhex(row['reply_hex']))['speed_left_rpm'],-.5)
        self.assertEqual(len(proof['zero_samples']),5)
        self.assertEqual(result['counts']['3'],2)
        self.assertNotIn('final_stop_recovery_attempted',result)

    def test_new_post_status_nonzero_sample_still_fails_without_a_zero_grace_period(self):
        result,port,_=self.run_trial('parking_new_nonzero',delay=.009)
        self.assertFalse(result['trial_complete'])
        self.assertIsNone(result['final_stop_chain'])
        self.assertIn('not zero and DISARMED',result['final_stop_error']['message'])
        row=next(row for row in result['rows'] if row['phase']=='final_stop'
                 and row['kind']==5 and row['packet']['speed_left_rpm']==-.5)
        self.assertTrue(row['parking_source_after_status'])
        self.assertEqual(set(row['parking_source_deltas_ms'].values()),{1})
        self.assertNotIn('parking_source_rejection',row)
        recovery=result['final_stop_recovery_chain']
        self.assertTrue(recovery['complete'])
        self.assertLess(recovery['stop_ack']['request_start_monotonic_s']-row['completed_monotonic_s'],.000001)
        self.assertEqual(result['counts']['3'],3)

    def test_corrupt_rx_is_kept_and_causes_original_300ms_failure(self):
        result, _, _ = self.run_trial('corrupt_setpoint_reply')
        self.assertFalse(result['trial_complete'])
        self.assertTrue(any(row.get('raw_hex') and bytes.fromhex(row['raw_hex'])[3] == 0x87
            for row in result['wire_events'] if row['direction'] == 'rx'))
        self.assertTrue(result['final_stop_chain']['complete'])

    def test_normal_goal_finish_only_waits_existing_original_ticket(self):
        result, _, _ = self.run_trial(duration=.117, delay=.009)
        self.assertTrue(result['trial_complete'], result['first_failure'])
        quiesce = result['normal_quiesce']
        self.assertEqual(quiesce['queries_started'], 0)
        self.assertTrue(quiesce['complete'])
        pending = quiesce['pending_at_completion']
        if pending:
            self.assertLess(quiesce['started_monotonic_s'] + quiesce['duration_s'], pending['deadline_monotonic_s'])

    def test_odometry_comes_from_measured_rpm_and_preserves_original_source(self):
        seen = []
        result, _, _ = self.run_trial(sink=lambda odom, packet, start, end: seen.append((odom, packet, start, end)))
        moving = [r for r in seen if r[1]['speed_left_rpm'] != 0]
        self.assertTrue(result['trial_complete'])
        self.assertTrue(moving)
        for odom, packet, start, end in moving:
            self.assertEqual(packet['speed_left_rpm'], 2.)
            self.assertEqual(packet['speed_right_rpm'], -2.)
            self.assertLessEqual(odom['source_monotonic_s'], start)
            self.assertGreater(odom['linear_x'], 0.)
            self.assertLess(odom['linear_x'], .03)
        self.assertGreater(max(r[0]['x'] for r in seen), 0.)

    def test_final_stop_is_independent_of_failed_ros_sink(self):
        def sink(*args):
            if args[1]['controller_state'] == 1 and seen_motion[0]:
                raise RuntimeError('publisher failed')
            if args[1]['controller_state'] in (5, 10):
                seen_motion[0] = True
        seen_motion = [False]
        result, _, _ = self.run_trial(sink=sink)
        self.assertTrue(result['final_stop_chain']['complete'])
        self.assertTrue(result['final_stop_callback_errors'])
        self.assertFalse(result['trial_complete'])

    def test_post_stop_tail_produces_new_true_odom_and_cannot_send_motion(self):
        owner_samples=[]
        def sink(odom,packet,start,end):owner_samples.append((odom,end))
        def observer(now,proof,port):
            boundary=proof['stopped_status']['completed_monotonic_s']
            self.assertEqual(port.state,1)
            self.assertEqual(len(proof['zero_samples']),5)
            return dict(complete=any(odom['source_monotonic_s']>boundary+.55 for odom,_ in owner_samples),failure=None)
        result,port,phases=self.run_trial(sink=sink,post=observer)
        self.assertTrue(result['trial_complete'],result['first_failure'])
        self.assertTrue(result['post_stop_observation']['complete'])
        boundary=dict(phases)['post_stop_observation']
        self.assertTrue(all(raw[3] in (2,5) for at,raw in port.writes if at>=boundary))
        self.assertGreater(result['post_stop_observation']['queries_started'],0)
        self.assertEqual(result['counts']['6'],1)

    def test_new_post_stop_fault_keeps_original_proof_and_sends_new_stop(self):
        result,port,_=self.run_trial(post=lambda now,proof,port:dict(complete=False,failure='new final AMCL rejected'))
        self.assertFalse(result['trial_complete'])
        self.assertTrue(result['final_stop_chain']['complete'])
        self.assertTrue(result['post_stop_fault_stop_chain']['complete'])
        self.assertEqual(result['counts']['3'],3)
        self.assertEqual(result['first_failure']['phase'],'post_stop_observation')

    def test_post_stop_success_drains_only_the_original_inflight_query(self):
        first=[None]
        def observer(now,proof,port):
            if first[0] is None:first[0]=now
            return dict(complete=now-first[0]>.004 and bool(port.queued),failure=None)
        result,port,phases=self.run_trial(delay=.009,post=observer)
        self.assertTrue(result['trial_complete'],result['first_failure'])
        observation=result['post_stop_observation'];quiesce=observation['normal_quiesce']
        self.assertIsNotNone(quiesce['pending'])
        self.assertEqual(quiesce['queries_started'],0)
        self.assertGreater(quiesce['duration_s'],0.)
        tail_at=dict(phases)['post_stop_observation']
        self.assertEqual(len([raw for at,raw in port.writes if at>=tail_at]),1)
        self.assertEqual(sum(result['complete_write_counts'].values()),result['total_rows'])

    def test_post_stop_timeout_is_bounded_and_never_rearms(self):
        result,port,_=self.run_trial(post=lambda now,proof,port:dict(complete=False,failure=None))
        self.assertFalse(result['trial_complete'])
        self.assertIn('timed out',result['first_failure']['message'])
        self.assertEqual(result['counts']['6'],1)
        self.assertTrue(result['post_stop_fault_stop_chain']['complete'])
        observation=result['post_stop_observation']
        # Observation itself is two seconds; the separate fresh parking proof
        # afterwards is recorded as cleanup, not silently added to that budget.
        self.assertLess(observation['ended_monotonic_s']-observation['started_monotonic_s'],2.4)


class PureBoundaryTests(unittest.TestCase):
    def test_physical_write_rechecks_original_command_and_feedback_deadlines(self):
        for at,command_stamp,feedback_stamp,lease_deadline in ((100.25,100.,100.,101.),(100.75,100.60,100.,101.),(100.,100.01,100.,101.),(100.1,100.,100.,100.1)):
            with self.subTest(at=at):
                clock=Clock();port=MotionMCU(clock);wire=trial.TrialWirePort(port,clock=clock.monotonic)
                wire.mode,wire.session='armed',9
                note=dict(original_command=dict(received_monotonic_s=command_stamp),feedback_source_monotonic_s=feedback_stamp,
                          ready_lease_deadline_monotonic_s=lease_deadline)
                wire.permit_target(9,2,-2,note)
                clock.value=at  # Advance after permit/deepcopy but before real IO.
                with self.assertRaisesRegex(trial.TrialFault,'Physical SETPOINT boundary|First SETPOINT'):
                    wire.write(api.request(7,1,2,-2,session=9))
                self.assertEqual(port.writes,[])
                self.assertEqual(wire.attempts['7'],0)
                self.assertFalse(wire.events[-1]['write_attempted'])
    def test_feedback_state_metadata_can_change_without_source_refresh(self):
        packet = dict(tick_ms=1000, sample_sequence=1, sample_tick_ms=990,
            left_tick_ms=990, right_tick_ms=990, speed_tick_ms=990,
            position_left_raw=0, position_right_raw=0, speed_raw=0,
            status_raw=0x70007, fault_raw=0, flags=2561, valid_mask=31)
        source = trial.MotionSourceTracker(.75)
        first = source.accept(packet, 10., 10.01)
        newer = dict(packet, tick_ms=1050, flags=1281)
        duplicate = source.accept(newer, 10.05, 10.06)
        self.assertFalse(duplicate['distinct'])
        self.assertEqual(first['source_monotonic_s'], duplicate['source_monotonic_s'])
        with self.assertRaisesRegex(Exception, 'changed its content'):
            source.accept(dict(newer, flags=1283), 10.06, 10.07)
        with self.assertRaisesRegex(Exception, 'timeout'):
            source.accept(dict(newer, sample_sequence=2), 10.74, 10.75)

    def test_command_identity_finites_duplicates_and_future(self):
        tracker = trial.CommandTracker('t', 10., .25)
        command = dict(task_id='t', sequence=1, received_monotonic_s=10.01, linear_mps=.02, angular_rad_s=0.)
        tracker.accept(command, 10.02)
        for bad in (dict(command, task_id='other'), dict(command, sequence=True),
                    dict(command, received_monotonic_s=10.03), dict(command, linear_mps=float('nan'))):
            with self.assertRaises(Exception):tracker.accept(bad, 10.02)
        with self.assertRaisesRegex(Exception, 'changed content'):
            tracker.accept(dict(command, received_monotonic_s=10.02), 10.03)

    def test_quantized_limits_and_forward_only_rejection(self):
        output = trial.quantize_forward(dict(linear_mps=.03, angular_rad_s=0.))
        self.assertEqual(output['wire_target'], dict(left_rpm=2, right_rpm=-2))
        for v,w in ((0., .05), (-.01,0.), (.001,.1)):
            with self.assertRaises(trial.TrialFault):trial.quantize_forward(dict(linear_mps=v, angular_rad_s=w))
        for v,w in ((.03,.12),(.02,.01),(.5,2.),(0.,0.)):
            out=trial.quantize_forward(dict(linear_mps=v,angular_rad_s=w))
            self.assertLessEqual(abs(out['estimated']['linear_mps']), .03)
            self.assertLessEqual(abs(out['estimated']['angular_deg_s']),6.9)
            self.assertGreaterEqual(out['physical_wheels']['left_rpm'],0)
            self.assertGreaterEqual(out['physical_wheels']['right_rpm'],0)

    def test_wire_requires_single_use_permit_and_never_blocks_stop_after_reject(self):
        clock=Clock();port=MotionMCU(clock);wire=trial.TrialWirePort(port,clock=clock.monotonic)
        for raw in (api.request(7,1,2,-2,session=9),api.request(6,1,session=9,challenge=123),
                    api.request(8,1),bytes(20)):
            with self.assertRaises(trial.TrialFault):wire.write(raw)
        self.assertEqual(port.writes,[])
        # The Client may consume a local sequence while a frame is rejected.
        wire.write(api.request(3,2))
        self.assertEqual(port.writes[-1][1][3],3)
        errors=[]
        thread=threading.Thread(target=lambda: self.capture_thread_error(wire,errors))
        thread.start();thread.join()
        self.assertEqual(len(errors),1)
        self.assertIn('PID and thread',str(errors[0]))

    @staticmethod
    def capture_thread_error(wire,errors):
        try:wire.write(api.request(3,3))
        except Exception as exc:errors.append(exc)


if __name__ == '__main__':
    unittest.main()
