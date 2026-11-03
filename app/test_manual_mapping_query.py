"""Real reviewed Client/core, synthetic hand-pushed feedback; no physical IO."""
import importlib.util
import math
from pathlib import Path
import struct
import threading
import unittest
from unittest.mock import patch

import manual_mapping_query as manual

ROOT = Path(__file__).resolve().parent
PROJECT = ROOT.parent.parent
if (ROOT / 'deps').exists():
    paths = {name: ROOT / 'deps' / (name + '.py') for name in ('control_client', 'core')}
    if not all(path.is_file() for path in paths.values()):
        raise RuntimeError('Complete Client/core dependency pair required, no partial fallback')
else:
    paths = dict(control_client=PROJECT / 'candidates/v9_nonblocking_20261102/transport/src/control_client.py',
                 core=PROJECT / 'navigation-demo/green_base_bridge/src/green_base_bridge/core.py')


def load(name):
    spec = importlib.util.spec_from_file_location('manual_test_' + name, str(paths[name]))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


api, core = load('control_client'), load('core')


class Clock:
    def __init__(self): self.value = 100.
    def monotonic(self): return self.value
    def sleep(self, seconds): self.value += seconds


class HandPushedMCU:
    def __init__(self, clock, mode='healthy', delay=0.):
        self.clock, self.mode, self.delay = clock, mode, delay
        self.timeout, self.write_timeout = .1, .1
        self.writes, self.queue, self.buffer = [], [], bytearray()

    def write(self, raw):
        raw = bytes(raw)
        self.writes.append((self.clock.value, raw))
        kind, seq = raw[3], struct.unpack_from('<H', raw, 4)[0]
        if kind not in (2, 5): raise AssertionError('Physical control frame reached fake MCU')
        tick = 100000 if self.mode == 'frozen' else int(self.clock.value * 1000)
        sample_tick = tick if self.mode == 'frozen' else int(tick / 100) * 100
        if self.mode == 'gap_once' and sample_tick == 100200:
            sample_tick = 100100  # One missing 100ms sample, next real step is 2 / dt .2.
        sample_seq = sample_tick // 100
        if kind == 2:
            words = [tick, 5 if self.mode == 'enabled' else 1, 0, 0, 0, 0, 0,
                     0, 257, api.IDENTITY, 0xffffffff, 4321, 0]
        else:
            old = 251 if self.mode == 'old_can' else 0
            # Signed raw tenths: left +0.1 / right -0.1, expected hand motion.
            speed = 0xffff0001 if self.mode != 'fast' else 0xff690097
            words = [tick, sample_seq, sample_tick-old, 31, sample_seq, (-sample_seq)&0xffffffff,
                     speed, sample_tick-old, sample_tick-old, sample_tick-old, 0, 0, 261]
        body = struct.pack('<BBBBHBB13IH', 0xaa, 0x55, 9, kind|0x80, seq, 0, 0, *words, 0)
        reply = body + struct.pack('<H', api.crc16(body))
        if self.mode == 'bad_crc' and kind == 5:
            reply = reply[:-1] + bytes([reply[-1]^1])
        if self.mode == 'short' and kind == 5: return 10
        self.queue.append((self.clock.value+self.delay, reply))
        return 20

    def read(self, count):
        ready = [entry for entry in self.queue if entry[0] <= self.clock.value]
        self.queue = [entry for entry in self.queue if entry[0] > self.clock.value]
        for _, raw in ready: self.buffer.extend(raw)
        raw = bytes(self.buffer[:count]); del self.buffer[:count]
        return raw


class ManualQueryTests(unittest.TestCase):
    def session(self, mode='healthy', duration=.45, delay=0., stop_pending=False, evidence_limit=4096,
                odom_class=None):
        clock = Clock(); port = HandPushedMCU(clock, mode, delay); seen = []
        def stopping(): return stop_pending and len(port.writes)>2 and bool(port.queue)
        with patch.object(api.time, 'monotonic', clock.monotonic), patch.object(api.time, 'sleep', clock.sleep):
            result = manual.run_manual_mapping_session(port, api,
                lambda: manual.make_raw_rpm_odometry(odom_class or core.MeasuredOdometry), boot_id='new-boot',
                duration_s=duration, stop_recording=stopping, odom_sink=lambda *args: seen.append(args),
                clock=clock.monotonic, sleep=clock.sleep, wall_clock=lambda:1000.+clock.value,
                evidence_limit=evidence_limit)
        return result, port, seen

    def test_real_client_only_queries_and_first_true_origin_retains_tiny_rpm(self):
        result,port,seen = self.session()
        self.assertTrue(result['recording_complete'],result['first_failure'])
        self.assertEqual({raw[3] for _,raw in port.writes},{2,5})
        self.assertEqual(result['query_counts']['3'],0)
        self.assertEqual(result['origin']['pose'],dict(x=0.,y=0.,yaw=0.))
        self.assertEqual(result['origin']['raw_packet']['speed_left_rpm'],.1)
        self.assertEqual(seen[0][0]['x'],0.)
        self.assertGreater(seen[-1][0]['x'],0.)
        self.assertAlmostEqual(seen[-1][0]['linear_x'],.1*math.pi*.2/60.)
        self.assertGreater(seen[-1][1]['position_left_raw'],seen[0][1]['position_left_raw'])
        self.assertLessEqual(result['origin']['source_monotonic_s'],result['origin']['request_start_monotonic_s'])

    def test_stop_recording_drains_original_ticket_without_a_stop_frame(self):
        result,port,_=self.session(delay=.009,stop_pending=True)
        self.assertTrue(result['recording_complete'],result['first_failure'])
        self.assertEqual(result['termination_reason'],'operator_stopped_recording')
        drain=result['normal_quiesce']
        self.assertIsNotNone(drain['pending'])
        self.assertTrue(drain['complete']); self.assertEqual(drain['queries_started'],0)
        self.assertLess(drain['started_monotonic_s']+drain['duration_s'],drain['pending']['deadline_monotonic_s'])
        self.assertTrue(all(raw[3] in (2,5) for _,raw in port.writes))
        self.assertEqual(len(result['rows']),len(port.writes))

    def test_frozen_source_fails_at_750ms_despite_repeated_successful_queries(self):
        result,port,_=self.session('frozen',duration=1.)
        self.assertFalse(result['recording_complete'])
        self.assertIn('Host feedback source age',result['first_failure']['message'])
        self.assertLess(result['elapsed_s'],.76)
        feedback=[row for row in result['rows'] if row['kind']==5]
        self.assertEqual(len({row['source_monotonic_s'] for row in feedback}),1)
        self.assertTrue(all(raw[3] in (2,5) for _,raw in port.writes))

    def test_enabled_old_can_fast_and_bad_crc_refuse_without_control(self):
        for mode in ('enabled','old_can','fast','bad_crc','short'):
            with self.subTest(mode=mode):
                result,port,_=self.session(mode)
                self.assertFalse(result['recording_complete'])
                self.assertIsNotNone(result['first_failure'])
                self.assertTrue(all(raw[3] in (2,5) for _,raw in port.writes))
                self.assertTrue(result['raw_events'])
                if mode=='short':
                    self.assertEqual(next(e for e in result['raw_events'] if e.get('kind')==5)['bytes_accepted'],10)

    def test_raw_budget_full_fails_before_unrecorded_io_without_tail_overwrite(self):
        result,port,_=self.session(duration=3.,evidence_limit=256)
        self.assertFalse(result['recording_complete'])
        self.assertTrue(result['raw_evidence_overflow'])
        self.assertIn('RAM budget',result['first_failure']['message'])
        tx=[entry for entry in result['raw_events'] if entry['direction']=='tx']
        self.assertEqual([entry['raw_hex'] for entry in tx],[raw.hex() for _,raw in port.writes])
        self.assertEqual(tx[0]['sequence'],1)
        self.assertLessEqual(len(result['raw_events']),256)

    def test_wire_rejects_stop_all_controls_bad_crc_fields_and_other_thread(self):
        clock=Clock(); port=HandPushedMCU(clock); fence=manual.ManualQueryPort(port,clock=clock.monotonic)
        frames=[api.request(kind,1) for kind in (1,3,8)]
        frames += [api.request(6,1,session=9,challenge=123),api.request(7,1,session=9),
                   api.request(7,1,1,-1,session=9),bytes(20),api.request(2,1)[:-1]+b'\x00']
        for raw in frames:
            with self.subTest(raw=raw.hex()),self.assertRaises(manual.ManualQueryFault): fence.write(raw)
        self.assertEqual(port.writes,[])
        failures=[]
        def other_thread():
            try: fence.write(api.request(2,1))
            except Exception as exc: failures.append(exc)
        thread=threading.Thread(target=other_thread);thread.start();thread.join()
        self.assertEqual(len(failures),1);self.assertIn('PID/thread',str(failures[0]))
        self.assertEqual(port.writes,[])

    def test_factory_preserves_original_duplicate_gap_and_mcu_time_policy(self):
        odom=manual.make_raw_rpm_odometry(core.MeasuredOdometry)
        packet=dict(sample_sequence=1,speed_tick_ms=100,speed_left_rpm=.1,speed_right_rpm=-.1)
        self.assertEqual(odom.update(packet)['x'],0.)
        self.assertIsNone(odom.update(packet))
        self.assertTrue(odom.update(dict(packet,sample_sequence=3,speed_tick_ms=200))['discontinuity'])
        odom.invalidate()
        self.assertFalse(odom.update(dict(packet,sample_sequence=4,speed_tick_ms=300))['integrated'])

    def test_current_source_gap_continues_without_integration_or_claiming_complete_trajectory(self):
        result,port,seen=self.session('gap_once',duration=.6)
        self.assertIsNone(result['first_failure'])
        self.assertTrue(result['recording_ended_cleanly'])
        self.assertTrue(result['normal_recording_end'])
        self.assertEqual(result['termination_reason'],'fixed_duration_complete')
        self.assertFalse(result['recording_complete'])
        self.assertFalse(result['odometry_continuity_validated'])
        self.assertEqual(result['metrics']['gap_count'],1)
        gap=result['odometry_discontinuities'][0]
        self.assertEqual(gap['sequence_step'],2)
        self.assertEqual(gap['dt_s'],.2)
        self.assertGreater(gap['source_gap_s'],.19)
        self.assertLess(gap['source_gap_s'],.23)
        gap_index=next(i for i,item in enumerate(seen) if item[0]['discontinuity'])
        before,current,after=seen[gap_index-1][0],seen[gap_index][0],seen[gap_index+1][0]
        self.assertFalse(current['integrated'])
        self.assertEqual(current['x'],before['x'])
        self.assertEqual(current['source_monotonic_s'],gap['source_monotonic_s'])
        self.assertGreater(after['x'],current['x'])
        self.assertEqual(result['origin']['sample_sequence'],seen[0][1]['sample_sequence'])
        self.assertTrue(all(raw[3] in (2,5) for _,raw in port.writes))

    def test_missing_observation_from_distinct_real_source_remains_fatal(self):
        class MissingObservation(core.MeasuredOdometry):
            def update(self,packet): return None
        result,port,_=self.session(odom_class=MissingObservation)
        self.assertFalse(result['normal_recording_end'])
        self.assertFalse(result['recording_complete'])
        self.assertIn('no odometry observation',result['first_failure']['message'])
        self.assertTrue(all(raw[3] in (2,5) for _,raw in port.writes))


if __name__=='__main__': unittest.main()
