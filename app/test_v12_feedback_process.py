"""Production worker and codec exercised through a real spawned process."""

from dataclasses import asdict
import importlib.util
import hashlib
import io
import json
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Thread
import time
import unittest
from contextlib import redirect_stdout
from unittest.mock import patch

from v12_feedback_worker import FeedbackWorker
from feedback_test_support import SimulatedPort, simulated_port
from mcu_v11 import (APPEND, BEGIN, COMMIT, HEADER, PERIOD_S, QUERY, STEER, STEER_VALID, STATUS,
                     STOP, LinkError, Rejected, command, crc16, decode_telemetry)
import struct
from v11_map_plan import load_map
from v11_ros import NS, OdomHistory


class GuidedSimulatedPort(SimulatedPort):
    def write(self, packet):
        if packet[3] != STEER:
            return super().write(packet)
        index, bias, basis = struct.unpack_from('<HhI', packet, 34)
        if len(packet) != 44 or index != 0 or not -2000 <= bias <= 2000:
            raise AssertionError('Invalid STEER payload from production master')
        boot, epoch, request = struct.unpack_from('<QII', packet, 6)
        self.extra_flags |= STEER_VALID
        super().write(command(QUERY, boot, epoch, request))
        if self.profile == 'steer_rejected':
            values = asdict(decode_telemetry(bytes(self.pending)))
            values['ack_result'] = 4
            body = HEADER[2:] + STATUS.pack(*values.values())
            self.pending[:] = b'\xa5\x5a' + body + struct.pack('<H', crc16(body))
        elif self.profile == 'steer_late':
            self.ready_at = time.monotonic() + .15
        return len(packet)


def guided_simulated_port(device, reply_timeout):
    return GuidedSimulatedPort(device, reply_timeout)


class FeedbackProcessTests(unittest.TestCase):
    @staticmethod
    def minimum_host_rate():
        # Python 3.10 on Windows uses a 15.625 ms monotonic clock. Its 20 ms
        # minimum send spacing quantizes to two ticks; RK uses a finer clock.
        resolution = time.get_clock_info('monotonic').resolution
        period = max(PERIOD_S, math.ceil(PERIOD_S/resolution)*resolution)
        return .85/period

    def launch(self, profile='normal', *, trace_timing=False):
        worker = FeedbackWorker(profile, port_factory=simulated_port, trace_timing=trace_timing)
        worker.launch()
        self.addCleanup(worker.close)
        worker.discover()
        return worker

    def test_query_timing_crosses_process_with_ordered_phases(self):
        worker = self.launch(trace_timing=True)
        deadline = time.monotonic() + 2.
        while True:
            snapshot = worker.timing.snapshot()
            if len(snapshot['rows']) >= 3 or time.monotonic() >= deadline:
                break
            time.sleep(.01)
        self.assertGreaterEqual(len(snapshot['rows']), 3)
        rows = [dict(zip(snapshot['stages'], row)) for row in snapshot['rows']]
        for previous, following in zip(rows[1:], rows[2:]):
            stages = ('cadence_ready', 'write_started', 'write_returned', 'read_returned',
                      'parsed', 'publish_waiting', 'publish_acquired', 'publish_released',
                      'send_cache_waiting', 'send_cache_acquired', 'send_cache_released',
                      'drop_cache_waiting', 'drop_cache_acquired', 'drop_cache_released')
            timestamps = [previous[stage] for stage in stages]
            self.assertGreater(timestamps[0], 0)
            self.assertEqual(timestamps, sorted(timestamps))
            self.assertLessEqual(timestamps[-1], following['write_started'])
            self.assertEqual(following['request'], previous['request'] + 1)

    def launch_guided(self, profile='normal'):
        worker = FeedbackWorker(profile, port_factory=guided_simulated_port).launch()
        self.addCleanup(worker.close)
        worker.discover()
        ack = worker.upload(dict(segments=[dict(kind='move', distance_m=.5, guided=True)]))[-1]
        worker.start(ack.status.task)
        return worker

    def test_steer_ack_is_cached_with_fractional_bias_bytes(self):
        worker = self.launch_guided()
        before = worker.snapshot()['cursor']
        latest = worker.latest.status
        result = worker.steer(latest.task, 0, -1250, latest.measurement_tick_ms)
        self.assertEqual(result.opcode, STEER)
        self.assertTrue(result.status.flags & STEER_VALID)
        self.assertEqual(struct.unpack_from('<HhI', bytes.fromhex(result.request_hex), 34),
                         (0, -1250, latest.measurement_tick_ms))
        replies = [reply for reply in worker.history_since(before)['exchanges'] if reply.opcode == STEER]
        self.assertEqual([asdict(reply) for reply in replies], [asdict(result)])

    def test_stop_cancels_queued_steer_before_any_steer_transmission(self):
        worker = self.launch_guided()
        latest = worker.latest.status
        revision = worker.stop_revision
        cursor = worker.transmissions_since(0)['cursor']
        worker.request_stop()
        with self.assertRaisesRegex(ValueError, 'cancelled by STOP'):
            worker.steer(latest.task, 0, 1000, latest.measurement_tick_ms, revision=revision)
        self.assertTrue(worker.stop()['stop_verified'])
        self.assertFalse(worker.failed)
        self.assertNotIn(STEER, [entry['opcode'] for entry in worker.transmissions_since(cursor)['entries']])

    def test_steer_rejection_is_not_retried_and_keeps_feedback_connected(self):
        worker = self.launch_guided('steer_rejected')
        latest = worker.latest.status
        cursor = worker.transmissions_since(0)['cursor']
        with self.assertRaises(Rejected):
            worker.steer(latest.task, 0, 750, latest.measurement_tick_ms)
        entries = worker.transmissions_since(cursor)['entries']
        self.assertEqual(sum(entry['opcode'] == STEER for entry in entries), 1)
        self.assertFalse(worker.failed)
        self.assertTrue(worker.snapshot()['connected'])

    def test_late_steer_ack_closes_worker_without_resending(self):
        worker = self.launch_guided('steer_late')
        latest = worker.latest.status
        cursor = worker.transmissions_since(0)['cursor']
        with self.assertRaisesRegex(LinkError, 'matching V13 reply'):
            worker.steer(latest.task, 0, 500, latest.measurement_tick_ms)
        entries = worker.transmissions_since(cursor)['entries']
        self.assertEqual(sum(entry['opcode'] == STEER for entry in entries), 1)
        worker.process.join(2.)
        self.assertTrue(worker.failed)
        self.assertFalse(worker.snapshot()['connected'])

    def free_grid(self):
        folder = TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        root = Path(folder.name)
        pgm, yaml = root/'map.pgm', root/'map.yaml'
        pgm.write_bytes(b'P5\n100 100\n255\n'+bytes([254])*10000)
        yaml.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
                        'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n', encoding='utf-8')
        return load_map(pgm, yaml, use_modeled_error_margin=False)

    def test_parent_pause_preserves_intermediate_measurements_and_does_not_stop_queries(self):
        worker = self.launch()
        before = worker.snapshot()['cursor']
        tx_before = worker.transmissions_since(0)['cursor']
        began = time.monotonic()
        time.sleep(1.2)
        duration = time.monotonic()-began
        batch = worker.history_since(before)
        tx = worker.transmissions_since(tx_before)
        self.assertEqual(batch['missed'], 0)
        self.assertEqual(tx['missed'], 0)
        self.assertGreater(len(tx['entries']), duration*self.minimum_host_rate())
        self.assertLess(len(tx['entries']), duration*53+2)
        self.assertTrue(all(entry['opcode'] == QUERY for entry in tx['entries']))
        identities = []
        for reply in batch['exchanges']:
            self.assertEqual(decode_telemetry(bytes.fromhex(reply.reply_hex)), reply.status)
            sequence = reply.status.sample_sequence
            if not identities or identities[-1] != sequence:
                identities.append(sequence)
        self.assertGreaterEqual(len(identities), 22)
        self.assertTrue(all(b-a == 1 for a, b in zip(identities, identities[1:])))
        self.assertGreater(len(batch['exchanges']), len(identities)*1.3)
        odom = OdomHistory(1_700_000_000*NS)
        for reply in batch['exchanges']:
            odom.add(reply)
        summary = odom.diagnostics(time.monotonic())
        self.assertEqual(summary['gap_count'], 0)
        self.assertEqual(summary['newest_sample_sequence'], identities[-1])

    def test_real_planner_cpu_load_does_not_own_serial(self):
        grid = self.free_grid()
        worker = self.launch()
        cursor = worker.transmissions_since(0)['cursor']
        start, goal = dict(x=1., y=1., yaw=0.), dict(x=3.5, y=3.5)
        began = time.monotonic()
        attempts = 0
        while time.monotonic()-began < 1.2:
            plan = grid.plan(start, goal)
            grid.validate_plan(start, plan)
            attempts += 1
        duration = time.monotonic()-began
        tx = worker.transmissions_since(cursor)
        times = [entry['sent_at'] for entry in tx['entries']]
        self.assertGreater(attempts, 0)
        self.assertEqual(tx['missed'], 0)
        self.assertGreater(len(times), duration*self.minimum_host_rate())
        self.assertLess(max(b-a for a, b in zip(times, times[1:])), .150)

    def test_memory_reads_do_not_request_extra_exchanges(self):
        worker = self.launch()
        before = worker.transmissions_since(0)['cursor']
        began = time.monotonic()
        for _ in range(200):
            latest = worker.snapshot()
            worker.history_since(latest['cursor'])
        duration = time.monotonic()-began
        tx = worker.transmissions_since(before)
        self.assertLess(duration, .6)
        self.assertLessEqual(len(tx['entries']), int(duration/.020)+2)
        self.assertTrue(all(entry['opcode'] == QUERY for entry in tx['entries']))
        same = next(reply for reply in worker.history_since(0)['exchanges']
                    if reply.request == latest['latest'].request)
        self.assertEqual(asdict(same), asdict(latest['latest']))

    def test_upload_acknowledgements_are_in_feedback_history(self):
        worker = self.launch()
        cursor = worker.snapshot()['cursor']
        replies = worker.upload(dict(segments=[dict(kind='move', distance_m=.5)]*8))
        batch = worker.history_since(cursor)
        acks = [reply for reply in batch['exchanges'] if reply.opcode != QUERY]
        self.assertEqual([reply.opcode for reply in acks], [BEGIN]+[APPEND]*8+[COMMIT])
        self.assertEqual([asdict(reply) for reply in acks], [asdict(reply) for reply in replies])
        self.assertEqual(replies[-1].status.state, 2)
        result = worker.stop()
        self.assertTrue(result['stop_verified'])

    def test_invalid_source_is_not_hidden_by_following_valid_samples(self):
        worker = self.launch('invalid')
        time.sleep(.65)
        replies = worker.history_since(0)['exchanges']
        invalid = [index for index, reply in enumerate(replies) if not reply.status.source_valid]
        self.assertTrue(invalid)
        self.assertTrue(any(reply.status.source_valid for reply in replies[invalid[-1]+1:]))
        odom = OdomHistory(1_700_000_000*NS)
        for reply in replies:
            odom.add(reply)
        self.assertGreater(odom.history.summary()['segment'], 1)
        self.assertTrue(worker.snapshot()['connected'])

    def test_stop_interrupts_upload_before_next_segment(self):
        worker = self.launch()
        cursor = worker.transmissions_since(0)['cursor']
        outcome = []

        def upload():
            try:
                worker.upload(dict(segments=[dict(kind='move', distance_m=.5)]*128))
                outcome.append('completed')
            except Exception as exc:
                outcome.append(exc)

        thread = Thread(target=upload)
        thread.start()
        until = time.monotonic()+2.
        while time.monotonic() < until:
            entries = worker.transmissions_since(cursor)['entries']
            if sum(entry['opcode'] == APPEND for entry in entries) >= 3:
                break
            time.sleep(.005)
        worker.request_stop()
        stop = worker.stop()
        thread.join(3.)
        self.assertFalse(thread.is_alive())
        self.assertEqual(len(outcome), 1)
        self.assertIsInstance(outcome[0], ValueError)
        self.assertTrue(stop['stop_verified'])
        entries = worker.transmissions_since(cursor)['entries']
        opcodes = [entry['opcode'] for entry in entries]
        self.assertIn(STOP, opcodes)
        self.assertNotIn(COMMIT, opcodes)
        self.assertLess(opcodes.count(APPEND), 128)
        self.assertNotIn(APPEND, opcodes[opcodes.index(STOP)+1:])

    def test_abrupt_child_exit_marks_old_snapshot_disconnected(self):
        worker = self.launch()
        old = worker.latest
        worker.process.terminate()
        worker.process.join(2.)
        self.assertFalse(worker.process.is_alive())
        state = worker.snapshot()
        self.assertFalse(state['connected'])
        self.assertIsNotNone(state['error'])
        self.assertEqual(state['latest'].sent_at, old.sent_at)
        with self.assertRaises(LinkError):
            worker.query()

    def test_cancelled_start_waiting_behind_stop_proof_does_not_poison_owner(self):
        worker = self.launch('stop_no_proof')
        uploaded = worker.upload(dict(segments=[dict(kind='move', distance_m=.5)]))
        previous_revision = worker.stop_revision
        worker.request_stop()
        with self.assertRaisesRegex(ValueError, 'cancelled by STOP'):
            worker.start(uploaded[-1].status.task, revision=previous_revision)
        result = worker.stop()
        self.assertFalse(result['stop_verified'])
        self.assertEqual(result['error']['code'], 'stop_proof_deadline')
        self.assertFalse(worker.failed)
        self.assertTrue(worker.snapshot()['connected'])
        self.assertIsNotNone(worker.query())

    def test_mcu_counter_wrap_survives_real_process_cache(self):
        worker = self.launch('wrap')
        time.sleep(.65)
        replies = worker.history_since(0)['exchanges']
        self.assertGreater(replies[0].status.publication_tick_ms, replies[-1].status.publication_tick_ms)
        self.assertGreater(replies[0].status.sample_sequence, replies[-1].status.sample_sequence)
        self.assertTrue(worker.snapshot()['connected'])
        self.assertIsNone(worker.snapshot()['error'])
        odom = OdomHistory(1_700_000_000*NS)
        for reply in replies:
            odom.add(reply)
        self.assertEqual(odom.history.summary()['gap_count'], 0)

    def test_device_failure_and_identity_change_disconnect_without_hanging(self):
        for profile in ('write_failure', 'boot_change', 'epoch_change'):
            with self.subTest(profile=profile):
                worker = self.launch(profile)
                until = time.monotonic()+2.
                while worker.snapshot()['connected'] and time.monotonic() < until:
                    time.sleep(.02)
                state = worker.snapshot()
                self.assertFalse(state['connected'])
                self.assertIsNotNone(state['error'])
                began = time.monotonic()
                with self.assertRaises(LinkError):
                    worker.query()
                self.assertLess(time.monotonic()-began, .3)


class FeedbackBenchmarkTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = Path(__file__).resolve().parents[1]/'scripts/benchmark_v12_feedback.py'
        spec = importlib.util.spec_from_file_location('feedback_benchmark', path)
        cls.benchmark = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.benchmark)

    def test_default_command_does_not_launch_worker_or_touch_output(self):
        with TemporaryDirectory() as directory:
            output = Path(directory)/'uncreated'
            stdout = io.StringIO()
            with patch.object(self.benchmark, 'FeedbackWorker') as factory, redirect_stdout(stdout):
                code = self.benchmark.main(['--port', '/dev/not-a-real-device', '--output', str(output)])
            self.assertEqual(code, 0)
            factory.assert_not_called()
            self.assertFalse(output.exists())
            self.assertFalse(json.loads(stdout.getvalue())['port_opened'])

    def test_comparison_uses_measured_send_cadence_and_rejects_missing_records(self):
        def phase(rate, p99):
            return dict(tx_rate_hz=rate, tx_gap_ms=dict(p99=p99), cache_missed=dict(feedback=0, transmissions=0),
                        query_only=True, reply_count=100, invalid_source_replies=0, moving_replies=0,
                        drive_fault_replies=0, mcu_fault_replies=0, boots=['42'], planning=dict(successful=2))
        idle, load = phase(49., 22.), phase(46., 25.)
        self.assertTrue(self.benchmark.compare_phases(idle, load)['passed'])
        load['tx_rate_hz'] = 40.
        self.assertFalse(self.benchmark.compare_phases(idle, load)['passed'])
        load['tx_rate_hz'] = 46.
        load['cache_missed']['feedback'] = 1
        self.assertFalse(self.benchmark.compare_phases(idle, load)['passed'])

    def test_full_benchmark_records_only_queries_in_both_phases(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            pgm, yaml, output = root/'map.pgm', root/'map.yaml', root/'result'
            pgm.write_bytes(b'P5\n100 100\n255\n'+bytes([254])*10000)
            yaml.write_text('image: map.pgm\nresolution: 0.05\norigin: [0, 0, 0]\n'
                            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n', encoding='utf-8')
            factory = lambda device: FeedbackWorker('normal', port_factory=simulated_port)
            args = ['--execute', '--port', 'simulation-only', '--duration', '1.2', '--output', str(output),
                    '--map-pgm', str(pgm), '--map-yaml', str(yaml),
                    '--map-sha256', hashlib.sha256(pgm.read_bytes()).hexdigest(),
                    '--yaml-sha256', hashlib.sha256(yaml.read_bytes()).hexdigest(),
                    '--start-x', '1', '--start-y', '1', '--goal-x', '3', '--goal-y', '3']
            with patch.object(self.benchmark, 'FeedbackWorker', side_effect=factory), redirect_stdout(io.StringIO()):
                code = self.benchmark.main(args)
            report = json.loads((output/'report.json').read_text(encoding='utf-8'))
            self.assertEqual(code, 0, report)
            self.assertEqual([phase['phase'] for phase in report['phases']], ['idle', 'planning'])
            self.assertTrue(report['comparison']['passed'])
            records = [json.loads(line) for line in (output/'records.jsonl').read_text().splitlines()]
            sent = [record for record in records if record['kind'] == 'transmit']
            self.assertGreater(len(sent), 50)
            self.assertTrue(all(record['opcode'] == QUERY for record in sent))
            self.assertTrue(all(phase['planning']['successful'] for phase in report['phases']
                                if phase['phase'] == 'planning'))


if __name__ == '__main__':
    unittest.main()
