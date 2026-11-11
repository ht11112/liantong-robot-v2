"""Offline tests for V13 codec, sequencing, polling and operator boundaries."""

from contextlib import contextmanager, redirect_stderr, redirect_stdout
from io import StringIO
import json
import os
from pathlib import Path
import struct
import sys
import threading
import unittest
import zlib
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import mcu_v11 as v11


BOOT = 0x1122334455667788
TASK = 0x0102030405060708
PAYLOAD = ('boot epoch publication_sequence publication_tick_ms sample_sequence measurement_tick_ms '
           'task ack_request ack_result state flags index count progress x y yaw v w position_left '
           'position_right left_tenths_rpm right_tenths_rpm first_error first_drive_error first_error_tick_ms '
           'lease_remaining_ms accepted_request_watermark').split()


def telemetry(**changes):
    values = dict.fromkeys(PAYLOAD, 0)
    values.update(boot=BOOT, epoch=7, publication_sequence=1, publication_tick_ms=1000,
                  sample_sequence=1, measurement_tick_ms=995, flags=v11.ACK_VALID | 3)
    values.update(changes)
    body = v11.HEADER[2:] + struct.pack('<QIIIIIQIBBHHHffffffIIhhHIIII', *(values[key] for key in PAYLOAD))
    return b'\xa5\x5a' + body + struct.pack('<H', v11.crc16(body))


def golden_reply():
    return telemetry(publication_sequence=9, publication_tick_ms=10000, sample_sequence=42,
                     measurement_tick_ms=9950, task=TASK, ack_request=5, ack_result=4,
                     state=5, flags=455, index=1, count=2, progress=.25, x=1.25, y=-.5,
                     yaw=-1., v=.02, w=-.05, position_left=0xFFFFFFFE, position_right=1234,
                     left_tenths_rpm=7, right_tenths_rpm=-9, first_error=3,
                     first_drive_error=0x12345678, first_error_tick_ms=9000,
                     lease_remaining_ms=29500, accepted_request_watermark=32)


class FakeClock:
    def __init__(self):
        self.value = 0.

    def __call__(self):
        return self.value

    def sleep(self, seconds):
        self.value += seconds


class FakePort:
    def __init__(self, clock, response=None):
        self.clock = clock
        self.response = response or self.regular_response
        self.writes = []
        self.inbox = []
        self.short_write = False

    def regular_response(self, index, kind, request, task):
        tick = 1000 + index * 20
        source_tick = tick - tick % 50
        frame = telemetry(publication_sequence=index + 1, publication_tick_ms=tick,
                          sample_sequence=source_tick // 50, measurement_tick_ms=source_tick,
                          ack_request=request, accepted_request_watermark=request, task=task)
        return [(.014, frame)]

    def write(self, packet):
        kind = packet[3]
        request = struct.unpack_from('<I', packet, 18)[0]
        task = struct.unpack_from('<Q', packet, 26)[0]
        index = len(self.writes)
        self.writes.append((self.clock(), packet))
        if self.short_write:
            return len(packet) - 1
        self.inbox.extend((self.clock() + delay, raw) for delay, raw in self.response(index, kind, request, task))
        return len(packet)

    def read(self, size):
        # A bounded fake serial read. No real sleep or I/O occurs.
        expires = self.clock() + .002
        if not self.inbox or self.inbox[0][0] > expires:
            self.clock.value = expires
            return b''
        ready, data = self.inbox.pop(0)
        self.clock.value = max(ready, self.clock())
        if len(data) > size:
            self.inbox.insert(0, (ready, data[size:]))
        return data[:size]


class CodecTests(unittest.TestCase):
    def test_discovery_and_regular_wire_layout(self):
        discovery = v11.command(v11.QUERY)
        self.assertEqual(discovery[:6], b'\xa5\x5a\x0d\x07\x1c\x00')
        self.assertEqual(discovery[6:34], bytes(28))
        self.assertEqual(len(discovery), 36)
        self.assertEqual(v11.crc16(discovery[2:-2]), int.from_bytes(discovery[-2:], 'little'))
        request = v11.command(v11.BEGIN, BOOT, 7, 33, TASK, struct.pack('<HII', 2, 0x615AABE9, 30000))
        self.assertEqual(len(request), 46)
        self.assertEqual(struct.unpack_from('<QIIIQ', request, 6), (BOOT, 7, 33, 0, TASK))
        with self.assertRaises(ValueError):
            v11.command(v11.START)
        with self.assertRaises(ValueError):
            v11.command(v11.QUERY, boot=BOOT)

    def test_real_c_golden(self):
        log = os.environ.get('MQ_V11_CODEC_GOLDEN_LOG')
        if not log:
            self.skipTest('Run the host C build to compare the actual V13 golden vector')
        lines = Path(log).read_text(encoding='utf-8').splitlines()
        line = next(line for line in lines if line.startswith('GOLDEN'))
        golden = json.loads(line.partition(' ')[2] if line.startswith('GOLDEN ') else line[line.index('{'):])
        self.assertEqual(golden['telemetry'], golden_reply().hex())
        status = v11.decode_telemetry(bytes.fromhex(golden['telemetry']))
        self.assertEqual(golden['query'], v11.command(v11.QUERY).hex())
        self.assertEqual(golden['begin'], v11.command(v11.BEGIN, status.boot, 7, 33, status.task + 1,
                                                    struct.pack('<HII', 2, 0x615AABE9, 30000)).hex())
        self.assertEqual(golden['append'], v11.command(v11.APPEND, status.boot, 7, 34, status.task + 1,
                                                      struct.pack('<HIf', 1, 2, -1.5)).hex())
        self.assertEqual(golden['queue_crc32'], 0x615AABE9)

    def test_fragmented_noise_crc_metadata_and_legacy_rejection(self):
        valid = telemetry()
        bad_crc = bytearray(valid)
        bad_crc[50] ^= 1
        old_body = b'\x0c' + valid[3:-2]
        old = valid[:2] + old_body + struct.pack('<H', v11.crc16(old_body))
        parser = v11.ReplyParser()
        self.assertEqual(list(parser.feed(b'noise' + old + bad_crc + telemetry(lease_remaining_ms=1) + valid[:50])), [])
        outputs = list(parser.feed(valid[50:]))
        self.assertEqual(outputs, [(v11.decode_telemetry(valid), valid)])
        self.assertEqual(parser.errors['invalid_headers'], 1)
        self.assertEqual(parser.errors['crc_errors'], 1)
        self.assertEqual(parser.errors['invalid_metadata'], 1)
        with self.assertRaises(ValueError):
            v11.decode_telemetry(telemetry(measurement_tick_ms=750))

    def test_lease_field_is_bounded_and_only_active_states_report_it(self):
        flags = v11.ACK_VALID | 3 | v11.EXECUTING | v11.STARTED | v11.COMMITTED
        for state, remaining in ((5, 30000), (5, 0), (7, 0)):
            raw = telemetry(state=state, flags=flags, lease_remaining_ms=remaining)
            self.assertEqual(struct.unpack_from('<I', raw, 100)[0], remaining)
            self.assertEqual(v11.decode_telemetry(raw).lease_remaining_ms, remaining)
        for state, remaining in ((5, 30001), (7, 1), (9, 30000)):
            with self.assertRaises(ValueError):
                v11.decode_telemetry(telemetry(state=state, flags=flags, lease_remaining_ms=remaining))

    def test_old_total_duration_is_not_implicitly_dropped(self):
        with self.assertRaisesRegex(ValueError, 'Remove duration_s'):
            v11.QueuePlan.from_document({'segments': [{'kind': 'move', 'distance_m': .5}], 'duration_s': 1200})

    def test_guided_move_uses_distinct_canonical_kind_and_crc(self):
        document = dict(segments=[dict(kind='move', distance_m=.5, guided=True),
                                  dict(kind='turn', angle_rad=-1.5),
                                  dict(kind='move', distance_m=.75, guided=False)])
        segments, checksum, duration = v11.canonical_segments(document)
        expected = [struct.pack('<If', 3, .5), struct.pack('<If', 2, -1.5), struct.pack('<If', 1, .75)]
        self.assertEqual(segments, expected)
        self.assertEqual(checksum, zlib.crc32(b''.join(expected)))
        self.assertEqual(duration, 1200000)
        self.assertEqual(v11.QueuePlan.from_document(document).segments, tuple(expected))
        for segment in (dict(kind='move', distance_m=.5, guided=1),
                        dict(kind='move', distance_m=.5, guided='true'),
                        dict(kind='turn', angle_rad=1., guided=True)):
            with self.subTest(segment=segment), self.assertRaisesRegex(ValueError, 'guided'):
                v11.canonical_segments(dict(segments=[segment]))

    def test_steer_wire_layout_uses_signed_millirpm_and_wraparound_basis(self):
        packet = v11.command(v11.STEER, BOOT, 7, 33, TASK, struct.pack('<HhI', 127, -1250, 0xffffffff))
        self.assertEqual(len(packet), 44)
        self.assertEqual(packet[:6], b'\xa5\x5a\x0d\x09\x24\x00')
        self.assertEqual(struct.unpack_from('<HhI', packet, 34), (127, -1250, 0xffffffff))
        self.assertEqual(v11.crc16(packet[2:-2]), int.from_bytes(packet[-2:], 'little'))
        self.assertEqual(v11.decode_telemetry(telemetry(flags=259 | v11.STEER_VALID)).flags, 771)
        with self.assertRaises(ValueError):
            v11.decode_telemetry(telemetry(flags=259 | 1024))
        for extra in (b'', bytes(10)):
            with self.assertRaises(ValueError):
                v11.command(v11.STEER, BOOT, 7, 33, TASK, extra)


def active_move_response(index, kind, request, task):
    tick = 1000 + index * 20
    return [(.014, telemetry(publication_sequence=index + 1, publication_tick_ms=tick,
                             measurement_tick_ms=tick - 5, sample_sequence=index + 1,
                             ack_request=request, accepted_request_watermark=request, task=TASK,
                             state=4, count=1, flags=3 | v11.ACK_VALID | v11.EXECUTING,
                             lease_remaining_ms=30000))]


class MasterTests(unittest.TestCase):
    def make_master(self, response=None):
        clock = FakeClock()
        port = FakePort(clock, response)
        master = v11.RkMaster(port, clock=clock, sleep=clock.sleep)
        return master, port

    def test_steer_fractional_bias_and_validation_use_one_real_transaction(self):
        master, port = self.make_master(active_move_response)
        master.discover()
        reply = master.steer(TASK, 0, 1250, 995)
        self.assertEqual(reply.opcode, v11.STEER)
        self.assertEqual(struct.unpack_from('<HhI', port.writes[-1][1], 34), (0, 1250, 995))
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.STEER])
        for bias in (-2001, 2001, True, 1.25):
            with self.subTest(bias=bias), self.assertRaises(ValueError):
                master.steer(TASK, 0, bias, 995)
        for index, tick in ((128, 995), (True, 995), (0, -1), (0, 0x100000000), (0, True)):
            with self.subTest(index=index, tick=tick), self.assertRaises(ValueError):
                master.steer(TASK, index, 1000, tick)
        with self.assertRaisesRegex(ValueError, 'exact active MOVE'):
            master.steer(TASK + 1, 0, 1000, 995)
        self.assertEqual(len(port.writes), 2)

    def test_late_steer_reply_times_out_from_actual_send_without_retry(self):
        def response(index, kind, request, task):
            rows = active_move_response(index, kind, request, task)
            return [(delay if kind != v11.STEER else .101, frame) for delay, frame in rows]

        master, port = self.make_master(response)
        master.discover()
        with self.assertRaises(v11.ReplyTimeout) as fault:
            master.steer(TASK, 0, 500, 995)
        self.assertEqual(fault.exception.details['sent_at'], port.writes[-1][0])
        self.assertGreaterEqual(master.clock() - port.writes[-1][0], .100)
        self.assertTrue(master.failed)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.STEER])

    def test_rejected_steer_reply_is_retained_without_resending(self):
        def response(index, kind, request, task):
            if kind != v11.STEER:
                return active_move_response(index, kind, request, task)
            return [(.014, telemetry(publication_sequence=index + 1, publication_tick_ms=1020,
                                     measurement_tick_ms=1015, sample_sequence=2,
                                     ack_request=request, accepted_request_watermark=request,
                                     ack_result=4, task=TASK, state=4, count=1,
                                     flags=3 | v11.ACK_VALID | v11.EXECUTING, lease_remaining_ms=30000))]

        master, port = self.make_master(response)
        master.discover()
        with self.assertRaises(v11.Rejected) as rejected:
            master.steer(TASK, 0, -750, 995)
        self.assertIs(master.latest, rejected.exception.exchange)
        self.assertFalse(master.failed)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.STEER])

    def test_stop_during_cadence_cancels_steer_before_actual_write(self):
        clock = FakeClock()
        port = FakePort(clock, active_move_response)
        cancelled = [False]

        @contextmanager
        def guard(kind):
            if kind == v11.STEER and cancelled[0]:
                raise v11.RequestCancelled('STOP precedes STEER')
            yield

        def sleep(seconds):
            cancelled[0] = True
            clock.sleep(seconds)

        master = v11.RkMaster(port, clock=clock, sleep=sleep, send_guard=guard)
        master.discover()
        with self.assertRaises(v11.RequestCancelled):
            master.steer(TASK, 0, 1000, 995)
        self.assertFalse(master.failed)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY])

    def test_discovery_resumes_watermark_and_wraps(self):
        def response(index, kind, request, task):
            value = 0xFFFFFFFF if index == 0 else request
            sequence = (index - 1) & v11.MASK32
            tick = (index * 20 - 10) & v11.MASK32
            return [(.014, telemetry(publication_sequence=sequence, publication_tick_ms=tick,
                                     measurement_tick_ms=(tick - 5) & v11.MASK32,
                                     ack_request=request, accepted_request_watermark=value))]

        master, port = self.make_master(response)
        first = master.discover()
        self.assertEqual(first.status.accepted_request_watermark, 0xFFFFFFFF)
        self.assertEqual(master.query().request, 1)
        self.assertEqual(struct.unpack_from('<QIIIQ', port.writes[-1][1], 6), (BOOT, 7, 1, 0, 0))

    def test_poll_repeated_real_sources_and_uart_fault_separate(self):
        def response(index, kind, request, task):
            tick = 1000 + index * 20
            measurement = tick - tick % 50
            return [(.014, telemetry(publication_sequence=index + 1, publication_tick_ms=tick,
                                     sample_sequence=measurement // 50, measurement_tick_ms=measurement,
                                     ack_request=request, accepted_request_watermark=request,
                                     first_error=7, flags=v11.ACK_VALID | v11.RX_FAULT | 3))]

        master, port = self.make_master(response)
        master.discover()
        report = master.poll(.21)
        self.assertTrue(report['success'])
        self.assertEqual(report['acknowledged_queries'], 11)
        self.assertLess(report['distinct_live_samples'], report['fresh_source_replies'])
        self.assertEqual(report['repeated_live_sample_replies'],
                         report['fresh_source_replies'] - report['distinct_live_samples'])
        self.assertEqual(report['uart_first_error_7_replies'], report['acknowledged_queries'])
        self.assertAlmostEqual(report['reply_rate_hz'], 50.)
        self.assertAlmostEqual(report['send_rate_hz'], 50.)
        self.assertAlmostEqual(report['max_send_gap_ms'], 20.)
        self.assertTrue(all(packet[3] == v11.QUERY for _, packet in port.writes))

    def test_no_catch_up_after_slow_reply(self):
        master, port = self.make_master()
        response = port.response

        def delay(index, *args):
            return [(.045 if index == 1 else time, raw) for time, raw in response(index, *args)]

        port.response = delay
        master.discover()
        result = master.poll(.13)
        self.assertTrue(result['success'])
        ticks = [stamp for stamp, _ in port.writes]
        self.assertAlmostEqual(ticks[2] - ticks[1], .045)
        self.assertTrue(all(new - old >= .020 - 1e-9 for old, new in zip(ticks, ticks[1:])))
        self.assertLess(result['send_rate_hz'], 50)
        self.assertAlmostEqual(result['max_send_gap_ms'], 45.)
        self.assertEqual(result['reply_latency_ms']['over_20_ms'], 1)

    def test_timeout_and_short_write_poison_without_retry(self):
        for short in (False, True):
            with self.subTest(short=short):
                master, port = self.make_master(lambda *unused: [])
                port.short_write = short
                with self.assertRaises(v11.LinkError) as failed:
                    master.discover()
                self.assertIn('request_hex', failed.exception.details)
                self.assertTrue(master.failed)
                with self.assertRaises(v11.LinkError):
                    master.discover()
                self.assertEqual(len(port.writes), 1)

    def test_regular_query_drops_loss_crc_wrong_ack_and_late_reply_without_retry(self):
        for loss in ('missing', 'bad_crc', 'wrong_ack', 'late'):
            with self.subTest(loss=loss):
                def response(index, kind, request, task):
                    if index == 0:
                        return [(.014, telemetry(accepted_request_watermark=31))]
                    ack = request + 3 if index == 1 and loss == 'wrong_ack' else request
                    raw = telemetry(publication_sequence=index + 1, publication_tick_ms=1000 + 20 * index,
                                    ack_request=ack, accepted_request_watermark=ack)
                    if index == 1:
                        if loss == 'missing':
                            return []
                        if loss == 'bad_crc':
                            raw = raw[:-1] + bytes((raw[-1] ^ 1,))
                        if loss == 'late':
                            return [(.105, raw)]
                    return [(.014, raw)]

                master, port = self.make_master(response)
                discovered = master.discover()
                self.assertIsNone(master.query())
                self.assertIs(master.latest, discovered)
                self.assertEqual(len(port.writes), 2)
                self.assertEqual(master.dropped_queries, 1)
                reply = master.query()
                self.assertEqual(reply.request, 33)
                self.assertEqual(reply.sent_at, port.writes[2][0])
                self.assertEqual(struct.unpack_from('<I', port.writes[1][1], 18)[0], 32)
                self.assertEqual(master.parser.errors['crc_errors'], int(loss == 'bad_crc'))
                self.assertEqual(master.ignored['unmatched_ack'], int(loss in ('wrong_ack', 'late')))
                self.assertFalse(master.failed)

    def test_repeated_query_drops_keep_owner_open_and_advance_request(self):
        master, port = self.make_master(lambda index, *unused: [(.014, telemetry())] if not index else [])
        discovery = master.discover()
        for _ in range(3):
            self.assertIsNone(master.query())
        self.assertEqual(master.dropped_queries, 3)
        self.assertFalse(master.failed)
        self.assertIs(master.latest, discovery)
        self.assertEqual([struct.unpack_from('<I', packet, 18)[0] for _, packet in port.writes], [0, 1, 2, 3])

    def test_mutating_request_skips_dropped_query_id_and_wraps(self):
        def response(index, kind, request, task):
            ready = index < 2
            raw = telemetry(publication_sequence=index + 1, publication_tick_ms=1000 + 20 * index,
                            accepted_request_watermark=0xFFFFFFFE if index == 0 else request,
                            ack_request=request, task=TASK, state=v11.READY if ready else 3,
                            flags=v11.ACK_VALID | 3 | v11.COMMITTED | (0 if ready else v11.STARTED))
            return [(.105 if index == 1 else .014, raw)]

        master, port = self.make_master(response)
        master.discover()
        self.assertIsNone(master.query())
        ack = master.start(TASK)
        self.assertEqual(ack.request, 1)
        self.assertEqual(master.ignored['unmatched_ack'], 1)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.QUERY, v11.START])
        self.assertEqual([struct.unpack_from('<I', packet, 18)[0] for _, packet in port.writes], [0, 0xFFFFFFFF, 1])

    def test_explicit_reject_can_report_a_dropped_query_watermark(self):
        def response(index, kind, request, task):
            if index in (1, 2):
                return []
            rejected = index == 3
            return [(.014, telemetry(publication_sequence=index + 1, publication_tick_ms=1000 + 20 * index,
                                     accepted_request_watermark=1 if rejected else request,
                                     ack_request=request, ack_result=4 if rejected else 0))]

        master, port = self.make_master(response)
        master.discover()
        self.assertIsNone(master.query())
        self.assertIsNone(master.query())
        with self.assertRaises(v11.Rejected):
            master.query()
        self.assertFalse(master.failed)
        self.assertEqual(master.query().request, 3)
        self.assertEqual([struct.unpack_from('<I', packet, 18)[0] for _, packet in port.writes], [0, 1, 2, 3, 3])

    def test_poll_counts_actual_attempts_and_drops_and_continues(self):
        master, port = self.make_master()
        response = port.response
        port.response = lambda index, *args: [] if index in (1, 4) else response(index, *args)
        master.discover()
        report = master.poll(.3)
        self.assertTrue(report['success'])
        self.assertIsNone(report['error'])
        self.assertEqual(report['dropped_queries'], 2)
        self.assertEqual(report['query_attempts'], len(port.writes) - 1)
        self.assertEqual(report['acknowledged_queries'], report['query_attempts'] - 2)
        self.assertEqual(report['first_reply']['request'], 2)
        self.assertAlmostEqual(report['max_send_gap_ms'], 100.)

    def test_short_regular_query_write_closes_without_retry(self):
        master, port = self.make_master()
        master.discover()
        port.short_write = True
        with self.assertRaisesRegex(v11.LinkError, 'Partial serial write'):
            master.query()
        self.assertEqual(len(port.writes), 2)
        self.assertEqual(master.dropped_queries, 0)
        self.assertTrue(master.failed)

    def test_query_write_timeout_preserves_owner_and_requires_on_time_ack(self):
        for elapsed in (.05, .12):
            with self.subTest(elapsed=elapsed):
                master, port = self.make_master()
                discovery = master.discover()
                write = port.write

                def write_then_timeout(packet):
                    write(packet)
                    master.sleep(elapsed)
                    raise TimeoutError('Write timeout')

                with patch.object(port, 'write', write_then_timeout):
                    reply = master.query()
                if elapsed > master.reply_timeout:
                    self.assertIsNone(reply)
                    self.assertIs(master.latest, discovery)
                    self.assertEqual(master.dropped_queries, 1)
                else:
                    self.assertEqual(reply.request, 1)
                    self.assertEqual(master.dropped_queries, 0)
                self.assertFalse(master.failed)
                self.assertEqual(master.query().request, 2)
                self.assertEqual(master.ignored['unmatched_ack'], int(reply is None))
                self.assertEqual(len(port.writes), 3)

    def test_discovery_and_stop_write_timeout_close_without_retry(self):
        for discover in (True, False):
            with self.subTest(discover=discover):
                master, port = self.make_master()
                if not discover:
                    master.discover()
                with patch.object(port, 'write', side_effect=TimeoutError('Write timeout')) as write:
                    with self.assertRaisesRegex(v11.LinkError, 'Serial exchange failed: Write timeout'):
                        (master.discover if discover else master.stop)()
                    write.assert_called_once()
                self.assertTrue(master.failed)
                self.assertEqual(master.dropped_queries, 0)

    def test_start_and_stop_timeouts_close_without_retry(self):
        discovery = telemetry(task=TASK, state=v11.READY, flags=v11.ACK_VALID | 3 | v11.COMMITTED)
        for opcode in (v11.START, v11.STOP):
            with self.subTest(opcode=opcode):
                master, port = self.make_master(lambda index, *unused: [(.014, discovery)] if not index else [])
                master.discover()
                with self.assertRaises(v11.ReplyTimeout):
                    if opcode == v11.START:
                        master.start(TASK)
                    else:
                        master.stop()
                self.assertTrue(master.failed)
                self.assertEqual(master.dropped_queries, 0)
                self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, opcode])

    def test_start_reply_deadline_is_one_second(self):
        discovery = telemetry(task=TASK, state=v11.READY, flags=v11.ACK_VALID | 3 | v11.COMMITTED)
        for delay in (.900, 1.001):
            with self.subTest(delay=delay):
                def response(index, kind, request, task):
                    if index == 0:
                        return [(.014, discovery)]
                    return [(delay, telemetry(publication_sequence=2, publication_tick_ms=1020,
                                              task=task, ack_request=request, accepted_request_watermark=request,
                                              state=3, flags=v11.ACK_VALID | 3 | v11.COMMITTED | v11.STARTED))]

                master, port = self.make_master(response)
                master.discover()
                if delay < 1.:
                    reply = master.start(TASK)
                    self.assertAlmostEqual(reply.received_at - reply.sent_at, delay)
                else:
                    with self.assertRaises(v11.ReplyTimeout) as failure:
                        master.start(TASK)
                    self.assertAlmostEqual(port.clock() - failure.exception.details['sent_at'], 1., delta=.0021)
                self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.START])

    def test_stale_ack_is_ignored_but_boot_change_poisoned(self):
        def response(index, kind, request, task):
            if index == 0:
                return [(.014, telemetry())]
            stale = telemetry(ack_request=request, accepted_request_watermark=request)
            new_boot = telemetry(boot=BOOT + 1, publication_sequence=2, publication_tick_ms=1020,
                                 ack_request=request, accepted_request_watermark=request)
            return [(.013, stale), (.026, new_boot)]

        master, port = self.make_master(response)
        master.discover()
        with self.assertRaises(v11.LinkError) as changed:
            master.query()
        self.assertEqual(master.ignored['stale_reply'], 1)
        self.assertEqual(changed.exception.exchange.status.boot, BOOT + 1)
        self.assertTrue(master.failed)
        self.assertEqual(len(port.writes), 2)

    def test_unmatched_old_identity_is_ignored_during_ordinary_query(self):
        def response(index, kind, request, task):
            if index == 0:
                return [(.014, telemetry())]
            status = dict(publication_sequence=index + 1, publication_tick_ms=1000 + 20 * index)
            if index == 1:
                return [(.014, telemetry(boot=BOOT - 1, ack_request=request + 1, **status)),
                        (.016, telemetry(epoch=6, ack_request=request + 1, **status))]
            return [(.014, telemetry(ack_request=request, accepted_request_watermark=request, **status))]

        master, port = self.make_master(response)
        original = master.discover()
        self.assertIsNone(master.query())
        self.assertIs(master.latest, original)
        self.assertEqual(master.ignored['unmatched_ack'], 2)
        self.assertFalse(master.failed)
        self.assertEqual(master.query().request, 2)
        self.assertEqual(len(port.writes), 3)

    def test_rejected_reply_keeps_raw_ack_and_uses_watermark(self):
        def response(index, kind, request, task):
            ack_result = 4 if index == 1 else 0
            watermark = 7 if index < 2 else request
            return [(.014, telemetry(publication_sequence=index + 1, publication_tick_ms=1000 + 20 * index,
                                     ack_request=request, ack_result=ack_result,
                                     accepted_request_watermark=watermark))]

        master, _ = self.make_master(response)
        master.discover()
        with self.assertRaises(v11.Rejected) as rejected:
            master.query()
        self.assertEqual(rejected.exception.exchange.status.ack_result, 4)
        self.assertEqual(rejected.exception.exchange.status.accepted_request_watermark, 7)
        self.assertEqual(len(rejected.exception.exchange.reply_hex), 220)
        self.assertFalse(master.failed)
        self.assertEqual(master.query().request, 8)

    def test_upload_accepts_delayed_append_replies_and_serializes_poll(self):
        begin_entered = threading.Event()
        poll_started = threading.Event()

        def response(index, kind, request, task):
            if kind == v11.BEGIN:
                begin_entered.set()
                if not poll_started.wait(1):
                    raise RuntimeError('Poll test did not reach the concurrent call')
            ready = kind == v11.COMMIT
            delay = .900 if kind == v11.APPEND else .014
            return [(delay, telemetry(publication_sequence=index + 1, publication_tick_ms=1000 + 20 * index,
                                     ack_request=request, accepted_request_watermark=request, task=task,
                                     state=v11.READY if ready else 0, count=2 if ready else 0,
                                     flags=v11.ACK_VALID | 3 | (v11.COMMITTED if ready else 0)))]

        master, port = self.make_master(response)
        master.discover()
        poll_result = []

        def poll():
            if begin_entered.wait(1):
                poll_started.set()
                poll_result.append(master.query())

        worker = threading.Thread(target=poll)
        worker.start()
        seen = []
        acks = master.upload({'segments': [{'kind': 'move', 'distance_m': .5},
                                           {'kind': 'turn', 'angle_rad': -1.5}]},
                             on_ack=lambda ack: seen.append((len(port.writes), ack)))
        worker.join(1)
        self.assertEqual(seen, list(zip(range(2, 6), acks)))
        self.assertEqual(acks[-1].status.state, v11.READY)
        self.assertEqual([ack.opcode for ack in acks], [v11.BEGIN, v11.APPEND, v11.APPEND, v11.COMMIT])
        for ack in acks[1:3]:
            self.assertAlmostEqual(ack.received_at - ack.sent_at, .900)
        self.assertEqual([packet[3] for _, packet in port.writes],
                         [v11.QUERY, v11.BEGIN, v11.APPEND, v11.APPEND, v11.COMMIT, v11.QUERY])
        self.assertEqual(len(poll_result), 1)
        self.assertEqual(struct.unpack_from('<HII', port.writes[1][1], 34), (2, 0x615AABE9, 30000))

    def test_upload_does_not_retry_after_opcode_specific_reply_deadline(self):
        for delayed, delay, expected_wait, prefix in (
                (v11.APPEND, 1.001, 1.000, [v11.BEGIN]),
                (v11.COMMIT, .150, .100, [v11.BEGIN, v11.APPEND])):
            with self.subTest(opcode=delayed):
                master, port = self.make_master()

                def response(index, kind, request, task):
                    _, frame = port.regular_response(index, kind, request, task)[0]
                    return [(delay if kind == delayed else .014, frame)]

                port.response = response
                master.discover()
                with self.assertRaises(v11.ReplyTimeout) as failure:
                    master.upload({'segments': [{'kind': 'move', 'distance_m': .5}]})
                details = failure.exception.details
                self.assertEqual(details['opcode'], delayed)
                self.assertEqual([ack['opcode'] for ack in details['accepted_upload_prefix']], prefix)
                self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, *prefix, delayed])
                self.assertAlmostEqual(port.clock() - details['sent_at'], expected_wait, delta=.0021)
                self.assertTrue(master.failed)

    def test_upload_failure_preserves_prior_ack_and_never_sends_commit(self):
        def response(index, kind, request, task):
            ack_result = 1 if kind == v11.APPEND else 0
            return [(.014, telemetry(publication_sequence=index + 1, publication_tick_ms=1000 + 20 * index,
                                     ack_request=request, accepted_request_watermark=request, ack_result=ack_result))]

        master, port = self.make_master(response)
        master.discover()
        with self.assertRaises(v11.Rejected) as rejected:
            master.upload({'segments': [{'kind': 'move', 'distance_m': .5}]})
        previous = rejected.exception.details['accepted_upload_prefix']
        self.assertEqual([ack['opcode'] for ack in previous], [v11.BEGIN])
        self.assertEqual(rejected.exception.exchange.opcode, v11.APPEND)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.BEGIN, v11.APPEND])

    def test_start_requires_exact_ready_task_and_sends_once(self):
        def response(index, kind, request, task):
            return [(.014, telemetry(publication_sequence=index + 1, publication_tick_ms=1000 + 20 * index,
                                     task=TASK, ack_request=request, accepted_request_watermark=request,
                                     state=v11.READY if index == 0 else 3,
                                     flags=v11.ACK_VALID | 3 | v11.COMMITTED | (v11.STARTED if index else 0)))]

        master, port = self.make_master(response)
        master.discover()
        with self.assertRaises(ValueError):
            master.start(TASK + 1)
        self.assertEqual(master.start(TASK).opcode, v11.START)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.START])

    def test_renew_names_current_active_task_and_accepts_the_ack(self):
        def response(index, kind, request, task):
            return [(.014, telemetry(publication_sequence=index + 1, publication_tick_ms=1000 + 20 * index,
                                     task=TASK, ack_request=request, accepted_request_watermark=request, state=4,
                                     flags=v11.ACK_VALID | 3 | v11.EXECUTING | v11.STARTED,
                                     lease_remaining_ms=30000 if index else 4990))]

        master, port = self.make_master(response)
        master.discover()
        with self.assertRaisesRegex(ValueError, 'exact active task'):
            master.renew(TASK + 1)
        reply = master.renew(TASK)
        self.assertEqual(reply.status.lease_remaining_ms, 30000)
        packet = port.writes[-1][1]
        self.assertEqual(packet[:6], b'\xa5\x5a\x0d\x08\x1c\x00')
        self.assertEqual(len(packet), 36)
        self.assertEqual(struct.unpack_from('<QIIIQ', packet, 6), (BOOT, 7, 1, 0, TASK))
        self.assertEqual([frame[3] for _, frame in port.writes], [v11.QUERY, v11.RENEW])

    def test_renew_is_never_retried_after_an_uncertain_ack(self):
        active = telemetry(task=TASK, state=4, flags=v11.ACK_VALID | 3 | v11.EXECUTING,
                           lease_remaining_ms=5000)
        master, port = self.make_master(lambda index, *unused: [(.014, active)] if not index else [])
        master.discover()
        with self.assertRaises(v11.ReplyTimeout):
            master.renew(TASK)
        with self.assertRaises(v11.LinkError):
            master.renew(TASK)
        self.assertTrue(master.failed)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.RENEW])

    def test_stop_ack_and_new_zero_sample_proof_in_cancelled_terminal_state(self):
        def response(index, kind, request, task):
            tick = 1000 + 20 * index
            # Initial STOP ACK reports STOPPING. The next STOP-confirmed status
            # repeats the ACK's sample, then a genuinely later sample proves stop.
            values = dict(publication_sequence=index + 1, publication_tick_ms=tick,
                          ack_request=request, accepted_request_watermark=request)
            if index:
                values.update(first_error=1, state=7 if index == 1 else 9,
                              flags=v11.ACK_VALID | 3 | (v11.EXECUTING if index == 1 else v11.STOP_CONFIRMED),
                              sample_sequence=10 if index < 3 else 11,
                              measurement_tick_ms=1010 if index < 3 else 1050)
            return [(.014, telemetry(**values))]

        master, port = self.make_master(response)
        master.discover()
        seen = []
        result = master.stop(on_reply=lambda reply: seen.append(reply.opcode))
        self.assertEqual(seen, [v11.STOP, v11.QUERY, v11.QUERY])
        self.assertTrue(result['stop_verified'])
        self.assertEqual(result['ack']['status']['state'], 7)
        self.assertEqual(result['proof']['status']['state'], 9)
        self.assertEqual(result['proof']['status']['first_error'], 1)
        self.assertEqual(result['proof']['status']['sample_sequence'], 11)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.STOP, v11.QUERY, v11.QUERY])

    def test_stop_proof_skips_final_query_and_leaves_owner_usable(self):
        master, port = self.make_master()
        master.discover()
        result = master.stop(proof_timeout=.125)
        self.assertFalse(result['stop_verified'])
        self.assertEqual(result['ack']['opcode'], v11.STOP)
        self.assertEqual(result['ack']['status']['ack_result'], 0)
        self.assertEqual(result['error']['code'], 'stop_proof_deadline')
        self.assertFalse(master.failed)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.STOP, v11.QUERY])
        self.assertEqual(master.query().request, 3)

    def test_stop_proof_rechecks_full_reply_window_after_cadence_sleep(self):
        master, port = self.make_master()
        master.discover()
        sleep = master.sleep

        def oversleep_after_stop_ack(reply):
            self.assertEqual(reply.opcode, v11.STOP)
            master.sleep = lambda seconds: sleep(seconds + .010)

        result = master.stop(proof_timeout=.110, on_reply=oversleep_after_stop_ack)
        self.assertFalse(result['stop_verified'])
        self.assertEqual(result['error']['code'], 'stop_proof_deadline')
        self.assertFalse(master.failed)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.STOP])

    def test_stop_proof_missing_query_does_not_poison_owner(self):
        master, port = self.make_master()
        response = port.response
        port.response = lambda index, *args: [] if index == 2 else response(index, *args)
        master.discover()
        result = master.stop(proof_timeout=.225)
        self.assertFalse(result['stop_verified'])
        self.assertEqual(result['ack']['opcode'], v11.STOP)
        self.assertEqual(result['error']['code'], 'stop_proof_deadline')
        self.assertFalse(master.failed)
        self.assertEqual(master.dropped_queries, 1)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.STOP, v11.QUERY, v11.QUERY])
        self.assertEqual([struct.unpack_from('<I', packet, 18)[0] for _, packet in port.writes], [0, 1, 2, 3])
        self.assertEqual(master.query().request, 4)

    def test_cli_defaults_to_offline_and_requires_a_separate_start_id(self):
        with patch.object(v11, 'open_port') as opening, redirect_stdout(StringIO()) as stream:
            self.assertEqual(v11.main(['poll', '--port', '/dev/ttyUSB0']), 0)
            self.assertFalse(json.loads(stream.getvalue())['port_opened'])
            with redirect_stderr(StringIO()), self.assertRaises(SystemExit):
                v11.main(['start', '--execute', '--port', '/dev/ttyUSB0'])
        opening.assert_not_called()

    def test_cli_reports_dropped_query_without_reusing_discovery_sample(self):
        master, _ = self.make_master(lambda index, *unused: [(.014, telemetry())] if not index else [])
        with patch.object(v11, 'open_port'), patch.object(v11, 'RkMaster', return_value=master):
            with redirect_stdout(StringIO()) as stream:
                self.assertEqual(v11.main(['query', '--execute', '--port', '/dev/ttyUSB0']), 0)
        report = json.loads(stream.getvalue())
        self.assertTrue(report['success'])
        self.assertTrue(report['sample_dropped'])
        self.assertIsNone(report['reply'])
        self.assertEqual(report['dropped_queries'], 1)
        self.assertFalse(report['owner_poisoned'])


if __name__ == '__main__':
    unittest.main()
