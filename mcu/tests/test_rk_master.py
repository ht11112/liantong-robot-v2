"""Offline tests for V11 codec, sequencing, polling and operator boundaries."""

from contextlib import redirect_stderr, redirect_stdout
from io import StringIO
import json
import os
from pathlib import Path
import struct
import sys
import threading
import unittest
from unittest.mock import patch


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import mcu_v11 as v11


BOOT = 0x1122334455667788
TASK = 0x0102030405060708
PAYLOAD = ('boot epoch publication_sequence publication_tick_ms sample_sequence measurement_tick_ms '
           'task ack_request ack_result state flags index count progress x y yaw v w position_left '
           'position_right left_tenths_rpm right_tenths_rpm first_error first_drive_error first_error_tick_ms '
           'reserved_100 reserved_102 accepted_request_watermark').split()


def telemetry(**changes):
    values = dict.fromkeys(PAYLOAD, 0)
    values.update(boot=BOOT, epoch=7, publication_sequence=1, publication_tick_ms=1000,
                  sample_sequence=1, measurement_tick_ms=995, flags=v11.ACK_VALID | 3)
    values.update(changes)
    body = v11.HEADER[2:] + struct.pack('<QIIIIIQIBBHHHffffffIIhhHIIHHI', *(values[key] for key in PAYLOAD))
    return b'\xa5\x5a' + body + struct.pack('<H', v11.crc16(body))


def golden_reply():
    return telemetry(publication_sequence=9, publication_tick_ms=10000, sample_sequence=42,
                     measurement_tick_ms=9950, task=TASK, ack_request=5, ack_result=4,
                     state=5, flags=455, index=1, count=2, progress=.25, x=1.25, y=-.5,
                     yaw=-1., v=.02, w=-.05, position_left=0xFFFFFFFE, position_right=1234,
                     left_tenths_rpm=7, right_tenths_rpm=-9, first_error=3,
                     first_drive_error=0x12345678, first_error_tick_ms=9000, accepted_request_watermark=32)


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
        self.assertEqual(discovery[:6], b'\xa5\x5a\x0b\x07\x1c\x00')
        self.assertEqual(discovery[6:34], bytes(28))
        self.assertEqual(len(discovery), 36)
        self.assertEqual(v11.crc16(discovery[2:-2]), int.from_bytes(discovery[-2:], 'little'))
        request = v11.command(v11.BEGIN, BOOT, 7, 33, TASK, struct.pack('<HII', 2, 0x615AABE9, 1200000))
        self.assertEqual(len(request), 46)
        self.assertEqual(struct.unpack_from('<QIIIQ', request, 6), (BOOT, 7, 33, 0, TASK))
        with self.assertRaises(ValueError):
            v11.command(v11.START)
        with self.assertRaises(ValueError):
            v11.command(v11.QUERY, boot=BOOT)

    def test_real_c_golden(self):
        log = os.environ.get('MQ_V11_CODEC_GOLDEN_LOG')
        if not log:
            self.skipTest('Run the host C build to compare the actual V11 golden vector')
        lines = Path(log).read_text(encoding='utf-8').splitlines()
        line = next(line for line in lines if line.startswith('GOLDEN'))
        golden = json.loads(line.partition(' ')[2] if line.startswith('GOLDEN ') else line[line.index('{'):])
        self.assertEqual(golden['telemetry'], golden_reply().hex())
        status = v11.decode_telemetry(bytes.fromhex(golden['telemetry']))
        self.assertEqual(golden['query'], v11.command(v11.QUERY).hex())
        self.assertEqual(golden['begin'], v11.command(v11.BEGIN, status.boot, 7, 33, status.task + 1,
                                                    struct.pack('<HII', 2, 0x615AABE9, 1200000)).hex())
        self.assertEqual(golden['append'], v11.command(v11.APPEND, status.boot, 7, 34, status.task + 1,
                                                      struct.pack('<HIf', 1, 2, -1.5)).hex())
        self.assertEqual(golden['queue_crc32'], 0x615AABE9)

    def test_fragmented_noise_crc_metadata_and_legacy_rejection(self):
        valid = telemetry()
        bad_crc = bytearray(valid)
        bad_crc[50] ^= 1
        old = bytearray(valid)
        old[2] = 10
        parser = v11.ReplyParser()
        self.assertEqual(list(parser.feed(b'noise' + old + bad_crc + telemetry(reserved_102=1) + valid[:50])), [])
        outputs = list(parser.feed(valid[50:]))
        self.assertEqual(outputs, [(v11.decode_telemetry(valid), valid)])
        self.assertEqual(parser.errors['invalid_headers'], 1)
        self.assertEqual(parser.errors['crc_errors'], 1)
        self.assertEqual(parser.errors['invalid_metadata'], 1)
        with self.assertRaises(ValueError):
            v11.decode_telemetry(telemetry(measurement_tick_ms=750))


class MasterTests(unittest.TestCase):
    def make_master(self, response=None):
        clock = FakeClock()
        port = FakePort(clock, response)
        master = v11.RkMaster(port, clock=clock, sleep=clock.sleep)
        return master, port

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

    def test_regular_query_retries_same_request_after_a_lost_or_delayed_exchange(self):
        for loss in ('request', 'reply', 'delayed_reply'):
            with self.subTest(loss=loss):
                accepted = []

                def response(index, kind, request, task):
                    if index == 0:
                        return [(.014, telemetry(accepted_request_watermark=31))]
                    if index == 1 and loss == 'request':
                        return []
                    if not accepted or accepted[-1] != request:
                        accepted.append(request)
                    if (index == 1 and loss == 'reply') or (index == 2 and loss == 'delayed_reply'):
                        return []
                    delay = .115 if index == 1 else .014
                    return [(delay, telemetry(publication_sequence=index + 1,
                                              publication_tick_ms=1000 + 20 * index,
                                              ack_request=request, accepted_request_watermark=request))]

                master, port = self.make_master(response)
                master.discover()
                reply = master.query()
                self.assertEqual(reply.request, 32)
                self.assertEqual(reply.sent_at, port.writes[1][0])
                self.assertEqual(port.writes[2][1], port.writes[1][1])
                self.assertGreater(reply.received_at, port.writes[2][0])
                self.assertEqual(master.query_retries, 1)
                self.assertEqual(master.last_query_retry['details']['request'], 32)
                self.assertFalse(master.failed)
                self.assertEqual(master.query().request, 33)
                self.assertEqual(accepted, [32, 33])

    def test_two_regular_query_timeouts_close_the_owner(self):
        master, port = self.make_master(lambda index, *unused: [(.014, telemetry())] if not index else [])
        master.discover()
        with self.assertRaises(v11.ReplyTimeout) as timed_out:
            master.query()
        details = timed_out.exception.details
        self.assertEqual(details['request'], details['first_query_attempt']['details']['request'])
        self.assertEqual(master.query_retries, 1)
        self.assertTrue(master.failed)
        with self.assertRaises(v11.LinkError):
            master.query()
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.QUERY, v11.QUERY])

    def test_short_regular_query_write_closes_without_retry(self):
        master, port = self.make_master()
        master.discover()
        port.short_write = True
        with self.assertRaisesRegex(v11.LinkError, 'Partial serial write'):
            master.query()
        self.assertEqual(len(port.writes), 2)
        self.assertEqual(master.query_retries, 0)
        self.assertTrue(master.failed)

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
                self.assertEqual(master.query_retries, 0)
                self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, opcode])

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

    def test_upload_reaches_ready_without_implicit_start_and_serializes_poll(self):
        begin_entered = threading.Event()
        poll_started = threading.Event()

        def response(index, kind, request, task):
            if kind == v11.BEGIN:
                begin_entered.set()
                if not poll_started.wait(1):
                    raise RuntimeError('Poll test did not reach the concurrent call')
            ready = kind == v11.COMMIT
            return [(.014, telemetry(publication_sequence=index + 1, publication_tick_ms=1000 + 20 * index,
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
        self.assertEqual([packet[3] for _, packet in port.writes],
                         [v11.QUERY, v11.BEGIN, v11.APPEND, v11.APPEND, v11.COMMIT, v11.QUERY])
        self.assertEqual(len(poll_result), 1)
        self.assertEqual(struct.unpack_from('<HII', port.writes[1][1], 34), (2, 0x615AABE9, 1200000))

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

    def test_stop_ack_preserved_when_later_proof_times_out(self):
        master, port = self.make_master()
        response = port.response
        port.response = lambda index, *args: response(index, *args) if index <= 1 else []
        master.discover()
        result = master.stop(proof_timeout=.055)
        self.assertFalse(result['stop_verified'])
        self.assertEqual(result['ack']['opcode'], v11.STOP)
        self.assertEqual(result['ack']['status']['ack_result'], 0)
        self.assertTrue(master.failed)
        self.assertEqual([packet[3] for _, packet in port.writes], [v11.QUERY, v11.STOP, v11.QUERY])

    def test_cli_defaults_to_offline_and_requires_a_separate_start_id(self):
        with patch.object(v11, 'open_port') as opening, redirect_stdout(StringIO()) as stream:
            self.assertEqual(v11.main(['poll', '--port', '/dev/ttyUSB0']), 0)
            self.assertFalse(json.loads(stream.getvalue())['port_opened'])
            with redirect_stderr(StringIO()), self.assertRaises(SystemExit):
                v11.main(['start', '--execute', '--port', '/dev/ttyUSB0'])
        opening.assert_not_called()


if __name__ == '__main__':
    unittest.main()
