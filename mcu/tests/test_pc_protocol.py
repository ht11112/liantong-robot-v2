"""Offline only. The serial module is never imported or a port opened."""
from dataclasses import replace
import importlib.util
import json
import math
import os
from pathlib import Path
import struct
import sys
import unittest

PATH = Path(__file__).resolve().parents[1] / 'tools' / 'mcu_v10.py'
spec = importlib.util.spec_from_file_location('mcu_v10_tested', PATH)
pc = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = pc
spec.loader.exec_module(pc)


def sample(pub=10000, seq=9, boot=0x1122334455667788):
    values = (boot, 7, seq, pub, 42, (pub-50) & 0xFFFFFFFF,
              0x0102030405060708, 5, 4, 5, 455, 1, 2, .25,
              1.25, -.5, -1., .02, -.05, 0xFFFFFFFE, 1234, 7, -9, 3,
              0x12345678, 9000, 14, 38, 32)
    return pc.frame(pc.TELEMETRY, struct.pack('<QIIIIIQIBBHHHffffffIIhhHIIHHI', *values))


class CodecTests(unittest.TestCase):
    def test_crc_and_canonical(self):
        self.assertEqual(pc.crc16(b'123456789'), 0x29B1)
        values, crc, duration = pc.canonical_segments({'segments': [
            {'kind': 'move', 'distance_m': .5}, {'kind': 'turn', 'angle_rad': -1.5}]})
        self.assertEqual(b''.join(values), struct.pack('<IfIf', 1, .5, 2, -1.5))
        self.assertEqual(crc, 0x615AABE9)
        self.assertEqual(duration, 1200000)
        self.assertEqual(len(pc.command(pc.BEGIN, 1, 1, 1, 1, 1, struct.pack('<HII', 2, crc, duration))), 46)

    def test_request_reconnect_uses_accepted_watermark_not_reject_ACK(self):
        value = pc.decode_telemetry(sample())
        self.assertEqual(pc.next_request(replace(value, ack_request=1)), 33)
        self.assertEqual(pc.next_request(replace(value, ack_request=0xF0000000)), 33)
        self.assertEqual(pc.next_request(replace(value, accepted_request_watermark=0xFFFFFFFF)), 1)

    def test_reject_bad_segments_and_durations(self):
        for segment in ({'kind': 'move', 'distance_m': 0}, {'kind': 'move', 'distance_m': -1},
                        {'kind': 'turn', 'angle_rad': 0}, {'kind': 'turn', 'angle_rad': 7},
                        {'kind': 'move', 'distance_m': float('nan')}, {'kind': 'move', 'distance_m': True},
                        {'kind': 'move', 'distance_m': 1e-100}, {'kind': 'arc', 'distance_m': 1}):
            with self.subTest(segment=segment), self.assertRaises(ValueError):
                pc.canonical_segments({'segments': [segment]})
        for duration in (0, -1, 1200.001, math.inf, True):
            with self.subTest(duration=duration), self.assertRaises(ValueError):
                pc.canonical_segments({'segments': [{'kind': 'move', 'distance_m': .5}], 'duration_s': duration})

    def test_stream_partial_crc_noise_duplicate_metadata(self):
        raw = sample()
        parser = pc.Parser()
        self.assertEqual(parser.feed(b'noise\xa5\xa5' + raw[:13]), [])
        values = parser.feed(raw[13:])
        self.assertEqual(len(values), 1)
        self.assertEqual(values[0].sample_sequence, 42)
        bad = bytearray(raw)
        bad[60] ^= 1
        self.assertEqual(len(parser.feed(bytes(bad) + raw)), 1)
        values = parser.feed(pc.frame(pc.QUERY, b'\x00'*28) + raw)
        self.assertEqual(len(values), 1)
        invalid = bytearray(raw)
        struct.pack_into('<H', invalid, 100, 8)
        struct.pack_into('<H', invalid, 108, pc.crc16(invalid[2:108]))
        with self.assertRaises(ValueError):
            pc.decode_telemetry(bytes(invalid))
        stale = bytearray(raw)
        struct.pack_into('<I', stale, 30, 9750)
        struct.pack_into('<H', stale, 108, pc.crc16(stale[2:108]))
        with self.assertRaises(ValueError):
            pc.decode_telemetry(bytes(stale))

    def test_real_C_golden_codec(self):
        log = os.environ.get('MQ_CODEC_GOLDEN_LOG')
        if not log:
            self.skipTest('Run mcu/build.py --host-tests for the actual C golden-vector comparison')
        line = next(line for line in Path(log).read_text(encoding='utf-8').splitlines() if line.startswith('GOLDEN'))
        c = json.loads(line[6:])
        self.assertEqual(c['telemetry'], sample().hex())
        telemetry = pc.decode_telemetry(bytes.fromhex(c['telemetry']))
        self.assertEqual(telemetry.boot, 0x1122334455667788)
        self.assertEqual(telemetry.first_drive_error, 0x12345678)
        self.assertEqual(telemetry.measurement_tick_ms, 9950)
        begin = pc.command(pc.BEGIN, telemetry.boot, 7, 33, 9, telemetry.task+1,
                           struct.pack('<HII', 2, 0x615AABE9, 1200000))
        append = pc.command(pc.APPEND, telemetry.boot, 7, 34, 10, telemetry.task+1,
                            struct.pack('<HIf', 1, 2, -1.5))
        self.assertEqual(c['begin'], begin.hex())
        self.assertEqual(c['append'], append.hex())
        self.assertEqual(c['queue_crc32'], 0x615AABE9)


class ClockTests(unittest.TestCase):
    def warm(self, *, maximum=2., tx=2., start=10000):
        budget = pc.TimingBudget(0., maximum, tx, 50., 'offline injected bounds')
        clock = pc.ClockWindow(budget)
        for i in range(5):
            raw_ms = (start + i*50) & 0xFFFFFFFF
            value = pc.decode_telemetry(sample(raw_ms, i+1))
            receipt = (start+i*50)/1000 + 20 + pc.TELEMETRY_SIZE*10/pc.BAUD + .001
            self.assertTrue(clock.observe(value, receipt))
        return clock, value, receipt

    def test_airtime_latency_drift_fits_bounded_window(self):
        clock, value, receipt = self.warm()
        window = clock.send_window(value, receipt, 46)
        self.assertIsNotNone(window)
        self.assertGreater(window[0], receipt)
        self.assertLess(window[1]-window[0], .024)
        # A 46-byte command is fully delivered before close for either offset endpoint.
        end_latest = window[1]-clock.low + .002 + 46*10/pc.BAUD
        self.assertLessEqual(end_latest, clock.unwrapped_ms/1000+.038)
        self.assertIsNone(clock.send_window(value, receipt+.2, 46))

    def test_uncertainty_refuses_and_stop_tail_is_bounded(self):
        clock, value, receipt = self.warm(maximum=25., tx=8.)
        self.assertIsNone(clock.send_window(value, receipt, 46))
        stop_window = clock.send_window(value, receipt, 36, True)
        # The quiet tail is only ten extra ms, not unlimited STOP transmission.
        if stop_window is not None:
            self.assertLess(stop_window[1]-clock.low, clock.unwrapped_ms/1000+.048)

    def test_source_ticks_wrap_boot_duplicates_and_delay_violation(self):
        clock, value, receipt = self.warm(start=0xFFFFFF90)
        self.assertGreater(clock.unwrapped_ms, 0xFFFFFFFF)
        self.assertFalse(clock.observe(value, receipt))
        delayed = pc.decode_telemetry(sample((value.publication_tick_ms+50)&0xFFFFFFFF, 6))
        self.assertFalse(clock.observe(delayed, receipt+.15))
        self.assertEqual(clock.good, 0)
        reboot = pc.decode_telemetry(sample(0, 1, 99))
        clock.observe(reboot, receipt+.20)
        self.assertEqual(clock.good, 1)
        self.assertIsNone(clock.send_window(reboot, receipt+.20, 36))

    def test_default_client_refuses_writes_without_timing_budget(self):
        class NoPort:
            def write(self, data):
                raise AssertionError('Must never write')
        with self.assertRaises(RuntimeError):
            pc.SerialClient(NoPort()).request(pc.START, 1)


if __name__ == '__main__':
    unittest.main()
