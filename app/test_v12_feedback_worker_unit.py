"""Shared-memory feedback storage, without opening ports or starting devices."""
from dataclasses import asdict
import struct
import unittest

from v12_feedback_worker import SharedFeedbackCache
from mcu_v11 import Exchange, Telemetry, HEADER, STATUS, QUERY, command, crc16


def exchange(sequence, sample=None, flags=259):
    fields = dict.fromkeys(Telemetry.__dataclass_fields__, 0)
    fields.update(boot=8, epoch=1, publication_sequence=sequence,
                  publication_tick_ms=1000 + sequence * 20,
                  sample_sequence=sequence if sample is None else sample,
                  measurement_tick_ms=1000 + sequence * 20, ack_request=sequence,
                  accepted_request_watermark=sequence, flags=flags)
    status = Telemetry(**fields)
    body = HEADER[2:] + STATUS.pack(*asdict(status).values())
    raw = b'\xa5\x5a' + body + struct.pack('<H', crc16(body))
    return Exchange(QUERY, sequence, 10. + sequence * .020, 10.014 + sequence * .020,
                    status, command(QUERY, 8, 1, sequence).hex(), raw.hex())


class SharedFeedbackTests(unittest.TestCase):
    def test_history_and_latest_keep_original_times_and_invalid_samples(self):
        cache = SharedFeedbackCache(capacity=3)
        frames = [exchange(1), exchange(2, flags=257), exchange(3)]
        for frame in frames:
            cache.publish(frame)
        self.assertEqual(cache.snapshot()['latest'], frames[-1])
        history = cache.history_since(0)
        self.assertEqual(history['exchanges'], frames)
        self.assertEqual(history['missed'], 0)
        self.assertFalse(history['exchanges'][1].status.source_valid)
        self.assertEqual(cache.history_since(3)['exchanges'], [])

    def test_ring_coverage_reports_loss_not_source_rewrite(self):
        cache = SharedFeedbackCache(capacity=3)
        for sequence in range(1, 6):
            cache.publish(exchange(sequence))
            cache.record_send(QUERY, sequence, 10. + sequence * .02)
        history = cache.history_since(0)
        self.assertEqual((history['cursor'], history['missed']), (5, 2))
        self.assertEqual([e.request for e in history['exchanges']], [3, 4, 5])
        self.assertEqual(cache.history_since(3)['missed'], 0)
        tx = cache.transmissions_since(0)
        self.assertEqual((tx['cursor'], tx['missed']), (5, 2))
        self.assertEqual([e['request'] for e in tx['entries']], [3, 4, 5])

    def test_duplicate_sources_and_failed_queries_have_separate_counters(self):
        cache = SharedFeedbackCache()
        for sequence in range(1, 6):
            cache.record_send(QUERY, sequence, 10. + sequence * .02)
            if sequence != 4:
                cache.publish(exchange(sequence, sample=1 if sequence < 3 else 2))
        cache.set_dropped(1)
        report = cache.diagnostics()
        self.assertEqual((report['sent_count'], report['reply_count'], report['dropped_queries']), (5, 4, 1))
        self.assertAlmostEqual(report['send_rate_hz'], 50.)
        self.assertAlmostEqual(report['real_sample_rate_hz'], 25.)
        self.assertAlmostEqual(report['reply_latency_ms']['median'], 14.)
        self.assertAlmostEqual(report['max_send_gap_ms'], 20.)

    def test_disconnect_retains_last_frame_without_claiming_connected(self):
        cache = SharedFeedbackCache()
        frame = exchange(1)
        cache.publish(frame)
        error = dict(type='LinkError', message='test loss')
        cache.disconnect(error)
        state = cache.snapshot()
        self.assertEqual(state['latest'], frame)
        self.assertFalse(state['connected'])
        self.assertEqual(state['error'], error)

    def test_disconnect_preserves_long_128_segment_upload_failure(self):
        cache = SharedFeedbackCache()
        error = dict(type='LinkError', message='COMMIT reply missing',
                     details=dict(accepted_upload_prefix=[asdict(exchange(n)) for n in range(1, 130)]))
        cache.disconnect(error)
        self.assertEqual(cache.snapshot()['error'], error)


if __name__ == '__main__':
    unittest.main()
