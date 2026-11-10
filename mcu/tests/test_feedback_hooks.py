"""The worker's hooks use the real master and do not change wire semantics."""
from contextlib import contextmanager
import unittest

from test_rk_master import FakeClock, FakePort, TASK, telemetry
import mcu_v11 as v12


class FeedbackHookTests(unittest.TestCase):
    def test_callbacks_include_every_sent_attempt_and_matched_reply(self):
        clock = FakeClock()
        sent, replies = [], []
        port = FakePort(clock)
        master = v12.RkMaster(port, clock=clock, sleep=clock.sleep,
                              on_send=lambda *args: sent.append(args), on_reply=replies.append)
        master.discover()
        master.query()
        self.assertEqual(len(sent), 2)
        self.assertEqual(len(replies), 2)
        self.assertEqual(sent[1], (v12.QUERY, replies[1].request, replies[1].sent_at))
        self.assertAlmostEqual(sent[1][2] - sent[0][2], .020)
        self.assertEqual(replies[-1], master.latest)

    def test_dropped_query_is_still_a_transmission_not_a_reply(self):
        clock = FakeClock()
        port = FakePort(clock)
        original = port.response
        port.response = lambda index, *args: original(index, *args) if index == 0 else []
        sent, replies = [], []
        master = v12.RkMaster(port, clock=clock, sleep=clock.sleep,
                              on_send=lambda *args: sent.append(args), on_reply=replies.append)
        master.discover()
        self.assertIsNone(master.query())
        self.assertEqual((len(sent), len(replies), master.dropped_queries), (2, 1, 1))

    def test_stop_during_cadence_prevents_actual_start_write(self):
        clock = FakeClock()
        cancelled = [False]

        def response(index, kind, request, task):
            tick = 1000 + index * 20
            return [(.014, telemetry(publication_sequence=index + 1, publication_tick_ms=tick,
                                     sample_sequence=index + 1, measurement_tick_ms=tick - 5,
                                     ack_request=request, accepted_request_watermark=request,
                                     task=TASK, state=v12.READY, flags=3 | v12.ACK_VALID | v12.COMMITTED))]

        @contextmanager
        def guard(kind):
            if kind == v12.START and cancelled[0]:
                raise v12.RequestCancelled('STOP accepted while waiting for cadence')
            yield

        def sleep(seconds):
            cancelled[0] = True
            clock.sleep(seconds)

        port = FakePort(clock, response)
        master = v12.RkMaster(port, clock=clock, sleep=sleep, send_guard=guard)
        master.discover()
        with self.assertRaises(v12.RequestCancelled):
            master.start(TASK)
        self.assertEqual(len(port.writes), 1)
        self.assertFalse(master.failed)
        self.assertIsNotNone(master.query())

    def test_rejected_reply_is_published_before_error_return(self):
        clock = FakeClock()
        port = FakePort(clock)
        original = port.response

        def response(index, kind, request, task):
            if index == 0:
                return original(index, kind, request, task)
            return [(.014, telemetry(publication_sequence=2, publication_tick_ms=1020,
                                     measurement_tick_ms=1015, sample_sequence=2,
                                     ack_request=request, accepted_request_watermark=request, ack_result=1))]

        port.response = response
        replies = []
        master = v12.RkMaster(port, clock=clock, sleep=clock.sleep, on_reply=replies.append)
        master.discover()
        with self.assertRaises(v12.Rejected):
            master.query()
        self.assertEqual(len(replies), 2)
        self.assertEqual(replies[-1].status.ack_result, 1)
        self.assertFalse(master.failed)


if __name__ == '__main__':
    unittest.main()
