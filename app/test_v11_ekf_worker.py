"""Exercise the real EKF worker process and feedback cache without ROS or devices."""

from dataclasses import replace
from functools import partial
import multiprocessing as mp
import time
from types import SimpleNamespace
import unittest

from test_v12_feedback_worker_unit import exchange
from v12_feedback_worker import SharedFeedbackCache
from v11_ekf_worker import EkfWorker, _SharedSnapshots


class RecordingAdapter:
    """A spawned ROS-boundary fixture with source ages and observable lifecycle."""

    def __init__(self, signals, imu_topic, executable=None):
        self.signals = signals
        self.source_at = None
        self.request = None
        signals.ready.set()

    def publish(self, frame):
        self.signals.published.value += 1
        self.signals.last_request.value = frame.request
        if not frame.status.source_valid:
            self.source_at = None
        elif self.request != frame.request:
            self.source_at = frame.sent_at
        self.request = frame.request

    def invalidate_history(self):
        self.signals.invalidated.value += 1
        self.source_at = None

    def imu_snapshot(self):
        if self.source_at is None:
            return None
        age_ms = (time.monotonic() - self.source_at) * 1000
        return dict(fresh=0 <= age_ms <= 250., age_ms=age_ms,
                    angular_velocity_rad_s=dict(x=0., y=0., z=self.request / 100.))

    def ekf_snapshot(self):
        source = self.imu_snapshot()
        if source is None:
            return None
        return dict(fresh=source['fresh'], age_ms=source['age_ms'],
                    yaw_rad=self.request / 100. if source['fresh'] else None)

    def worker_snapshot(self):
        valid_until = self.source_at + .25 if self.source_at is not None else 0.
        return dict(written_mono=time.monotonic(), imu=self.imu_snapshot(), ekf=self.ekf_snapshot(),
                    imu_valid_until_mono=valid_until, ekf_valid_until_mono=valid_until)

    def close(self):
        self.signals.closed.set()


class EkfWorkerTests(unittest.TestCase):
    def setUp(self):
        self.context = mp.get_context('spawn')
        self.signals = SimpleNamespace(
            published=self.context.RawValue('I', 0),
            last_request=self.context.RawValue('I', 0),
            invalidated=self.context.RawValue('I', 0),
            ready=self.context.Event(), closed=self.context.Event())

    def launch(self, cache=None):
        cache = cache or SharedFeedbackCache(self.context)
        with cache.lock:
            cache.connected.value = True
        worker = EkfWorker('/test/imu', context=self.context,
                           adapter_factory=partial(RecordingAdapter, self.signals))
        worker.launch(cache)
        self.addCleanup(worker.close)
        self.assertTrue(self.signals.ready.wait(4.))
        return worker, cache

    def wait_for(self, condition):
        deadline = time.monotonic() + 2.
        while not condition() and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertTrue(condition())

    def publish(self, cache, request, *, valid=True):
        now = time.monotonic()
        frame = replace(exchange(request, flags=259 if valid else 257),
                        sent_at=now, received_at=now)
        cache.publish(frame)
        return frame

    def test_child_consumes_cache_without_parent_publish(self):
        worker, cache = self.launch()
        self.publish(cache, 1)
        self.wait_for(lambda: self.signals.published.value == 1)
        self.wait_for(lambda: bool((worker.ekf_snapshot() or {}).get('fresh')))
        self.assertAlmostEqual(worker.ekf_snapshot()['yaw_rad'], .01)
        self.assertAlmostEqual(worker.imu_snapshot()['angular_velocity_rad_s']['z'], .01)
        worker.publish(exchange(100))
        time.sleep(.03)
        self.assertEqual(self.signals.published.value, 1)
        self.assertEqual(cache.snapshot()['cursor'], 1)

    def test_repeated_reads_cannot_keep_old_sources_fresh(self):
        worker, cache = self.launch()
        self.publish(cache, 1)
        self.wait_for(lambda: bool((worker.ekf_snapshot() or {}).get('fresh')))
        earlier = worker.ekf_snapshot()['age_ms']
        time.sleep(.10)
        self.assertGreater(worker.ekf_snapshot()['age_ms'], earlier + 50.)
        self.wait_for(lambda: not (worker.ekf_snapshot() or {}).get('fresh', False))
        self.assertFalse((worker.imu_snapshot() or {}).get('fresh', False))
        self.assertIsNone((worker.ekf_snapshot() or {}).get('yaw_rad'))
        self.assertTrue(worker.process.is_alive())

    def test_exited_child_immediately_invalidates_published_snapshot(self):
        worker, cache = self.launch()
        self.publish(cache, 1)
        self.wait_for(lambda: bool((worker.ekf_snapshot() or {}).get('fresh')))
        worker.process.terminate()
        worker.process.join(2.)
        self.assertFalse(worker.process.is_alive())
        self.assertFalse((worker.ekf_snapshot() or {}).get('fresh', False))
        self.assertFalse((worker.imu_snapshot() or {}).get('fresh', False))

    def test_cache_overflow_invalidates_before_delivering_remaining_history(self):
        cache = SharedFeedbackCache(self.context, capacity=2)
        for request in range(1, 6):
            self.publish(cache, request)
        worker, _ = self.launch(cache)
        self.wait_for(lambda: self.signals.last_request.value == 5)
        self.assertEqual(self.signals.published.value, 2)
        self.assertGreaterEqual(self.signals.invalidated.value, 1)

    def test_parent_invalidation_is_consumed_and_close_releases_adapter(self):
        worker, cache = self.launch()
        self.publish(cache, 1)
        self.wait_for(lambda: bool((worker.ekf_snapshot() or {}).get('fresh')))
        worker.invalidate_history()
        self.assertFalse((worker.ekf_snapshot() or {}).get('fresh', False))
        self.wait_for(lambda: self.signals.invalidated.value >= 1)
        self.publish(cache, 2, valid=False)
        self.wait_for(lambda: self.signals.last_request.value == 2)
        self.assertFalse((worker.ekf_snapshot() or {}).get('fresh', False))
        worker.close()
        self.assertTrue(self.signals.closed.wait(1.))
        self.assertFalse(worker.process.is_alive())

    def test_invalidated_revision_rejects_an_older_inflight_snapshot(self):
        snapshots = _SharedSnapshots(self.context)
        revision, _ = snapshots.read()
        record = dict(written_mono=time.monotonic(), imu=None, ekf=None,
                      imu_valid_until_mono=0., ekf_valid_until_mono=0.)
        self.assertTrue(snapshots.write(record, revision))
        self.assertEqual(snapshots.read(), (revision, record))
        current = snapshots.invalidate()
        self.assertGreater(current, revision)
        self.assertFalse(snapshots.write(record, revision))
        self.assertEqual(snapshots.read(), (current, None))

    def test_serial_cache_disconnect_closes_child_and_invalidates_estimate(self):
        worker, cache = self.launch()
        self.publish(cache, 1)
        self.wait_for(lambda: bool((worker.ekf_snapshot() or {}).get('fresh')))
        cache.disconnect(dict(type='LinkError', message='test serial disconnect'))
        worker.process.join(2.)
        self.assertFalse(worker.process.is_alive())
        self.assertTrue(self.signals.closed.is_set())
        self.assertFalse((worker.ekf_snapshot() or {}).get('fresh', False))
        _, record = worker.snapshots.read()
        self.assertEqual(record['error'], 'test serial disconnect')


if __name__ == '__main__':
    unittest.main()
