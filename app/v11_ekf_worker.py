"""Independent ROS EKF consumer of the existing read-only MCU feedback cache."""

import multiprocessing as mp
import pickle
import time

from v11_ekf import RosEkfOdometry, pose_at_stamp


class _StopFlag:
    """One parent writer; a terminated child cannot strand an Event lock."""
    def __init__(self, ctx):
        self.value = ctx.RawValue('b', False)

    def set(self):
        self.value.value = True

    def is_set(self):
        return bool(self.value.value)

    def wait(self, timeout):
        if not self.is_set():
            time.sleep(timeout)
        return self.is_set()


class _SharedSnapshots:
    def __init__(self, ctx, capacity=8192):
        self.data = ctx.RawArray('B', capacity)
        self.length = ctx.RawValue('I', 0)
        self.revision = ctx.RawValue('Q', 0)
        self.lock = ctx.Lock()

    def write(self, payload, revision):
        raw = pickle.dumps(payload, protocol=4)
        if len(raw) > len(self.data):
            raise ValueError('EKF snapshot exceeds the bounded mailbox')
        with self.lock:
            if revision != self.revision.value:
                return False
            memoryview(self.data).cast('B')[:len(raw)] = raw
            self.length.value = len(raw)
        return True

    def read(self):
        # A dead child cannot leave an HTTP or STOP caller waiting on its lock.
        if not self.lock.acquire(timeout=.01):
            return None, None
        try:
            revision = self.revision.value
            raw = bytes(memoryview(self.data).cast('B')[:self.length.value])
        finally:
            self.lock.release()
        return revision, pickle.loads(raw) if raw else None

    def invalidate(self):
        if not self.lock.acquire(timeout=.01):
            raise RuntimeError('EKF snapshot lock unavailable')
        try:
            self.revision.value += 1
            self.length.value = 0
            return self.revision.value
        finally:
            self.lock.release()


def _worker_main(imu_topic, executable, cache, snapshots, closed, adapter_factory):
    adapter = None
    cursor = missed = 0
    revision = 0
    error = None
    try:
        adapter = adapter_factory(imu_topic, executable)
        while not closed.is_set():
            parent = mp.parent_process()
            if parent is not None and not parent.is_alive():
                break
            requested, _ = snapshots.read()
            if requested is None:
                continue
            if requested != revision:
                adapter.invalidate_history()
                revision = requested
            batch = cache.history_since(cursor)
            if not batch['connected']:
                error = (batch['error'] or {}).get('message', '底盘反馈进程已断开')
                break
            if batch['missed']:
                missed += batch['missed']
                adapter.invalidate_history()
            for exchange in batch['exchanges']:
                adapter.publish(exchange)
            cursor = batch['cursor']
            snapshot = adapter.worker_snapshot()
            snapshots.write(dict(snapshot, cursor=cursor, missed=missed, error=None), revision)
            closed.wait(.01)
    except Exception as exc:
        error = '{}: {}'.format(type(exc).__name__, exc)
    finally:
        try:
            if adapter is not None:
                adapter.close()
        finally:
            revision, _ = snapshots.read()
            if revision is not None:
                snapshots.write(dict(written_mono=time.monotonic(), imu=None, ekf=None,
                                     imu_valid_until_mono=0., ekf_valid_until_mono=0.,
                                     cursor=cursor, missed=missed, error=error), revision)


class EkfWorker:
    """Navigation-side snapshots; the child has no serial or command capability."""
    def __init__(self, imu_topic, executable=None, *, context=None, adapter_factory=None):
        self.context = context or mp.get_context('spawn')
        self.imu_topic, self.executable = imu_topic, executable
        self.adapter_factory = adapter_factory or RosEkfOdometry
        self.snapshots = _SharedSnapshots(self.context)
        self.closed = _StopFlag(self.context)
        self.process = None

    def launch(self, cache):
        if self.process is not None:
            raise RuntimeError('EKF worker has already been launched')
        self.process = self.context.Process(target=_worker_main,
            args=(self.imu_topic, self.executable, cache, self.snapshots, self.closed, self.adapter_factory),
            name='v12-ekf-ros')
        self.process.start()

    def publish(self, exchange):
        # The parent may retain its adapter interface, but never feeds the ROS
        # estimator through the navigation/map thread again.
        return None

    def invalidate_history(self):
        try:
            self.snapshots.invalidate()
        except RuntimeError:
            self.closed.set()
            raise

    def _snapshot(self, kind, include_history=False):
        _, record = self.snapshots.read()
        if record is None or record[kind] is None:
            return None
        value = dict(record[kind])
        now = time.monotonic()
        elapsed = now - record['written_mono']
        if value['age_ms'] is not None and elapsed >= 0:
            value['age_ms'] += elapsed * 1000.
        value['fresh'] = bool(value['fresh'] and self.process is not None and self.process.is_alive()
                              and not self.closed.is_set() and 0 <= elapsed
                              and now <= record[kind + '_valid_until_mono'])
        if kind == 'ekf' and not value['fresh']:
            value['yaw_rad'] = None
            value['pose'] = value['angular_z_rad_s'] = None
            value['linear_x_m_s'] = None
        if include_history:
            value['pose_history'] = record.get('pose_history', ())
        return value

    def imu_snapshot(self):
        return self._snapshot('imu')

    def ekf_snapshot(self):
        return self._snapshot('ekf')

    def pose_at(self, stamp_ns):
        snapshot = self._snapshot('ekf', include_history=True)
        if snapshot is None or not snapshot['fresh']:
            return None
        return pose_at_stamp(snapshot['pose_history'], stamp_ns)

    def close(self):
        self.closed.set()
        if self.process is not None:
            self.process.join(timeout=6.)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2.)
