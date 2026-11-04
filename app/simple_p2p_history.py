"""Bounded history of already integrated poses; no IO or RPM integration.

The optional measurement_monotonic_s is the interpolation axis. By default it
is source_monotonic_s. These clocks must be aligned by the caller; this module
does not synchronize clocks. Freshness always uses the unmodified original
source_monotonic_s, which may be a conservative lower bound and may regress
between different samples when a separate measurement axis is supplied.

Only the current continuous epoch/segment is retained. Missing measurements
are never recovered, and queries outside the observed interval never predict.
"""
from collections import deque
from collections.abc import Mapping
import math
from threading import RLock


class PoseHistoryError(ValueError):
    pass


def _finite(value, name):
    try:
        valid = type(value) in (int, float) and math.isfinite(value)
    except (OverflowError, ValueError, TypeError):
        valid = False
    if not valid:
        raise PoseHistoryError(name + ' must be a finite number')
    return float(value)


def _time(value, name):
    value = _finite(value, name)
    if value < 0:
        raise PoseHistoryError(name + ' must be nonnegative')
    return value


def _epoch(value):
    if value is not None and (not isinstance(value, str) or not value):
        raise PoseHistoryError('epoch must be a nonempty string or None')
    return value


def _measurement_time(pose):
    return pose.get('measurement_monotonic_s', pose['source_monotonic_s'])


def _copy_pose(pose, measurement_monotonic_s):
    if not isinstance(pose, Mapping):
        raise PoseHistoryError('pose must be a flat mapping of observed scalars')
    result = dict(pose)
    for key, value in result.items():
        if not isinstance(key, str) or type(value) not in (int, float, bool, str, type(None)):
            raise PoseHistoryError('pose fields must be named immutable scalars')
        if type(value) in (int, float):
            _finite(value, key)
    for key in ('x', 'y', 'yaw', 'source_monotonic_s', 'sample_sequence'):
        if key not in result:
            raise PoseHistoryError('missing pose field: ' + key)
    for key in ('x', 'y', 'yaw'):
        _finite(result[key], key)
    _time(result['source_monotonic_s'], 'source_monotonic_s')
    sequence = result['sample_sequence']
    if type(sequence) is not int or not 0 <= sequence <= 0xffffffff:
        raise PoseHistoryError('sample_sequence must be a uint32 observed identity')
    if 'discontinuity' in result and type(result['discontinuity']) is not bool:
        raise PoseHistoryError('discontinuity must be boolean')
    if 'measurement_monotonic_s' in result:
        _time(result['measurement_monotonic_s'], 'measurement_monotonic_s')
    if measurement_monotonic_s is not None:
        _time(measurement_monotonic_s, 'measurement_monotonic_s')
        if ('measurement_monotonic_s' in result
                and result['measurement_monotonic_s'] != measurement_monotonic_s):
            raise PoseHistoryError('conflicting measurement timestamps')
        result['measurement_monotonic_s'] = measurement_monotonic_s
    return result


class PoseHistory:
    """Owned flat pose copies, bounded by capacity (2..4096 samples).

    push returns False only for an identical duplicate and cannot renew its
    source. Rewritten identities, regressing measurement times and backwards
    sequence numbers raise. Explicit/core gaps and forward sequence skips cut
    the segment. A history cut is not permission to resume a motion task.

    reset/invalidate clear interpolation data while retaining the last identity
    and measurement-time fence in the same epoch. Only an explicit new epoch
    permits a restarted clock/sequence. epoch=None in methods keeps the epoch.
    """
    def __init__(self, capacity=256, epoch=None):
        if type(capacity) is not int or not 2 <= capacity <= 4096:
            raise PoseHistoryError('capacity must be an integer from 2 to 4096')
        self._capacity = capacity
        self._epoch = _epoch(epoch)
        self._poses = deque(maxlen=capacity)
        self._last = None
        self._segment = 0
        self._accepted = self._duplicates = self._gaps = 0
        self._lock = RLock()

    def push(self, observation, *, epoch=None, gap=False, measurement_monotonic_s=None):
        """Store an observed accumulated pose, not raw wheel measurements."""
        proposed_epoch = _epoch(epoch)
        if type(gap) is not bool:
            raise PoseHistoryError('gap must be boolean')
        pose = _copy_pose(observation, measurement_monotonic_s)
        measurement_time = _measurement_time(pose)
        with self._lock:
            new_epoch = proposed_epoch is not None and proposed_epoch != self._epoch
            previous = None if new_epoch else self._last
            step = 1
            if previous is not None:
                if pose['sample_sequence'] == previous['sample_sequence']:
                    if pose == previous:
                        self._duplicates += 1
                        return False
                    raise PoseHistoryError('a sample identity cannot be rewritten or renewed')
                if measurement_time <= _measurement_time(previous):
                    raise PoseHistoryError('measurement time must strictly increase')
                step = (pose['sample_sequence'] - previous['sample_sequence']) & 0xffffffff
                if step >= 0x80000000:
                    raise PoseHistoryError('sample_sequence moved backwards')
            cut = gap or pose.get('discontinuity', False) or step != 1
            if new_epoch or cut:
                self._poses.clear()
                self._segment += 1
                if cut:
                    self._gaps += 1
            if new_epoch:
                self._epoch = proposed_epoch
            self._poses.append(pose)
            self._last = pose
            self._accepted += 1
            return True

    def latest(self, now, max_age=.75):
        """Return the latest true sample copy, or raise if its source is stale."""
        now = _time(now, 'now')
        max_age = _finite(max_age, 'max_age')
        if max_age <= 0:
            raise PoseHistoryError('max_age must be positive')
        with self._lock:
            if not self._poses:
                raise PoseHistoryError('no observed pose in this segment')
            pose = self._poses[-1]
            age = now - pose['source_monotonic_s']
            if not 0 <= age < max_age:
                raise PoseHistoryError('latest original source is future or expired')
            return dict(pose)

    def sample_at(self, source_time):
        """Interpolate only between observed measurement-axis endpoints.

        Exact endpoints return unmodified observed copies. Interior results are
        marked synthetic with sample_sequence=None, both bracket identities,
        and the older original source bound. They cannot be pushed as samples.
        This method is a historical query, not a freshness assertion.
        """
        target = _time(source_time, 'source_time')
        with self._lock:
            if (not self._poses or target < _measurement_time(self._poses[0])
                    or target > _measurement_time(self._poses[-1])):
                raise PoseHistoryError('query is outside the observed segment')
            lower = None
            for upper in self._poses:
                upper_time = _measurement_time(upper)
                if target == upper_time:
                    return dict(upper)
                if target < upper_time:
                    break
                lower = upper
            lower = dict(lower)
            upper = dict(upper)
        lower_time = _measurement_time(lower)
        upper_time = _measurement_time(upper)
        alpha = (target - lower_time) / (upper_time - lower_time)
        # Normalize each yaw first so subtraction of large finite angles cannot
        # overflow. Never normalize/rewrite a stored observed endpoint.
        yaw0 = math.atan2(math.sin(lower['yaw']), math.cos(lower['yaw']))
        yaw1 = math.atan2(math.sin(upper['yaw']), math.cos(upper['yaw']))
        delta = math.atan2(math.sin(yaw1-yaw0), math.cos(yaw1-yaw0))
        yaw = yaw0 + alpha * delta
        result = dict(x=(1-alpha)*lower['x'] + alpha*upper['x'],
                      y=(1-alpha)*lower['y'] + alpha*upper['y'],
                      yaw=math.atan2(math.sin(yaw), math.cos(yaw)),
                      source_monotonic_s=min(lower['source_monotonic_s'], upper['source_monotonic_s']),
                      measurement_monotonic_s=target, sample_sequence=None,
                      interpolated=True, query_monotonic_s=target, alpha=alpha,
                      lower_source_monotonic_s=lower['source_monotonic_s'],
                      upper_source_monotonic_s=upper['source_monotonic_s'],
                      lower_measurement_monotonic_s=lower_time,
                      upper_measurement_monotonic_s=upper_time,
                      lower_sample_sequence=lower['sample_sequence'],
                      upper_sample_sequence=upper['sample_sequence'])
        for key in ('x', 'y', 'yaw'):
            _finite(result[key], 'interpolated.' + key)
        return result

    at = sample_at

    def reset(self, epoch=None):
        proposed_epoch = _epoch(epoch)
        with self._lock:
            if proposed_epoch is not None and proposed_epoch != self._epoch:
                self._epoch = proposed_epoch
                self._last = None
            self._poses.clear()
            self._segment += 1

    def invalidate(self):
        self.reset()

    def summary(self):
        with self._lock:
            first = self._poses[0] if self._poses else None
            last = self._poses[-1] if self._poses else None
            return dict(count=len(self._poses), capacity=self._capacity,
                        epoch=self._epoch, segment=self._segment,
                        oldest_measurement_monotonic_s=None if first is None else _measurement_time(first),
                        newest_measurement_monotonic_s=None if last is None else _measurement_time(last),
                        oldest_source_monotonic_s=None if first is None else first['source_monotonic_s'],
                        newest_source_monotonic_s=None if last is None else last['source_monotonic_s'],
                        newest_sample_sequence=None if last is None else last['sample_sequence'],
                        accepted_count=self._accepted, duplicate_count=self._duplicates,
                        gap_count=self._gaps)
