"""Offline feedback-source watchdog. An adapter must supply true source time."""
import math


def _finite(value, name):
    try:
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(name + " must be finite")
        return float(value)
    except OverflowError:
        raise ValueError(name + " must be finite")


class FeedbackWatchdog:
    """Latched failure; duplicate/reordered observations do not renew freshness.

    Uses one monotonic clock domain. A future hardware adapter derives a
    conservative source_stamp from original request time and MCU CAN age;
    polling a cached status file cannot create a new observation. This model
    makes no motor commands and does not replace MCU protocol checks.
    """

    def __init__(self, timeout_ms):
        self.timeout_s = _finite(timeout_ms, "timeout_ms") / 1000.
        if self.timeout_s <= 0:
            raise ValueError("timeout_ms must be positive")
        self.sequence = self.source_stamp = self.last_now = None
        self.failure = None

    def _now(self, now):
        now = _finite(now, "now")
        if self.last_now is not None and now < self.last_now:
            raise ValueError("Monotonic clock moved backwards")
        self.last_now = now
        return now

    def observe(self, sequence, source_stamp, now):
        now = self._now(now)
        source_stamp = _finite(source_stamp, "source_stamp")
        if type(sequence) is not int or not 0 <= sequence <= 0xffffffff:
            raise ValueError("sequence must be a uint32 sample sequence")
        if source_stamp > now:
            raise ValueError("Future source timestamps are invalid")
        # Check the previous source BEFORE processing a late arriving sample.
        # A late new sample may not erase a timeout that already occurred.
        self._expire(now)
        if self.failure:
            return False
        if self.sequence is not None:
            step = (sequence - self.sequence) & 0xffffffff
            if not 0 < step <= 0x7fffffff or source_stamp <= self.source_stamp:
                return False
        if now - source_stamp >= self.timeout_s:
            self.failure = "feedback_timeout"
            return False
        self.sequence, self.source_stamp = sequence, source_stamp
        return True

    def _expire(self, now):
        if self.source_stamp is not None and now - self.source_stamp >= self.timeout_s:
            self.failure = self.failure or "feedback_timeout"

    def fault(self, reason, now):
        self._now(now)
        if type(reason) is not str or not reason.strip():
            raise ValueError("A concrete fault reason is required")
        self.failure = self.failure or reason
        return self.status(now)

    def status(self, now):
        now = self._now(now)
        self._expire(now)
        age = None if self.source_stamp is None else now - self.source_stamp
        return dict(healthy=self.source_stamp is not None and self.failure is None,
                    reason=self.failure or ("feedback_missing" if age is None else None),
                    source_stamp=self.source_stamp, source_age_s=age,
                    timeout_s=self.timeout_s, stop_required=self.failure is not None,
                    start_allowed=self.source_stamp is not None and self.failure is None,
                    hardware_execution_authorized=False)
