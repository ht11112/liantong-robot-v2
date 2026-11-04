"""v10 codec and conservative half-duplex CLI. Import never opens a port.

No command means listen. Upload commits only; START is a separate invocation.
Writes require explicit measured transport timing bounds, never a fixed sleep.
"""
import argparse
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import struct
import time
import zlib

VERSION = 10
BAUD = 115200
TELEMETRY_SIZE = 110
BEGIN, APPEND, COMMIT, START, STOP, RESET, QUERY, TELEMETRY = 1, 2, 3, 4, 5, 6, 7, 128
ACK_VALID = 256


def crc16(raw):
    value = 0xFFFF
    for byte in raw:
        value ^= byte << 8
        for _ in range(8):
            value = ((value << 1) ^ (0x1021 if value & 0x8000 else 0)) & 0xFFFF
    return value


def frame(kind, payload):
    body = struct.pack('<BBH', VERSION, kind, len(payload)) + payload
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))


def command(kind, boot, epoch, request, grant, task=0, extra=b''):
    if kind not in range(1, 8) or not 0 < request <= 0xFFFFFFFF:
        raise ValueError('Invalid command/request identity')
    expected = 10 if kind in (BEGIN, APPEND) else 0
    if len(extra) != expected:
        raise ValueError('Unexpected command payload')
    result = frame(kind, struct.pack('<QIIIQ', boot, epoch, request, grant, task) + extra)
    if len(result) > 64:
        raise ValueError('Command exceeds the wire budget')
    return result


def canonical_segments(document):
    """Return exact float32 wire bytes; CRC does not depend on C padding."""
    segments = document['segments']
    if not isinstance(segments, list) or not 1 <= len(segments) <= 128:
        raise ValueError('Queue needs 1..128 segments')
    result = []
    for segment in segments:
        if not isinstance(segment, dict):
            raise ValueError('Each segment must be an object')
        kind = segment.get('kind')
        if kind == 'move':
            value, code = segment.get('distance_m'), 1
        elif kind == 'turn':
            value, code = segment.get('angle_rad'), 2
        else:
            raise ValueError('Segment kind must be move or turn')
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError('Segment value must be a finite SI value')
        try:
            raw = struct.pack('<If', code, value)
        except (OverflowError, struct.error):
            raise ValueError('Segment cannot be represented as float32') from None
        quantized = struct.unpack('<If', raw)[1]
        limit = struct.unpack('<f', struct.pack('<f', math.pi))[0] * 2
        if not (0 < quantized <= 1000 if code == 1 else 0 < abs(quantized) <= limit):
            raise ValueError('MOVE requires (0,1000]m; TURN requires nonzero [-2pi,2pi]rad')
        result.append(raw)
    duration = document.get('duration_s', 1200)
    if isinstance(duration, bool) or not isinstance(duration, (int, float)) or not math.isfinite(duration):
        raise ValueError('Invalid queue duration')
    duration_ms = round(duration * 1000)
    if not 0 < duration_ms <= 1200000:
        raise ValueError('Duration must be within (0,1200] seconds')
    return result, zlib.crc32(b''.join(result)), duration_ms


@dataclass(frozen=True)
class Telemetry:
    boot: int
    epoch: int
    publication_sequence: int
    publication_tick_ms: int
    sample_sequence: int
    measurement_tick_ms: int
    task: int
    ack_request: int
    ack_result: int
    state: int
    flags: int
    index: int
    count: int
    progress: float
    x: float
    y: float
    yaw: float
    v: float
    w: float
    position_left: int
    position_right: int
    left_tenths_rpm: int
    right_tenths_rpm: int
    first_error: int
    first_drive_error: int
    first_error_tick_ms: int
    grant_open_ms: int
    grant_close_ms: int
    accepted_request_watermark: int

    @property
    def grant(self):
        return self.publication_sequence


def decode_telemetry(raw):
    if (len(raw) != TELEMETRY_SIZE or raw[:4] != b'\xa5\x5a\x0a\x80'
            or struct.unpack_from('<H', raw, 4)[0] != 102
            or struct.unpack_from('<H', raw, 108)[0] != crc16(raw[2:108])):
        raise ValueError('Invalid v10 telemetry frame')
    values = struct.unpack_from('<QIIIIIQIBBHHHffffffIIhhHIIHHI', raw, 6)
    result = Telemetry(*values)
    if (not result.boot or not result.epoch or result.state > 9 or result.flags & ~511
            or not 13 <= result.grant_open_ms <= 14
            or result.grant_close_ms - result.grant_open_ms != 24
            or (result.flags & 2 and ((result.publication_tick_ms-result.measurement_tick_ms) & 0xFFFFFFFF) >= 250)
            or not all(math.isfinite(getattr(result, key)) for key in ('progress', 'x', 'y', 'yaw', 'v', 'w'))
            or abs(result.yaw) > math.pi + 0.000001):
        raise ValueError('Telemetry metadata is inconsistent')
    return result


class Parser:
    def __init__(self):
        self.buffer = bytearray()

    def feed(self, data):
        self.buffer.extend(data)
        output = []
        while True:
            position = self.buffer.find(b'\xa5\x5a')
            if position < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b'\xa5' else b''
                break
            if position:
                del self.buffer[:position]
            if len(self.buffer) < 6:
                break
            length = struct.unpack_from('<H', self.buffer, 4)[0] + 8
            if self.buffer[2] != VERSION or not 8 <= length <= TELEMETRY_SIZE:
                del self.buffer[0]
                continue
            if len(self.buffer) < length:
                break
            raw = bytes(self.buffer[:length])
            if crc16(raw[2:-2]) != struct.unpack_from('<H', raw, length - 2)[0]:
                del self.buffer[0]
                continue
            del self.buffer[:length]
            if raw[3] == TELEMETRY:
                try:
                    output.append(decode_telemetry(raw))
                except ValueError:
                    pass
        return output


def next_request(value):
    # A rejected request can change ACK identity; it cannot change this watermark.
    return ((value.accepted_request_watermark + 1) & 0xFFFFFFFF) or 1


@dataclass(frozen=True)
class TimingBudget:
    rx_latency_min_ms: float
    rx_latency_max_ms: float
    tx_latency_max_ms: float
    clock_drift_ppm: float
    provenance: str

    @classmethod
    def load(cls, path):
        value = cls(**json.loads(Path(path).read_text(encoding='utf-8')))
        numeric = (value.rx_latency_min_ms, value.rx_latency_max_ms,
                   value.tx_latency_max_ms, value.clock_drift_ppm)
        if (any(isinstance(item, bool) or not isinstance(item, (int, float))
                or not math.isfinite(item) or item < 0 for item in numeric)
                or value.rx_latency_max_ms < value.rx_latency_min_ms
                or not isinstance(value.provenance, str) or not value.provenance.strip()):
            raise ValueError('Measured timing bounds and provenance are required')
        return value


class ClockWindow:
    """Bound H-M, including complete-frame airtime, USB/scheduler delay and drift.

    This is conditional on measured bounds supplied by the operator; desktop
    Python/USB is not a hard real-time link. Firmware independently rejects late
    normal commands. No average latency is treated as an upper bound.
    """
    def __init__(self, budget):
        self.budget = budget
        self.boot = self.last_raw = self.unwrapped_ms = self.sequence = None
        self.low = self.high = self.receipt = None
        self.good = 0

    def observe(self, value, receipt):
        if self.boot != value.boot:
            self.__init__(self.budget)
            self.boot = value.boot
        if self.sequence is not None:
            step = (value.publication_sequence - self.sequence) & 0xFFFFFFFF
            if not 0 < step < 0x80000000:
                return False
        if self.last_raw is None:
            unwrapped = value.publication_tick_ms
        else:
            step = (value.publication_tick_ms - self.last_raw) & 0xFFFFFFFF
            if not 0 < step < 0x80000000:
                self.good = 0
                return False
            unwrapped = self.unwrapped_ms + step
        b = self.budget
        end = unwrapped / 1000 + TELEMETRY_SIZE * 10 / BAUD
        low = receipt - end - b.rx_latency_max_ms / 1000
        # The integer MCU publication tick denotes [tick,tick+1ms), not an exact instant.
        low -= .001
        high = receipt - end - b.rx_latency_min_ms / 1000
        if self.receipt is not None:
            drift = max(0, receipt - self.receipt) * b.clock_drift_ppm / 1000000
            low, high = max(low, self.low - drift), min(high, self.high + drift)
        if low > high:
            # An observed violation discards old authority, never widens it and sends.
            self.low, self.high, self.receipt, self.good = None, None, None, 0
            self.last_raw = self.sequence = self.unwrapped_ms = None
            return False
        self.low, self.high, self.receipt = low, high, receipt
        self.last_raw, self.unwrapped_ms = value.publication_tick_ms, unwrapped
        self.sequence, self.good = value.publication_sequence, self.good + 1
        return True

    def send_window(self, value, now, size, stop=False):
        if (self.good < 5 or self.sequence != value.publication_sequence
                or self.boot != value.boot or self.receipt is None
                or not 0 <= now - self.receipt <= .10):
            return None
        drift = (now - self.receipt) * self.budget.clock_drift_ppm / 1000000
        low, high = self.low - drift, self.high + drift
        publication = self.unwrapped_ms / 1000
        earliest = publication + value.grant_open_ms / 1000 + high
        # STOP can also use the quiet tail before the next telemetry reservation.
        close_ms = value.grant_close_ms + (10 if stop else 0)
        latest = (publication + close_ms / 1000 + low
                  - self.budget.tx_latency_max_ms / 1000 - size * 10 / BAUD - .001)
        return (earliest, latest) if earliest <= latest else None


class SerialClient:
    def __init__(self, port, budget=None):
        self.port, self.parser = port, Parser()
        self.clock = None if budget is None else ClockWindow(budget)
        self.latest = None
        self.last_receipt = None
        self.last_grant = None

    def receive(self, timeout=2.):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            data = self.port.read(128)
            receipt = time.monotonic()
            accepted = None
            for value in self.parser.feed(data):
                if self.latest is not None and value.boot == self.latest.boot:
                    step = (value.publication_sequence - self.latest.publication_sequence) & 0xFFFFFFFF
                    if not 0 < step < 0x80000000:
                        continue
                self.latest, self.last_receipt = value, receipt
                if self.clock:
                    self.clock.observe(value, receipt)
                accepted = value
            if accepted is not None:
                return accepted
        raise TimeoutError('No fresh v10 telemetry; no command sent')

    def request(self, kind, task=0, extra=b''):
        if self.clock is None:
            raise RuntimeError('Writes require an explicit measured timing budget')
        first = self.latest or self.receive()
        boot, epoch = first.boot, first.epoch
        req = next_request(first)
        deadline = time.monotonic() + 3.
        packet = None
        while time.monotonic() < deadline:
            value = self.latest
            if value.boot != boot or value.epoch != epoch:
                raise RuntimeError('MCU boot/odom epoch changed before command; re-prepare')
            now = time.monotonic()
            candidate = command(kind, boot, epoch, req, value.grant, task, extra)
            window = self.clock.send_window(value, now, len(candidate), kind == STOP)
            if window and self.last_grant != value.grant and window[0] <= now <= window[1]:
                # Re-evaluate immediately; tx bound includes this check-to-write delay.
                now = time.monotonic()
                window = self.clock.send_window(value, now, len(candidate), kind == STOP)
                if window and window[0] <= now <= window[1]:
                    packet = candidate
                    if self.port.write(packet) != len(packet):
                        raise RuntimeError('Partial serial write; outcome unknown, do not retry/start')
                    self.last_grant = value.grant
                    break
            if window and now < window[0] and window[0] - now < .01:
                time.sleep(min(.001, window[0] - now))
            else:
                self.receive(min(.2, max(.01, deadline - now)))
        if packet is None:
            raise TimeoutError('No provably fitting transmit window; no command sent')
        deadline = time.monotonic() + 2.
        while time.monotonic() < deadline:
            value = self.receive(max(.01, deadline - time.monotonic()))
            if value.boot != boot:
                raise RuntimeError('MCU rebooted; prior request outcome is not reusable')
            if value.flags & ACK_VALID and value.ack_request == req:
                if value.ack_result:
                    raise RuntimeError('MCU rejected request {} with result {}'.format(req, value.ack_result))
                return value
        raise TimeoutError('No matching ACK; outcome unknown, no automatic retry/start')


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', nargs='?', default='listen',
                        choices=('listen', 'upload', 'start', 'stop', 'reset', 'query'))
    parser.add_argument('--port', required=True)
    parser.add_argument('--timing-budget', help='Measured USB/host delay bounds JSON, required for writes')
    parser.add_argument('--queue', help='JSON segments/duration_s, upload only')
    parser.add_argument('--task-id', type=int, help='Monotonic uint64 task ID; upload defaults to current+1')
    args = parser.parse_args(argv)
    if args.operation != 'listen' and not args.timing_budget:
        parser.error('Writes require --timing-budget with measured bounds and provenance')
    if args.operation == 'upload' and not args.queue:
        parser.error('Upload requires --queue')
    if args.operation == 'start' and args.task_id is None:
        parser.error('START requires explicit --task-id')
    budget = TimingBudget.load(args.timing_budget) if args.timing_budget else None
    import serial  # Optional PC dependency; import/codec/test never opens a port.
    with serial.Serial(args.port, BAUD, timeout=.003, write_timeout=.02, exclusive=True) as port:
        client = SerialClient(port, budget)
        value = client.receive()
        if args.operation == 'listen':
            while True:
                print(json.dumps(asdict(value), allow_nan=False), flush=True)
                value = client.receive()
        elif args.operation == 'upload':
            segments, checksum, duration = canonical_segments(json.loads(Path(args.queue).read_text(encoding='utf-8')))
            task = value.task + 1 if args.task_id is None else args.task_id
            if not value.task < task <= 0xFFFFFFFFFFFFFFFF:
                parser.error('New upload task ID must be greater than current MCU task ID')
            value = client.request(BEGIN, task, struct.pack('<HII', len(segments), checksum, duration))
            for index, segment in enumerate(segments):
                value = client.request(APPEND, task, struct.pack('<H', index) + segment)
            value = client.request(COMMIT, task)
        elif args.operation == 'start':
            if not 0 < args.task_id <= 0xFFFFFFFFFFFFFFFF or args.task_id != value.task:
                parser.error('START must name the current committed task ID')
            value = client.request(START, args.task_id)
        else:
            value = client.request({'stop': STOP, 'reset': RESET, 'query': QUERY}[args.operation], value.task)
        print(json.dumps(asdict(value), allow_nan=False))
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (ValueError, RuntimeError, TimeoutError, OSError) as exc:
        raise SystemExit(str(exc))
