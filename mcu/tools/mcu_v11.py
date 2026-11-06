"""RK master for the V11 request/reply link. Import does not access serial."""

import argparse
from collections import Counter
from contextlib import closing
from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path
import statistics
import struct
import threading
import time

from mcu_v10 import BAUD, canonical_segments, crc16


VERSION = 11
BEGIN, APPEND, COMMIT, START, STOP, RESET, QUERY, TELEMETRY = 1, 2, 3, 4, 5, 6, 7, 128
TELEMETRY_SIZE = 110
PERIOD_S = .020
MASK32 = 0xFFFFFFFF
ODOM_INITIALIZED, ODOM_VALID, EXECUTING = 1, 2, 4
STOP_CONFIRMED, RX_FAULT, COMMITTED, STARTED, ACK_VALID = 16, 32, 64, 128, 256
READY = 2
UINT64_MAX = 0xFFFFFFFFFFFFFFFF
HEADER = b'\xa5\x5a\x0b\x80\x66\x00'
STATUS = struct.Struct('<QIIIIIQIBBHHHffffffIIhhHIIHHI')


def command(kind, boot=0, epoch=0, request=0, task=0, extra=b''):
    if not 1 <= kind <= 7 or len(extra) != (10 if kind in (BEGIN, APPEND) else 0):
        raise ValueError('Invalid command opcode or payload length')
    if request == 0:
        if kind != QUERY or boot or epoch or task:
            raise ValueError('Only discovery QUERY can use a zero request and identity')
    elif not 0 < request <= MASK32 or not boot or not epoch:
        raise ValueError('Regular requests require current boot, epoch and a nonzero request')
    body = struct.pack('<BBHQIIIQ', VERSION, kind, 28 + len(extra), boot, epoch, request, 0, task) + extra
    return b'\xa5\x5a' + body + struct.pack('<H', crc16(body))


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
    reserved_100: int
    reserved_102: int
    accepted_request_watermark: int

    @property
    def source_age_ms(self):
        return (self.publication_tick_ms - self.measurement_tick_ms) & MASK32

    @property
    def source_valid(self):
        both = ODOM_INITIALIZED | ODOM_VALID
        return self.flags & both == both and self.source_age_ms < 250


def decode_telemetry(raw):
    if len(raw) != TELEMETRY_SIZE or raw[:6] != HEADER:
        raise ValueError('Invalid V11 reply header')
    if crc16(raw[2:108]) != struct.unpack_from('<H', raw, 108)[0]:
        raise ValueError('Invalid V11 reply CRC')
    return decode_status(raw)


def decode_status(raw):
    value = Telemetry(*STATUS.unpack_from(raw, 6))
    if (not value.boot or not value.epoch or value.state > 9 or value.flags & ~511
            or value.reserved_100 or value.reserved_102 or value.ack_result > 8
            or value.flags & ODOM_VALID and not value.source_valid
            or not all(math.isfinite(v) for v in (value.progress, value.x, value.y, value.yaw, value.v, value.w))
            or abs(value.yaw) > math.pi + .000001):
        raise ValueError('Inconsistent V11 reply metadata')
    return value


class ReplyParser:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = Counter(noise_bytes=0, invalid_headers=0, crc_errors=0, invalid_metadata=0)

    def feed(self, data):
        self.buffer.extend(data)
        while self.buffer:
            start = self.buffer.find(HEADER[:2])
            if start < 0:
                tail = self.buffer[-1:] if self.buffer[-1] == HEADER[0] else b''
                self.errors['noise_bytes'] += len(self.buffer) - len(tail)
                self.buffer[:] = tail
                return
            if start:
                self.errors['noise_bytes'] += start
                del self.buffer[:start]
            header = self.buffer[:len(HEADER)]
            if header != HEADER[:len(header)]:
                self.errors['invalid_headers'] += 1
                del self.buffer[0]
                continue
            if len(self.buffer) < TELEMETRY_SIZE:
                return
            raw = bytes(self.buffer[:TELEMETRY_SIZE])
            if crc16(raw[2:108]) != struct.unpack_from('<H', raw, 108)[0]:
                self.errors['crc_errors'] += 1
                del self.buffer[0]
                continue
            del self.buffer[:TELEMETRY_SIZE]
            try:
                status = decode_status(raw)
            except ValueError:
                self.errors['invalid_metadata'] += 1
                continue
            yield status, raw


def forward(new, old):
    return 0 < ((new - old) & MASK32) < 0x80000000


@dataclass(frozen=True)
class Exchange:
    opcode: int
    request: int
    sent_at: float
    received_at: float
    status: Telemetry
    request_hex: str
    reply_hex: str


@dataclass(frozen=True)
class QueuePlan:
    segments: tuple
    checksum: int
    duration_ms: int

    @classmethod
    def from_document(cls, document):
        segments, checksum, duration_ms = canonical_segments(document)
        return cls(tuple(segments), checksum, duration_ms)


class LinkError(RuntimeError):
    def __init__(self, message, exchange=None, *, details=None):
        super().__init__(message)
        self.exchange = exchange
        self.details = details or {}


class Rejected(LinkError):
    pass


class ReplyTimeout(LinkError):
    pass


class RkMaster:
    """One serial owner, one transaction at a time. Ordinary queries may drop."""

    def __init__(self, port, *, reply_timeout=.100, clock=time.monotonic, sleep=time.sleep):
        if not math.isfinite(reply_timeout) or reply_timeout <= 0:
            raise ValueError('Reply timeout must be finite and positive')
        self.port = port
        self.reply_timeout = reply_timeout
        self.clock, self.sleep = clock, sleep
        self.parser = ReplyParser()
        self.lock = threading.Lock()
        self.latest = None
        self.last_sent = None
        self.sent = 0
        self.dropped_queries = 0
        self._dropped_request = None
        self.ignored = Counter(unmatched_ack=0, stale_reply=0)
        self.failed = False

    def _exchange(self, kind, *, task=0, extra=b'', discover=False, start_before=None, drop_missing=False):
        if self.failed:
            raise LinkError('Owner closed to new requests after an uncertain transaction')
        previous = self.latest.status if self.latest else None
        if discover:
            if previous is not None or kind != QUERY:
                raise ValueError('Discovery is only valid as the first transaction')
            request = 0
            packet = command(QUERY)
        else:
            if previous is None:
                raise ValueError('Discover the MCU before regular requests')
            watermark = previous.accepted_request_watermark
            base = watermark if self._dropped_request is None else self._dropped_request
            request = (base + 1) & MASK32 or 1
            if not forward(request, watermark):
                self.failed = True
                raise LinkError('Request sequence has exhausted the forward range of the last known watermark')
            packet = command(kind, previous.boot, previous.epoch, request, task, extra)

        earliest = self.clock() if self.last_sent is None else self.last_sent + PERIOD_S
        if start_before is not None and max(earliest, self.clock()) >= start_before:
            return None
        now = self.clock()
        while now < earliest:
            self.sleep(earliest - now)
            now = self.clock()
        sent_at = self.clock()
        if start_before is not None and sent_at >= start_before:
            return None
        self.last_sent, self.sent = sent_at, self.sent + 1
        details = dict(opcode=kind, request=request, request_hex=packet.hex(), sent_at=sent_at)
        deadline = sent_at + self.reply_timeout
        exchange = None
        try:
            written = self.port.write(packet)
            details['bytes_written'] = written
            if written != len(packet):
                raise LinkError('Partial serial write, command outcome unknown', details=details)
            while self.clock() < deadline:
                data = self.port.read(max(1, TELEMETRY_SIZE - len(self.parser.buffer)))
                arrived = self.clock()
                if arrived > deadline:
                    break
                for value, raw in self.parser.feed(data):
                    exchange = Exchange(kind, request, sent_at, arrived, value, packet.hex(), raw.hex())
                    matches = value.flags & ACK_VALID and value.ack_request == request
                    if previous and (matches or not drop_missing) and (value.boot, value.epoch) != (previous.boot, previous.epoch):
                        raise LinkError('MCU boot or odometry epoch changed during request', exchange, details=details)
                    if not matches:
                        self.ignored['unmatched_ack'] += 1
                        continue
                    if previous and (not forward(value.publication_sequence, previous.publication_sequence)
                                     or not forward(value.publication_tick_ms, previous.publication_tick_ms)):
                        self.ignored['stale_reply'] += 1
                        continue
                    if not discover:
                        accepted = value.accepted_request_watermark
                        # A rejected request may reveal acceptance of an earlier dropped QUERY.
                        known_or_dropped = accepted == watermark or (accepted != 0 and forward(accepted, watermark)
                                                                     and forward(request, accepted))
                        if accepted != request and not (value.ack_result and known_or_dropped):
                            raise LinkError('Reply acceptance watermark does not match request', exchange, details=details)
                        if accepted == request:
                            self._dropped_request = None
                    self.latest = exchange
                    if value.ack_result:
                        raise Rejected('MCU rejected opcode {} with result {}'.format(kind, value.ack_result),
                                       exchange, details=details)
                    return exchange
            if drop_missing and kind == QUERY and not discover:
                # The next request must not accept a late ACK for this sample.
                self._dropped_request = request
                self.dropped_queries += 1
                return None
            raise ReplyTimeout('Timed out without a matching V11 reply', exchange, details=details)
        except Rejected:
            if discover:
                self.failed = True
            raise
        except (OSError, LinkError) as exc:
            self.failed = True
            if isinstance(exc, LinkError):
                raise
            raise LinkError('Serial exchange failed: {}'.format(exc), exchange, details=details) from exc
        except BaseException:
            self.failed = True
            raise

    def discover(self):
        with self.lock:
            return self._exchange(QUERY, discover=True)

    def query(self):
        """Return the current exchange or None for a dropped status sample."""
        with self.lock:
            return self._exchange(QUERY, drop_missing=True)

    def upload(self, document, task=None, *, on_ack=None):
        plan = document if isinstance(document, QueuePlan) else QueuePlan.from_document(document)
        with self.lock:
            if self.latest is None:
                raise ValueError('Discover the MCU before upload')
            current = self.latest.status.task
            task = current + 1 if task is None else task
            if type(task) is not int or not current < task <= UINT64_MAX:
                raise ValueError('New upload requires a strictly increasing uint64 task ID')
            acks = []

            def record(reply):
                acks.append(reply)
                if on_ack is not None:
                    on_ack(reply)

            try:
                metadata = struct.pack('<HII', len(plan.segments), plan.checksum, plan.duration_ms)
                record(self._exchange(BEGIN, task=task, extra=metadata))
                for index, segment in enumerate(plan.segments):
                    record(self._exchange(APPEND, task=task, extra=struct.pack('<H', index) + segment))
                reply = self._exchange(COMMIT, task=task)
                value = reply.status
                if (value.task != task or value.state != READY or not value.flags & COMMITTED
                        or value.flags & (STARTED | EXECUTING) or value.count != len(plan.segments)):
                    self.failed = True
                    raise LinkError('COMMIT ACK did not prove the new task is READY', reply)
                record(reply)
            except LinkError as exc:
                exc.details['accepted_upload_prefix'] = [asdict(ack) for ack in acks]
                raise
            return acks

    def start(self, task):
        with self.lock:
            value = self.latest.status if self.latest else None
            if (type(task) is not int or not 0 < task <= UINT64_MAX or value is None
                    or value.task != task or value.state != READY or not value.flags & COMMITTED
                    or value.flags & (STARTED | EXECUTING)):
                raise ValueError('START requires the explicitly selected READY task on this MCU')
            return self._exchange(START, task=task)

    def stop(self, *, proof_timeout=3., on_reply=None):
        if not math.isfinite(proof_timeout) or proof_timeout <= 0:
            raise ValueError('Stop proof timeout must be finite and positive')
        with self.lock:
            ack = self._exchange(STOP)
            if on_reply is not None:
                on_reply(ack)
            deadline = self.clock() + proof_timeout
            # Each proof query needs its full response window, including after cadence sleep.
            start_before = deadline - self.reply_timeout
            while self.clock() < deadline:
                previous_sends = self.sent
                try:
                    reply = self._exchange(QUERY, start_before=start_before, drop_missing=True)
                except LinkError as exc:
                    return dict(ack=asdict(ack), stop_verified=False, error=error_record(exc))
                if reply is None:
                    if self.sent == previous_sends:
                        break
                    continue
                if on_reply is not None:
                    on_reply(reply)
                value, before = reply.status, ack.status
                if (value.flags & STOP_CONFIRMED and value.source_valid and not value.flags & EXECUTING
                        and value.state in (0, 8, 9)
                        and forward(value.sample_sequence, before.sample_sequence)
                        and forward(value.measurement_tick_ms, before.publication_tick_ms)
                        and max(abs(value.left_tenths_rpm), abs(value.right_tenths_rpm)) <= 2):
                    return dict(ack=asdict(ack), stop_verified=True, proof=asdict(reply))
            return dict(ack=asdict(ack), stop_verified=False,
                        error=dict(code='stop_proof_deadline', message='No fresh measured stop proof before deadline'))

    def poll(self, duration):
        if not math.isfinite(duration) or duration <= 0:
            raise ValueError('Poll duration must be finite and positive')
        started = self.clock()
        deadline = started + duration
        replies = []
        sends = []
        drops = 0
        failure = None
        while self.clock() < deadline:
            try:
                with self.lock:
                    previous_sends = self.sent
                    try:
                        reply = self._exchange(QUERY, start_before=deadline, drop_missing=True)
                    finally:
                        sent = self.sent != previous_sends
                        if sent:
                            sends.append(self.last_sent)
            except LinkError as exc:
                failure = error_record(exc)
                break
            if reply is None:
                if not sent:
                    break
                drops += 1
                continue
            replies.append(reply)
        result = poll_report(replies, sends, drops, duration, self.clock() - started)
        result.update(error=failure, success=failure is None)
        return result


def error_record(exc):
    report = dict(type=type(exc).__name__, message=str(exc))
    if isinstance(exc, LinkError):
        report.update(exchange=asdict(exc.exchange) if exc.exchange else None, details=exc.details)
    return report


def poll_report(replies, sends, drops, duration, elapsed):
    def cadence(values):
        if len(values) < 2 or values[-1] <= values[0]:
            return None
        return (len(values) - 1) / (values[-1] - values[0])

    statuses = [reply.status for reply in replies]
    sources = [status for status in statuses if status.source_valid]
    samples = {(status.boot, status.epoch, status.sample_sequence) for status in sources}
    latencies = [1000 * (reply.received_at - reply.sent_at) for reply in replies]
    send_gaps = [1000 * (new - old) for old, new in zip(sends, sends[1:])]
    return dict(duration_requested_s=duration, elapsed_s=elapsed, target_hz=1 / PERIOD_S,
                query_attempts=len(sends), acknowledged_queries=len(replies), dropped_queries=drops,
                send_rate_hz=cadence(sends),
                reply_rate_hz=cadence([reply.received_at for reply in replies]),
                max_send_gap_ms=max(send_gaps, default=None),
                reply_latency_ms=dict(median=statistics.median(latencies) if latencies else None,
                                      max=max(latencies, default=None), over_20_ms=sum(ms > 20. for ms in latencies)),
                fresh_source_replies=len(sources), distinct_live_samples=len(samples),
                repeated_live_sample_replies=len(sources) - len(samples),
                max_source_age_ms=max((status.source_age_ms for status in sources), default=None),
                uart_first_error_7_replies=sum(status.first_error == 7 for status in statuses),
                uart_rx_fault_flag_replies=sum(bool(status.flags & RX_FAULT) for status in statuses),
                first_reply=asdict(replies[0]) if replies else None,
                last_reply=asdict(replies[-1]) if replies else None)


def open_port(device, reply_timeout):
    # The RK is Linux. PySerial's lock and kernel exclusive mode protect this owner.
    import fcntl
    import serial
    import termios

    port = serial.Serial(port=None, baudrate=BAUD, bytesize=8, parity='N', stopbits=1,
                         timeout=min(.002, reply_timeout), write_timeout=reply_timeout,
                         xonxoff=False, rtscts=False, dsrdtr=False, exclusive=True)
    port.dtr = False
    port.rts = False
    port.port = device
    try:
        port.open()
        fcntl.ioctl(port.fileno(), termios.TIOCEXCL)
    except BaseException:
        port.close()
        raise
    return port


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('poll', 'benchmark', 'query', 'upload', 'start', 'stop'))
    parser.add_argument('--port', required=True)
    parser.add_argument('--execute', action='store_true', help='Open the specified RK serial port')
    parser.add_argument('--duration', type=float, default=65., help='Poll/benchmark duration in seconds')
    parser.add_argument('--reply-timeout', type=float, default=.100, help='Per-request reply timeout in seconds')
    parser.add_argument('--stop-proof-timeout', type=float, default=3., help='Time to observe proof after STOP ACK')
    parser.add_argument('--queue', type=Path)
    parser.add_argument('--task-id', type=lambda value: int(value, 0))
    args = parser.parse_args(argv)
    if any(not math.isfinite(value) or value <= 0 for value in
           (args.duration, args.reply_timeout, args.stop_proof_timeout)):
        parser.error('Durations and timeouts must be finite and positive')
    if args.operation == 'upload' and args.queue is None:
        parser.error('Upload requires --queue')
    if args.operation == 'start' and args.task_id is None:
        parser.error('START requires --task-id')
    if args.task_id is not None and not 0 < args.task_id <= UINT64_MAX:
        parser.error('Task ID must be a positive uint64')
    if not args.execute:
        print(json.dumps(dict(mode='offline', operation=args.operation, port_opened=False)))
        return 0

    report = dict(operation=args.operation, port=args.port, success=False)
    master = None
    try:
        plan = None
        if args.operation == 'upload':
            plan = QueuePlan.from_document(json.loads(args.queue.read_text(encoding='utf-8')))
        with closing(open_port(args.port, args.reply_timeout)) as port:
            master = RkMaster(port, reply_timeout=args.reply_timeout)
            report['discovery'] = asdict(master.discover())
            if args.operation in ('poll', 'benchmark'):
                report.update(master.poll(args.duration))
            elif args.operation == 'stop':
                report.update(master.stop(proof_timeout=args.stop_proof_timeout))
                report['success'] = report['stop_verified']
            else:
                if args.operation == 'upload':
                    acks = master.upload(plan, args.task_id)
                    report['acknowledgements'] = [asdict(ack) for ack in acks]
                    reply = acks[-1]
                elif args.operation == 'start':
                    reply = master.start(args.task_id)
                else:
                    reply = master.query()
                    report['sample_dropped'] = reply is None
                report.update(success=True, reply=asdict(reply) if reply is not None else None)
    except (ImportError, OSError, ValueError, KeyError, TypeError, LinkError) as exc:
        report.update(success=False, error=error_record(exc))
    if master is not None:
        report.update(transmit_attempts=master.sent, parser_errors=dict(master.parser.errors),
                      ignored_replies=dict(master.ignored), owner_poisoned=master.failed,
                      dropped_queries=master.dropped_queries)
    print(json.dumps(report, indent=2, allow_nan=False))
    return 0 if report['success'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
