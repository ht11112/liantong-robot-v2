"""Single V12 serial process and bounded shared feedback; importing performs no IO."""

from contextlib import closing, contextmanager
import math
import multiprocessing as mp
from pathlib import Path
import pickle
import queue
import statistics
import struct
import sys
import threading
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mcu' / 'tools'))
from mcu_v11 import (APPEND_REPLY_TIMEOUT_S, Exchange, LinkError, PERIOD_S, QUERY, QueuePlan, Rejected,
                     RequestCancelled, RkMaster, STATUS, STOP, Telemetry,
                     error_record, open_port)


CAPACITY = 512
STOP_PROOF_TIMEOUT_S = 3.
# Only the serial owner validates wire frames. Consumers copy trusted internal records.
FRAME = struct.Struct('<BIddH46s110s')
TRANSMISSION = struct.Struct('<BId')


def _pack(exchange):
    request = bytes.fromhex(exchange.request_hex)
    return FRAME.pack(exchange.opcode, exchange.request, exchange.sent_at, exchange.received_at,
                      len(request), request, bytes.fromhex(exchange.reply_hex))


def _unpack(raw):
    opcode, request, sent_at, received_at, size, packet, reply = FRAME.unpack(raw)
    status = Telemetry(*STATUS.unpack_from(reply, 6))
    return Exchange(opcode, request, sent_at, received_at, status, packet[:size].hex(), reply.hex())


class _Mailbox:
    """One outstanding operation has one bounded result; a slow reader never stalls IO."""
    def __init__(self, ctx, capacity=262144):
        self.data = ctx.RawArray('B', capacity)
        self.length = ctx.RawValue('I', 0)
        self.lock = ctx.Lock()

    def put(self, identifier, success, result):
        raw = pickle.dumps((identifier, success, result), protocol=4)
        if len(raw) > len(self.data):
            raise ValueError('Feedback result exceeds the bounded mailbox')
        with self.lock:
            self.data[:len(raw)] = raw
            self.length.value = len(raw)

    def get(self):
        with self.lock:
            raw = bytes(self.data[:self.length.value])
        return pickle.loads(raw) if raw else None


class SharedFeedbackCache:
    def __init__(self, ctx=None, capacity=CAPACITY):
        ctx = ctx or mp.get_context('spawn')
        self.capacity = capacity
        self.frames = ctx.RawArray('B', capacity * FRAME.size)
        self.transmissions = ctx.RawArray('B', capacity * TRANSMISSION.size)
        self.sequence = ctx.RawValue('Q', 0)
        self.tx_sequence = ctx.RawValue('Q', 0)
        self.query_count = ctx.RawValue('Q', 0)
        self.dropped_queries = ctx.RawValue('Q', 0)
        self.connected = ctx.RawValue('b', False)
        self.failed = ctx.RawValue('b', False)
        self.lock = ctx.Lock()
        self.failure = _Mailbox(ctx)

    def publish(self, exchange):
        raw = _pack(exchange)
        with self.lock:
            index = self.sequence.value % self.capacity
            start = index * FRAME.size
            self.frames[start:start + FRAME.size] = raw
            self.sequence.value += 1
            self.connected.value = True

    def record_send(self, opcode, request, sent_at):
        raw = TRANSMISSION.pack(opcode, request, sent_at)
        with self.lock:
            index = self.tx_sequence.value % self.capacity
            start = index * TRANSMISSION.size
            self.transmissions[start:start + TRANSMISSION.size] = raw
            self.tx_sequence.value += 1
            self.query_count.value += opcode == QUERY

    def set_dropped(self, count):
        with self.lock:
            self.dropped_queries.value = count

    def disconnect(self, error=None):
        if error is not None:
            self.failure.put(1, False, error)
        with self.lock:
            self.connected.value = False
            self.failed.value = error is not None

    def _read(self, cursor, transmissions=False):
        stride = TRANSMISSION.size if transmissions else FRAME.size
        with self.lock:
            latest = (self.tx_sequence if transmissions else self.sequence).value
            first = max(cursor + 1, latest - self.capacity + 1, 1)
            missed = max(0, first - cursor - 1)
            data = self.transmissions if transmissions else self.frames
            records = []
            for sequence in range(first, latest + 1):
                index = (sequence - 1) % self.capacity
                start = index * stride
                records.append((sequence, bytes(data[start:start + stride])))
            connected = bool(self.connected.value)
        return latest, records, missed, connected

    def history_since(self, cursor):
        latest, records, missed, connected = self._read(cursor)
        failure = self.failure.get()
        return dict(cursor=latest, exchanges=[_unpack(raw) for _, raw in records],
                    missed=missed, connected=connected, error=failure[2] if failure else None)

    def snapshot(self):
        with self.lock:
            sequence = self.sequence.value
            connected = bool(self.connected.value)
            start = ((sequence - 1) % self.capacity) * FRAME.size
            raw = bytes(self.frames[start:start + FRAME.size]) if sequence else None
        failure = self.failure.get()
        return dict(cursor=sequence, latest=_unpack(raw) if raw else None, connected=connected,
                    error=failure[2] if failure else None)

    def transmissions_since(self, cursor):
        latest, records, missed, _ = self._read(cursor, True)
        entries = []
        for sequence, raw in records:
            opcode, request, sent_at = TRANSMISSION.unpack(raw)
            entries.append(dict(sequence=sequence, opcode=opcode, request=request, sent_at=sent_at))
        return dict(cursor=latest, entries=entries, missed=missed)

    def diagnostics(self):
        frames = self.history_since(0)
        transmissions = self.transmissions_since(0)
        replies, tx = frames['exchanges'], transmissions['entries']
        sends = [item['sent_at'] for item in tx]
        queries = [item['sent_at'] for item in tx if item['opcode'] == QUERY]
        gaps = [1000 * (new - old) for old, new in zip(sends, sends[1:])]
        latencies = [1000 * (item.received_at - item.sent_at) for item in replies]
        sources = {}
        for item in replies:
            sample = item.status
            if sample.source_valid:
                sources.setdefault((sample.boot, sample.epoch, sample.sample_sequence),
                                   item.sent_at - (sample.source_age_ms + 1) / 1000.)
        with self.lock:
            query_count, dropped = self.query_count.value, self.dropped_queries.value
        return dict(target_hz=1 / PERIOD_S, send_rate_hz=_rate(sends), query_rate_hz=_rate(queries),
                    reply_rate_hz=_rate([item.received_at for item in replies]),
                    reply_latency_ms=_distribution(latencies), send_gap_ms=_distribution(gaps),
                    max_send_gap_ms=max(gaps, default=None), real_sample_rate_hz=_rate(list(sources.values())),
                    dropped_queries=dropped, sent_count=transmissions['cursor'], query_count=query_count,
                    reply_count=frames['cursor'], cache_sequence=frames['cursor'], cache_capacity=self.capacity,
                    cache_overwrites=max(0, frames['cursor'] - self.capacity),
                    connected=frames['connected'], error=frames['error'])


def _rate(values):
    return ((len(values) - 1) / (values[-1] - values[0])
            if len(values) > 1 and values[-1] > values[0] else None)


def _distribution(values):
    ordered = sorted(values)
    return dict(median=statistics.median(ordered) if ordered else None,
                p99=ordered[max(0, math.ceil(len(ordered) * .99) - 1)] if ordered else None,
                max=ordered[-1] if ordered else None)


def _worker_main(device, reply_timeout, cache, commands, results, stop_results,
                 stop_revision, send_gate, closed, port_factory):
    command_revision = 0
    handled_stop = 0
    master = None

    @contextmanager
    def guard(kind):
        with send_gate:
            if closed.is_set():
                raise RequestCancelled('Feedback owner is closing')
            if kind not in (QUERY, STOP) and command_revision != stop_revision.value:
                raise RequestCancelled('Control request cancelled by STOP')
            yield

    try:
        with closing(port_factory(device, reply_timeout)) as port:
            master = RkMaster(port, reply_timeout=reply_timeout, send_guard=guard,
                              on_send=cache.record_send, on_reply=cache.publish)
            master.discover()
            while not closed.is_set():
                parent = mp.parent_process()
                if parent is not None and not parent.is_alive():
                    break
                revision = stop_revision.value
                if revision != handled_stop:
                    try:
                        result = master.stop(proof_timeout=STOP_PROOF_TIMEOUT_S)
                        stop_results.put(revision, True, result)
                    except (ValueError, LinkError) as exc:
                        stop_results.put(revision, False, error_record(exc))
                        if master.failed:
                            raise
                    if master.failed:
                        cache.disconnect(result['error'])
                        break
                    handled_stop = revision
                else:
                    try:
                        identifier, operation, command_revision, payload = commands.get_nowait()
                    except queue.Empty:
                        master.query()
                    else:
                        try:
                            if command_revision != stop_revision.value:
                                raise RequestCancelled('Control request cancelled by STOP')
                            if operation == 'upload':
                                result = master.upload(payload[0], payload[1])
                            elif operation == 'start':
                                result = master.start(payload)
                            elif operation == 'renew':
                                result = master.renew(payload)
                            else:
                                raise ValueError('Unknown feedback operation')
                            results.put(identifier, True, result)
                        except (ValueError, LinkError) as exc:
                            results.put(identifier, False, error_record(exc))
                            if master.failed:
                                raise
                cache.set_dropped(master.dropped_queries)
    except Exception as exc:
        if not closed.is_set():
            cache.disconnect(error_record(exc))
    finally:
        if master is not None:
            cache.set_dropped(master.dropped_queries)
        with cache.lock:
            cache.connected.value = False


def _raise_error(record):
    exchange = record.get('exchange')
    if exchange is not None:
        exchange = dict(exchange)
        exchange['status'] = Telemetry(**exchange['status'])
        exchange = Exchange(**exchange)
    if record.get('type') in ('ValueError', 'RequestCancelled'):
        raise RequestCancelled(record['message']) if record['type'] == 'RequestCancelled' else ValueError(record['message'])
    error = Rejected if record.get('type') == 'Rejected' else LinkError
    raise error(record.get('message', 'Feedback worker failed'), exchange, details=record.get('details'))


class FeedbackWorker:
    """Navigation-side proxy. query/discover only wait for shared memory changes."""
    def __init__(self, device, *, reply_timeout=.100, context=None, port_factory=None):
        if not math.isfinite(reply_timeout) or reply_timeout <= 0:
            raise ValueError('Reply timeout must be finite and positive')
        self.device, self.reply_timeout = device, reply_timeout
        self.context = context or mp.get_context('spawn')
        self.cache = SharedFeedbackCache(self.context)
        self.commands = self.context.Queue(maxsize=1)
        self.results, self.stop_results = _Mailbox(self.context), _Mailbox(self.context)
        self._stop_revision = self.context.RawValue('Q', 0)
        self.send_gate = self.context.Lock()
        self.closed = self.context.Event()
        self.process = None
        self.port_factory = port_factory or open_port
        self.rpc_lock = threading.Lock()
        self.query_cursor = self.command_id = self.last_stop_waited = 0
        self.local_error = None
        self.sleep = time.sleep

    def launch(self):
        if self.process is not None:
            raise ValueError('Feedback worker already launched')
        self.process = self.context.Process(target=_worker_main,
            args=(self.device, self.reply_timeout, self.cache, self.commands, self.results,
                  self.stop_results, self._stop_revision, self.send_gate, self.closed, self.port_factory),
            name='v12-serial-owner')
        self.process.start()
        return self

    @property
    def stop_revision(self):
        return self._stop_revision.value

    @property
    def latest(self):
        return self.snapshot()['latest']

    @property
    def failed(self):
        return bool(self.local_error or self.cache.failed.value or
                    self.process is not None and not self.process.is_alive() and not self.closed.is_set())

    def _check_owner(self):
        if self.local_error:
            _raise_error(self.local_error)
        if self.process is None:
            raise LinkError('Feedback worker is not launched')
        if self.closed.is_set() or not self.process.is_alive() or self.cache.failed.value:
            state = self.cache.snapshot()
            _raise_error(state['error'] or dict(type='LinkError', message='Feedback worker disconnected'))

    def snapshot(self):
        result = self.cache.snapshot()
        if self.local_error is not None:
            result['connected'], result['error'] = False, self.local_error
        if self.process is not None and not self.process.is_alive():
            result['connected'] = False
            result['error'] = result['error'] or dict(type='LinkError', message='Feedback worker disconnected')
        return result

    def history_since(self, cursor):
        result = self.cache.history_since(cursor)
        if self.local_error is not None:
            result['connected'], result['error'] = False, self.local_error
        if self.process is not None and not self.process.is_alive():
            result['connected'] = False
            result['error'] = result['error'] or dict(type='LinkError', message='Feedback worker disconnected')
        return result

    def transmissions_since(self, cursor):
        return self.cache.transmissions_since(cursor)

    def diagnostics(self):
        result = self.cache.diagnostics()
        result['worker_alive'] = self.process is not None and self.process.is_alive()
        result['connected'] = result['connected'] and result['worker_alive']
        if self.local_error is not None:
            result['connected'], result['error'] = False, self.local_error
        if not result['worker_alive'] and self.process is not None:
            result['error'] = result['error'] or dict(type='LinkError', message='Feedback worker disconnected')
        return result

    def discover(self):
        deadline = time.monotonic() + 5.
        while True:
            self._check_owner()
            state = self.snapshot()
            if state['latest'] is not None:
                self.query_cursor = state['cursor']
                return state['latest']
            if time.monotonic() >= deadline:
                raise LinkError('Feedback worker discovery did not complete')
            self.sleep(.002)

    def query(self, *, start_before=None):
        deadline = time.monotonic() + self.reply_timeout
        if start_before is not None:
            deadline = min(deadline, start_before)
        while True:
            self._check_owner()
            state = self.snapshot()
            if state['cursor'] > self.query_cursor:
                self.query_cursor = state['cursor']
                return state['latest']
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self.sleep(min(.002, remaining))

    def _wait_result(self, mailbox, identifier, timeout):
        deadline = time.monotonic() + timeout
        while True:
            result = mailbox.get()
            if result is not None and result[0] >= identifier:
                if not result[1]:
                    _raise_error(result[2])
                return result[2]
            self._check_owner()
            if time.monotonic() >= deadline:
                self.local_error = dict(type='LinkError', message='Feedback operation did not complete')
                _raise_error(self.local_error)
            self.sleep(.002)

    def _submit(self, operation, payload, revision, timeout):
        with self.rpc_lock:
            self._check_owner()
            self.command_id += 1
            identifier = self.command_id
            revision = self.stop_revision if revision is None else revision
            self.commands.put_nowait((identifier, operation, revision, payload))
            return self._wait_result(self.results, identifier, timeout)

    def upload(self, document, task=None, *, on_ack=None, revision=None):
        plan = document if isinstance(document, QueuePlan) else QueuePlan.from_document(document)
        append_timeout = max(self.reply_timeout, APPEND_REPLY_TIMEOUT_S)
        timeout = (len(plan.segments) * (append_timeout + PERIOD_S)
                   + 2 * (self.reply_timeout + PERIOD_S) + STOP_PROOF_TIMEOUT_S + 2.)
        acks = self._submit('upload', (plan, task), revision, timeout)
        if on_ack is not None:
            for ack in acks:
                on_ack(ack)
        return acks

    def start(self, task, *, revision=None):
        return self._submit('start', task, revision, self.reply_timeout + STOP_PROOF_TIMEOUT_S + 2.)

    def renew(self, task, *, revision=None):
        return self._submit('renew', task, revision, self.reply_timeout + STOP_PROOF_TIMEOUT_S + 2.)

    def request_stop(self):
        self._check_owner()
        with self.send_gate:
            self._stop_revision.value += 1
            return self._stop_revision.value

    def stop(self, *, proof_timeout=3., on_reply=None):
        if proof_timeout != 3.:
            raise ValueError('Feedback worker uses the existing 3 second stop-proof window')
        revision = self.stop_revision
        if revision <= self.last_stop_waited:
            revision = self.request_stop()
        result = self._wait_result(self.stop_results, revision, proof_timeout + self.reply_timeout + 2.)
        self.last_stop_waited = revision
        if on_reply is not None:
            for key in ('ack', 'proof'):
                raw = result.get(key)
                if raw is not None:
                    raw = dict(raw)
                    raw['status'] = Telemetry(**raw['status'])
                    on_reply(Exchange(**raw))
        return result

    def close(self):
        self.closed.set()
        if self.process is not None:
            self.process.join(timeout=4.)
            if self.process.is_alive():
                self.process.terminate()
                self.process.join(timeout=2.)
        self.commands.close()
        self.commands.join_thread()
