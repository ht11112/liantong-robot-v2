"""Read-only v10 static evidence capture and offline analysis (Python 3.8+).

Default operation is analyze; only capture --execute --port opens a serial port.
No UART write, flush, protocol command or automatic reset operation exists.
"""
import argparse
from bisect import bisect_left
from dataclasses import asdict
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import statistics
import struct
import sys
import time


CODEC_PATH = Path(__file__).resolve().with_name('mcu_v10.py')
_SPEC = importlib.util.spec_from_file_location('_passive_mcu_v10', CODEC_PATH)
codec = importlib.util.module_from_spec(_SPEC)
sys.modules[_SPEC.name] = codec
_SPEC.loader.exec_module(codec)
LAYOUT = '<QIIIIIQIBBHHHffffffIIhhHIIHHI'
MASK32 = 0xFFFFFFFF
LIMITS = dict(rate_min_hz=19., rate_max_hz=21., median_min_ms=49.,
              median_max_ms=51., p99_max_ms=60., interval_max_ms=100.,
              source_age_max_ms_exclusive=250, stationary_rpm_tenths_max=2,
              minimum_complete_frame_span_s=60.)


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def software_identity():
    return {'passive_capture.py': sha256(__file__), 'mcu_v10.py': sha256(CODEC_PATH)}


def finite(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)


def save_json(path, document):
    with Path(path).open('w', encoding='utf-8', newline='\n') as output:
        json.dump(document, output, indent=2, ensure_ascii=False, allow_nan=False)
        output.write('\n')
        output.flush()
        os.fsync(output.fileno())


class ReadOnlyViolation(RuntimeError):
    pass


class ReadOnlyReader:
    """Capability boundary: exposes read/close and denies every outgoing API."""
    def __init__(self, reader):
        self._reader = reader
        self.forbidden_attempts = []

    def read(self, size):
        return self._reader.read(size)

    def close(self):
        close = getattr(self._reader, 'close', None)
        if close is not None:
            close()

    def _forbid(self, name):
        self.forbidden_attempts.append(name)
        raise ReadOnlyViolation('Passive transport forbids ' + name)

    def write(self, *args, **kwargs):
        return self._forbid('write')

    def flush(self, *args, **kwargs):
        return self._forbid('flush')

    def reset_input_buffer(self, *args, **kwargs):
        return self._forbid('reset_input_buffer')

    def reset_output_buffer(self, *args, **kwargs):
        return self._forbid('reset_output_buffer')

    def send_break(self, *args, **kwargs):
        return self._forbid('send_break')


def serial_reader(port):
    # Lazily loaded only by an explicitly authorized capture CLI invocation.
    import serial
    connection = serial.Serial(port=None, baudrate=codec.BAUD, bytesize=8,
                               parity='N', stopbits=1, timeout=.10,
                               xonxoff=False, rtscts=False, dsrdtr=False,
                               exclusive=True)
    # Set before open: no reset pulse or RTS transmit control is requested.
    connection.dtr = False
    connection.rts = False
    connection.port = port
    try:
        connection.open()
    except BaseException:
        connection.close()
        raise
    return connection


def _duration(value):
    if not finite(value) or not 0 < value <= 3600:
        raise ValueError('Duration must be finite and within (0,3600] seconds')
    return float(value)


def capture_stream(reader, output_dir, duration_s=65., *, monotonic=time.monotonic,
                   wall=time.time, source_label='injected read-only source',
                   expected_boot_id=None, expected_epoch=None):
    """Offline-testable read-only acquisition; provided reader is never opened here."""
    return _capture(lambda: reader, output_dir, duration_s, monotonic, wall,
                    source_label, expected_boot_id, expected_epoch)


def _capture(opener, output_dir, duration_s, monotonic, wall, source_label,
             expected_boot_id, expected_epoch):
    duration_s = _duration(duration_s)
    folder = Path(output_dir)
    folder.mkdir(parents=True, exist_ok=False)
    started, started_wall = monotonic(), wall()
    info = dict(schema='mcu-v10-passive-capture-v1', source_label=source_label,
        duration_requested_s=duration_s, read_size=codec.TELEMETRY_SIZE,
        baudrate=codec.BAUD, start_monotonic_s=started, start_wall_s=started_wall,
        capture_complete=False, read_error=None, cleanup_errors=[], read_count=0,
        byte_count=0, forbidden_attempts=[], outbound_operations=0,
        serial_commands_supported=False, serial_flush_supported=False,
        raw_flush_after_every_read=True, fsync_at_close=True,
        expected_boot_id=expected_boot_id, expected_epoch=expected_epoch,
        software_sha256=software_identity(), operation_start_monotonic_s=started)
    save_json(folder / 'capture.json', info)
    reader = None
    with (folder / 'raw.bin').open('xb') as raw, (folder / 'chunks.jsonl').open('x', encoding='utf-8', newline='\n') as chunks:
        try:
            supplied = opener()
            reader = supplied if isinstance(supplied, ReadOnlyReader) else ReadOnlyReader(supplied)
            started, started_wall = monotonic(), wall()
            info.update(start_monotonic_s=started, start_wall_s=started_wall)
            while monotonic() - started < duration_s:
                read_started = monotonic()
                try:
                    data = reader.read(codec.TELEMETRY_SIZE)
                except BaseException as exc:
                    receipt, receipt_wall = monotonic(), wall()
                    entry = dict(read_index=info['read_count'], read_start_monotonic_s=read_started,
                        receipt_monotonic_s=receipt, receipt_wall_s=receipt_wall,
                        offset=info['byte_count'], length=0,
                        read_error=type(exc).__name__ + ': ' + str(exc), raw_flushed=True)
                    chunks.write(json.dumps(entry, allow_nan=False) + '\n')
                    chunks.flush()
                    info['read_count'] += 1
                    raise
                receipt, receipt_wall = monotonic(), wall()
                if not isinstance(data, (bytes, bytearray)):
                    raise TypeError('read() must return original bytes')
                data = bytes(data)
                raw.write(data)
                raw.flush()
                entry = dict(read_index=info['read_count'], read_start_monotonic_s=read_started,
                    receipt_monotonic_s=receipt, receipt_wall_s=receipt_wall,
                    offset=info['byte_count'], length=len(data), raw_flushed=True)
                chunks.write(json.dumps(entry, allow_nan=False) + '\n')
                chunks.flush()
                info['read_count'] += 1
                info['byte_count'] += len(data)
            info['capture_complete'] = True
        except BaseException as exc:
            info['read_error'] = dict(type=type(exc).__name__, message=str(exc))
        finally:
            if reader is not None:
                info['forbidden_attempts'] = list(reader.forbidden_attempts)
                if reader.forbidden_attempts:
                    info['capture_complete'] = False
                try:
                    reader.close()
                except BaseException as exc:
                    info['cleanup_errors'].append(type(exc).__name__ + ': ' + str(exc))
                    info['capture_complete'] = False
            raw.flush()
            chunks.flush()
            os.fsync(raw.fileno())
            os.fsync(chunks.fileno())
    info.update(end_monotonic_s=monotonic(), end_wall_s=wall())
    info['elapsed_s'] = info['end_monotonic_s'] - started
    info['capture_complete'] = bool(info['capture_complete'] and info['elapsed_s'] >= duration_s
                                    and not info['read_error'] and not info['cleanup_errors'])
    info['raw_sha256'], info['chunks_sha256'] = sha256(folder / 'raw.bin'), sha256(folder / 'chunks.jsonl')
    save_json(folder / 'capture.json', info)
    report, events = analyze_files(folder / 'raw.bin', folder / 'chunks.jsonl', info)
    _write_analysis(folder, report, events)
    return report


def _chunk_index(raw, path):
    chunks, errors = [], []
    offset, previous_receipt = 0, None
    try:
        lines = Path(path).read_text(encoding='utf-8').splitlines()
    except (OSError, UnicodeError) as exc:
        return [], ['chunks_unreadable: ' + str(exc)]
    for number, line in enumerate(lines):
        try:
            item = json.loads(line)
            length = item['length']
            if (not isinstance(length, int) or isinstance(length, bool) or length < 0
                    or item['offset'] != offset or item['read_index'] != number
                    or item.get('raw_flushed') is not True
                    or not finite(item['read_start_monotonic_s'])
                    or not finite(item['receipt_monotonic_s']) or not finite(item['receipt_wall_s'])
                    or item['receipt_monotonic_s'] < item['read_start_monotonic_s']
                    or (previous_receipt is not None and item['read_start_monotonic_s'] < previous_receipt)
                    or offset + length > len(raw)):
                raise ValueError('offset, read identity, timing or flush invariant differs')
            offset += length
            previous_receipt = item['receipt_monotonic_s']
            if item.get('read_error'):
                errors.append('read_error_in_chunks: ' + str(item['read_error']))
            chunks.append(item)
        except (ValueError, KeyError, TypeError) as exc:
            errors.append('invalid_chunk_line_{}: {}'.format(number + 1, exc))
    if offset != len(raw):
        errors.append('raw_byte_coverage_mismatch: chunks={} raw={}'.format(offset, len(raw)))
    return chunks, errors


def _scan(raw, chunks):
    """Retain all bytes; malformed candidates are logged before resynchronizing."""
    nonempty = [item for item in chunks if item['length']]
    ends = [item['offset'] + item['length'] for item in nonempty]
    events, position = [], 0

    def event(kind, start, length, **extra):
        end = start + length
        index = bisect_left(ends, end)
        chunk = nonempty[index] if index < len(nonempty) else None
        value = dict(kind=kind, offset=start, length=length,
            raw_hex=raw[start:end].hex(), receipt_semantics='chunk containing final byte',
            receipt_monotonic_s=None if chunk is None else chunk['receipt_monotonic_s'],
            receipt_wall_s=None if chunk is None else chunk['receipt_wall_s'],
            final_byte_read_index=None if chunk is None else chunk['read_index'])
        value.update(extra)
        events.append(value)
        return value

    while position < len(raw):
        start = raw.find(b'\xa5\x5a', position)
        if start < 0:
            event('noise', position, len(raw)-position)
            break
        if start > position:
            event('noise', position, start-position)
        position = start
        if len(raw)-position < 6:
            event('truncated_header', position, len(raw)-position)
            break
        version, opcode, payload_length = struct.unpack_from('<BBH', raw, position+2)
        length = payload_length+8
        if version != codec.VERSION or not 8 <= length <= codec.TELEMETRY_SIZE:
            event('invalid_header', position, 2, version=version, opcode=opcode,
                  declared_payload_length=payload_length)
            position += 2
            continue
        if len(raw)-position < length:
            event('truncated_frame', position, len(raw)-position, declared_length=length)
            break
        packet = raw[position:position+length]
        stored, calculated = struct.unpack_from('<H', packet, length-2)[0], codec.crc16(packet[2:-2])
        if stored != calculated:
            # Candidate spans may overlap later resynchronization events; raw.bin is authoritative.
            event('crc_error', position, length, crc_stored=stored, crc_calculated=calculated)
            position += 1
            continue
        if opcode != codec.TELEMETRY or length != codec.TELEMETRY_SIZE:
            event('unexpected_frame', position, length, opcode=opcode, crc_valid=True)
        else:
            value = codec.Telemetry(*struct.unpack_from(LAYOUT, packet, 6))
            decoded = {key:None if isinstance(number,float) and not math.isfinite(number) else number
                       for key,number in asdict(value).items()}
            item = event('telemetry', position, length, crc_valid=True,
                         decoded=decoded, metadata_valid=True, metadata_error=None)
            try:
                codec.decode_telemetry(packet)
            except ValueError as exc:
                item['metadata_valid'], item['metadata_error'] = False, str(exc)
        position += length
    frames = [item for item in events if item['kind'] == 'telemetry']
    if frames and events[0]['kind'] == 'noise' and events[0]['offset'] == 0 and events[0]['length'] <= 109:
        events[0]['kind'] = 'boundary_prefix'
        events[0]['reason'] = 'bounded initial fragment before first complete frame; original bytes retained'
    if frames and events[-1]['kind'] in ('noise','truncated_header','truncated_frame'):
        tail = events[-1]
        packet = bytes.fromhex(tail['raw_hex'])
        header = b'\xa5\x5a\x0a\x80\x66\x00'
        if 0 < len(packet) <= 109 and packet[:min(6,len(packet))] == header[:min(6,len(packet))]:
            tail['kind'] = 'boundary_suffix'
            tail['reason'] = 'unfinished final telemetry frame at capture end; original bytes retained'
    return events


def _stats(intervals_ms):
    if not intervals_ms:
        return dict(interval_count=0, rate_hz=None, median_ms=None, p99_ms=None, max_ms=None, min_ms=None)
    values = sorted(intervals_ms)
    total = sum(values)
    return dict(interval_count=len(values), rate_hz=None if total <= 0 else len(values)*1000/total,
                median_ms=statistics.median(values), p99_ms=values[max(0, math.ceil(len(values)*.99)-1)],
                max_ms=values[-1], min_ms=values[0])


def _timing_ok(value):
    return (value['rate_hz'] is not None and LIMITS['rate_min_hz'] <= value['rate_hz'] <= LIMITS['rate_max_hz']
            and LIMITS['median_min_ms'] <= value['median_ms'] <= LIMITS['median_max_ms']
            and value['p99_ms'] <= LIMITS['p99_max_ms'] + 1e-6
            and value['max_ms'] <= LIMITS['interval_max_ms'] + 1e-6
            and value['min_ms'] > 0)


def analyze_files(raw_path, chunks_path, capture_info):
    raw_path, chunks_path = Path(raw_path), Path(chunks_path)
    raw = raw_path.read_bytes()
    chunks, errors = _chunk_index(raw, chunks_path)
    events = _scan(raw, chunks)
    frames = [item for item in events if item['kind'] == 'telemetry']
    counts = {kind: sum(item['kind'] == kind for item in events) for kind in
              ('telemetry', 'noise', 'crc_error', 'invalid_header', 'truncated_header', 'truncated_frame',
               'unexpected_frame', 'boundary_prefix', 'boundary_suffix')}
    counts['metadata_error'] = sum(not item['metadata_valid'] for item in frames)
    publication = dict(duplicates=0, missing=0, backwards=0, tick_backwards_or_equal=0)
    samples = dict(distinct_observations=0, unchanged_repeats=0, changed_repeats=0,
                   skipped_observations=0, backwards=0, tick_backwards_or_equal=0,
                   nominal_rate_hz=None, sequence_span=0, measurement_span_ms=0,
                   skipped_observations_are_not_proof_of_MCU_loss=True)
    pub_intervals, host_intervals, ages, boots, epochs = [], [], [], [], []
    previous, previous_sample = None, None
    source_valid, stationary, errors_zero, idle, no_commands = True, True, True, True, True
    for item in frames:
        value, receipt = item['decoded'], item['receipt_monotonic_s']
        identity = (value['boot'], value['epoch'])
        if value['boot'] not in boots:
            boots.append(value['boot'])
        if identity not in epochs:
            epochs.append(identity)
        age = (value['publication_tick_ms']-value['measurement_tick_ms']) & MASK32
        ages.append(age)
        source_valid &= bool(value['flags'] & 1 and value['flags'] & 2 and age < 250)
        stationary &= max(abs(value['left_tenths_rpm']), abs(value['right_tenths_rpm'])) <= 2
        errors_zero &= value['first_error'] == 0 and value['first_drive_error'] == 0 and not value['flags'] & 32
        idle &= value['state'] == 0 and value['count'] == 0 and value['index'] == 0
        no_commands &= (value['task'] == 0 and value['ack_request'] == 0 and value['ack_result'] == 0
                        and value['accepted_request_watermark'] == 0 and not value['flags'] & (4 | 8 | 64 | 128 | 256))
        if previous is not None and identity == previous['identity']:
            seq_step = (value['publication_sequence']-previous['value']['publication_sequence']) & MASK32
            if not seq_step:
                publication['duplicates'] += 1
            elif seq_step >= 0x80000000:
                publication['backwards'] += 1
            elif seq_step > 1:
                publication['missing'] += seq_step-1
            tick = (value['publication_tick_ms']-previous['value']['publication_tick_ms']) & MASK32
            if not 0 < tick < 0x80000000:
                publication['tick_backwards_or_equal'] += 1
            else:
                pub_intervals.append(tick)
            if receipt is not None and previous['receipt'] is not None:
                host_intervals.append((receipt-previous['receipt'])*1000)
        previous = dict(identity=identity, value=value, receipt=receipt)
        packet = bytes.fromhex(item['raw_hex'])
        signature = packet[30:34]+packet[58:90]
        if previous_sample is None or identity != previous_sample['identity']:
            samples['distinct_observations'] += 1
            item['sample_observation'] = 'first_in_identity'
        else:
            step = (value['sample_sequence']-previous_sample['value']['sample_sequence']) & MASK32
            tick = (value['measurement_tick_ms']-previous_sample['value']['measurement_tick_ms']) & MASK32
            if step == 0:
                if signature != previous_sample['signature']:
                    samples['changed_repeats'] += 1
                    item['sample_observation'] = 'same_sequence_changed_source_or_values'
                else:
                    samples['unchanged_repeats'] += 1
                    item['sample_observation'] = 'same_original_sample'
            elif step >= 0x80000000:
                samples['backwards'] += 1
                item['sample_observation'] = 'sequence_backwards'
            else:
                samples['distinct_observations'] += 1
                samples['skipped_observations'] += step-1
                item['sample_observation'] = 'forward_original_sample'
                if not 0 < tick < 0x80000000:
                    samples['tick_backwards_or_equal'] += 1
                else:
                    samples['sequence_span'] += step
                    samples['measurement_span_ms'] += tick
        previous_sample = dict(identity=identity, value=value, signature=signature)
    if samples['measurement_span_ms']:
        samples['nominal_rate_hz'] = samples['sequence_span']*1000/samples['measurement_span_ms']
    for name in ('raw', 'chunks'):
        expected = capture_info.get(name+'_sha256')
        path = raw_path if name == 'raw' else chunks_path
        if expected is not None and expected != sha256(path):
            errors.append(name+'_SHA_mismatch')
    expected_boot, expected_epoch = capture_info.get('expected_boot_id'), capture_info.get('expected_epoch')
    expected_identity = ((expected_boot is None or boots == [expected_boot])
                         and (expected_epoch is None or all(epoch == expected_epoch for _, epoch in epochs)))
    started, ended = capture_info.get('start_monotonic_s'), capture_info.get('end_monotonic_s')
    requested = capture_info.get('duration_requested_s', 65.)
    duration_valid = (finite(started) and finite(ended) and finite(requested) and requested > 0
                      and ended-started >= requested)
    complete = bool(capture_info.get('capture_complete') is True and duration_valid
                    and not capture_info.get('read_error') and not capture_info.get('cleanup_errors')
                    and not capture_info.get('forbidden_attempts') and capture_info.get('outbound_operations') == 0)
    coverage, full_frame_span = False, None
    if frames and duration_valid:
        first_receipt, last_receipt = frames[0]['receipt_monotonic_s'], frames[-1]['receipt_monotonic_s']
        full_frame_span = None if first_receipt is None or last_receipt is None else last_receipt-first_receipt
        coverage = (first_receipt is not None and last_receipt is not None
                    and 0 <= first_receipt-started <= .1+1e-9 and 0 <= ended-last_receipt <= .1+1e-9
                    and full_frame_span >= LIMITS['minimum_complete_frame_span_s']-1e-9)
    integrity = (not errors and len(frames) >= 2
                 and counts['telemetry'] + counts['boundary_prefix'] + counts['boundary_suffix'] == len(events)
                 and not counts['metadata_error'] and all(not value for value in publication.values())
                 and samples['distinct_observations'] >= 2 and samples['changed_repeats'] == 0
                 and samples['backwards'] == 0 and samples['tick_backwards_or_equal'] == 0
                 and len(boots) == 1 and len(epochs) == 1 and source_valid)
    timing = dict(MCU_publication=_stats(pub_intervals), host_receipt=_stats(host_intervals))
    checks = dict(capture_complete=complete, data_integrity=integrity, full_duration_coverage=coverage,
                  stable_expected_boot_epoch=expected_identity and len(boots) == 1 and len(epochs) == 1,
                  MCU_publication_timing=_timing_ok(timing['MCU_publication']),
                  host_receipt_timing=_timing_ok(timing['host_receipt']),
                  sample_sources_initialized_valid_fresh=source_valid and bool(frames),
                  stationary_RPM=stationary and bool(frames), zero_errors_no_UART=errors_zero and bool(frames),
                  idle_empty_queue=idle and bool(frames),
                  no_command_evidence=no_commands and bool(frames) and capture_info.get('outbound_operations') == 0
                                      and not capture_info.get('forbidden_attempts'))
    drift = None
    if frames and all(finite(item['decoded'][key]) for item in frames for key in ('x','y','yaw')):
        first, last = frames[0]['decoded'], frames[-1]['decoded']
        dx, dy = last['x']-first['x'], last['y']-first['y']
        drift = dict(first_pose={k:first[k] for k in ('x','y','yaw')},
                     last_pose={k:last[k] for k in ('x','y','yaw')}, dx_m=dx, dy_m=dy,
                     displacement_m=math.hypot(dx,dy),
                     yaw_change_rad=math.atan2(math.sin(last['yaw']-first['yaw']),math.cos(last['yaw']-first['yaw'])),
                     maximum_distance_from_first_m=max(math.hypot(item['decoded']['x']-first['x'],
                         item['decoded']['y']-first['y']) for item in frames),
                     automatic_zero_performed=False, acceptance_threshold_applied=False)
    report = dict(schema='mcu-v10-passive-analysis-v1', acceptance_scope='passive_telemetry_only',
        capture_complete=complete, data_valid=bool(complete and integrity), acceptance_pass=all(checks.values()),
        checks=checks, failure_checks=[name for name, okay in checks.items() if not okay],
        acquisition_errors=errors, capture_error=capture_info.get('read_error'),
        forbidden_attempts=capture_info.get('forbidden_attempts', []),
        limits=dict(LIMITS), duration_requested_s=requested,
        elapsed_s=None if not duration_valid else ended-started,
        complete_frame_span_s=full_frame_span,
        raw_bytes=len(raw), raw_sha256=sha256(raw_path), chunks_sha256=sha256(chunks_path),
        read_count=len(chunks), event_counts=counts, publication_sequence=publication,
        samples=samples, source_age_ms=dict(min=None if not ages else min(ages), max=None if not ages else max(ages)),
        timing=timing, boots=[str(value) for value in boots], boot_epochs=[dict(boot=str(boot),epoch=epoch) for boot,epoch in epochs],
        expected_boot_id=None if expected_boot is None else str(expected_boot), expected_epoch=expected_epoch,
        odometry_drift=drift, source_label=capture_info.get('source_label'),
        captured_software_sha256=capture_info.get('software_sha256'), analysis_software_sha256=software_identity(),
        driver_guard_verified_by_tool=False, physical_stop_verified_by_tool=False,
        startup_STOP_CONFIRMED_required=False, UART_outgoing_operations=capture_info.get('outbound_operations'),
        no_command_evidence_is_not_an_external_bus_sniffer=True,
        receipt_semantics='original monotonic return time of the read containing the frame final byte',
        startup_errors_or_other_boots_ignored=False)
    return report, events


def _write_analysis(folder, report, events):
    with (Path(folder)/'frames.jsonl').open('x', encoding='utf-8', newline='\n') as output:
        for item in events:
            output.write(json.dumps(item, ensure_ascii=False, allow_nan=False) + '\n')
        output.flush()
        os.fsync(output.fileno())
    save_json(Path(folder)/'report.json', report)


def analyze_capture(input_dir, output_dir=None, *, expected_boot_id=None, expected_epoch=None):
    source = Path(input_dir)
    info = json.loads((source/'capture.json').read_text(encoding='utf-8'))
    captured_boot, captured_epoch = info.get('expected_boot_id'), info.get('expected_epoch')
    if expected_boot_id is not None:
        info['expected_boot_id'] = expected_boot_id
    if expected_epoch is not None:
        info['expected_epoch'] = expected_epoch
    report, events = analyze_files(source/'raw.bin', source/'chunks.jsonl', info)
    report['capture_expected_identity'] = dict(boot=None if captured_boot is None else str(captured_boot),epoch=captured_epoch)
    if ((expected_boot_id is not None and captured_boot is not None and expected_boot_id != captured_boot)
            or (expected_epoch is not None and captured_epoch is not None and expected_epoch != captured_epoch)):
        report['checks']['capture_vs_analysis_expected_identity'] = False
        report['failure_checks'].append('capture_vs_analysis_expected_identity')
        report['acceptance_pass'] = False
    if output_dir is not None:
        destination = Path(output_dir)
        destination.mkdir(parents=True, exist_ok=False)
        report['analyzed_input_directory'] = str(source.resolve())
        _write_analysis(destination, report, events)
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', nargs='?', default='analyze', choices=('analyze','capture'))
    parser.add_argument('--input-dir')
    parser.add_argument('--output-dir')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port')
    parser.add_argument('--duration', type=float, default=65.)
    parser.add_argument('--expected-boot', '--expected-boot-id', dest='expected_boot_id', type=lambda value:int(value,0))
    parser.add_argument('--expected-epoch', type=int)
    args = parser.parse_args(argv)
    if args.operation == 'analyze':
        if not args.input_dir or args.execute or args.port:
            parser.error('Offline analyze requires --input-dir; no --execute or --port is allowed')
        report = analyze_capture(args.input_dir, args.output_dir,
                                 expected_boot_id=args.expected_boot_id,expected_epoch=args.expected_epoch)
    else:
        if not args.execute or not args.port or not args.output_dir or args.input_dir:
            parser.error('Serial capture requires capture --execute --port --output-dir')
        report = _capture(lambda: serial_reader(args.port), args.output_dir, args.duration,
                          time.monotonic, time.time, 'exclusive passive serial: '+args.port,
                          args.expected_boot_id, args.expected_epoch)
    print(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False))
    return 0 if report['acceptance_pass'] else 2


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit('Passive capture/analyze failed: ' + str(exc))
