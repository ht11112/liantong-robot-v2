"""Read-only V12 feedback benchmark; hardware access requires --execute."""

import argparse
from dataclasses import asdict
import json
import math
from pathlib import Path
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/'app'), str(ROOT/'mcu/tools')]
from mcu_v11 import QUERY
from v11_map_plan import PlanningError, load_map
from v12_feedback_worker import FeedbackWorker


def percentile(values, percentile_value):
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(percentile_value*len(ordered))-1)]


def phase_report(name, duration, transmissions, exchanges, missed, planning, diagnostics):
    times = [entry['sent_at'] for entry in transmissions]
    gaps = [1000*(b-a) for a, b in zip(times, times[1:])]
    latencies = [1000*(reply.received_at-reply.sent_at) for reply in exchanges]
    samples = {(reply.status.boot, reply.status.epoch, reply.status.sample_sequence)
               for reply in exchanges if reply.status.source_valid}
    opcodes = {}
    for entry in transmissions:
        key = str(entry['opcode'])
        opcodes[key] = opcodes.get(key, 0)+1
    sample_ticks = {}
    for reply in exchanges:
        status = reply.status
        if status.source_valid:
            sample_ticks.setdefault((status.boot, status.epoch, status.sample_sequence), reply.sent_at)
    real_times = list(sample_ticks.values())
    rate = ((len(times)-1)/(times[-1]-times[0]) if len(times)>1 else None)
    real_rate = ((len(real_times)-1)/(real_times[-1]-real_times[0]) if len(real_times)>1 else None)
    statuses = [reply.status for reply in exchanges]
    return dict(phase=name, elapsed_s=duration, tx_count=len(transmissions), reply_count=len(exchanges),
                tx_rate_hz=rate, query_only=set(opcodes).issubset({str(QUERY)}), opcode_counts=opcodes,
                tx_gap_ms=dict(median=statistics.median(gaps) if gaps else None,
                               p99=percentile(gaps, .99), maximum=max(gaps, default=None)),
                reply_latency_ms=dict(median=statistics.median(latencies) if latencies else None,
                                      p99=percentile(latencies, .99), maximum=max(latencies, default=None)),
                distinct_samples=len(samples), real_sample_rate_hz=real_rate,
                max_source_age_ms=max((s.source_age_ms for s in statuses), default=None),
                invalid_source_replies=sum(not s.source_valid for s in statuses),
                moving_replies=sum(bool(s.flags & 4) or max(abs(s.left_tenths_rpm), abs(s.right_tenths_rpm))>2
                                   for s in statuses),
                drive_fault_replies=sum(bool(s.first_drive_error) for s in statuses),
                mcu_fault_replies=sum(s.first_error not in (0, 1) for s in statuses),
                boots=sorted({str(s.boot) for s in statuses}),
                cache_missed=missed, planning=planning, diagnostics=diagnostics)


def run_phase(worker, name, duration, stream, grid=None, start=None, goal=None):
    cursor = worker.snapshot()['cursor']
    tx_cursor = worker.transmissions_since(0)['cursor']
    began = time.monotonic()
    replies, transmissions = [], []
    missed = dict(feedback=0, transmissions=0)
    planning = dict(attempts=0, successful=0, failures=0, max_duration_ms=0., last_error=None)
    while time.monotonic()-began < duration:
        if name == 'planning':
            attempt = time.monotonic()
            planning['attempts'] += 1
            try:
                plan = grid.plan(start, goal)
                grid.validate_plan(start, plan)
                planning['successful'] += 1
            except PlanningError as exc:
                planning['failures'] += 1
                planning['last_error'] = str(exc)
            planning['max_duration_ms'] = max(planning['max_duration_ms'], (time.monotonic()-attempt)*1000)
        else:
            time.sleep(.02)
        batch = worker.history_since(cursor)
        tx_batch = worker.transmissions_since(tx_cursor)
        cursor, tx_cursor = batch['cursor'], tx_batch['cursor']
        missed['feedback'] += batch['missed']
        missed['transmissions'] += tx_batch['missed']
        for reply in batch['exchanges']:
            replies.append(reply)
            stream.write(json.dumps(dict(phase=name, kind='reply', exchange=asdict(reply)))+'\n')
        for entry in tx_batch['entries']:
            transmissions.append(entry)
            stream.write(json.dumps(dict(phase=name, kind='transmit', **entry))+'\n')
        if not batch['connected']:
            raise RuntimeError('Feedback process disconnected: {}'.format(batch.get('error')))
    return phase_report(name, time.monotonic()-began, transmissions, replies, missed, planning,
                        worker.diagnostics())


def compare_phases(idle, planning):
    idle_rate, load_rate = idle['tx_rate_hz'], planning['tx_rate_hz']
    idle_gap, load_gap = idle['tx_gap_ms']['p99'], planning['tx_gap_ms']['p99']
    drop = (1-load_rate/idle_rate if idle_rate and load_rate else None)
    increase = (load_gap-idle_gap if idle_gap is not None and load_gap is not None else None)
    records_intact = all(not any(phase['cache_missed'].values()) for phase in (idle, planning))
    healthy = all(phase['query_only'] and phase['reply_count'] and not phase['invalid_source_replies']
                  and not phase['moving_replies'] and not phase['drive_fault_replies']
                  and not phase['mcu_fault_replies'] for phase in (idle, planning))
    stable_boot = len(set(idle['boots']) | set(planning['boots'])) == 1
    passed = (drop is not None and drop <= .10 and increase is not None and increase <= 20
              and records_intact and healthy and stable_boot and planning['planning']['successful'] > 0)
    return dict(passed=passed, send_rate_drop_fraction=drop, p99_gap_increase_ms=increase,
                max_rate_drop_fraction=.10, max_p99_gap_increase_ms=20,
                records_intact=records_intact, healthy_stationary=bool(healthy), stable_boot=stable_boot)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--port', required=True)
    parser.add_argument('--duration', type=float, default=60.)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--map-pgm', type=Path)
    parser.add_argument('--map-yaml', type=Path)
    parser.add_argument('--map-sha256')
    parser.add_argument('--yaml-sha256')
    parser.add_argument('--start-x', type=float)
    parser.add_argument('--start-y', type=float)
    parser.add_argument('--start-yaw', type=float, default=0.)
    parser.add_argument('--goal-x', type=float)
    parser.add_argument('--goal-y', type=float)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration <= 0:
        parser.error('--duration must be finite and positive')
    if not args.execute:
        print(json.dumps(dict(mode='offline', port_opened=False, target_hz=50,
                              phases=['idle', 'planning'], duration_each_s=args.duration)))
        return 0
    coordinates = [args.start_x, args.start_y, args.start_yaw, args.goal_x, args.goal_y]
    if (not all(value is not None and math.isfinite(value) for value in coordinates)
            or not all((args.map_pgm, args.map_yaml, args.map_sha256, args.yaml_sha256))):
        parser.error('Execution requires the pinned map and all start/goal coordinates')
    grid = load_map(args.map_pgm, args.map_yaml, args.map_sha256, args.yaml_sha256,
                    use_modeled_error_margin=False)
    start, goal = dict(x=args.start_x, y=args.start_y, yaw=args.start_yaw), dict(x=args.goal_x, y=args.goal_y)
    args.output.mkdir(parents=True, exist_ok=False)
    report = dict(mode='read_only_query', target_hz=50, duration_each_s=args.duration,
                  map=grid.metadata(), planning_start=start, planning_goal=goal, phases=[], error=None)
    worker = FeedbackWorker(args.port)
    try:
        worker.launch()
        worker.discover()
        with (args.output/'records.jsonl').open('x', encoding='utf-8') as stream:
            for name in ('idle', 'planning'):
                phase = run_phase(worker, name, args.duration, stream, grid, start, goal)
                report['phases'].append(phase)
                print(json.dumps(dict(phase=name, report=phase)), flush=True)
        report['comparison'] = compare_phases(*report['phases'])
    except Exception as exc:
        report['error'] = type(exc).__name__+': '+str(exc)
        report['diagnostics'] = worker.diagnostics()
    finally:
        worker.close()
        (args.output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding='utf-8')
    print(json.dumps(dict(report=str(args.output/'report.json'), error=report['error'],
                          comparison=report.get('comparison')), ensure_ascii=False), flush=True)
    return 0 if not report['error'] and report.get('comparison', {}).get('passed') else 2


if __name__ == '__main__':
    raise SystemExit(main())
