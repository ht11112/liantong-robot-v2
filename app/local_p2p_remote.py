"""RK-local JSON-line RPC. Fixed operations and fixed job paths; no shell API.

Import performs no device IO and starts no task. The PC owns its coordinate
epoch; this module probes a software zero or runs one independently bounded
relative goal. PC disconnect never kills/restarts an RK-local motion owner.
"""
import argparse
from contextlib import contextmanager
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import uuid

from simple_p2p import SimplePointController
from simple_p2p_base import run_session
from simple_p2p_runtime import DEFAULT_CONFIG, load_config


HERE = Path(__file__).resolve().parent
MAX_LINE = 16384


def strict_json(text):
    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError('Duplicate JSON key: '+key)
            result[key] = value
        return result
    def constant(value):
        raise ValueError('Non-finite JSON value: '+value)
    return json.loads(text, object_pairs_hook=pairs, parse_constant=constant)


def canonical_id(value):
    if type(value) is not str or str(uuid.UUID(value)) != value:
        raise ValueError('Canonical UUID required')
    return value


def encoders(value):
    if (type(value) is not dict or set(value) != {'left_raw','right_raw'}
            or any(type(v) is not int or not 0 <= v <= 0xffffffff for v in value.values())):
        raise ValueError('Expected encoder must contain exact original uint32 left_raw/right_raw')
    return dict(value)


def finite_goal(value):
    if type(value) not in (int,float) or not math.isfinite(value) or abs(value) > 100:
        raise ValueError('Goal must be finite and within 100 metres of this task origin')
    return float(value)


def atomic_json(path, value):
    temporary = path.with_name(path.name+'.tmp')
    temporary.write_text(json.dumps(value,sort_keys=True,allow_nan=False),encoding='utf-8')
    os.replace(str(temporary),str(path))


def read_json(path, limit=128*1024*1024):
    if path.is_symlink():
        raise RuntimeError('Evidence file symlinks are forbidden')
    if not path.is_file():
        return None
    if path.stat().st_size > limit:
        raise RuntimeError('Job evidence exceeds bounded read limit')
    return strict_json(path.read_text(encoding='utf-8'))


def compact_result(result):
    if result is None:
        return None
    keys = ('schema','boot_id','origin','latest_odometry','first_failure','session_complete',
            'motion_complete','position_verified','final_distance_m','query_counts',
            'complete_write_counts','exclusive_port_closed_normally','elapsed_s','arm_count',
            'position_verification','cleanup_errors','live_status_writer_complete')
    compact = {key:result.get(key) for key in keys}
    proof = result.get('final_stop_chain') or {}
    samples = proof.get('zero_samples') or []
    compact['latest_feedback'] = samples[-1] if samples else None
    compact['final_stop_chain'] = {key:proof.get(key) for key in
        ('complete','parking_position_error','position_verified','error')}
    compact['final_stop_chain']['zero_sample_count'] = len(samples)
    compact['full_evidence_retained_on_RK'] = True
    return compact


def boot_id():
    return canonical_id(Path('/proc/sys/kernel/random/boot_id').read_text().strip())


def process_identity(pid):
    if type(pid) is not int or pid <= 0:
        raise ValueError('Invalid recorded PID')
    try:
        raw = Path('/proc/{}/stat'.format(pid)).read_text()
    except FileNotFoundError:
        return None
    fields = raw.rsplit(')',1)[1].split()
    return dict(pid=pid,start_ticks=int(fields[19]),state=fields[0])


@contextmanager
def job_lock(path):
    import fcntl
    with path.open('a') as handle:
        fcntl.flock(handle.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        try:
            yield
        finally:
            fcntl.flock(handle.fileno(),fcntl.LOCK_UN)


def ros_environment():
    """Explicit baseline ROS environment; no request supplies environment/code."""
    env = dict(os.environ)
    env.update(PYTHONNOUSERSITE='1',ROS_DISTRO='noetic',ROS_VERSION='1',
        ROS_PYTHON_VERSION='3',ROS_MASTER_URI='http://localhost:11311',
        CMAKE_PREFIX_PATH='/opt/ros/noetic')
    env['PYTHONPATH'] = '/opt/ros/noetic/lib/python3/dist-packages:/usr/lib/python3/dist-packages'
    env['LD_LIBRARY_PATH'] = '/opt/ros/noetic/lib:/opt/ros/noetic/lib/aarch64-linux-gnu'
    env['PATH'] = '/opt/ros/noetic/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin'
    return env


def probe_base(expected_boot=None):
    current = boot_id()
    if expected_boot is not None and expected_boot != current:
        raise RuntimeError('RK boot identity changed')
    spec = importlib.util.spec_from_file_location('_p2p_remote_transport',HERE/'deps/transport.py')
    transport = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(transport)
    api = transport.load_dependency(HERE/'deps','control_client')
    core = transport.load_dependency(HERE/'deps','core')
    link = transport.load_dependency(HERE/'deps','rk_link')
    config,_ = load_config(DEFAULT_CONFIG)
    target = link.locate()
    with transport.exclusive_port(target['port']) as physical:
        result = run_session(physical,api,core,SimplePointController(0.,0.,config=config,
            started_at=time.monotonic()),scan_provider=None,boot_id=current,execute=False)
    if not result['session_complete']:
        raise RuntimeError('Read-only probe failed: '+json.dumps(result['first_failure']))
    origin = result['origin']
    return dict(boot_id=current,now_monotonic_s=time.monotonic(),origin=origin,
        latest_feedback=origin['raw_packet'],latest_odometry=result['latest_odometry'],
        query_counts=result['query_counts'],complete_write_counts=result['complete_write_counts'],
        session_complete=True,pose0=dict(x=0.,y=0.,yaw=0.),actuation_requested=False)


class Remote:
    def __init__(self, directory=HERE, *, boot_reader=boot_id, pid_reader=process_identity,
                 popen=subprocess.Popen, lock_factory=job_lock, probe=probe_base,
                 clock=time.monotonic):
        self.directory = Path(directory).resolve()
        self.jobs = self.directory/'jobs'
        self.boot, self.pid, self.popen = boot_reader,pid_reader,popen
        self.lock_factory,self.probe,self.clock = lock_factory,probe,clock

    def paths(self, task):
        task = canonical_id(task)
        path = self.jobs/task
        if path.is_symlink() or self.jobs.is_symlink():
            raise RuntimeError('Job directory symlinks are forbidden')
        if path.resolve().parent != self.jobs.resolve():
            raise RuntimeError('Job path escaped fixed jobs directory')
        return path

    def active(self, record):
        if not record or record.get('pid') is None:
            if record and record.get('launch_state') in ('reserved','uncertain'):
                raise RuntimeError('Unresolved local task startup; no second owner permitted')
            return False
        if record.get('launch_state') == 'uncertain':
            raise RuntimeError('Uncertain local child cleanup; no second owner permitted')
        if record['boot_id'] != self.boot():
            return False
        identity = self.pid(record['pid'])
        return bool(identity is not None and identity['state'] != 'Z'
            and identity['start_ticks'] == record['start_ticks'])

    def inventory(self):
        active, latest = [], None
        if self.jobs.exists():
            for entry in self.jobs.iterdir():
                if not entry.is_dir() or entry.is_symlink():
                    continue
                try:
                    canonical_id(entry.name)
                except (ValueError,TypeError,AttributeError):
                    continue
                record = read_json(entry/'job.json',65536)
                if record is None:
                    raise RuntimeError('Unaccounted job directory; cannot authorize another owner')
                if latest is None or record['created_monotonic_s'] > latest['created_monotonic_s']:
                    latest = record
                if self.active(record):
                    active.append(dict(task_id=entry.name,pid=record['pid'],start_ticks=record['start_ticks']))
        return dict(boot_id=self.boot(),active_jobs=active,
            active_task_id=active[0]['task_id'] if len(active) == 1 else None,
            last_task_id=None if latest is None else latest['task_id'],now_monotonic_s=self.clock())

    def status(self, task):
        path = self.paths(task)
        if (path/'run').is_symlink():
            raise RuntimeError('Job output symlinks are forbidden')
        record = read_json(path/'job.json',65536)
        if record is None:
            raise ValueError('Unknown task')
        result_file = path/'run/result.json'
        result = compact_result(read_json(result_file))
        live = read_json(path/'run/live_status.json',65536)
        active = self.active(record)
        return dict(task_id=task,boot_id=self.boot(),record=record,active=active,
            live_status=live,result=result,now_monotonic_s=self.clock(),
            terminal_without_result=not active and result is None,
            startup_error=record.get('startup_error'),result_file=str(result_file))

    def handle(self, request):
        if type(request) is not dict or type(request.get('op')) is not str:
            raise ValueError('JSON object with a fixed op required')
        operation = request['op']
        if operation == 'poll':
            if set(request)-{'op','task_id'}:
                raise ValueError('Unexpected poll fields')
            return self.status(request['task_id']) if 'task_id' in request else self.inventory()
        if operation == 'cancel':
            if set(request) != {'op','task_id'}:
                raise ValueError('Cancel requires only task_id')
            path = self.paths(request['task_id'])
            status = self.status(request['task_id'])
            # No signals, PID kills, serial frames or foreign namespace cleanup.
            if status['active']:
                if (path/'cancel.request').is_symlink():
                    raise RuntimeError('Cancellation file symlinks are forbidden')
                (path/'cancel.request').write_text('operator_cancel\n',encoding='ascii')
            return dict(status,cancel_requested=status['active'])
        if operation not in ('probe','start'):
            raise ValueError('Only probe/start/poll/cancel are supported')
        self.jobs.mkdir(exist_ok=True)
        if self.jobs.is_symlink():
            raise RuntimeError('Job directory symlinks are forbidden')
        with self.lock_factory(self.jobs/'.broker.lock'):
            if operation == 'probe':
                if set(request)-{'op','expected_boot_id'}:
                    raise ValueError('Unexpected probe fields')
                expected = request.get('expected_boot_id')
                if expected is not None:
                    canonical_id(expected)
                if self.inventory()['active_jobs']:
                    raise RuntimeError('Active local task owns the base; probe is forbidden')
                return self.probe(expected)
            expected_fields = {'op','goal_x','goal_y','expected_start_encoders','expected_boot_id','area_clear','request_id'}
            if set(request) != expected_fields or request['area_clear'] is not True:
                raise ValueError('Explicit area_clear=true and exact start fields required')
            task = canonical_id(request['request_id'])
            expected = encoders(request['expected_start_encoders'])
            current = canonical_id(request['expected_boot_id'])
            goal = tuple(finite_goal(request[key]) for key in ('goal_x','goal_y'))
            if current != self.boot():
                raise RuntimeError('RK boot identity changed; reset the local frame')
            digest = hashlib.sha256(json.dumps(request,sort_keys=True,allow_nan=False).encode()).hexdigest()
            path = self.paths(task)
            previous = read_json(path/'job.json',65536)
            if previous is not None:
                if previous['request_digest'] != digest:
                    raise ValueError('request_id was already used with different task parameters')
                return self.status(task)  # Never relaunch a completed/failed identical request.
            if self.inventory()['active_jobs']:
                raise RuntimeError('Another local task is active')
            path.mkdir()
            argv = [sys.executable,'-s',str(self.directory/'simple_p2p_runtime.py'),
                '--config',str(DEFAULT_CONFIG),'--execute','--reset-odom',
                '--expected-boot-id',current,'--expected-start-encoders',
                str(expected['left_raw']),str(expected['right_raw']),
                '--output-dir',str(path/'run'),'--cancel-request-file',str(path/'cancel.request'),
                '--goal-x',str(goal[0]),'--goal-y',str(goal[1]),'--turn-area-clear']
            record = dict(task_id=task,request_digest=digest,boot_id=current,
                request=request,created_monotonic_s=self.clock(),pid=None,start_ticks=None,
                argv=argv,startup_error=None,launch_state='reserved')
            atomic_json(path/'job.json',record)
            child = None
            try:
                with (path/'runtime.log').open('ab') as log:
                    child = self.popen(argv,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,
                        cwd=str(self.directory),env=ros_environment(),start_new_session=True)
                identity = self.pid(child.pid)
                if identity is None:
                    raise RuntimeError('Spawned task exited before PID birth could be recorded')
                record.update(pid=child.pid,start_ticks=identity['start_ticks'],launch_state='running')
                atomic_json(path/'job.json',record)
            except BaseException as exc:
                record['startup_error'] = type(exc).__name__+': '+str(exc)
                record['launch_state'] = 'failed'
                if child is not None:
                    # Only this Popen, never a PID rediscovered from a request.
                    try:
                        child.send_signal(2)
                        child.wait(timeout=8.)
                    except subprocess.TimeoutExpired:
                        record['launch_state'] = 'uncertain'
                        record['startup_error'] += '; own task did not exit within 8s; no forced kill'
                    except BaseException as cleanup:
                        record['launch_state'] = 'uncertain'
                        record['startup_error'] += '; own task cleanup: '+str(cleanup)
                atomic_json(path/'job.json',record)
                raise RuntimeError(record['startup_error'])
            return self.status(task)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--stdio',action='store_true',required=True)
    parser.parse_args(argv)
    remote = Remote()
    while True:
        line = sys.stdin.readline(MAX_LINE+1)
        if not line:
            return 0
        try:
            if len(line) > MAX_LINE or not line.endswith('\n'):
                raise ValueError('RPC line exceeds bound or is incomplete')
            result = dict(ok=True,**remote.handle(strict_json(line)))
        except Exception as exc:
            result = dict(ok=False,error=dict(type=type(exc).__name__,message=str(exc)))
        print(json.dumps(result,allow_nan=False),flush=True)
        if len(line) > MAX_LINE:
            return 2  # Never reinterpret the remainder as another command.


if __name__ == '__main__':
    raise SystemExit(main())
