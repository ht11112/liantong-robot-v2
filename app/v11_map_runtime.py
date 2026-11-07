"""RK V12 map navigation. The serial loop owns every MCU request."""

import argparse
from collections import deque
from contextlib import contextmanager
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
import sys
import threading
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'mcu' / 'tools'))
from mcu_v11 import (EXECUTING, LEASE_STATES, RX_FAULT, STOP, STOP_CONFIRMED, LinkError, Rejected,
                     RkMaster, error_record, open_port)
from v11_map_plan import PlanningError, load_map


PHASES = ('idle', 'loading', 'ready', 'arming', 'move', 'turn',
          'barrier', 'stopping', 'complete', 'failed')
SOURCE_MAX_AGE_S = .250
RENEW_BEFORE_S = 5.
REPLAN_INTERVAL_S = 1.
MCU_CANCELLED = 1
ARRIVAL_REACHED = 8


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def pose_of(sample):
    return dict(x=sample.x, y=sample.y, yaw=sample.yaw)


def same_pose(current, reference):
    # Allow float32 odometry round-off, including yaw wrapping at ±π.
    return (current is not None
            and math.hypot(current['x'] - reference['x'], current['y'] - reference['y']) <= 1e-6
            and abs(angle(current['yaw'] - reference['yaw'])) <= 1e-5)


def compose(first, second):
    c, s = math.cos(first['yaw']), math.sin(first['yaw'])
    return dict(x=first['x'] + c * second['x'] - s * second['y'],
                y=first['y'] + s * second['x'] + c * second['y'],
                yaw=angle(first['yaw'] + second['yaw']))


def inverse(pose):
    c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
    return dict(x=-c * pose['x'] - s * pose['y'],
                y=s * pose['x'] - c * pose['y'], yaw=-pose['yaw'])


class Navigation:
    def __init__(self, grid, localizer=None, *, mode='ready', clock=time.monotonic, scanner=None):
        self.grid, self.localizer, self.mode, self.clock = grid, localizer, mode, clock
        self.scanner = scanner
        self.avoidance_phase, self.avoidance_message, self.replans = 'watching', None, 0
        self.detour = None
        self.lock = threading.RLock()
        self.latest = None
        self.connected = False
        self.source = self.source_time = None
        self.alignment = self.origin = None
        self.localization = dict(mode='unlocalized', reason='请在地图上标出当前位置和朝向')
        self.trail = deque(maxlen=1200)
        self.plan = self.route = None
        self.work = None
        self.pending = None
        self.lease_task = None
        self.stop_requested = False
        self.stop_revision = 0
        self.stop_ack = self.stop_verified = None
        self.stop_evidence = None
        self.error = None

    def _invalidate(self, reason):
        if self.detour is not None:
            self._cancel_detour(reason)
        self.alignment = self.origin = self.plan = self.route = None
        self.trail.clear()
        self.localization = dict(mode='unlocalized', reason=reason)

    def _fresh(self):
        if not self.connected or self.latest is None:
            return False
        sample = self.latest.status
        return (sample.source_valid and self.source_time is not None
                and 0 <= self.clock() - self.source_time < SOURCE_MAX_AGE_S)

    def _stationary(self):
        if not self._fresh():
            return False
        sample = self.latest.status
        return (not sample.flags & (EXECUTING | RX_FAULT) and not sample.first_drive_error
                and max(abs(sample.left_tenths_rpm), abs(sample.right_tenths_rpm)) <= 2)

    def _require_stationary(self):
        if not self._stationary():
            raise ValueError('需要新鲜有效的里程计、底盘无故障且小车已停稳')
        return self.latest.status

    def _map_pose(self):
        if self.alignment is None or not self._fresh():
            return None
        return compose(self.alignment, pose_of(self.latest.status))

    def observe(self, exchange):
        sample = exchange.status
        identity = sample.boot, sample.epoch
        key = (*identity, sample.sample_sequence, sample.measurement_tick_ms)
        with self.lock:
            previous = self.latest
            self.latest, self.connected = exchange, True
            if previous and (previous.status.boot, previous.status.epoch) != identity:
                self._invalidate('MCU 会话已变化，请重新定位')
                self.stop_ack = self.stop_verified = None
                self.stop_evidence = None
            if exchange.opcode == STOP and sample.ack_result == 0:
                self.stop_ack = True
            distinct = sample.source_valid and key != self.source
            if distinct:
                self.source = key
                self.source_time = exchange.sent_at - (sample.source_age_ms + 1) / 1000
            if not sample.source_valid:
                if self.alignment is not None:
                    self._invalidate('里程计失效，请重新定位')
            elif distinct and self._fresh():
                pose = self._map_pose()
                if pose is not None and (not self.trail or pose != self.trail[-1]):
                    self.trail.append(pose)
        if self.localizer is not None:
            self.localizer.publish(exchange)
        if self.scanner is not None:
            self.scanner.publish(exchange)

    def _scan_points(self):
        if self.scanner is None or self.alignment is None or not self._fresh():
            return None
        frame = self.scanner.latest()
        if frame is None:
            return None
        odom, points = frame
        status = self.latest.status
        if (odom['boot'], odom['epoch']) != (status.boot, status.epoch):
            return None
        pose = compose(self.alignment, odom)
        c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
        return tuple((pose['x'] + c*x - s*y, pose['y'] + s*x + c*y) for x, y in points)

    def _upload_obstacles(self):
        points = self._scan_points()
        if points is None:
            raise ValueError('避障测试需要定位后有效的雷达数据')
        cells = self.grid.obstacle_cells(points)
        return cells | self.detour['blockers'] if self.detour is not None else cells

    def _cancel_detour(self, reason):
        self.detour = None
        self.avoidance_phase, self.avoidance_message = 'cancelled', reason

    @contextmanager
    def _stationary_work(self, name):
        with self.lock:
            if self.work or self.stop_requested or self.detour is not None:
                raise ValueError('当前操作正在处理')
            self._require_stationary()
            self.work, self.error = name, None
            stop_revision = self.stop_revision
        try:
            yield stop_revision
        finally:
            with self.lock:
                self.work = None

    def align(self, seed, mode):
        with self._stationary_work('localizing' if mode == 'amcl' else 'aligning') as stop_revision:
            with self.lock:
                before = self.latest.status
                initial = pose_of(before)
                self.plan = self.route = None
            try:
                if mode == 'amcl':
                    if self.localizer is None:
                        raise ValueError('AMCL 当前不可用，可手动对齐')
                    map_pose, odom_pose = self.localizer.localize(seed)
                else:
                    map_pose = seed
                with self.lock:
                    current = self._require_stationary()
                    if (current.boot, current.epoch) != (before.boot, before.epoch):
                        raise ValueError('定位期间 MCU 会话已变化')
                    if self.stop_revision != stop_revision:
                        raise ValueError('定位已取消')
                    if mode == 'manual':
                        odom_pose = pose_of(current)
                        if not same_pose(odom_pose, initial):
                            raise ValueError('置零期间车体位置发生变化，请停稳后重试')
                    self.alignment = compose(map_pose, inverse(odom_pose))
                    self.origin = dict(x=odom_pose['x'], y=odom_pose['y'], yaw=odom_pose['yaw'])
                    self.localization = dict(mode=mode, reason=None, stamp_ns=map_pose.get('stamp_ns'),
                                             boot=str(current.boot), epoch=current.epoch)
                    self.trail.clear()
                    self.trail.append(self._map_pose())
            except Exception as exc:
                with self.lock:
                    self._invalidate(str(exc))
                raise

    def preview(self, goal):
        with self._stationary_work('planning') as stop_revision:
            with self.lock:
                anchor, pose = self.alignment, self._map_pose()
                self.plan = None
                if pose is None:
                    raise ValueError('请先完成地图定位')
                points = self._scan_points()
            plan = (self.grid.plan(pose, goal, obstacle_cells=self.grid.obstacle_cells(points))
                    if points is not None else self.grid.plan(pose, goal))
            with self.lock:
                if (self.stop_revision != stop_revision or anchor is not self.alignment
                        or not same_pose(self._map_pose(), pose)):
                    raise ValueError('规划期间起点已变化，请重新预览')
                self.plan = dict(plan, id=uuid.uuid4().hex)

    def request_start(self, plan_id):
        with self.lock:
            self._require_stationary()
            if self.work or self.stop_requested or self.detour is not None:
                raise ValueError('当前操作正在处理')
            if self.plan is None or self.plan['id'] != plan_id or not self.plan['segments']:
                raise ValueError('请预览一条可执行的新路径')
            if self.scanner is not None and self._scan_points() is None:
                raise ValueError('避障测试需要定位后有效的雷达数据')
            self.avoidance_phase, self.avoidance_message, self.replans = 'watching', None, 0
            self.pending = self.route = self.plan
            self.plan = None
            self.work, self.error = 'uploading', None
            self.stop_ack = self.stop_verified = None
            self.stop_evidence = None

    def request_stop(self):
        with self.lock:
            if self.mode != 'drive':
                raise ValueError('静止验收模式使用查询和上传')
            if not self.connected:
                raise ValueError('串口状态未知，请在现场停车')
            self._cancel_detour('已人工取消')
            self._queue_stop()

    def _queue_stop(self):
        self.plan = self.pending = None
        self.lease_task = None
        self.stop_requested = True
        self.stop_revision += 1
        self.stop_ack = self.stop_verified = None
        self.stop_evidence = None

    def _observe_upload(self, exchange):
        self.observe(exchange)
        with self.lock:
            if self.stop_requested:
                raise ValueError('上传已取消')

    def _upload(self, master, plan):
        with self.lock:
            self._require_stationary()
            pose = self._map_pose()
            if pose is None:
                raise ValueError('地图定位已失效，请重新定位')
            if not same_pose(pose, plan['start']):
                raise ValueError('起点已变化，请重新预览')
            cells = self._upload_obstacles() if self.scanner is not None else frozenset()
        if cells:
            self.grid.validate_plan(pose, plan, obstacle_cells=cells)
        else:
            self.grid.validate_plan(pose, plan)
        with self.lock:
            if self.stop_requested:
                return
        reply = master.upload(plan, on_ack=self._observe_upload)[-1]
        with self.lock:
            if self.mode == 'ready' or self.stop_requested:
                return
            self._require_stationary()
            if not same_pose(self._map_pose(), pose):
                raise ValueError('上传期间起点已变化，请重新预览')
            if self.scanner is not None:
                current_cells = self._upload_obstacles()
                if current_cells != cells:
                    self.grid.validate_plan(pose, plan, obstacle_cells=current_cells)
            self.work = 'starting'
        started = master.start(reply.status.task)
        self.observe(started)
        status = started.status
        with self.lock:
            if not self.stop_requested and status.state in LEASE_STATES and status.lease_remaining_ms:
                self.lease_task = status.boot, status.epoch, status.task
                if self.detour is not None:
                    self.detour = None
                    self.replans += 1
                    self.stop_ack = self.stop_verified = self.stop_evidence = None
                    self.avoidance_phase, self.avoidance_message = 'watching', '已换路线，继续前往原目标'
            elif self.detour is not None:
                self._cancel_detour('底盘未进入续行状态，请人工处理')

    def _lease_seconds(self, exchange):
        # The MCU computes its remaining time after this send, so this is a lower bound.
        return max(0., exchange.sent_at + exchange.status.lease_remaining_ms / 1000. - self.clock())

    def _renew_if_due(self, master, reply):
        status = reply.status
        with self.lock:
            if self.lease_task is None:
                return
            if (self.lease_task != (status.boot, status.epoch, status.task)
                    or status.state not in LEASE_STATES or not status.lease_remaining_ms):
                self.lease_task = None
                return
            if self.stop_requested or not self._fresh() or not 0 < self._lease_seconds(reply) <= RENEW_BEFORE_S:
                return
        try:
            self.observe(master.renew(status.task))
        except Rejected as exc:
            # The MCU may complete or stop the task while RENEW is in flight.
            self.observe(exc.exchange)
            with self.lock:
                self.lease_task = None
                self.error = '任务续约未获底盘接受'

    def _stop(self, master):
        result = master.stop(on_reply=self.observe)
        error = result.get('error', {})
        with self.lock:
            self.stop_evidence = result
            self.stop_ack = True
            self.stop_verified = result['stop_verified']
            if result['stop_verified']:
                self.error = None
            elif error.get('code') == 'stop_proof_deadline':
                self.error = '停稳确认到期，仍在更新底盘状态'
            else:
                self.error = error.get('message', '停车未确认')

            if self.detour is not None:
                if self.stop_verified:
                    self.avoidance_phase, self.avoidance_message = 'waiting', '已停稳，等待重新规划'
                else:
                    self._cancel_detour(self.error)

    def _check_route(self, reply):
        status = reply.status
        identity = status.boot, status.epoch, status.task
        if (self.stop_requested or self.lease_task != identity or status.state not in LEASE_STATES
                or status.first_error or status.first_drive_error):
            return
        pose, points = self._map_pose(), self._scan_points()
        if points is None:
            return
        cells = self.grid.obstacle_cells(points)
        # In the handoff state the reported segment has already finished.
        index = status.index + (status.state == 6)
        progress = status.progress if status.state in (4, 5) else 0.
        if self.grid.route_blocked(pose, self.route, index, progress, cells):
            # Keep the blocking scan during replanning if the object leaves the front fan.
            self.detour = dict(goal=self.route['goal'], blockers=cells, attempt=None, retry_at=0.)
            self.avoidance_phase, self.avoidance_message = 'stopping', '前方路线被挡，正在停车'
            self._queue_stop()

    def _avoidance_cycle(self, master, reply):
        with self.lock:
            detour = self.detour
            if detour is None:
                self._check_route(reply)
                return
            if not self.stop_verified or self.stop_requested or not self._stationary():
                return
            status = reply.status
            if status.first_error not in (0, MCU_CANCELLED):
                self._cancel_detour('底盘有故障，取消自动续行')
                return
            if status.flags & ARRIVAL_REACHED:
                self.detour = None
                self.avoidance_phase, self.avoidance_message = 'watching', '原任务已到点'
                return
            points = self._scan_points()
            if points is None or self.clock() < detour['retry_at']:
                return
            cells = self.grid.obstacle_cells(points) | detour['blockers']
            if cells == detour['attempt']:
                return
            detour['attempt'], detour['retry_at'] = cells, self.clock() + REPLAN_INTERVAL_S
            pose = self._map_pose()
            self.avoidance_phase, self.avoidance_message = 'replanning', '正在重新规划到原目标'

        try:
            plan = self.grid.plan(pose, detour['goal'], obstacle_cells=cells)
        except PlanningError:
            with self.lock:
                if self.detour is detour:
                    self.avoidance_phase, self.avoidance_message = 'waiting', '暂无绕行路线，保持停车'
            return

        # A* can outlast the freshness window while this owner is stopped.
        fresh = master.query()
        if fresh is not None:
            self.observe(fresh)
        with self.lock:
            if self.detour is not detour:
                return
            if fresh is None or not self._stationary() or not same_pose(self._map_pose(), pose):
                detour['attempt'] = None
                self.avoidance_phase, self.avoidance_message = 'waiting', '等待新鲜停稳位姿'
                return
            if plan['no_motion']:
                self.detour = None
                self.avoidance_phase, self.avoidance_message = 'watching', '当前位置已在目标容差内'
                return
            self.pending = self.route = plan
            self.work, self.error = 'uploading', None
            self.avoidance_phase, self.avoidance_message = 'resuming', '已找到绕行路线，正在下发'

    def run(self, device, closed):
        try:
            with open_port(device, .1) as port:
                master = RkMaster(port)
                self.observe(master.discover())
                while not closed.is_set():
                    with self.lock:
                        stop = self.stop_requested
                        plan, self.pending = self.pending, None
                        if stop and self.work in (None, 'uploading', 'starting'):
                            self.work = 'stopping'
                    try:
                        if stop:
                            self._stop(master)
                        elif plan is not None:
                            self._upload(master, plan)
                        else:
                            reply = master.query()
                            if reply is not None:
                                self.observe(reply)
                                if self.scanner is not None and self.mode == 'drive':
                                    self._avoidance_cycle(master, reply)
                                self._renew_if_due(master, reply)
                        if master.failed:
                            raise LinkError(self.error or 'V12 会话已失效')
                    except (ValueError, LinkError) as exc:
                        if master.failed or not (stop or plan is not None):
                            raise
                        if isinstance(exc, LinkError) and exc.exchange is not None:
                            self.observe(exc.exchange)
                        with self.lock:
                            self.error = str(exc)
                            if self.detour is not None:
                                self._cancel_detour(self.error)
                            if stop:
                                self.stop_ack, self.stop_verified = False, False
                    finally:
                        with self.lock:
                            if stop:
                                self.stop_requested = False
                            if self.work in ('uploading', 'starting', 'stopping') and (stop or plan is not None):
                                self.work = None
        except Exception as exc:
            with self.lock:
                self.error = str(exc)
                task = self.latest.status.task if self.latest is not None else None
            print(json.dumps(dict(event='v12_owner_closed', task=task, error=error_record(exc))),
                  flush=True)
        finally:
            with self.lock:
                self.connected = False
                self.pending = None
                self.lease_task = None
                self.stop_requested = False
                self._invalidate('串口已关闭，请重新连接并定位')

    def snapshot(self):
        with self.lock:
            status = self.latest.status if self.latest else None
            pose = self._map_pose()
            fresh = self._fresh()
            status_fresh = (self.connected and self.latest is not None
                            and 0 <= self.clock() - self.latest.sent_at < SOURCE_MAX_AGE_S)
            preparing = not self.work and not self.stop_requested and self.detour is None and self._stationary()
            avoidance = None
            points = None
            if self.scanner is not None:
                points = self._scan_points()
                message = self.avoidance_message
                if points is None and self.avoidance_phase == 'watching':
                    message = '等待定位和有效雷达数据'
                avoidance = dict(phase=self.avoidance_phase, message=message, replans=self.replans,
                                 points=[dict(x=x, y=y) for x, y in points or ()])
            task = dict(id=str(status.task) if status else None,
                        phase=(self.work if self.work in ('uploading', 'starting', 'stopping')
                               else PHASES[status.state] if status_fresh else 'unknown'),
                        index=status.index if status else 0, count=status.count if status else 0,
                        progress=status.progress if status else 0,
                        lease_remaining_s=self._lease_seconds(self.latest) if status_fresh
                                          and status.lease_remaining_ms else None,
                        arrival=bool(status_fresh and status.flags & ARRIVAL_REACHED),
                        stop_ack=self.stop_ack,
                        stop_verified=(None if not status_fresh or
                                       (self.stop_requested and self.stop_ack is None) else
                                       self.stop_verified if self.stop_ack is not None else
                                       bool(status.flags & STOP_CONFIRMED) if fresh and status else None),
                        stop_evidence=self.stop_evidence,
                        error=self.error or (f'MCU 错误 {status.first_error} / 驱动 {status.first_drive_error}'
                                             if status and (status.first_error or status.first_drive_error)
                                             else None))
            return dict(map=self.grid.metadata(), localization=dict(self.localization), pose=pose,
                        local_pose=(compose(inverse(self.origin), pose_of(status))
                                    if pose is not None and self.origin else None),
                        trail=list(self.trail), preview=self.plan, route=self.route,
                        connected=self.connected,
                        source_fresh=fresh, busy=bool(self.work or self.stop_requested), mode=self.mode,
                        task=task, avoidance=avoidance, can_prepare=preparing,
                        can_start=bool(preparing and pose and self.plan and self.plan['segments']
                                       and (self.scanner is None or points is not None)),
                        can_stop=self.connected and self.mode == 'drive', error=self.error)


def checked_body(body, keys):
    if not isinstance(body, dict) or set(body) != set(keys):
        raise ValueError('请求字段不正确')
    if keys == ('plan_id',):
        if not isinstance(body['plan_id'], str):
            raise ValueError('路径标识不正确')
    elif any(type(body[k]) not in (int, float) or not math.isfinite(body[k]) for k in keys):
        raise ValueError('地图坐标必须是有限数值')
    return body


def handler_for(nav):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *args):
            pass

        def send(self, code, value, content_type='application/json; charset=utf-8'):
            raw = (json.dumps(value, ensure_ascii=False, allow_nan=False).encode('utf-8')
                   if isinstance(value, dict) else value)
            self.send_response(code)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(raw)

        def do_GET(self):
            if self.path == '/api/state':
                self.send(200, nav.snapshot())
            elif self.path == '/map.pgm':
                self.send(200, nav.grid.pgm_bytes, 'image/x-portable-graymap')
            else:
                self.send(404, dict(error='未知页面'))

        def do_POST(self):
            if (self.headers.get('Host') != '127.0.0.1:8767'
                    or self.headers.get('Origin') is not None
                    or self.headers.get_content_type() != 'application/json'):
                self.send(403, dict(error='请使用本机导航网页'))
                return
            try:
                length = int(self.headers.get('Content-Length', '0'))
                if not 0 < length <= 1024:
                    raise ValueError('请求大小不正确')
                body = json.loads(self.rfile.read(length))
                if self.path in ('/api/localize', '/api/zero'):
                    pose = checked_body(body, ('x', 'y', 'yaw'))
                    nav.align(pose, 'amcl' if self.path.endswith('localize') else 'manual')
                elif self.path == '/api/preview':
                    nav.preview(checked_body(body, ('x', 'y')))
                elif self.path == '/api/start':
                    nav.request_start(checked_body(body, ('plan_id',))['plan_id'])
                elif self.path == '/api/stop':
                    checked_body(body, ())
                    nav.request_stop()
                else:
                    self.send(404, dict(error='未知操作'))
                    return
            except Exception as exc:
                self.send(409, dict(error=str(exc)))
                return
            self.send(200, nav.snapshot())
    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True, help='已核实的 RK RS485 设备路径')
    parser.add_argument('--map-yaml', required=True, type=Path)
    parser.add_argument('--map-pgm', required=True, type=Path)
    parser.add_argument('--map-sha256', required=True)
    parser.add_argument('--yaml-sha256', required=True)
    parser.add_argument('--scan-topic', default='/lidar1/laser_fan')
    parser.add_argument('--mode', choices=('ready', 'drive'), default='ready')
    parser.add_argument('--manual-only', action='store_true', help='只启用地图手动对齐')
    parser.add_argument('--no-modeled-error-margin', action='store_true',
                        help='路线扫掠只检查车体包络，不叠加动作结束误差')
    parser.add_argument('--obstacle-test', action='store_true', help='启用前方雷达挡路停车与自动重规划')
    args = parser.parse_args(argv)
    grid = load_map(args.map_pgm, args.map_yaml, args.map_sha256, args.yaml_sha256,
                    use_modeled_error_margin=not args.no_modeled_error_margin)
    localizer = None
    if not args.manual_only:
        from v11_ros import RosLocalizer
        localizer = RosLocalizer(args.map_yaml, args.scan_topic)
    scanner = None
    if args.obstacle_test:
        from v11_ros import RosObstacleScanner
        scanner = RosObstacleScanner(args.scan_topic)
    nav = Navigation(grid, localizer, mode=args.mode, scanner=scanner)
    closed = threading.Event()
    owner = threading.Thread(target=nav.run, args=(args.port, closed), name='v12-owner')
    try:
        with ThreadingHTTPServer(('127.0.0.1', 8767), handler_for(nav)) as server:
            owner.start()
            print('RK V12 map API: http://127.0.0.1:8767 ({})'.format(args.mode), flush=True)
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        closed.set()
        if owner.ident is not None:
            owner.join()
        if localizer is not None:
            localizer.close()
        if scanner is not None:
            scanner.close()


if __name__ == '__main__':
    main()
