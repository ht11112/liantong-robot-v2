"""RK V12 map navigation consuming an independent serial feedback worker."""

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
                     RequestCancelled, error_record)
from v12_feedback_worker import FeedbackWorker
from v11_map_plan import PlanningError, load_map
from v11_obstacles import DynamicObstacles


PHASES = ('idle', 'loading', 'ready', 'arming', 'move', 'turn',
          'barrier', 'stopping', 'complete', 'failed')
SOURCE_MAX_AGE_S = .500
RENEW_BEFORE_S = 5.
REPLAN_INTERVAL_S = 1.
# Reuse the existing simple-p2p host scan timeout, not an MCU fault threshold.
OBSTACLE_SCAN_TIMEOUT_S = .75
START_SCAN_WAIT_S = 2.
OBSTACLE_WORK_INTERVAL_S = .02
MCU_CANCELLED = 1
ARRIVAL_REACHED = 8
POSE_DISTANCE_TOLERANCE_M = .02
POSE_HEADING_TOLERANCE_RAD = math.radians(2.)


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def pose_of(sample):
    return dict(x=sample.x, y=sample.y, yaw=sample.yaw)


def same_pose(current, reference):
    if current is None:
        return False
    distance = math.hypot(current['x'] - reference['x'], current['y'] - reference['y'])
    heading = abs(angle(current['yaw'] - reference['yaw']))
    return distance <= POSE_DISTANCE_TOLERANCE_M and heading <= POSE_HEADING_TOLERANCE_RAD


def compose(first, second):
    c, s = math.cos(first['yaw']), math.sin(first['yaw'])
    return dict(x=first['x'] + c * second['x'] - s * second['y'],
                y=first['y'] + s * second['x'] + c * second['y'],
                yaw=angle(first['yaw'] + second['yaw']))


def inverse(pose):
    c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])
    return dict(x=-c * pose['x'] - s * pose['y'],
                y=s * pose['x'] - c * pose['y'], yaw=-pose['yaw'])


# 串口在独立进程持续查询；导航与 ROS 只消费内存中的反馈历史。
class Navigation:
    def __init__(self, grid, localizer=None, *, mode='ready', clock=time.monotonic, scanner=None,
                 obstacle_scan_timeout_s=OBSTACLE_SCAN_TIMEOUT_S):
        if not math.isfinite(obstacle_scan_timeout_s) or obstacle_scan_timeout_s <= 0:
            raise ValueError('Obstacle scan timeout must be positive and finite')
        self.grid, self.localizer, self.mode, self.clock = grid, localizer, mode, clock
        self.scanner = scanner
        self.obstacles = DynamicObstacles(grid)
        self.obstacle_scan_timeout_s = obstacle_scan_timeout_s
        self.scan_stamp = self.last_usable_scan = None
        self.scan_points = self.scan_expires = None
        self.scan_status = 'unavailable'
        self.obstacle_hits = {}
        self.clear_work = None
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
        self.last_scan_failure = None
        self.feedback = None
        self.feedback_cursor = 0
        self.feedback_missed = 0
        self.feedback_lock = threading.Lock()
        self.feedback_error = None

    def _invalidate(self, reason):
        if self.detour is not None:
            self._cancel_detour(reason)
        self.alignment = self.origin = self.plan = self.route = None
        self._clear_obstacles()
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

    # 只有新的有效 MCU 测量能续鲜；boot 或 epoch 改变时必须重新定位。
    def observe(self, exchange):
        if self.feedback is not None:
            self._consume_feedback()
            return
        self._observe_exchange(exchange)

    def _observe_exchange(self, exchange):
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

    def _consume_feedback(self):
        # 命令结果与后台消费都走同一游标，避免较新的 ACK 越过尚未读到的失效样本。
        with self.feedback_lock:
            batch = self.feedback.history_since(self.feedback_cursor)
            if batch['missed']:
                self.feedback_missed += batch['missed']
                for adapter in (self.scanner, self.localizer):
                    if adapter is not None:
                        adapter.invalidate_history()
                with self.lock:
                    self._invalidate('反馈缓存遗漏，请重新定位')
                    self.pending = self.lease_task = None
                    self.error = '反馈缓存遗漏（cache_overrun），已取消待执行路线'
                    sample = batch['exchanges'][-1].status
                    if self.mode == 'drive' and sample.flags & EXECUTING:
                        self._queue_stop()
                print(json.dumps(dict(event='v12_feedback_overrun', missed=batch['missed'],
                                      cursor=batch['cursor'])), flush=True)
            for exchange in batch['exchanges']:
                self._observe_exchange(exchange)
            self.feedback_cursor = batch['cursor']
            if not batch['connected']:
                error = batch['error']
                raise LinkError(error['message'] if error else '串口采集进程已退出')

    def _run_feedback_consumer(self, closed):
        try:
            while not closed.wait(.01):
                self._consume_feedback()
        except Exception as exc:
            with self.lock:
                self.feedback_error = exc
                self.error = str(exc)
                self.connected = False
                self.pending = self.lease_task = None
                self._invalidate('串口已关闭，请重新连接并定位')

    def _clear_obstacles(self):
        self.obstacles.clear()
        self.scan_stamp = self.last_usable_scan = None
        self.scan_points = self.scan_expires = None
        self.scan_status = 'unavailable'
        self.obstacle_hits.clear()
        self.clear_work = None

    def _scan_points(self):
        if self._scan_current():
            return self.scan_points
        return None

    def _scan_current(self):
        return self.scan_expires is not None and self.clock() < self.scan_expires and self._fresh()

    def _scan_diagnostics(self):
        diagnostic = getattr(self.scanner, 'diagnostics', None)
        now = self.clock()
        return dict(localization=self.localization['mode'], source_fresh=self._fresh(),
                    source_age_ms=(None if self.source_time is None else (now-self.source_time)*1000),
                    cached_scan_stamp_ns=self.scan_stamp,
                    cached_scan_remaining_ms=(None if self.scan_expires is None else
                                              (self.scan_expires-now)*1000),
                    pairing=diagnostic() if callable(diagnostic) else None)

    def _record_scan_failure(self, event, stage, error):
        self.last_scan_failure = dict(event=event, stage=stage, error=error,
                                      automatic_resume=self.detour is not None,
                                      task=self.latest.status.task if self.latest else None,
                                      monotonic_s=self.clock(), diagnostic=self._scan_diagnostics())
        print(json.dumps(self.last_scan_failure, ensure_ascii=False), flush=True)

    # 按扫描时刻的实测里程计投影障碍；提交前再次核对定位及数据时效。
    def update_scan(self):
        with self.lock:
            if self.alignment is None or not self._fresh():
                return
            anchor, last_stamp = self.alignment, self.scan_stamp
            identity = self.latest.status.boot, self.latest.status.epoch
        frame = self.scanner.latest_observation()
        if frame is None or (last_stamp is not None and frame['stamp_ns'] <= last_stamp):
            return
        odom = frame['pose']
        if (odom['boot'], odom['epoch']) != identity:
            return
        stamp = frame['stamp_ns']
        if not frame['usable']:
            scan_status = ('filtered_ranges' if frame.get('finite_count', 0) else
                           'invalid_ranges' if frame.get('invalid_count', 0) else
                           'no_return' if frame.get('no_return_count', 0) else 'no_usable_ranges')
            points, hits, rays = None, frozenset(), ()
        else:
            pose = compose(anchor, odom)
            c, s = math.cos(pose['yaw']), math.sin(pose['yaw'])

            def project(x, y):
                return pose['x'] + c*x - s*y, pose['y'] + s*x + c*y

            scan_status = 'usable'
            points = tuple(project(x, y) for x, y in frame['points'])
            hits = self.grid.obstacle_cells(points)
            rays = tuple((*project(x0, y0), *project(x1, y1)) for x0, y0, x1, y1 in frame['clear_rays'])
        with self.lock:
            if self.alignment is not anchor or not self._fresh() or self.clock() >= frame['expires_mono']:
                return
            self.scan_stamp, self.scan_expires = stamp, frame['expires_mono']
            self.scan_status, self.scan_points = scan_status, points
            self.clear_work = None
            if points is not None:
                self.last_usable_scan = frame['source_mono']
                older_cells = self.obstacles.cells - hits
                self.obstacles.cells |= hits
                self.obstacle_hits.update(dict.fromkeys(hits, stamp))
                if older_cells and rays:
                    self.clear_work = anchor, stamp, frame['source_mono'], older_cells, rays

    # 射线在锁外计算，清除时保留同一帧及更新帧命中的障碍。
    def clear_obstacle_rays(self):
        with self.lock:
            work, self.clear_work = self.clear_work, None
        if work is None:
            return
        anchor, stamp, source_mono, cells, rays = work
        cleared = self.obstacles.cleared_cells(cells, rays)
        with self.lock:
            if (self.alignment is not anchor or self._scan_points() is None
                    or self.clock() - source_mono >= self.obstacle_scan_timeout_s):
                return
            # A hit from this scan or a newer one overrides the older clear ray.
            cleared = {cell for cell in cleared if self.obstacle_hits.get(cell, stamp) < stamp}
            self.obstacles.cells -= cleared
            for cell in cleared:
                del self.obstacle_hits[cell]

    def run_obstacle_worker(self, closed, work):
        while not closed.wait(OBSTACLE_WORK_INTERVAL_S):
            work()

    def _upload_obstacles(self):
        points = self._scan_points()
        if points is None:
            raise ValueError('避障测试需要定位后有效的雷达数据')
        return self.obstacles.cells

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

    # 手动对齐建立地图到里程计的变换；对齐期间车体移动则放弃。
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
                    self._clear_obstacles()
                    self.origin = dict(x=odom_pose['x'], y=odom_pose['y'], yaw=odom_pose['yaw'])
                    self.localization = dict(mode=mode, reason=None, stamp_ns=map_pose.get('stamp_ns'),
                                             boot=str(current.boot), epoch=current.epoch)
                    self.trail.clear()
                    self.trail.append(self._map_pose())
            except Exception as exc:
                with self.lock:
                    self._invalidate(str(exc))
                raise

    # 规划后重新核对起点和定位，避免执行过期预览。
    def preview(self, goal):
        with self._stationary_work('planning') as stop_revision:
            with self.lock:
                anchor, pose = self.alignment, self._map_pose()
                self.plan = None
                if pose is None:
                    raise ValueError('请先完成地图定位')
                cells = self.obstacles.cells
            plan = (self.grid.plan(pose, goal, obstacle_cells=cells)
                    if self.scanner is not None else self.grid.plan(pose, goal))
            with self.lock:
                if (self.stop_revision != stop_revision or anchor is not self.alignment
                        or not same_pose(self._map_pose(), pose)):
                    raise ValueError('规划期间起点已变化，请重新预览')
                self.plan = dict(plan, id=uuid.uuid4().hex)

    # 开始请求消费预览并排队，真正的串口上传由唯一 owner 完成。
    def request_start(self, plan_id):
        with self.lock:
            self._require_stationary()
            if self.work or self.stop_requested or self.detour is not None:
                raise ValueError('当前操作正在处理')
            if self.plan is None or self.plan['id'] != plan_id or not self.plan['segments']:
                raise ValueError('请预览一条可执行的新路径')
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
        if self.feedback is not None:
            self.feedback.request_stop()

    def _observe_upload(self, exchange):
        self.observe(exchange)
        with self.lock:
            if self.stop_requested:
                raise ValueError('上传已取消')

    # 截止时已有有效配对仍可使用；query 在此只等待缓存推进，不发送串口。
    def _wait_start_scan(self, master, deadline, stage):
        while True:
            with self.lock:
                if self.stop_requested:
                    return False
                now = self.clock()
                if self._scan_points() is not None:
                    return True
                if now >= deadline:
                    message = '启动前等待有效的雷达与里程计配对超时（{:g} s）'.format(START_SCAN_WAIT_S)
                    self._record_scan_failure('v12_scan_wait_expired', stage, message)
                    if self.detour is not None:
                        self.pending = self.work = None
                        self.detour['attempt'] = None
                        self.detour['retry_at'] = now + REPLAN_INTERVAL_S
                        self.avoidance_phase = 'waiting'
                        self.avoidance_message = '续行等待雷达与里程计配对超时，保持停车'
                        return False
                    raise ValueError(message)
            # Leave the existing wait budget intact; the serial worker continues independently.
            query_cutoff = deadline - master.reply_timeout
            if now < query_cutoff:
                reply = master.query(start_before=query_cutoff)
                if reply is not None:
                    self.observe(reply)
                    continue
            remaining = deadline - self.clock()
            if remaining > 0:
                master.sleep(min(remaining, OBSTACLE_WORK_INTERVAL_S))

    # 上传前和 START 前共用扫描等待期限，并重新核对停稳、起点和障碍。
    def _upload(self, master, plan):
        wait_for_scan = self.scanner is not None
        scan_deadline = self.clock() + START_SCAN_WAIT_S
        if wait_for_scan and not self._wait_start_scan(master, scan_deadline, 'before_upload'):
            return
        with self.lock:
            if self.stop_requested:
                return
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
            revision = self.feedback.stop_revision if self.feedback is not None else None
        reply = (master.upload(plan, on_ack=self._observe_upload, revision=revision) if self.feedback is not None
                 else master.upload(plan, on_ack=self._observe_upload))[-1]
        with self.lock:
            if self.mode == 'ready' or self.stop_requested:
                return
            wait_again = wait_for_scan and self._scan_points() is None
        if wait_again and not self._wait_start_scan(master, scan_deadline, 'before_start'):
            return
        with self.lock:
            if self.stop_requested:
                return
            self._require_stationary()
            current_pose = self._map_pose()
            if not same_pose(current_pose, pose):
                raise ValueError('上传期间起点已变化，请重新预览')
            current_cells = self._upload_obstacles() if self.scanner is not None else cells
        if current_pose != pose or current_cells != cells:
            self.grid.validate_plan(current_pose, plan, obstacle_cells=current_cells)
        with self.lock:
            if self.stop_requested:
                return
            self._require_stationary()
            if not same_pose(self._map_pose(), current_pose):
                raise ValueError('上传期间起点已变化，请重新预览')
            if self.scanner is not None and self._upload_obstacles() != current_cells:
                raise ValueError('上传期间障碍已变化，请重新预览')
            self.work = 'starting'
            revision = self.feedback.stop_revision if self.feedback is not None else None
        # 子进程在实际发送前复核这一代次，STOP 无需等待导航锁或 ACK。
        started = (master.start(reply.status.task, revision=revision) if self.feedback is not None
                   else master.start(reply.status.task))
        self.observe(started)
        status = started.status
        with self.lock:
            if not self.stop_requested and status.state in LEASE_STATES and status.lease_remaining_ms:
                self.lease_task = status.boot, status.epoch, status.task
                if self.detour is not None:
                    rejoining = self.detour.get('rejoining', False)
                    self.detour = None
                    if not rejoining:
                        self.replans += 1
                    self.stop_ack = self.stop_verified = self.stop_evidence = None
                    self.avoidance_phase = 'watching'
                    self.avoidance_message = ('沿原路线继续前往目标' if rejoining
                                              else '已换路线，继续前往原目标')
            elif self.detour is not None:
                self._cancel_detour('底盘未进入续行状态，请人工处理')

    def _lease_seconds(self, exchange):
        # The MCU computes its remaining time after this send, so this is a lower bound.
        return max(0., exchange.sent_at + exchange.status.lease_remaining_ms / 1000. - self.clock())

    # 仅为当前会话、当前任务且反馈新鲜的有效租约续期。
    def _renew_if_due(self, master, reply):
        if self.feedback is not None:
            self._consume_feedback()
        with self.lock:
            if self.lease_task is None:
                return
            reply = self.latest
            status = reply.status
            if (self.lease_task != (status.boot, status.epoch, status.task)
                    or status.state not in LEASE_STATES or not status.lease_remaining_ms):
                self.lease_task = None
                return
            if self.stop_requested or not self._fresh() or not 0 < self._lease_seconds(reply) <= RENEW_BEFORE_S:
                return
            revision = self.feedback.stop_revision if self.feedback is not None else None
        try:
            renewed = (master.renew(status.task, revision=revision) if self.feedback is not None
                       else master.renew(status.task))
            self.observe(renewed)
        except RequestCancelled:
            with self.lock:
                self.lease_task = None
        except ValueError:
            # 子进程可能在 RPC 排队期间已查询到任务结束；这不是串口失效。
            if self.feedback is not None:
                self._consume_feedback()
            with self.lock:
                self.lease_task = None
        except Rejected as exc:
            # The MCU may complete or stop the task while RENEW is in flight.
            self.observe(exc.exchange)
            with self.lock:
                self.lease_task = None
                self.error = '任务续约未获底盘接受'

    # STOP 应答与后续新鲜零速证据分别记录；应答不等于实测停稳。
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
                    self.avoidance_phase = 'waiting'
                    self.avoidance_message = ('已停稳，等待配对恢复后复验原路线' if 'remaining' in self.detour
                                              else '已停稳，等待重新规划')
                else:
                    self._cancel_detour(self.error)

    # 扫描持续缺失或剩余路线受阻时先停车，不在行驶中直接换路线。
    def _check_route(self, reply):
        status = reply.status
        identity = status.boot, status.epoch, status.task
        # In the handoff state the reported segment has already finished.
        index = status.index + (status.state == 6)
        progress = status.progress if status.state in (4, 5) else 0.
        with self.lock:
            if (self.stop_requested or self.lease_task != identity or status.state not in LEASE_STATES
                    or status.first_error or status.first_drive_error):
                return
            pose, points = self._map_pose(), self._scan_points()
            route, cells = self.route, self.obstacles.cells
            if points is None:
                if (self.last_usable_scan is None or
                        self.clock() - self.last_usable_scan >= self.obstacle_scan_timeout_s):
                    if route is not None:
                        self.detour = dict(goal=route['goal'], attempt=None, retry_at=0.,
                                           remaining=(route, index, progress))
                    self.avoidance_phase, self.avoidance_message = 'stopping', '雷达持续无有效观测，停车等待恢复'
                    self._queue_stop()
                return
        blocked = self.grid.route_blocked(pose, route, index, progress, cells)
        with self.lock:
            if blocked and not self.stop_requested and self.route is route and self.lease_task == identity:
                # Memory survives leaving the fan, but measured free rays can clear it.
                self.detour = dict(goal=route['goal'], attempt=None, retry_at=0.)
                self.avoidance_phase, self.avoidance_message = 'stopping', '前方路线被挡，正在停车'
                self._queue_stop()

    # 缺测停车先复验原路线。仅剩余扫掠被障碍占用时搜索绕行。
    def _avoidance_cycle(self, master, reply):
        with self.lock:
            detour = self.detour
        if detour is None:
            self._check_route(reply)
            return
        with self.lock:
            if self.detour is not detour:
                return
            if not self.stop_verified or self.stop_requested or not self._stationary():
                return
            status = self.latest.status
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
            cells = self.obstacles.cells
            if cells == detour['attempt']:
                return
            detour['attempt'], detour['retry_at'] = cells, self.clock() + REPLAN_INTERVAL_S
            pose = self._map_pose()
            remaining = detour.get('remaining')

        rejoining = remaining is not None
        try:
            plan = self.grid.resume_route(pose, *remaining, obstacle_cells=cells) if rejoining else None
            if plan is None:
                rejoining = False
                with self.lock:
                    if self.detour is not detour:
                        return
                    self.avoidance_phase, self.avoidance_message = 'replanning', '正在重新规划到原目标'
                plan = self.grid.plan(pose, detour['goal'], obstacle_cells=cells)
        except PlanningError:
            with self.lock:
                if self.detour is detour:
                    self.avoidance_phase = 'waiting'
                    self.avoidance_message = ('无法安全接回原路线，保持停车' if rejoining
                                              else '暂无绕行路线，保持停车')
            return

        # 等新的缓存反馈后复核起点。
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
            detour['rejoining'] = rejoining
            self.avoidance_phase = 'resuming'
            self.avoidance_message = ('已复验原路线剩余部分，正在下发' if rejoining
                                      else '已找到绕行路线，正在下发')

    # 导航线程只读缓存并提交命令；串口进程的查询不依赖本循环进度。
    def run(self, device, closed, trace_feedback_timing=False):
        master = FeedbackWorker(device, trace_timing=trace_feedback_timing)
        self.feedback = master
        self.feedback_cursor = self.feedback_missed = 0
        self.feedback_error = None
        consumer_closed = threading.Event()
        consumer = None
        try:
            master.launch()
            self.observe(master.discover())
            consumer = threading.Thread(target=self._run_feedback_consumer,
                                        args=(consumer_closed,), name='v12-cache-consumer')
            consumer.start()
            while not closed.is_set():
                if self.feedback_error is not None:
                    raise self.feedback_error
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
                            with self.lock:
                                reply = self.latest
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
                        if self.scanner is not None:
                            self._record_scan_failure('v12_navigation_rejected',
                                                      self.work or 'query', self.error)
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
            consumer_closed.set()
            if consumer is not None:
                consumer.join()
            master.close()
            with self.lock:
                self.connected = False
                self.pending = None
                self.lease_task = None
                self.stop_requested = False
                self._invalidate('串口已关闭，请重新连接并定位')

    # 网页读取缓存快照；过期反馈不能证明当前位置或停车状态。
    def snapshot(self):
        feedback = self.feedback.snapshot() if self.feedback is not None else None
        communication = self.feedback.cached_diagnostics(feedback) if feedback is not None else None
        cached = feedback['latest'] if feedback is not None else None
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
                scan_status = self.scan_status if self._scan_current() else 'unavailable'
                message = self.avoidance_message
                if points is None and self.work == 'uploading':
                    message = '启动前等待有效的雷达与里程计配对'
                elif points is None and self.avoidance_phase == 'watching':
                    message = {'filtered_ranges': '雷达回波被距离配置过滤，等待可用观测',
                               'invalid_ranges': '雷达观测无效，等待有效数据',
                               'no_return': '雷达全无回波，暂不能确认通道情况'}.get(
                                   scan_status, '等待定位和有效雷达数据')
                avoidance = dict(phase=self.avoidance_phase, message=message, replans=self.replans,
                                 scan_status=scan_status, remembered_cells=len(self.obstacles.cells),
                                 diagnostic=self._scan_diagnostics(), last_failure=self.last_scan_failure)
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
            result = dict(map=self.grid.metadata(), localization=dict(self.localization), pose=pose,
                        local_pose=(compose(inverse(self.origin), pose_of(status))
                                    if pose is not None and self.origin else None),
                        trail=list(self.trail), preview=self.plan, route=self.route,
                        connected=self.connected,
                        source_fresh=fresh, busy=bool(self.work or self.stop_requested), mode=self.mode,
                        task=task, avoidance=avoidance, can_prepare=preparing,
                        can_start=bool(preparing and pose and self.plan and self.plan['segments']),
                        can_stop=self.connected and self.mode == 'drive', error=self.error)
            if communication is not None:
                communication = dict(communication, consumer_missed=self.feedback_missed,
                                     consumer_lag_ms=(None if cached is None or self.latest is None else
                                         max(0., cached.received_at-self.latest.received_at)*1000))
            result['communication'] = communication
        if avoidance is not None:
            avoidance['points'] = [dict(x=x, y=y) for x, y in points or ()]
        return result


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
            elif (self.path == '/api/feedback-timing' and nav.feedback is not None
                  and nav.feedback.timing is not None):
                self.send(200, nav.feedback.timing.snapshot())
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
    parser.add_argument('--laser-x', type=float, default=.13, help='雷达相对车体的前向偏移（米）')
    parser.add_argument('--laser-y', type=float, default=0., help='雷达相对车体的左向偏移（米）')
    parser.add_argument('--laser-yaw', type=float, default=0., help='雷达相对车体的偏航（弧度）')
    parser.add_argument('--obstacle-min-range', type=float, default=1.,
                        help='避障采用的最小雷达距离（米），按实际安装确认后设置')
    parser.add_argument('--obstacle-scan-timeout', type=float, default=OBSTACLE_SCAN_TIMEOUT_S,
                        help='持续缺少可用扫描后的 RK 停车等待时间（秒）')
    parser.add_argument('--mode', choices=('ready', 'drive'), default='ready')
    parser.add_argument('--manual-only', action='store_true', help='只启用地图手动对齐')
    parser.add_argument('--no-modeled-error-margin', action='store_true',
                        help='路线扫掠只检查车体包络，不叠加动作结束误差')
    parser.add_argument('--obstacle-test', action='store_true', help='启用前方雷达挡路停车与自动重规划')
    parser.add_argument('--trace-feedback-timing', action='store_true',
                        help='在 RK 本机接口记录最近 4096 条 QUERY 的分段计时')
    args = parser.parse_args(argv)
    if (not all(math.isfinite(v) for v in (args.laser_x, args.laser_y, args.laser_yaw,
                                          args.obstacle_min_range, args.obstacle_scan_timeout))
            or args.obstacle_min_range < 0 or args.obstacle_scan_timeout <= 0):
        parser.error('雷达参数必须为有限数；最小距离不能为负，缺测等待时间必须为正')
    mounting = dict(laser_x=args.laser_x, laser_y=args.laser_y, laser_yaw=args.laser_yaw)
    grid = load_map(args.map_pgm, args.map_yaml, args.map_sha256, args.yaml_sha256,
                    use_modeled_error_margin=not args.no_modeled_error_margin)
    localizer = None
    if not args.manual_only:
        from v11_ros import RosLocalizer
        localizer = RosLocalizer(args.map_yaml, args.scan_topic, **mounting)
    scanner = None
    if args.obstacle_test:
        from v11_ros import RosObstacleScanner
        scanner = RosObstacleScanner(args.scan_topic, min_range_m=args.obstacle_min_range, **mounting)
    nav = Navigation(grid, localizer, mode=args.mode, scanner=scanner,
                     obstacle_scan_timeout_s=args.obstacle_scan_timeout)
    closed = threading.Event()
    workers = [threading.Thread(target=nav.run, args=(args.port, closed, args.trace_feedback_timing),
                                name='v12-navigation')]
    if scanner is not None:
        for work in (nav.update_scan, nav.clear_obstacle_rays):
            workers.append(threading.Thread(target=nav.run_obstacle_worker, args=(closed, work), name=work.__name__))
    try:
        with ThreadingHTTPServer(('127.0.0.1', 8767), handler_for(nav)) as server:
            for worker in workers:
                worker.start()
            print('RK V12 map API: http://127.0.0.1:8767 ({})'.format(args.mode), flush=True)
            server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        closed.set()
        for worker in workers:
            if worker.ident is not None:
                worker.join()
        if localizer is not None:
            localizer.close()
        if scanner is not None:
            scanner.close()


if __name__ == '__main__':
    main()
