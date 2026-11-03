"""Loopback web broker for explicit, local-frame single-goal jobs.

The saved map is an immutable picture. No map pose or map-to-odom transform is
invented here. Serial access and the motion loop remain on RK, never HTTP.
"""
import argparse
import copy
import getpass
import hashlib
import json
import math
from pathlib import Path
import queue
import re
import secrets
import shlex
import sys
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit

from manual_mapping_viewer import pgm_to_png

HERE = Path(__file__).resolve().parent
ORIGIN = "http://127.0.0.1:8766"


class RequestError(ValueError):
    def __init__(self, message, status=409):
        super().__init__(message)
        self.status = status


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def request_id(value):
    try:
        if type(value) is not str or str(uuid.UUID(value)) != value:
            raise ValueError()
    except (ValueError, AttributeError):
        raise RequestError("请求标识必须是标准 UUID", 400)
    return value


def pose_of(value):
    if not isinstance(value, dict) or not all(number(value.get(k)) for k in ("x", "y", "yaw")):
        raise ValueError("Missing finite measured pose")
    return {k: value[k] for k in ("x", "y", "yaw")}


def encoders(packet):
    if not isinstance(packet, dict):
        raise ValueError("Missing original encoder packet")
    result = {"left_raw": packet.get("position_left_raw"), "right_raw": packet.get("position_right_raw")}
    if not all(type(x) is int and 0 <= x < 2**32 for x in result.values()):
        raise ValueError("Invalid original uint32 encoder packet")
    return result


def compose(start, local):
    c, s = math.cos(start["yaw"]), math.sin(start["yaw"])
    return dict(x=start["x"]+c*local["x"]-s*local["y"],
                y=start["y"]+s*local["x"]+c*local["y"],
                yaw=math.atan2(math.sin(start["yaw"]+local["yaw"]), math.cos(start["yaw"]+local["yaw"])))


def relative_goal(pose, x, y):
    dx, dy = x-pose["x"], y-pose["y"]
    c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
    return dict(x=c*dx+s*dy, y=-s*dx+c*dy)


class SSHRemote:
    """One broker thread owns this connection. Auth stays in process memory."""
    def __init__(self, host, user, known_hosts, host_key_alias, directory, password):
        if not re.fullmatch(r"/home/cat/p2p_navigation_[A-Za-z0-9_]+", directory):
            raise ValueError("Use a dedicated installed RK package directory")
        self.host, self.user, self.password = host, user, password
        self.known_hosts, self.alias, self.directory = str(known_hosts), host_key_alias, directory
        self.client = None

    def close(self):
        if self.client is not None:
            self.client.close()
        self.client = None

    def request(self, payload):
        import paramiko
        if self.client is None or not self.client.get_transport() or not self.client.get_transport().is_active():
            self.close()
            client = paramiko.SSHClient()
            client.load_host_keys(self.known_hosts)
            if self.alias:
                keys = paramiko.HostKeys(self.known_hosts).lookup(self.alias)
                if not keys:
                    raise ValueError("Pinned RK host key alias is missing")
                for kind, key in keys.items():
                    client.get_host_keys().add(self.host, kind, key)
            client.set_missing_host_key_policy(paramiko.RejectPolicy())
            client.connect(self.host, username=self.user, password=self.password, allow_agent=False,
                           look_for_keys=False, timeout=6, banner_timeout=12, auth_timeout=12)
            self.client = client
        command = "/usr/bin/python3 -B " + shlex.quote(self.directory+"/local_p2p_remote.py") + " --stdio"
        try:
            stdin, out, err = self.client.exec_command(command, timeout=12)
            stdin.write(json.dumps(payload, allow_nan=False)+"\n")
            stdin.flush()
            stdin.channel.shutdown_write()
            raw, errors = out.read(8*1024*1024+1), err.read(16385)
            code = out.channel.recv_exit_status()
            if len(raw) > 8*1024*1024 or len(errors) > 16384:
                raise ValueError("RK response exceeds bounded size")
            result = json.loads(raw.decode("utf-8"))
            if code or result.get("ok") is not True:
                detail = result.get("error", {})
                raise RuntimeError(detail.get("message", str(detail)) if isinstance(detail, dict) else str(detail))
            return result
        except BaseException:
            self.close()
            raise


class Broker:
    def __init__(self, remote, map_pgm, map_sha256, map_resolution, state_file=None, clock=time.monotonic):
        raw = Path(map_pgm).read_bytes()
        digest = hashlib.sha256(raw).hexdigest()
        if digest != map_sha256:
            raise ValueError("Saved map hash mismatch; original map was not loaded")
        if not number(map_resolution) or map_resolution <= 0:
            raise ValueError("Invalid saved map resolution")
        self.png, width, height = pgm_to_png(raw)
        self.map = dict(width=width, height=height, resolution=map_resolution,
                        sha256=digest, image_url="/saved-map.png", reference_only=True)
        self.remote, self.clock = remote, clock
        self.lock, self.stop = threading.RLock(), threading.Event()
        self.queue = queue.Queue(maxsize=1)
        self.csrf = secrets.token_urlsafe(32)
        self.state_file = None if state_file is None else Path(state_file)
        self.reference = dict(epoch=None, valid=False, pose=dict(x=0., y=0., yaw=0.),
                              reason="请停稳后点击里程计置零", version=0)
        self.task = dict(id=None, phase="idle", active=False, complete=False,
                         stop_confirmed=False, position_verified=False, first_failure=None, goal=None)
        self.anchor, self.boot_id, self.task_start = None, None, None
        self.history, self.preview, self.seen_requests = [], None, {}
        self.last_received, self.error, self.busy = None, "等待 RK 连接", False
        self.odometry_age_at_receipt = None
        self.odometry_reference_mono = None
        self.last_probe = None
        self.discovery_needed, self.next_inventory = True, 0.
        if self.state_file and self.state_file.is_file():
            saved = json.loads(self.state_file.read_text(encoding="utf-8"))
            self.task = saved.get("task", self.task)
            self.task_start = saved.get("task_start")
            self.reference["pose"] = pose_of(saved.get("reference", {}).get("pose", self.reference["pose"]))
            self.reference["reason"] = "网页服务已重启，请重新置零；正在核对 RK 任务"
        self.thread = threading.Thread(target=self.run, name="RK-local-goal-broker", daemon=True)

    def persist(self):
        if self.state_file:
            self.state_file.parent.mkdir(parents=True, exist_ok=True)
            pending = self.state_file.with_suffix(".pending")
            pending.write_text(json.dumps(dict(reference=self.reference, task=self.task,
                                              task_start=self.task_start), ensure_ascii=False), encoding="utf-8")
            pending.replace(self.state_file)

    def snapshot(self):
        with self.lock:
            age = None if self.last_received is None else self.clock()-self.last_received
            connected = age is not None and 0 <= age < 3.5 and not self.error
            ref = copy.deepcopy(self.reference)
            ref["valid"] = bool(ref["valid"] and connected and not self.discovery_needed)
            return dict(csrf_token=self.csrf, connected=connected, error=self.error,
                        busy=self.busy, reference=ref, task=copy.deepcopy(self.task),
                        saved_map=self.map, history=copy.deepcopy(self.history),
                        snapshot_age_s=age, boot_id=self.boot_id,
                        last_probe=copy.deepcopy(self.last_probe),
                        odometry_age_s=(None if self.odometry_reference_mono is None else
                                        self.odometry_age_at_receipt+self.clock()-self.odometry_reference_mono),
                        limits=dict(linear_m_s=.03, angular_deg_s=6.9, arrival_m=.1,
                                    clearance_m=.2, timeout_s=200, feedback_timeout_ms=750))

    def invalidate(self, reason):
        self.reference.update(valid=False, reason=reason, version=self.reference["version"]+1)
        self.preview = None

    def call(self, data):
        began = self.clock()
        result = self.remote.request(data)
        result["_pc_request_started_monotonic_s"] = began
        with self.lock:
            # Include the entire SSH request/response delay in displayed ages.
            # A late response must never renew a source timestamp.
            self.last_received, self.error = began, None
        return result

    def available(self):
        state = self.snapshot()
        if self.busy or self.task["active"]:
            raise RequestError("正在处理任务，请等待或取消当前任务")
        if not state["connected"] or self.discovery_needed:
            raise RequestError("RK 尚未完成当前状态核对")

    def goal(self, data):
        if not self.reference["valid"] or data.get("epoch") != self.reference["epoch"]:
            raise RequestError("局部原点已失效，请停稳后重新置零")
        x, y = data.get("x"), data.get("y")
        if not all(number(v) and abs(v) <= 10 for v in (x, y)):
            raise RequestError("目标必须是 -10 到 10 米内的有限坐标", 400)
        local = relative_goal(self.reference["pose"], x, y)
        return dict(goal=dict(x=x, y=y), relative_goal=local,
                    distance_m=math.hypot(local["x"], local["y"]),
                    heading_deg=math.degrees(math.atan2(local["y"], local["x"])))

    def submit(self, op, data):
        with self.lock:
            if op == "preview":
                self.available()
                preview = dict(self.goal(data), preview_id=str(uuid.uuid4()), epoch=self.reference["epoch"],
                               version=self.reference["version"])
                self.preview = preview
                return copy.deepcopy(preview)
            if op == "cancel":
                if not self.task["active"] or data.get("task_id") != self.task["id"]:
                    raise RequestError("没有匹配的活动任务")
                if self.busy:
                    raise RequestError("任务请求处理中，稍后可取消")
                self.busy = True
                self.queue.put_nowait((op, dict(task_id=self.task["id"])))
                return dict(queued=True, task_id=self.task["id"])
            rid = request_id(data.get("request_id"))
            signature = json.dumps([op, data], sort_keys=True, allow_nan=False)
            if rid in self.seen_requests:
                if self.seen_requests[rid] != signature:
                    raise RequestError("同一请求标识不能更改参数", 400)
                return dict(queued=True, request_id=rid, duplicate=True)
            self.available()
            command = dict(request_id=rid)
            if op == "execute":
                goal = self.goal(data)
                preview = self.preview
                if (not preview or data.get("preview_id") != preview["preview_id"]
                        or preview["epoch"] != self.reference["epoch"]
                        or preview["version"] != self.reference["version"] or preview["goal"] != goal["goal"]):
                    raise RequestError("目标或原点已变，请重新预览")
                if data.get("area_clear") is not True:
                    raise RequestError("请先确认路径、四周及后方空位，且可人工停车", 400)
                command.update(goal, epoch=self.reference["epoch"], anchor=dict(self.anchor),
                               boot_id=self.boot_id, start=copy.deepcopy(self.reference["pose"]))
            elif op != "reset":
                raise RequestError("未知操作", 404)
            self.seen_requests[rid] = signature
            if len(self.seen_requests) > 512:
                self.seen_requests.pop(next(iter(self.seen_requests)))
            self.busy = True
            self.queue.put_nowait((op, command))
            return dict(queued=True, request_id=rid)

    def discover(self):
        inventory = self.call(dict(op="poll"))
        if len(inventory.get("active_jobs", [])) > 1:
            raise RuntimeError("RK 检测到多个任务，拒绝新控制")
        active = inventory.get("active_task_id")
        finished_task_id = None
        with self.lock:
            current_boot = inventory.get("boot_id")
            if not isinstance(current_boot, str):
                raise ValueError("Missing RK inventory boot identity")
            if self.boot_id is not None and current_boot != self.boot_id:
                self.invalidate("RK 已重启，旧坐标只作历史参考，请重新置零")
                self.task_start = None
            self.boot_id = current_boot
            if active:
                if self.task["id"] != active:
                    self.task_start = None
                    self.invalidate("发现 RK 活动任务，本地坐标待任务结束后重新置零")
                    self.task.update(id=active, goal=None)
                self.task.update(active=True, phase="reconnecting")
            elif self.task["active"]:
                # A lost start response must be resolved before any new origin/job.
                finished_task_id = self.task["id"]
        if finished_task_id:
            # SSH must never hold the HTTP snapshot lock, including recovery.
            try:
                self.update_task(self.call(dict(op="poll", task_id=finished_task_id)))
            except Exception:
                with self.lock:
                    self.task.update(active=False, phase="unknown", first_failure="RK 无活动任务；上次启动结果未取到")
                    self.invalidate("上次任务位置未核实，请重新置零")
        with self.lock:
            self.discovery_needed = False
            self.next_inventory = self.clock()+5.
            self.persist()

    def probe(self):
        result = self.call(dict(op="probe"))
        if not result.get("session_complete"):
            raise RuntimeError("底盘只读反馈未通过，保持原点无效")
        if any(result.get("complete_write_counts", {}).get(str(k), 0) for k in (3, 6, 7)):
            raise RuntimeError("只读探测出现禁止的控制请求")
        packet = result.get("latest_feedback")
        anchor = encoders(packet)
        if not isinstance(result.get("boot_id"), str):
            raise ValueError("Missing current RK boot identity")
        source = (result.get("origin") or {}).get("source_monotonic_s")
        current = result.get("now_monotonic_s")
        if not number(source) or not number(current) or not 0 <= current-source < .75:
            raise RuntimeError("原始轮速来源已经过期")
        with self.lock:
            self.odometry_age_at_receipt = current-source
            self.odometry_reference_mono = result["_pc_request_started_monotonic_s"]
            self.last_probe = dict(boot_id=result["boot_id"], source_monotonic_s=source,
                                   raw_packet=packet, query_counts=result.get("query_counts"),
                                   complete_write_counts=result.get("complete_write_counts"),
                                   actuation_requested=False)
        return result, anchor

    def idle_probe(self):
        result, anchor = self.probe()
        with self.lock:
            if self.reference["valid"] and (self.boot_id != result["boot_id"] or self.anchor != anchor):
                self.invalidate("静止期间编码器变化或 RK 重启，请停稳后重新置零")
            self.boot_id = result["boot_id"]

    def do_reset(self, data):
        self.discover()
        with self.lock:
            if self.task["active"]:
                raise RuntimeError("RK 仍有活动任务，不能置零")
        result, anchor = self.probe()
        with self.lock:
            self.anchor, self.boot_id = anchor, result["boot_id"]
            self.reference = dict(epoch=str(uuid.uuid4()), valid=True, pose=dict(x=0., y=0., yaw=0.),
                                  reason="当前静止位置和车头已设为局部原点", version=0,
                                  origin=copy.deepcopy(result.get("origin")))
            self.task.update(phase="idle", complete=False, stop_confirmed=False,
                             position_verified=False, first_failure=None, goal=None)
            self.history, self.preview = [dict(self.reference["pose"])], None
            self.persist()

    def do_execute(self, data):
        self.discover()
        if self.task["active"]:
            raise RuntimeError("RK 已有活动任务")
        probe, anchor = self.probe()
        with self.lock:
            if (not self.reference["valid"] or self.reference["epoch"] != data["epoch"]
                    or anchor != data["anchor"] or probe["boot_id"] != data["boot_id"]):
                self.invalidate("执行前编码器或原点已变化，请重新置零")
                raise RuntimeError(self.reference["reason"])
            self.task_start = data["start"]
            self.task = dict(id=data["request_id"], phase="starting", active=True,
                             goal=data["goal"], complete=False, stop_confirmed=False,
                             position_verified=False, first_failure=None)
            self.preview = None
            self.persist()
        response = self.call(dict(op="start", request_id=data["request_id"],
                                  goal_x=data["relative_goal"]["x"], goal_y=data["relative_goal"]["y"],
                                  expected_boot_id=data["boot_id"], expected_start_encoders=anchor,
                                  area_clear=True))
        self.update_task(response)

    def update_task(self, response):
        with self.lock:
            if response.get("task_id") != self.task["id"]:
                raise ValueError("RK task identity mismatch")
            live, result = response.get("live_status") or {}, response.get("result")
            active = response.get("active") is True
            current_boot = response.get("boot_id")
            if not isinstance(current_boot, str):
                raise ValueError("Missing RK task boot identity")
            if current_boot != self.boot_id:
                self.invalidate("RK 已重启，旧任务不能更新当前坐标，请重新置零")
                self.task_start, self.boot_id = None, current_boot
            identities = [(response.get("record") or {}).get("boot_id", current_boot)]
            if live:
                identities.append(live.get("boot_id"))
            if result:
                identities.append(result.get("boot_id"))
            if any(identity != current_boot for identity in identities):
                self.invalidate("任务记录来自此前 RK 启动，只作历史参考，请重新置零")
                self.task_start = None
                self.task.update(active=active, phase="historical", complete=False,
                                 stop_confirmed=False, position_verified=False,
                                 first_failure="旧启动的任务结果不能证明当前位姿或停车状态")
                self.persist()
                return
            self.task.update(active=active, phase=live.get("phase", "running" if active else "finished"))
            if not active and not result:
                self.task.update(phase="failed", complete=False, stop_confirmed=False,
                                 position_verified=False, first_failure=response.get("startup_error") or
                                 "RK 进程已退出，但没有完整结果；请检查现场并重新置零")
            if response.get("result_file"):
                self.task["result_file"] = response["result_file"]
            local_pose = live.get("pose")
            source, current = live.get("source_monotonic_s"), response.get("now_monotonic_s")
            if number(source) and number(current) and current >= source:
                self.odometry_age_at_receipt = current-source
                self.odometry_reference_mono = response.get("_pc_request_started_monotonic_s", self.last_received)
            if result:
                local_pose = result.get("latest_odometry")
                stop = result.get("final_stop_chain") or {}
                self.task.update(complete=result.get("session_complete") is True,
                                 stop_confirmed=stop.get("complete") is True,
                                 position_verified=result.get("position_verified") is True,
                                 first_failure=result.get("first_failure"),
                                 phase="completed" if result.get("session_complete") else "failed")
            if local_pose and self.task_start:
                pose = compose(self.task_start, pose_of(local_pose))
                self.reference["pose"] = pose
                if not self.history or self.history[-1] != pose:
                    self.history.append(pose)
                    self.history = self.history[-3000:]
            if not active:
                if result and self.task["position_verified"] and self.task["stop_confirmed"] and self.task_start:
                    packet = result.get("latest_feedback") or live.get("raw_packet")
                    if not packet:
                        rows = result.get("rows") or []
                        packet = next((row.get("packet") for row in reversed(rows)
                                       if isinstance(row.get("packet"), dict) and "position_left_raw" in row["packet"]), None)
                    if packet:
                        self.anchor = encoders(packet)
                        self.reference["version"] += 1
                    else:
                        self.invalidate("停车编码器未返回，请重新置零")
                else:
                    self.invalidate("任务结束位置未完整核实，请停稳后重新置零")
            self.persist()

    def cycle(self, operation=None):
        if self.discovery_needed:
            self.discover()
        if operation:
            op, data = operation
            if op == "reset":
                self.do_reset(data)
            elif op == "execute":
                self.do_execute(data)
            elif op == "cancel":
                self.update_task(self.call(dict(op="cancel", task_id=data["task_id"])))
        elif self.task["active"]:
            self.update_task(self.call(dict(op="poll", task_id=self.task["id"])))
        else:
            if self.clock() >= self.next_inventory:
                self.discover()
            if not self.task["active"]:
                self.idle_probe()

    def run(self):
        while not self.stop.is_set():
            try:
                operation = self.queue.get(timeout=.5 if self.task["active"] else 1.)
            except queue.Empty:
                operation = None
            try:
                self.cycle(operation)
            except Exception as exc:
                with self.lock:
                    self.error = str(exc)
                    self.discovery_needed = True
                    self.invalidate("连接或状态核对中断，请等待重连并重新置零")
            finally:
                if operation:
                    with self.lock:
                        self.busy = False
                    self.queue.task_done()
        self.remote.close()


def handler_for(broker):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(3.)

        def log_message(self, fmt, *args):
            pass

        def send(self, code, payload, kind="application/json; charset=utf-8"):
            raw = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode() if isinstance(payload, dict) else payload
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(raw)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.send_header("Content-Security-Policy", "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; img-src 'self'; frame-ancestors 'none'; connect-src 'self'")
            self.end_headers()
            self.wfile.write(raw)

        def allowed_host(self):
            return self.headers.get("Host") == "127.0.0.1:8766"

        def do_GET(self):
            if not self.allowed_host():
                return self.send(403, dict(error="仅允许本机指定地址"))
            path = urlsplit(self.path).path
            if path == "/":
                return self.send(200, (HERE/"local_p2p_viewer.html").read_bytes(), "text/html; charset=utf-8")
            if path == "/api/state":
                return self.send(200, broker.snapshot())
            if path == "/saved-map.png":
                return self.send(200, broker.png, "image/png")
            return self.send(404, dict(error="未找到"))

        def do_POST(self):
            try:
                if self.headers.get("Content-Type", "").split(";", 1)[0] != "application/json":
                    raise RequestError("仅接受 JSON 请求", 415)
                if self.headers.get("Transfer-Encoding"):
                    raise RequestError("不支持分块请求", 400)
                length = int(self.headers.get("Content-Length", "0"))
                if not 1 <= length <= 4096:
                    raise RequestError("请求大小无效", 400)
                data = json.loads(self.rfile.read(length))
                if not isinstance(data, dict):
                    raise RequestError("JSON 对象格式无效", 400)
                # Drain the bounded request before replying, so rejected POSTs
                # do not close TCP with an unread body and lose their response.
                if (not self.allowed_host() or self.headers.get("Origin") != ORIGIN
                        or not secrets.compare_digest(self.headers.get("X-CSRF-Token", ""), broker.csrf)):
                    raise RequestError("本机页面令牌或来源校验失败", 403)
                path = urlsplit(self.path).path
                if path not in ("/api/reset", "/api/preview", "/api/execute", "/api/cancel"):
                    raise RequestError("未知操作", 404)
                return self.send(200, broker.submit(path.rsplit("/", 1)[1], data))
            except RequestError as exc:
                return self.send(exc.status, dict(error=str(exc)))
            except (ValueError, TypeError, json.JSONDecodeError):
                return self.send(400, dict(error="请求格式无效"))
    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.1.228")
    parser.add_argument("--user", default="cat")
    parser.add_argument("--known-hosts", required=True)
    parser.add_argument("--host-key-alias")
    parser.add_argument("--remote-directory", required=True)
    parser.add_argument("--map-pgm", required=True)
    parser.add_argument("--map-sha256", required=True)
    parser.add_argument("--map-resolution", type=float, required=True)
    parser.add_argument("--state-file", default=str(HERE.parent/"runtime"/"local-p2p.json"))
    args = parser.parse_args(argv)
    password = getpass.getpass("RK SSH password (memory only): ")
    remote = SSHRemote(args.host, args.user, args.known_hosts, args.host_key_alias, args.remote_directory, password)
    password = None
    broker = Broker(remote, args.map_pgm, args.map_sha256, args.map_resolution, args.state_file)
    server = ThreadingHTTPServer(("127.0.0.1", 8766), handler_for(broker))
    broker.thread.start()
    print("Local point-to-point page: "+ORIGIN+"/", flush=True)
    try:
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        broker.stop.set()
        server.server_close()
        broker.thread.join(timeout=15)


if __name__ == "__main__":
    main()
