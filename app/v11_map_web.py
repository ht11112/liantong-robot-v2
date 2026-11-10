"""Loopback map console and fixed HTTP tunnel to the RK navigation daemon."""
import argparse
import getpass
import hashlib
import http.client
import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import secrets
import threading
from urllib.parse import urlsplit

from local_p2p_server import ORIGIN, RequestError
from manual_mapping_viewer import pgm_to_png
from v11_map_plan import load_map


HERE = Path(__file__).resolve().parent
HOST, PC_PORT, RK_PORT = '127.0.0.1', 8766, 8767
ACTION_PATHS = frozenset(('/api/localize', '/api/zero', '/api/preview', '/api/start', '/api/stop'))


# PC 通过已校验主机密钥的 SSH 通道访问 RK 本机 API。
class SSHHTTP:
    """Reuse the pinned SSH connection; each HTTP request has its own channel."""
    def __init__(self, host, user, known_hosts, alias, password):
        self.host, self.user, self.password = host, user, password
        self.known_hosts, self.alias = str(known_hosts), alias
        self.client = None
        self.lock = threading.Lock()

    def close(self):
        with self.lock:
            if self.client is not None:
                self.client.close()
                self.client = None

    def transport(self):
        import paramiko
        with self.lock:
            if self.client is not None:
                transport = self.client.get_transport()
                if transport is not None and transport.is_active():
                    return transport
                self.client.close()
                self.client = None
            client = paramiko.SSHClient()
            try:
                client.load_host_keys(self.known_hosts)
                if self.alias:
                    keys = paramiko.HostKeys(self.known_hosts).lookup(self.alias)
                    if not keys:
                        raise ValueError('Pinned RK host key alias is missing')
                    for kind, key in keys.items():
                        client.get_host_keys().add(self.host, kind, key)
                client.set_missing_host_key_policy(paramiko.RejectPolicy())
                client.connect(self.host, username=self.user, password=self.password,
                    allow_agent=False, look_for_keys=False, timeout=6, banner_timeout=12, auth_timeout=12)
            except BaseException:
                client.close()
                raise
            self.client = client
            return client.get_transport()

    def request(self, method, path, body=None):
        timeout = 35 if method == 'POST' else 6
        channel = self.transport().open_channel('direct-tcpip', (HOST, RK_PORT), (HOST, PC_PORT), timeout=6)
        connection = http.client.HTTPConnection(HOST, RK_PORT, timeout=timeout)
        connection.sock = channel
        try:
            channel.settimeout(timeout)
            headers = {'Connection': 'close'}
            if body is not None:
                headers['Content-Type'] = 'application/json'
            connection.request(method, path, body=body, headers=headers)
            # getresponse() closes a connection-close socket after its headers;
            # unlike a native socket file, ChannelFile cannot outlive its channel.
            with http.client.HTTPResponse(channel, method=method) as response:
                response.begin()
                limit = 8*1024*1024
                raw = response.read(limit+1)
                if len(raw) > limit:
                    raise ValueError('RK response exceeds the size limit')
                payload = json.loads(raw)
                if not isinstance(payload, dict):
                    raise ValueError('Expected a JSON object from RK')
                return response.status, payload
        finally:
            connection.close()


# 本地地图先核对哈希，并与 RK 返回的地图身份比较。
class MapGateway:
    def __init__(self, remote, map_pgm, map_sha256, *, costmap_grid=None):
        self.remote = remote
        raw = Path(map_pgm).read_bytes()
        self.sha256 = hashlib.sha256(raw).hexdigest()
        if self.sha256 != map_sha256.lower():
            raise ValueError('Saved map SHA-256 does not match')
        self.png, self.width, self.height = pgm_to_png(raw)
        self.costmap_png = self.costmap_id = None
        if costmap_grid is not None:
            if costmap_grid.pgm_sha256 != self.sha256:
                raise ValueError('Costmap PGM does not match the saved map')
            self.costmap_png, _, _ = pgm_to_png(costmap_grid.costmap_pgm())
            self.costmap_id = costmap_grid.map_id
        self.html = (HERE/'v11_map_viewer.html').read_bytes()
        self.csrf = secrets.token_urlsafe(32)

    def request(self, method, path, body=None):
        try:
            status, payload = self.remote.request(method, path, body)
        except Exception:
            if method == 'GET':
                return 200, dict(connected=False, can_prepare=False, can_start=False, can_stop=False,
                    csrf_token=self.csrf, error='未能读取 RK 状态，请检查 SSH 和 RK 服务。')
            return 502, dict(error='未收到 RK 操作应答。请查看最新状态后再操作。')
        if 'connected' in payload:
            metadata = payload.get('map') or {}
            map_matches = (metadata.get('pgm_sha256') == self.sha256 and metadata.get('width') == self.width
                           and metadata.get('height') == self.height)
            payload = dict(payload, csrf_token=self.csrf, map_image_available=map_matches,
                           costmap_image_available=(map_matches and self.costmap_id is not None
                                                    and metadata.get('map_id') == self.costmap_id))
        return status, payload


# 网页服务限本机访问，操作请求还需携带当前会话令牌。
def handler_for(gateway):
    class Handler(BaseHTTPRequestHandler):
        def setup(self):
            super().setup()
            self.connection.settimeout(3.)

        def log_message(self, fmt, *args):
            pass

        def send(self, status, payload, kind='application/json; charset=utf-8'):
            raw = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode('utf-8') if isinstance(payload, dict) else payload
            self.send_response(status)
            self.send_header('Content-Type', kind)
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; frame-ancestors 'none'; form-action 'none'")
            self.end_headers()
            self.wfile.write(raw)

        def allowed_host(self):
            return self.headers.get('Host') == '127.0.0.1:8766'

        def do_GET(self):
            if not self.allowed_host():
                return self.send(403, dict(error='仅允许本机指定地址'))
            target = urlsplit(self.path)
            if target.path == '/':
                return self.send(200, gateway.html, 'text/html; charset=utf-8')
            if target.path == '/api/state' and not target.query:
                return self.send(*gateway.request('GET', '/api/state'))
            if target.path == '/map.png' and target.query == 'v='+gateway.sha256:
                return self.send(200, gateway.png, 'image/png')
            if (target.path == '/costmap.png' and gateway.costmap_id is not None
                    and target.query == 'v='+gateway.costmap_id):
                _, state = gateway.request('GET', '/api/state')
                if state.get('costmap_image_available'):
                    return self.send(200, gateway.costmap_png, 'image/png')
            return self.send(404, dict(error='未找到'))

        def do_POST(self):
            try:
                if self.headers.get('Content-Type', '').split(';', 1)[0] != 'application/json':
                    raise RequestError('仅接受 JSON 请求', 415)
                if self.headers.get('Transfer-Encoding'):
                    raise RequestError('不支持分块请求', 400)
                length = int(self.headers.get('Content-Length', '0'))
                if not 1 <= length <= 1024:
                    raise RequestError('请求大小无效', 400)
                body = self.rfile.read(length)
                if len(body) != length:
                    raise RequestError('请求未完整接收', 400)
                if (not self.allowed_host() or self.headers.get('Origin') != ORIGIN
                        or not secrets.compare_digest(self.headers.get('X-CSRF-Token', ''), gateway.csrf)):
                    raise RequestError('本机页面令牌或来源校验失败', 403)
                if self.path not in ACTION_PATHS:
                    raise RequestError('未知操作', 404)
                # The RK API validates the JSON fields at the command boundary.
                return self.send(*gateway.request('POST', self.path, body))
            except RequestError as exc:
                return self.send(exc.status, dict(error=str(exc)))
            except (ValueError, TimeoutError):
                return self.send(400, dict(error='请求格式无效'))
    return Handler


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.1.228')
    parser.add_argument('--user', default='cat')
    parser.add_argument('--known-hosts', required=True)
    parser.add_argument('--host-key-alias')
    parser.add_argument('--map-pgm', required=True)
    parser.add_argument('--map-sha256', required=True)
    parser.add_argument('--map-yaml', help='Saved map YAML for the costmap panel')
    parser.add_argument('--yaml-sha256', help='Pinned SHA-256 of the saved map YAML')
    args = parser.parse_args(argv)
    if (args.map_yaml is None) != (args.yaml_sha256 is None):
        parser.error('--map-yaml and --yaml-sha256 must be specified together')
    grid = (load_map(args.map_pgm, args.map_yaml, args.map_sha256, args.yaml_sha256)
            if args.map_yaml is not None else None)
    remote = SSHHTTP(args.host, args.user, args.known_hosts, args.host_key_alias,
                     getpass.getpass('RK SSH password (memory only): '))
    gateway = MapGateway(remote, args.map_pgm, args.map_sha256, costmap_grid=grid)
    server = ThreadingHTTPServer((HOST, PC_PORT), handler_for(gateway))
    print('V12 map console: '+ORIGIN+'/', flush=True)
    try:
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        remote.close()
