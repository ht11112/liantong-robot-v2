"""Local read-only viewer: SFTP files -> memory cache -> HTTP, no robot commands."""
import argparse
import copy
import getpass
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlsplit, parse_qs
import zlib

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent


def pgm_to_png(raw):
    """P5 is already top-down. Preserve every pixel, including whitespace bytes."""
    position = 0
    def token():
        nonlocal position
        while position < len(raw):
            if raw[position] in b' \t\r\n':
                position += 1
            elif raw[position] == 35:
                end = raw.find(b'\n', position)
                if end < 0:
                    raise ValueError('Unterminated PGM comment')
                position = end+1
            else:
                break
        start = position
        while position < len(raw) and raw[position] not in b' \t\r\n':
            position += 1
        return raw[start:position]
    if token() != b'P5':
        raise ValueError('Only grayscale P5 maps are accepted')
    width, height, maximum = int(token()), int(token()), int(token())
    if not 1 <= width <= 10000 or not 1 <= height <= 10000 or width*height > 8000000 or maximum != 255:
        raise ValueError('Invalid map dimensions or depth')
    if raw[position:position+2] == b'\r\n':
        position += 2
    elif raw[position:position+1] in (b' ', b'\t', b'\r', b'\n'):
        position += 1
    else:
        raise ValueError('Missing PGM pixel separator')
    pixels = raw[position:]
    if len(pixels) != width*height:
        raise ValueError('Incomplete PGM pixels')
    rows = b''.join(b'\0'+pixels[y*width:(y+1)*width] for y in range(height))
    def chunk(kind, data):
        return struct.pack('>I', len(data))+kind+data+struct.pack('>I', zlib.crc32(kind+data) & 0xffffffff)
    png = (b'\x89PNG\r\n\x1a\n'+chunk(b'IHDR', struct.pack('>IIBBBBB', width, height, 8, 0, 0, 0, 0))
           +chunk(b'IDAT', zlib.compress(rows, 3))+chunk(b'IEND', b''))
    return png, width, height


def checked_record(path):
    record = json.loads(Path(path).read_text(encoding='utf-8'))
    directory = record.get('remote_directory', '')
    if (record.get('motion_requested') is not False or record.get('new_blank_map') is not True
            or record.get('serial_allowed_kinds') != [2, 5]
            or not re.fullmatch(r'/home/cat/p2p_navigation_20261103_manual_mapping_[A-Za-z0-9_]+', directory)):
        raise ValueError('Viewer only accepts a dedicated new hand-pushed mapping record')
    return record


def finite_number(value):
    return type(value) in (int, float) and math.isfinite(value)


class ReadOnlyCache:
    def __init__(self, record_path, connect):
        self.record_path, self.connect = Path(record_path), connect
        self.lock, self.stop = threading.Lock(), threading.Event()
        self.state = dict(connected=False, error='等待只读数据连接', status=None, origin=None, map=None,
                          observed_wall_s=None, read_only=True, map_to_odom=None, history=[])
        self.png, self.epoch, self.map_signature = None, None, None
        self.png_versions = {}
        self.age_reference_mono = self.observed_mono = None
        self.history, self.last_odom_key = [], None
        self.thread = threading.Thread(target=self.run, name='readonly-SFTP-poller', daemon=True)

    @staticmethod
    def read(sftp, path, maximum=262144):
        with sftp.open(path, 'rb') as stream:
            raw = stream.read(maximum+1)
        if len(raw) > maximum:
            raise ValueError('Read-only input exceeds bounded size: '+Path(path).name)
        return raw

    def snapshot(self):
        with self.lock:
            result = copy.deepcopy(self.state)
            now = time.monotonic()
            elapsed = None if self.age_reference_mono is None else max(0., now-self.age_reference_mono)
            result['http_cache_age_s'] = (None if self.observed_mono is None else
                                         max(0., now-self.observed_mono))
        for name in ('status_age_s', 'odom_current_age_s'):
            if finite_number(result.get(name)):
                result[name] = None if elapsed is None else result[name]+elapsed
        return result

    def run(self):
        client = sftp = None
        while not self.stop.is_set():
            began = time.monotonic()
            try:
                record = checked_record(self.record_path)
                epoch = (record['remote_directory'], record['nonce'], record['expected_boot_id'])
                if epoch != self.epoch:
                    with self.lock:
                        self.state.update(connected=False, status=None, origin=None, map=None, history=[],
                                          status_age_s=None, odom_current_age_s=None, error='新建图会话准备中')
                        self.age_reference_mono = self.observed_mono = None
                        self.png = None
                        self.png_versions.clear()
                    self.epoch, self.map_signature = epoch, None
                    self.history, self.last_odom_key = [], None
                if client is None or not client.get_transport() or not client.get_transport().is_active():
                    if client:
                        client.close()
                    client = self.connect()
                    sftp = client.open_sftp()
                    sftp.get_channel().settimeout(4.)
                directory, nonce, boot = epoch
                actual_boot = self.read(sftp, '/proc/sys/kernel/random/boot_id', 128).decode().strip()
                if actual_boot != boot:
                    raise ValueError('RK已重启，当前会话记录需要重新核对')
                status = json.loads(self.read(sftp, directory+'/mapping_status.json').decode())
                origin = json.loads(self.read(sftp, directory+'/origin_record.json').decode())
                if any(item.get('boot_id') != boot or item.get('nonce') != nonce for item in (status, origin)):
                    raise ValueError('状态或原点不属于选定会话')
                # Anchor before the read: conservatively include SFTP latency and all
                # later map transfer/conversion time. HTTP receipt never renews it.
                age_reference_mono = time.monotonic()
                uptime = float(self.read(sftp, '/proc/uptime', 128).split()[0])
                status_mono = status.get('monotonic_s')
                status_age = max(0., uptime-status_mono)+.02 if finite_number(status_mono) else None
                obs = ((status.get('latest') or {}).get('odom') or {}).get('observation') or {}
                source = obs.get('source_monotonic_s')
                odom_current_age = max(0., uptime-source)+.02 if finite_number(source) else None
                key = (obs.get('sample_sequence'), source)
                if source is not None and key != self.last_odom_key:
                    if all(finite_number(obs.get(name)) for name in ('x', 'y', 'yaw')):
                        self.history.append(dict(x=obs['x'], y=obs['y'], yaw=obs['yaw'],
                                                 source_monotonic_s=source, discontinuity=bool(obs.get('discontinuity'))))
                        self.history[:] = self.history[-5000:]
                    self.last_odom_key = key
                image_error = None
                try:
                    metadata_path = directory+'/new_map/latest_metadata.json'
                    metadata_raw = self.read(sftp, metadata_path, 32768)
                    metadata = json.loads(metadata_raw.decode())
                    if metadata['header'].get('frame_id') != origin['frames']['map']:
                        raise ValueError('地图坐标系不属于当前建图原点')
                    if (not finite_number(metadata.get('resolution')) or metadata['resolution'] <= 0
                            or len(metadata.get('origin', [])) != 3 or not all(map(finite_number, metadata['origin']))):
                        raise ValueError('地图坐标参数无效')
                    signature = (metadata['pgm_sha256'], json.dumps(metadata['header'], sort_keys=True))
                    if signature != self.map_signature:
                        pgm = self.read(sftp, directory+'/new_map/latest.pgm', 16000000)
                        if self.read(sftp, metadata_path, 32768) != metadata_raw:
                            raise ValueError('地图正在更新，暂保留上一帧')
                        if hashlib.sha256(pgm).hexdigest() != metadata['pgm_sha256']:
                            raise ValueError('地图图像与元数据尚未同一版本，暂保留上一帧')
                        png, width, height = pgm_to_png(pgm)
                        if width != metadata['width'] or height != metadata['height']:
                            raise ValueError('地图图像尺寸与元数据不一致')
                        map_info = dict(metadata, image_url='/api/map.png?v='+metadata['pgm_sha256'],
                                        map_stamp=metadata['header'], source_header_validated=True,
                                        overlay_alignment_validated=False)
                        with self.lock:
                            self.png, self.state['map'] = png, map_info
                            self.png_versions[metadata['pgm_sha256']] = png
                            while len(self.png_versions) > 3:
                                self.png_versions.pop(next(iter(self.png_versions)))
                        self.map_signature = signature
                except OSError as exc:
                    image_error = '地图元数据读取失败，保留已有快照: '+str(exc)
                    # Earlier ended sessions have PGM/YAML, no small sidecar. Label the pair unverified.
                    try:
                        if self.map_signature is None and status.get('phase') == 'finished' and status.get('recording_active') is False:
                            prefix = 'final' if status.get('phase') == 'finished' else 'latest'
                            pgm = self.read(sftp, directory+'/new_map/'+prefix+'.pgm', 16000000)
                            yaml = self.read(sftp, directory+'/new_map/'+prefix+'.yaml', 8192).decode()
                            resolution_match = re.search(r'^resolution:\s*([0-9.eE+-]+)\s*$', yaml, re.M)
                            origin_match = re.search(r'^origin:\s*(\[[^\n]+\])\s*$', yaml, re.M)
                            if not resolution_match or not origin_match:
                                raise ValueError('地图YAML缺少分辨率或原点')
                            resolution, offset = float(resolution_match[1]), json.loads(origin_match[1])
                            if not finite_number(resolution) or resolution <= 0 or len(offset) != 3 or not all(map(finite_number, offset)):
                                raise ValueError('地图坐标参数无效')
                            png, width, height = pgm_to_png(pgm)
                            digest = hashlib.sha256(pgm).hexdigest()
                            map_info = dict(width=width, height=height, resolution=resolution, origin=offset,
                                            image_url='/api/map.png?v='+digest, pgm_sha256=digest,
                                            map_stamp=None, known_cells=None, source_header_validated=False,
                                            overlay_alignment_validated=False, historical_pair_unverified=True)
                            with self.lock:
                                self.png, self.state['map'] = png, map_info
                                self.png_versions[digest] = png
                            self.map_signature = ('legacy', digest)
                            image_error = '历史地图图像与YAML配对未核验，不用于定位叠加'
                        elif self.map_signature is None:
                            image_error = '当前地图元数据暂不可读，等待同版本图像与元数据'
                    except (OSError, ValueError, KeyError, TypeError) as exc:
                        image_error = str(exc)
                except (ValueError, KeyError, TypeError) as exc:
                    image_error = str(exc)
                with self.lock:
                    self.age_reference_mono = age_reference_mono
                    self.observed_mono = time.monotonic()
                    self.state.update(connected=True, error=None, image_error=image_error,
                        status=status, origin=origin, session=dict(nonce=nonce, boot_id=boot, remote_directory=directory),
                        nonce=nonce, boot_id=boot, remote_dir=directory,
                        observed_wall_s=time.time(), status_age_s=status_age, odom_current_age_s=odom_current_age,
                        age_basis='RK /proc/uptime + PC monotonic elapsed; conservative +20ms; receipt is not sensor time',
                        map_to_odom=None, history=list(self.history), read_only=True)
            except Exception as exc:
                with self.lock:
                    self.state.update(connected=False, error=type(exc).__name__+': '+str(exc),
                                      connection_checked_wall_s=time.time())
                if client:
                    client.close()
                client = sftp = None
            self.stop.wait(max(.1, 1.-(time.monotonic()-began)))
        if client:
            client.close()


def make_handler(cache):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass
        def do_GET(self):
            path = urlsplit(self.path).path
            if path == '/api/state':
                raw = json.dumps(cache.snapshot(), allow_nan=False, separators=(',', ':')).encode()
                content_type = 'application/json; charset=utf-8'
            elif path == '/api/map.png':
                with cache.lock:
                    version = parse_qs(urlsplit(self.path).query).get('v', [None])[0]
                    raw = cache.png_versions.get(version) if version else cache.png
                if raw is None:
                    self.send_error(404, 'No verified or labelled historical map image yet')
                    return
                content_type = 'image/png'
            elif path in ('/', '/index.html'):
                raw = (HERE/'manual_mapping_viewer.html').read_bytes()
                content_type = 'text/html; charset=utf-8'
            else:
                self.send_error(404)
                return
            self.send_response(200)
            self.send_header('Content-Type', content_type)
            self.send_header('Content-Length', str(len(raw)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            self.wfile.write(raw)
        def do_POST(self):
            self.send_error(405, 'Read-only viewer')
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', default=str(ROOT/'output/p2p_redesign_20261103/manual_mapping_package_current.json'))
    parser.add_argument('--port', type=int, default=8766)
    args = parser.parse_args()
    checked_record(args.record)
    if not (HERE/'manual_mapping_viewer.html').is_file():
        raise RuntimeError('Viewer HTML has not been installed')
    sys.path.insert(0, str(ROOT/'navigation-demo/.pc-runtime/site'))
    import paramiko
    password = getpass.getpass('RK SSH password (memory only): ')
    def connect():
        keys = paramiko.HostKeys(str(ROOT/'motor-diagnostics/build/rk_known_hosts'))
        client = paramiko.SSHClient()
        existing = keys.lookup('fe80::ecdf:6b65:24f6:9595%10')
        if not existing:
            raise RuntimeError('Pinned RK host key missing')
        for kind, key in existing.items():
            client.get_host_keys().add('192.168.1.228', kind, key)
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
        client.connect('192.168.1.228', username='cat', password=password, allow_agent=False,
                       look_for_keys=False, timeout=5, banner_timeout=8, auth_timeout=8)
        return client
    cache = ReadOnlyCache(args.record, connect)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(cache))
    server.daemon_threads = True
    cache.thread.start()
    print(json.dumps(dict(url='http://127.0.0.1:'+str(args.port)+'/', read_only=True,
                          robot_commands=False, credential_saved=False)), flush=True)
    try:
        server.serve_forever(poll_interval=.25)
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        cache.stop.set()
        cache.thread.join(timeout=6.)


if __name__ == '__main__':
    main()
