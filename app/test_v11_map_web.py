"""HTTP framing and saved-map routes with no RK or motion commands."""
from contextlib import closing
from http.client import HTTPConnection, IncompleteRead
from http.server import ThreadingHTTPServer
import io
import sys
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Thread
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from v11_map_plan import load_map
from v11_map_web import MapGateway, SSHHTTP, handler_for


class DelayedBodyChannel:
    """The first read yields headers; explicit channel close cuts off later data."""
    def __init__(self, body):
        self.chunks = [b'HTTP/1.0 200 OK\r\nContent-Length: '+str(len(body)).encode()+b'\r\n\r\n', body]
        self.closed = False

    def settimeout(self, timeout):
        self.timeout = timeout

    def sendall(self, data):
        pass

    def close(self):
        self.closed = True

    def makefile(self, mode):
        channel = self

        class Stream(io.RawIOBase):
            def readable(self):
                return True

            def readinto(self, buffer):
                if channel.closed or not channel.chunks:
                    return 0
                chunk = channel.chunks.pop(0)
                buffer[:len(chunk)] = chunk
                return len(chunk)

        return io.BufferedReader(Stream())


class TunnelTests(unittest.TestCase):
    def test_frame_bytes_are_not_json_decoded(self):
        jpeg = b'\xff\xd8\xff\xd9'
        channel = DelayedBodyChannel(jpeg)
        class Transport:
            def open_channel(self, *args, **kwargs):
                return channel
        remote = SSHHTTP('unused', 'cat', 'unused', None, None)
        remote.transport = Transport
        self.assertEqual(remote.request('GET', '/api/dock/frame.jpg', binary=True), (200, jpeg))

    def test_connection_close_does_not_discard_delayed_http_body(self):
        channel = DelayedBodyChannel(b'{"connected":true}')

        class Transport:
            def open_channel(self, *args, **kwargs):
                return channel

        remote = SSHHTTP('unused', 'cat', 'unused', None, None)
        remote.transport = Transport
        self.assertEqual(remote.request('GET', '/api/state'), (200, {'connected': True}))
        self.assertTrue(channel.closed)
        self.assertEqual(channel.chunks, [])

    def test_failed_channel_is_discarded_and_next_get_reconnects(self):
        first = Mock()
        first.get_transport.return_value.is_active.return_value = True
        failure = TimeoutError('channel open timed out')
        first.get_transport.return_value.open_channel.side_effect = failure
        second = Mock()
        channel = DelayedBodyChannel(b'{"connected":true}')
        second.get_transport.return_value.open_channel.return_value = channel
        paramiko = SimpleNamespace(SSHClient=Mock(side_effect=[first, second]), RejectPolicy=Mock())
        remote = SSHHTTP('unused', 'cat', 'unused', None, None)
        with patch.dict(sys.modules, paramiko=paramiko):
            with self.assertRaises(TimeoutError) as caught:
                remote.request('GET', '/api/state')
            self.assertIs(caught.exception, failure)
            self.assertIsNone(remote.client)
            first.close.assert_called_once()
            self.assertEqual(remote.request('GET', '/api/state'), (200, {'connected': True}))
        self.assertEqual(paramiko.SSHClient.call_count, 2)
        first.get_transport.return_value.open_channel.assert_called_once()
        second.get_transport.return_value.open_channel.assert_called_once()
        second.close.assert_not_called()

    def test_truncated_http_body_invalidates_transport_without_replaying_post(self):
        channel = DelayedBodyChannel(b'{"connected":true}')
        channel.chunks[0] = b'HTTP/1.0 200 OK\r\nContent-Length: 100\r\n\r\n'
        channel.sendall = Mock()
        client = Mock()
        client.get_transport.return_value.is_active.return_value = True
        client.get_transport.return_value.open_channel.return_value = channel
        remote = SSHHTTP('unused', 'cat', 'unused', None, None)
        remote.client = client
        with patch.dict(sys.modules, paramiko=SimpleNamespace()):
            with self.assertRaises(IncompleteRead):
                remote.request('POST', '/api/start', b'{"plan_id":"test"}')
        sent = b''.join(call.args[0] for call in channel.sendall.call_args_list)
        self.assertEqual(sent.count(b'POST /api/start HTTP/1.1'), 1)
        client.get_transport.return_value.open_channel.assert_called_once()
        client.close.assert_called_once()
        self.assertIsNone(remote.client)
        self.assertTrue(channel.closed)

    def test_old_request_failure_does_not_close_replacement_transport(self):
        old, replacement = Mock(), Mock()
        old.get_transport.return_value.is_active.return_value = True
        remote = SSHHTTP('unused', 'cat', 'unused', None, None)
        remote.client = old

        def fail_after_reconnect(*args, **kwargs):
            remote.client = replacement
            raise TimeoutError('old channel timed out')

        old.get_transport.return_value.open_channel.side_effect = fail_after_reconnect
        with patch.dict(sys.modules, paramiko=SimpleNamespace()):
            with self.assertRaises(TimeoutError):
                remote.request('GET', '/api/state')
        self.assertIs(remote.client, replacement)
        replacement.close.assert_not_called()


class ActionRouteTests(unittest.TestCase):
    def test_ekf_turn_uses_the_same_origin_and_token_guard(self):
        gateway = SimpleNamespace(csrf='test-token', request=Mock(return_value=(200, dict(connected=True))))
        body = b'{"direction":-1}'
        with ThreadingHTTPServer(('127.0.0.1', 0), handler_for(gateway)) as server:
            thread = Thread(target=server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
            thread.start()
            try:
                def post(origin, token):
                    with closing(HTTPConnection('127.0.0.1', server.server_port, timeout=2)) as client:
                        client.request('POST', '/api/ekf-turn', body, headers={
                            'Host': '127.0.0.1:8766', 'Origin': origin, 'Content-Type': 'application/json',
                            'X-CSRF-Token': token})
                        with client.getresponse() as response:
                            return response.status

                self.assertEqual(post('http://127.0.0.1:8766', 'wrong'), 403)
                self.assertEqual(post('http://127.0.0.1:8767', 'test-token'), 403)
                gateway.request.assert_not_called()
                self.assertEqual(post('http://127.0.0.1:8766', 'test-token'), 200)
                gateway.request.assert_called_once_with('POST', '/api/ekf-turn', body)
            finally:
                server.shutdown()
                thread.join()


class CostmapRouteTests(unittest.TestCase):
    def test_costmap_png_requires_the_rk_map_identity(self):
        with TemporaryDirectory() as directory:
            pgm = Path(directory) / 'map.pgm'
            yaml = pgm.with_suffix('.yaml')
            pgm.write_bytes(b'P5\n8 8\n255\n' + bytes([254]) * 64)
            yaml.write_text('image: map.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
                            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n', encoding='utf-8')
            grid = load_map(pgm, yaml)

            class Remote:
                metadata = grid.metadata()

                def request(self, method, path, body=None):
                    return 200, dict(connected=True, map=self.metadata)

            remote = Remote()
            gateway = MapGateway(remote, pgm, grid.pgm_sha256, costmap_grid=grid)
            with ThreadingHTTPServer(('127.0.0.1', 0), handler_for(gateway)) as server:
                thread = Thread(target=server.serve_forever, kwargs={'poll_interval': .01}, daemon=True)
                thread.start()
                try:
                    def get(version):
                        with closing(HTTPConnection('127.0.0.1', server.server_port, timeout=2)) as client:
                            client.request('GET', '/costmap.png?v=' + version, headers={'Host': '127.0.0.1:8766'})
                            with client.getresponse() as response:
                                return response.status, response.getheader('Content-Type'), response.read()

                    self.assertTrue(gateway.request('GET', '/api/state')[1]['costmap_image_available'])
                    self.assertEqual(get('0' * 64)[0], 404)
                    status, content_type, body = get(grid.map_id)
                    self.assertEqual((status, content_type), (200, 'image/png'))
                    self.assertTrue(body.startswith(b'\x89PNG\r\n\x1a\n'))
                    remote.metadata['map_id'] = '0' * 64
                    self.assertFalse(gateway.request('GET', '/api/state')[1]['costmap_image_available'])
                    self.assertEqual(get(grid.map_id)[0], 404)
                finally:
                    server.shutdown()
                    thread.join()


if __name__ == '__main__':
    unittest.main()
