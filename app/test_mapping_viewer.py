"""Read-only viewer checks; no SSH credentials, ROS or real devices."""
import http.client
import json
from pathlib import Path
import struct
import tempfile
import threading
import unittest
from unittest.mock import patch
import zlib
from http.server import ThreadingHTTPServer

from manual_mapping_viewer import pgm_to_png, checked_record, ReadOnlyCache, make_handler


class ViewerTests(unittest.TestCase):
    def test_png_preserves_top_down_rows_and_whitespace_pixels(self):
        png, width, height = pgm_to_png(b'P5\n# map\n2 2\n255\n'+bytes([10, 13, 0, 255]))
        self.assertEqual((width, height), (2, 2))
        cursor, payload = 8, b''
        while cursor < len(png):
            size = struct.unpack('>I', png[cursor:cursor+4])[0]
            kind, data = png[cursor+4:cursor+8], png[cursor+8:cursor+8+size]
            crc = struct.unpack('>I', png[cursor+8+size:cursor+12+size])[0]
            self.assertEqual(crc, zlib.crc32(kind+data) & 0xffffffff)
            if kind == b'IDAT':
                payload += data
            cursor += 12+size
        self.assertEqual(zlib.decompress(payload), bytes([0, 10, 13, 0, 0, 255]))

    def test_incomplete_or_wrong_depth_map_rejected(self):
        for raw in (b'P5\n2 2\n255\nx', b'P2\n1 1\n255\nx', b'P5\n1 1\n65535\nx'):
            with self.subTest(raw=raw), self.assertRaises(ValueError):
                pgm_to_png(raw)

    def test_only_dedicated_new_mapping_record_is_accepted(self):
        record = dict(remote_directory='/home/cat/p2p_navigation_20261103_manual_mapping_unit',
                      motion_requested=False, new_blank_map=True, serial_allowed_kinds=[2, 5])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'record.json'
            path.write_text(json.dumps(record))
            self.assertEqual(checked_record(path), record)
            record['motion_requested'] = True
            path.write_text(json.dumps(record))
            with self.assertRaises(ValueError):
                checked_record(path)

    def test_http_cache_receipt_does_not_refresh_sensor_source(self):
        cache = ReadOnlyCache('unused', lambda: None)
        # Uptime sampled before a three-second map download. Cache receipt is later.
        cache.age_reference_mono, cache.observed_mono = 100., 103.
        cache.state.update(observed_wall_s=1000., status_age_s=.1, odom_current_age_s=.2,
                           status={'monotonic_s': 42.})
        with patch('manual_mapping_viewer.time.monotonic', side_effect=[103., 110.]), \
                patch('manual_mapping_viewer.time.time', return_value=-1000.):
            first, result = cache.snapshot(), cache.snapshot()
        self.assertAlmostEqual(first['status_age_s'], 3.1)
        self.assertEqual(result['http_cache_age_s'], 7.)
        self.assertAlmostEqual(result['status_age_s'], 10.1)
        self.assertAlmostEqual(result['odom_current_age_s'], 10.2)
        self.assertEqual(result['status']['monotonic_s'], 42.)
        self.assertEqual(cache.state['status_age_s'], .1)

    def test_http_control_requests_are_rejected(self):
        cache = ReadOnlyCache('unused', lambda: None)
        server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(cache))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            conn = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=2.)
            conn.request('POST', '/api/stop')
            response = conn.getresponse()
            self.assertEqual(response.status, 405)
            response.read()
            conn.request('GET', '/api/state')
            response = conn.getresponse()
            self.assertEqual(response.status, 200)
            self.assertTrue(json.loads(response.read())['read_only'])
            conn.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2.)

    def test_image_hash_selects_its_own_snapshot_instead_of_newest_map(self):
        cache = ReadOnlyCache('unused', lambda: None)
        cache.png, cache.png_versions = b'new-map', {'old-hash': b'old-map', 'new-hash': b'new-map'}
        server = ThreadingHTTPServer(('127.0.0.1', 0), make_handler(cache))
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        try:
            conn = http.client.HTTPConnection('127.0.0.1', server.server_port, timeout=2.)
            conn.request('GET', '/api/map.png?v=old-hash')
            response = conn.getresponse()
            self.assertEqual(response.status, 200)
            self.assertEqual(response.read(), b'old-map')
            conn.request('GET', '/api/map.png?v=expired-hash')
            response = conn.getresponse()
            self.assertEqual(response.status, 404)
            response.read()
            conn.close()
        finally:
            server.shutdown()
            server.server_close()
            thread.join(2.)


if __name__ == '__main__':
    unittest.main()
