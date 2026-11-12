import math
import threading
import time
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from v12_dock_camera import DockCamera

try:
    import cv2
    import numpy as np
    HAVE_ARUCO = hasattr(cv2, 'aruco') and hasattr(cv2.aruco, 'DICT_APRILTAG_36h11')
except ImportError:
    HAVE_ARUCO = False


class CameraInputTest(unittest.TestCase):
    def test_bad_inputs_and_default_gate(self):
        for kwargs in (dict(tag_size_m=float('nan')), dict(camera_height_m=-1),
                       dict(mount_yaw_deg=float('inf')), dict(target_id=587)):
            with self.assertRaises(ValueError):
                DockCamera('/dev/video0', **kwargs)
        camera = DockCamera('/dev/video0')
        self.assertFalse(camera.snapshot()['ready'])
        self.assertFalse(camera.snapshot()['fresh'])


@unittest.skipUnless(HAVE_ARUCO, 'OpenCV aruco is unavailable')
class RealTagTest(unittest.TestCase):
    def test_real_detection_overlay_and_pnp_signs(self):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = (cv2.aruco.generateImageMarker(dictionary, 42, 300)
                  if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(dictionary, 42, 300))
        matrix = np.array([[477.0511, 0, 330.1754], [0, 479.6547, 231.7071], [0, 0, 1.]])
        distortion = np.zeros(5)
        size = .08
        points = np.array([[-size/2, -size/2, 0], [size/2, -size/2, 0],
                           [size/2, size/2, 0], [-size/2, size/2, 0]], dtype=float)
        camera = DockCamera('/dev/video0', target_id=42, tag_size_m=size,
                            mount_yaw_deg=0, mount_pitch_deg=0, mount_roll_deg=0,
                            camera_height_m=.14, verified=True)
        for quarter_turns in range(4):
            for tx, theta in ((0, 0), (-.04, 0), (.04, 0), (0, 10), (0, -10)):
                with self.subTest(pattern_rotation=90 * quarter_turns, tx=tx, theta=theta):
                    rotation = np.array([0., math.radians(theta), 0.])
                    projected, _ = cv2.projectPoints(points, rotation, np.array([tx, 0., .4]), matrix, distortion)
                    destination = projected.reshape(4, 2).astype(np.float32)
                    source = np.array([[0, 0], [299, 0], [299, 299], [0, 299]], dtype=np.float32)
                    transform = cv2.getPerspectiveTransform(source, destination)
                    frame = cv2.warpPerspective(np.rot90(marker, quarter_turns).copy(), transform, (640, 480), borderValue=255)
                    corners, ids, _ = (cv2.aruco.ArucoDetector(dictionary).detectMarkers(frame)
                                       if hasattr(cv2.aruco, 'ArucoDetector') else cv2.aruco.detectMarkers(frame, dictionary))
                    self.assertIsNotNone(ids)
                    image = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
                    before = image.copy()
                    observed = camera._observe(cv2, np, image, corners, ids, matrix, distortion)
                    self.assertTrue(observed['ready'])
                    self.assertEqual(observed['tag_id'], 42)
                    self.assertEqual(len(observed['corners']), 4)
                    self.assertTrue(np.any(image != before))
                    self.assertAlmostEqual(observed['distance_m'], .4 * math.cos(math.radians(theta)) + tx * math.sin(math.radians(theta)), delta=.012)
                    expected_y = -tx * math.cos(math.radians(theta)) + .6 * math.sin(math.radians(theta))
                    self.assertAlmostEqual(observed['lateral_m'], expected_y, delta=.015)
                    self.assertAlmostEqual(observed['yaw_deg'], theta, delta=4.)

        vertical_normal = np.array([[1., 0., 0.], [0., 0., -1.], [0., 1., 0.]])
        with patch.object(cv2, 'Rodrigues', return_value=(vertical_normal, None)):
            observed = camera._observe(cv2, np, image, corners, ids, matrix, distortion)
        self.assertFalse(observed['ready'])
        self.assertIsNone(observed['lateral_m'])
        self.assertIn('水平法线', observed['reason'])

    def test_stationary_photo_reopens_camera_and_is_immutable(self):
        self._exercise_photos('success')

    def test_cancel_and_timeout_discard_late_camera_results(self):
        for outcome in ('cancel', 'timeout'):
            with self.subTest(outcome=outcome):
                self._exercise_photos(outcome)

    def _exercise_photos(self, outcome):
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        marker = (cv2.aruco.generateImageMarker(dictionary, 1, 160)
                  if hasattr(cv2.aruco, 'generateImageMarker') else cv2.aruco.drawMarker(dictionary, 1, 160))
        image = np.full((480, 640), 255, dtype=np.uint8)
        image[160:320, 240:400] = marker
        with TemporaryDirectory() as folder:
            video = str(Path(folder) / 'rear.avi')
            writer = cv2.VideoWriter(video, cv2.VideoWriter_fourcc(*'MJPG'), 10., (640, 480))
            self.assertTrue(writer.isOpened())
            for _ in range(3):
                writer.write(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
            writer.release()
            real_capture = cv2.VideoCapture
            read_started, release_read = threading.Event(), threading.Event()
            events = []

            class Capture:
                def __init__(self, index):
                    self.index = index
                    self.capture = real_capture(video)
                    self.first = True
                    events.append(('open', index))
                def isOpened(self): return self.capture.isOpened()
                def set(self, key, value): return True
                def get(self, key): return cv2.VideoWriter_fourcc(*'MJPG')
                def release(self):
                    events.append(('release', self.index))
                    self.capture.release()
                def read(self):
                    warmup = self.index == 2 and self.first
                    if warmup:
                        self.first = False
                        read_started.set()
                        if outcome == 'success':
                            time.sleep(.607)
                        else:
                            release_read.wait(4)
                    else:
                        time.sleep(.01)
                    ok, frame = self.capture.read()
                    if not ok:
                        self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        ok, frame = self.capture.read()
                    if warmup and ok:
                        frame[:] = 255
                    return ok, frame

            def open_capture(*args):
                return Capture(sum(event[0] == 'open' for event in events) + 1)

            def wait_for(predicate, timeout=3):
                end = time.monotonic() + timeout
                while time.monotonic() < end:
                    value = predicate()
                    if value:
                        return value
                    time.sleep(.01)
                self.fail('等待相机测试结果超时')

            camera = DockCamera('/dev/video0', target_id=1, tag_size_m=.08,
                                mount_yaw_deg=0, mount_pitch_deg=0, mount_roll_deg=0,
                                camera_height_m=.14, verified=True)
            with patch.object(cv2, 'VideoCapture', side_effect=open_capture), \
                 patch('v12_dock_camera.PHOTO_TIMEOUT_S', .2 if outcome == 'timeout' else 3.):
                try:
                    camera.start()
                    wait_for(camera.frame)
                    self.assertFalse(camera.snapshot()['ready'])
                    first = camera.request_photo()
                    self.assertEqual(first, 1)
                    self.assertTrue(read_started.wait(2))
                    self.assertIsNone(camera.photo_result(first))
                    if outcome == 'cancel':
                        camera.cancel_photo(first)
                    elif outcome == 'timeout':
                        time.sleep(.22)
                        self.assertIn('超时', camera.photo_result(first)['reason'])
                    if outcome != 'success':
                        second = camera.request_photo()
                        self.assertEqual(second, 2)
                        release_read.set()
                        result = wait_for(lambda: camera.photo_result(second))
                        self.assertIsNone(camera.photo_result(first))
                        self.assertIsNone(camera.reference_frame(first))
                    else:
                        second = first
                        result = wait_for(lambda: camera.photo_result(first))
                    self.assertEqual(result['photo_id'], second)
                    self.assertTrue(result['ready'], result['reason'])
                    self.assertEqual(result['tag_id'], 1)
                    reference = camera.reference_frame(second)
                    self.assertIsNotNone(reference)
                    self.assertEqual(reference[:2], b'\xff\xd8')
                    time.sleep(.05)
                    self.assertEqual(reference, camera.reference_frame(second))
                    self.assertLess(events.index(('release', 1)), events.index(('open', 2)))
                    self.assertFalse(camera.snapshot()['ready'])
                    camera.cancel_photo(second)
                    self.assertIsNone(camera.reference_frame(second))
                    self.assertIsNone(camera.photo_result(second))
                finally:
                    release_read.set()
                    camera.close()


if __name__ == '__main__':
    unittest.main()
