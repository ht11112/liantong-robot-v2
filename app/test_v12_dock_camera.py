import math
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


if __name__ == '__main__':
    unittest.main()
