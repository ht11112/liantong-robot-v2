"""Independent rear-camera preview and AprilTag observations for docking."""
import math
from pathlib import Path
import threading
import time


MAX_AGE_S = 0.5
PHOTO_TIMEOUT_S = 3.0


class DockCamera:
    def __init__(self, device, target_id=None, tag_size_m=None, mount_yaw_deg=None,
                 mount_pitch_deg=None, mount_roll_deg=None, camera_height_m=None,
                 verified=False):
        if not isinstance(device, str) or not device:
            raise ValueError('必须指定相机设备')
        if target_id is not None and (isinstance(target_id, bool) or not isinstance(target_id, int) or not 0 <= target_id < 587):
            raise ValueError('tag36h11 ID 必须为 0 至 586')
        for value, name, maximum in ((tag_size_m, '标签边长', 1.), (camera_height_m, '相机高度', 5.)):
            if value is not None and (not math.isfinite(value) or not 0 < value <= maximum):
                raise ValueError(name + '必须为正的有限米数')
        for value in (mount_yaw_deg, mount_pitch_deg, mount_roll_deg):
            if value is not None and (not math.isfinite(value) or abs(value) > 180):
                raise ValueError('相机安装角必须为有限角度（±180° 内）')
        self.device = device
        self.target_id, self.tag_size_m = target_id, tag_size_m
        self.mount = (mount_yaw_deg, mount_pitch_deg, mount_roll_deg)
        self.camera_height_m, self.verified = camera_height_m, verified
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.thread = None
        self.jpeg = None
        self.failed = False
        self.observation = dict(frame_seq=0, age_ms=None, fresh=False, ready=False,
                                reason='相机尚未启动', tag_id=None, distance_m=None,
                                lateral_m=None, yaw_deg=None, corners=[])
        self.stamp = None
        self.photo_id = 0
        self.photo = None

    def start(self):
        self.thread = threading.Thread(target=self._run, name='dock-camera', daemon=True)
        self.thread.start()

    def close(self):
        self.stop.set()
        with self.lock:
            self.photo = None
        if self.thread:
            self.thread.join(timeout=2)

    def snapshot(self):
        with self.lock:
            result = dict(self.observation)
            stamp = self.stamp
            failed = self.failed
        age = None if stamp is None else max(0., time.monotonic() - stamp)
        result['age_ms'] = None if age is None else age * 1000
        result['fresh'] = not failed and age is not None and age < MAX_AGE_S
        if not result['fresh']:
            result['ready'] = False
            if stamp is not None and not failed:
                result['reason'] = '画面已过期'
        result['ready'] = False
        if result['fresh']:
            result['reason'] = '参考预览，请以停稳后拍摄的照片为准'
        return result

    def frame(self):
        with self.lock:
            return self.jpeg

    def request_photo(self):
        with self.lock:
            if self.failed or self.stop.is_set():
                raise RuntimeError('后视相机不可用')
            if self.photo is not None and self.photo['status'] == 'pending':
                raise RuntimeError('拍照正在进行')
            self.photo_id += 1
            self.photo = dict(id=self.photo_id, status='pending',
                              deadline=time.monotonic() + PHOTO_TIMEOUT_S,
                              result=None, jpeg=None)
            return self.photo_id

    def photo_result(self, photo_id):
        with self.lock:
            photo = self.photo
            if photo is None or photo['id'] != photo_id:
                return None
            if photo['status'] == 'pending' and time.monotonic() >= photo['deadline']:
                photo['status'] = 'failed'
                photo['result'] = dict(photo_id=photo_id, ready=False, reason='拍照超时')
            return None if photo['result'] is None else dict(photo['result'])

    def cancel_photo(self, photo_id):
        with self.lock:
            if self.photo is not None and self.photo['id'] == photo_id:
                self.photo = None

    def reference_frame(self, photo_id):
        with self.lock:
            photo = self.photo
            return photo['jpeg'] if photo is not None and photo['id'] == photo_id else None

    def _open_camera(self, cv2):
        cap = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if not cap.isOpened():
                raise RuntimeError('无法打开后视相机')
            mode_ok = int(cap.get(cv2.CAP_PROP_FOURCC)) == cv2.VideoWriter_fourcc(*'MJPG')
            return cap, mode_ok
        except Exception:
            cap.release()
            raise

    def _run(self):
        cap = None
        try:
            import cv2
            import numpy as np
            import yaml
            calibration = yaml.safe_load((Path(__file__).parent / 'mf200_640x480.yaml').read_text())
            matrix = np.array(calibration['camera_matrix']['data'], dtype=float).reshape(3, 3)
            distortion = np.array(calibration['distortion_coefficients']['data'], dtype=float)
            has_tags = hasattr(cv2, 'aruco') and hasattr(cv2.aruco, 'DICT_APRILTAG_36h11')
            dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11) if has_tags else None
            detector = (cv2.aruco.ArucoDetector(dictionary) if has_tags and hasattr(cv2.aruco, 'ArucoDetector') else None)
            cap, mode_ok = self._open_camera(cv2)
            capture_id = None
            while not self.stop.is_set():
                with self.lock:
                    pending = self.photo
                    photo_id = pending['id'] if pending is not None and pending['status'] == 'pending' else None
                if photo_id is not None and photo_id != capture_id:
                    cap.release()
                    cap = None
                    cap, mode_ok = self._open_camera(cv2)
                    capture_id = photo_id
                    ok, _ = cap.read()
                    if not ok:
                        raise RuntimeError('后视相机预热读取失败')
                    with self.lock:
                        photo = self.photo
                        if photo is None or photo['id'] != photo_id or photo['status'] != 'pending':
                            continue
                        if time.monotonic() >= photo['deadline']:
                            photo['status'] = 'failed'
                            photo['result'] = dict(photo_id=photo_id, ready=False, reason='拍照超时')
                            continue
                read_start = time.monotonic()
                ok, image = cap.read()
                if not ok:
                    raise RuntimeError('后视相机读取失败')
                if image.shape[:2] != (480, 640):
                    raise RuntimeError('后视相机不是 640×480')
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                corners, ids, _ = ((detector.detectMarkers(gray) if detector else
                                   cv2.aruco.detectMarkers(gray, dictionary)) if has_tags else ([], None, []))
                result = self._observe(cv2, np, image, corners, ids, matrix, distortion)
                if not has_tags:
                    result.update(ready=False, reason='缺少 OpenCV aruco AprilTag 36h11 支持')
                elif not mode_ok:
                    result.update(ready=False, reason='相机未协商到 MJPEG，几何尚未验证')
                cv2.line(image, (320, 0), (320, 479), (0, 220, 255), 1)
                if result['distance_m'] is not None:
                    label = 'ID {}  D {:.3f}m  Y {:+.3f}m  yaw {:+.1f}deg'.format(
                        result['tag_id'], result['distance_m'], result['lateral_m'], result['yaw_deg'])
                    cv2.putText(image, label, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, .48, (0, 255, 255), 1)
                ok, encoded = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 80])
                if not ok:
                    raise RuntimeError('画面编码失败')
                jpeg = encoded.tobytes()
                with self.lock:
                    result['frame_seq'] = self.observation['frame_seq'] + 1
                    self.observation, self.jpeg, self.stamp = result, jpeg, read_start
                    photo = self.photo
                    if photo_id is not None and photo is not None and photo['id'] == photo_id and photo['status'] == 'pending':
                        if time.monotonic() >= photo['deadline']:
                            photo['status'] = 'failed'
                            photo['result'] = dict(photo_id=photo_id, ready=False, reason='拍照超时')
                        elif ids is not None or not has_tags or not mode_ok:
                            photo['status'] = 'complete'
                            photo['result'] = dict(result, photo_id=photo_id)
                            photo['jpeg'] = jpeg
        except Exception as exc:
            with self.lock:
                self.failed = True
                self.observation = dict(self.observation, ready=False, reason=str(exc) + '；请重启相机')
                if self.photo is not None and self.photo['status'] == 'pending':
                    self.photo['status'] = 'failed'
                    self.photo['result'] = dict(photo_id=self.photo['id'], ready=False, reason=str(exc))
        finally:
            if cap is not None:
                cap.release()

    def _observe(self, cv2, np, image, corners, ids, matrix, distortion):
        result = dict(frame_seq=0, age_ms=0., fresh=True, ready=False, reason='未识别目标标签',
                      tag_id=None, distance_m=None, lateral_m=None, yaw_deg=None, corners=[])
        if ids is None:
            return result
        cv2.aruco.drawDetectedMarkers(image, corners, ids)
        ids = ids.flatten().tolist()
        index = ids.index(self.target_id) if self.target_id in ids else (0 if self.target_id is None else None)
        if index is None:
            result['reason'] = '未识别指定 ID'
            return result
        result['tag_id'] = int(ids[index])
        result['corners'] = corners[index].reshape(4, 2).tolist()
        if (self.target_id is None or self.tag_size_m is None or self.tag_size_m <= 0
                or any(v is None for v in self.mount) or self.camera_height_m is None):
            result['reason'] = '等待目标 ID、标签边长和相机安装参数'
            return result
        half = self.tag_size_m / 2
        points = np.array([[-half, -half, 0], [half, -half, 0],
                           [half, half, 0], [-half, half, 0]], dtype=float)
        ok, rvec, tvec = cv2.solvePnP(points, corners[index].reshape(4, 2), matrix, distortion,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            result['reason'] = '标签位姿求解失败'
            return result
        rotation = cv2.Rodrigues(rvec)[0]
        tvec = tvec.reshape(3)
        yaw, pitch, roll = [math.radians(v) for v in self.mount]
        cz, sz, cy, sy, cx, sx = math.cos(yaw), math.sin(yaw), math.cos(pitch), math.sin(pitch), math.cos(roll), math.sin(roll)
        rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
        ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
        rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
        camera_to_body = rz @ ry @ rx @ np.array([[0, 0, -1], [1, 0, 0], [0, -1, 0]])
        outward = -camera_to_body @ rotation[:, 2]
        horizontal = math.hypot(outward[0], outward[1])
        if horizontal < 1e-6:
            result['reason'] = '标签水平法线无法确定'
            return result
        nx, ny = outward[0] / horizontal, outward[1] / horizontal
        tag_position = camera_to_body @ tvec + np.array([-.20, 0., 0.])
        result['distance_m'] = float(rotation[:, 2] @ tvec)
        result['lateral_m'] = float(-(-ny * tag_position[0] + nx * tag_position[1]))
        result['yaw_deg'] = math.degrees(-math.atan2(ny, nx))
        if result['distance_m'] <= 0:
            result.update(distance_m=None, lateral_m=None, yaw_deg=None, reason='标签位姿无效')
            return result
        result['reason'] = '视觉参数待现场验证' if not self.verified else '视觉观测有效'
        result['ready'] = bool(self.verified)
        return result
