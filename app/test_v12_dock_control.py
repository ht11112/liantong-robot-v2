import math
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
import time
import unittest
from dataclasses import replace
from http.server import ThreadingHTTPServer
from threading import Event, Thread
from unittest.mock import Mock, patch
from urllib.error import HTTPError
from urllib.request import urlopen

import test_v11_map_runtime as base
from mcu_v11 import EXECUTING, RX_FAULT, STOP_CONFIRMED, Rejected
from v12_dock_control import dock_segment, next_action
from v11_map_runtime import handler_for

try:
    import cv2
    import numpy as np
    HAVE_ARUCO = hasattr(cv2, 'aruco') and hasattr(cv2.aruco, 'DICT_APRILTAG_36h11')
except ImportError:
    HAVE_ARUCO = False


class Camera:
    def __init__(self):
        self.sequence = 0
        self.results = {}
        self.cancelled = []
        self.value = dict(fresh=False, ready=False, reason='预览未连接')

    def snapshot(self):
        return dict(self.value)

    def request_photo(self):
        self.sequence += 1
        return self.sequence

    def photo_result(self, photo_id):
        return self.results.get(photo_id)

    def cancel_photo(self, photo_id):
        self.cancelled.append(photo_id)
        self.results.pop(photo_id, None)

    def reference_frame(self, photo_id):
        return b'jpeg' if photo_id in self.results else None

    def finish(self, photo_id=None, **changes):
        photo_id = self.sequence if photo_id is None else photo_id
        result = dict(photo_id=photo_id, frame_seq=photo_id, ready=True, reason=None,
                      distance_m=.5, lateral_m=0., yaw_deg=0., tag_id=1)
        result.update(changes)
        self.results[photo_id] = result


class DockTest(unittest.TestCase):
    setUp = base.NavigationTest.setUp
    observe = base.NavigationTest.observe

    def enable(self):
        self.nav.mode = 'drive'
        self.nav.camera = Camera()
        self.observe(flags=self.sample.flags | STOP_CONFIRMED)

    def stop_proof(self, verified=True, settle=True):
        master = Mock()
        master.stop.return_value = dict(stop_verified=verified)
        self.nav._stop(master)
        self.nav.stop_requested = False
        self.nav.work = None
        if settle and self.nav.dock['phase'] == 'settling':
            self.settle()
        return master

    def settle(self, samples=5):
        for _ in range(samples):
            self.now += .05
            tick = self.sample.measurement_tick_ms + 50
            self.observe(sample_sequence=self.sample.sample_sequence + 1,
                         measurement_tick_ms=tick, publication_tick_ms=tick + 5)
            self.nav._dock_cycle()

    def begin(self, single=False):
        self.enable()
        if single:
            self.nav.dock_single_step_test = True
            self.nav.request_dock_step('move', -.03)
        else:
            self.nav.request_dock()
        self.assertTrue(self.nav.stop_requested)
        self.assertEqual(self.nav.dock['phase'], 'stopping')
        self.stop_proof()
        self.assertEqual(self.nav.dock['phase'], 'capturing')

    def photo(self, **changes):
        self.nav.camera.finish(**changes)
        self.nav._dock_cycle()
        return self.nav.pending

    def test_segment_wire_limits_and_geometry(self):
        self.assertEqual(struct.unpack('<If', dock_segment('move', -.05).segments[0])[0], 3)
        self.assertEqual(struct.unpack('<If', dock_segment('turn', math.radians(5)).segments[0])[0], 4)
        for kind, value in [('move', -.005), ('move', .01), ('move', -.051),
                            ('turn', math.radians(1)), ('turn', math.radians(5.1))]:
            with self.assertRaises(ValueError):
                dock_segment(kind, value)
        self.assertGreater(next_action(dict(distance_m=.5, lateral_m=.04, yaw_deg=0.))[1], 0.)
        self.assertLess(next_action(dict(distance_m=.5, lateral_m=-.04, yaw_deg=0.))[1], 0.)
        self.assertEqual(next_action(dict(distance_m=.5, lateral_m=0., yaw_deg=0.)), ('move', -.05))
        self.assertIsNone(next_action(dict(distance_m=.20, lateral_m=.01, yaw_deg=2.)))

    def test_start_stops_and_photographs_without_preview_readiness(self):
        self.begin()
        self.assertFalse(self.nav.camera.snapshot()['ready'])
        self.assertIsNone(self.nav.dock['photo_id'])
        self.assertIsNone(self.nav.pending)
        self.photo()
        self.assertEqual(self.nav.dock['photo_id'], 1)
        self.assertEqual(self.nav.dock['phase'], 'uploading')
        self.assertIsNotNone(self.nav.pending)
        with self.assertRaises(ValueError):
            self.nav.align(dict(x=0., y=0., yaw=0.), 'manual')

    def test_settling_requires_five_distinct_identical_samples_after_last_change(self):
        self.enable()
        self.nav.request_dock()
        self.stop_proof(settle=False)
        self.assertEqual(self.nav.dock['phase'], 'settling')
        self.settle(samples=3)
        for _ in range(5):
            self.nav._dock_cycle()
        self.assertEqual(self.nav.dock_capture['stable_samples'], 3)
        self.assertEqual(self.nav.camera.sequence, 0)
        self.now += .05
        tick = self.sample.measurement_tick_ms + 50
        self.observe(sample_sequence=self.sample.sample_sequence + 1, measurement_tick_ms=tick,
                     publication_tick_ms=tick + 5, position_left=4, position_right=1)
        self.nav._dock_cycle()
        self.assertEqual(self.nav.dock_capture['stable_samples'], 1)
        self.settle(samples=3)
        self.assertEqual(self.nav.camera.sequence, 0)
        self.settle(samples=1)
        self.assertEqual(self.nav.dock['phase'], 'capturing')
        self.assertEqual(self.nav.dock_capture['positions'], (4, 1))
        self.assertEqual(self.nav.camera.sequence, 1)

    def test_settling_and_capture_share_original_three_second_deadline(self):
        self.enable()
        self.nav.request_dock()
        self.stop_proof(settle=False)
        deadline = self.nav.dock_capture['deadline']
        self.settle()
        self.assertEqual(self.nav.dock_capture['deadline'], deadline)
        self.now = deadline + .01
        self.observe(sample_sequence=self.sample.sample_sequence + 1,
                     measurement_tick_ms=self.sample.measurement_tick_ms + 50,
                     publication_tick_ms=self.sample.measurement_tick_ms + 55)
        self.nav._dock_cycle()
        self.assertEqual(self.nav.camera.cancelled, [1])
        self.assertEqual(self.nav.dock_stop_outcome, 'failed')
        self.assertIsNone(self.nav.pending)

    def test_settling_timeout_or_fault_never_requests_photo(self):
        for failure in ('timeout', 'stale', 'drive', 'rx', 'manual', 'boot'):
            with self.subTest(failure=failure):
                self.setUp()
                self.enable()
                self.nav.request_dock()
                self.stop_proof(settle=False)
                if failure == 'timeout':
                    self.now += 3.01
                    self.observe(sample_sequence=2, measurement_tick_ms=145, publication_tick_ms=150)
                elif failure == 'stale':
                    self.now += .6
                elif failure == 'drive':
                    self.observe(first_drive_error=72)
                elif failure == 'rx':
                    self.observe(flags=self.sample.flags | RX_FAULT)
                elif failure == 'manual':
                    self.nav.request_stop()
                else:
                    self.observe(boot=43)
                self.nav._dock_cycle()
                self.assertEqual(self.nav.camera.sequence, 0)
                self.assertIsNone(self.nav.pending)
                self.assertTrue(self.nav.stop_requested)
                self.assertEqual(self.nav.dock_stop_outcome, 'cancelled' if failure == 'manual' else 'failed')

    def test_change_after_settling_still_cancels_photo(self):
        self.begin()
        self.observe(position_left=1, sample_sequence=self.sample.sample_sequence + 1,
                     measurement_tick_ms=self.sample.measurement_tick_ms + 50,
                     publication_tick_ms=self.sample.measurement_tick_ms + 55)
        self.nav._dock_cycle()
        self.assertEqual(self.nav.camera.cancelled, [1])
        self.assertEqual(self.nav.dock_stop_outcome, 'failed')
        self.assertIsNone(self.nav.pending)

    def test_photo_timeout_cancels_and_late_photo_cannot_start_next_task(self):
        self.begin()
        self.now += 3.01
        self.observe(sample_sequence=2, measurement_tick_ms=145, publication_tick_ms=150)
        self.nav._dock_cycle()
        self.assertEqual(self.nav.camera.cancelled, [1])
        self.assertTrue(self.nav.stop_requested)
        self.assertEqual(self.nav.dock_stop_outcome, 'failed')
        self.nav.camera.finish(photo_id=1)
        self.stop_proof()
        self.nav.request_dock()
        self.stop_proof()
        self.assertEqual(self.nav.dock_capture['id'], 2)
        self.nav._dock_cycle()
        self.assertEqual(self.nav.dock['phase'], 'capturing')
        self.assertIsNone(self.nav.pending)
        self.nav.camera.finish(photo_id=2)
        self.nav._dock_cycle()
        self.assertEqual(self.nav.dock['photo_id'], 2)

    def test_failed_photo_and_mismatched_number_stop(self):
        for changes in (dict(ready=False, reason='未识别目标'), dict(photo_id=99)):
            with self.subTest(changes=changes):
                self.setUp()
                self.begin()
                self.nav.camera.finish()
                self.nav.camera.results[1].update(changes)
                self.nav._dock_cycle()
                self.assertTrue(self.nav.stop_requested)
                self.assertEqual(self.nav.dock_stop_outcome, 'failed')
                self.assertIsNone(self.nav.pending)

    def test_capture_feedback_loss_fault_and_manual_stop(self):
        for failure in ('stale', 'drive', 'rx', 'manual'):
            with self.subTest(failure=failure):
                self.setUp()
                self.begin()
                if failure == 'stale':
                    self.now += .6
                    self.nav._dock_cycle()
                elif failure == 'drive':
                    self.observe(first_drive_error=72)
                    self.nav._dock_cycle()
                elif failure == 'rx':
                    self.observe(flags=self.sample.flags | RX_FAULT)
                    self.nav._dock_cycle()
                else:
                    self.nav.request_stop()
                self.assertTrue(self.nav.stop_requested)
                self.assertEqual(self.nav.camera.cancelled, [1])
                self.assertEqual(self.nav.dock_stop_outcome, 'cancelled' if failure == 'manual' else 'failed')
                self.nav.camera.finish(photo_id=1)
                self.nav._dock_cycle()
                self.assertIsNone(self.nav.pending)

    def test_feedback_loss_during_stop_blocks_photo(self):
        self.enable()
        self.nav.request_dock()
        self.now += .6
        self.nav._dock_cycle()
        self.assertEqual(self.nav.dock_stop_outcome, 'failed')
        self.stop_proof()
        self.assertEqual(self.nav.dock['phase'], 'failed')
        self.assertEqual(self.nav.camera.sequence, 0)

    def test_start_rechecks_encoder_identity_and_cancel_after_upload(self):
        for failure in ('left', 'right', 'boot', 'cancel'):
            with self.subTest(failure=failure):
                self.setUp()
                self.begin()
                plan = self.photo()
                master = Mock()
                def upload(document, *, on_ack):
                    if failure == 'cancel':
                        self.nav.request_stop()
                    else:
                        self.observe(**({'position_left': 1} if failure == 'left' else
                                        {'position_right': 1} if failure == 'right' else {'boot': 43}))
                    return [self.nav.latest]
                master.upload.side_effect = upload
                if failure in ('cancel', 'boot'):
                    self.nav._upload_dock(master, plan)
                else:
                    with self.assertRaises(ValueError):
                        self.nav._upload_dock(master, plan)
                master.start.assert_not_called()

    def test_old_firmware_rejection_never_starts(self):
        self.begin()
        plan = self.photo()
        master = Mock()
        master.upload.side_effect = Rejected('MCU rejected APPEND')
        with self.assertRaises(Rejected):
            self.nav._upload_dock(master, plan)
        master.start.assert_not_called()

    def test_photo_age_during_upload_and_preview_loss_during_motion_do_not_cancel(self):
        self.begin()
        plan = self.photo()
        self.now += 1.
        self.observe(sample_sequence=2, measurement_tick_ms=145, publication_tick_ms=150)
        master = Mock()
        master.upload.return_value = [self.nav.latest]
        started = replace(self.nav.latest, status=replace(self.sample, task=7, state=4,
                          flags=self.sample.flags | EXECUTING, lease_remaining_ms=30000))
        master.start.return_value = started
        self.nav._upload_dock(master, plan)
        self.assertEqual(self.nav.dock['phase'], 'moving')
        self.nav.camera.value.update(fresh=False, ready=False, reason='标签丢失')
        self.nav._dock_cycle()
        self.assertFalse(self.nav.stop_requested)
        self.observe(task=7, state=4, flags=self.sample.flags | EXECUTING,
                     lease_remaining_ms=4000, sample_sequence=3, measurement_tick_ms=195,
                     publication_tick_ms=200)
        self.nav._renew_if_due(master, self.nav.latest)
        master.renew.assert_not_called()

    def test_motion_feedback_fault_stops_without_visual_dependency(self):
        for failure in ('stale', 'drive', 'rx', 'manual'):
            with self.subTest(failure=failure):
                self.setUp()
                self.begin()
                self.photo()
                self.nav.work = None
                self.nav.dock['phase'] = 'moving'
                self.observe(task=7, state=4, flags=self.sample.flags | EXECUTING, lease_remaining_ms=30000)
                self.nav.lease_task = (42, 1, 7)
                if failure == 'stale':
                    self.now += .6
                    self.nav._dock_cycle()
                elif failure == 'drive':
                    self.observe(first_drive_error=72)
                    self.nav._dock_cycle()
                elif failure == 'rx':
                    self.observe(flags=self.sample.flags | RX_FAULT)
                    self.nav._dock_cycle()
                else:
                    self.nav.request_stop()
                self.assertTrue(self.nav.stop_requested)
                self.assertIsNone(self.nav.lease_task)
                self.assertEqual(self.nav.dock_stop_outcome, 'cancelled' if failure == 'manual' else 'failed')

    def test_cancel_after_stop_proof_before_cleanup_does_not_leave_stopping(self):
        self.enable()
        self.nav.request_dock()
        master = Mock()
        master.stop.return_value = dict(stop_verified=True)
        self.nav._stop(master)
        self.assertTrue(self.nav.stop_requested)
        self.assertEqual(self.nav.dock['phase'], 'settling')
        self.nav.request_stop()
        self.assertEqual(self.nav.dock['phase'], 'cancelled')
        self.assertEqual(self.nav.camera.cancelled, [])
        self.nav.stop_requested = False
        self.nav._dock_cycle()
        self.assertEqual(self.nav.dock['phase'], 'cancelled')
        self.assertIsNone(self.nav.pending)

    def test_cancel_while_start_reply_is_pending_fences_worker(self):
        self.begin()
        plan = self.photo()
        master = Mock()
        master.stop_revision = 0
        master.request_stop.side_effect = lambda: setattr(master, 'stop_revision', master.stop_revision + 1)
        master.history_since.return_value = dict(cursor=0, exchanges=[], missed=0, connected=True, error=None)
        master.upload.return_value = [self.nav.latest]
        self.nav.feedback = master
        entered, release = Event(), Event()
        def start(task, *, revision):
            self.assertEqual(revision, 0)
            entered.set()
            self.assertTrue(release.wait(2))
            return self.nav.latest
        master.start.side_effect = start
        thread = Thread(target=self.nav._upload_dock, args=(master, plan))
        thread.start()
        self.assertTrue(entered.wait(2))
        self.nav.request_stop()
        release.set()
        thread.join(2)
        self.assertFalse(thread.is_alive())
        self.assertEqual(master.stop_revision, 1)
        self.assertIsNone(self.nav.lease_task)
        self.assertEqual(self.nav.dock_stop_outcome, 'cancelled')

    def test_final_completion_needs_new_stopped_photo(self):
        self.begin()
        self.photo()
        self.nav.work = None
        self.nav.dock['phase'] = 'moving'
        self.observe(state=8, flags=self.sample.flags | 8)
        self.nav._dock_cycle()
        self.assertTrue(self.nav.stop_requested)
        self.stop_proof()
        self.assertEqual(self.nav.dock['phase'], 'capturing')
        self.assertEqual(self.nav.camera.sequence, 2)
        self.nav._dock_cycle()
        self.assertNotEqual(self.nav.dock['phase'], 'complete')
        self.photo(distance_m=.2, lateral_m=.01, yaw_deg=2.)
        self.assertEqual(self.nav.dock['phase'], 'complete')
        self.assertEqual(self.nav.dock['photo_id'], 2)

    def test_single_step_stops_then_photographs_without_continuing(self):
        self.begin(single=True)
        plan = self.photo()
        self.assertAlmostEqual(struct.unpack('<If', plan.segments[0])[1], -.03)
        self.nav.pending = None
        self.nav.work = None
        self.nav.dock['phase'] = 'moving'
        self.observe(state=8, flags=self.sample.flags | 8)
        self.nav._dock_cycle()
        self.stop_proof()
        self.photo(distance_m=.5)
        self.assertEqual(self.nav.dock['phase'], 'step_complete')
        self.assertEqual(self.nav.dock['photo_id'], 2)
        self.assertIsNone(self.nav.pending)

    def test_reference_jpeg_requires_exact_current_photo_number(self):
        self.begin()
        self.photo()
        server = ThreadingHTTPServer(('127.0.0.1', 0), handler_for(self.nav))
        worker = Thread(target=server.serve_forever)
        worker.start()
        try:
            root = 'http://127.0.0.1:{}'.format(server.server_port)
            with urlopen(root + '/api/dock/reference.jpg?photo_id=1', timeout=2) as response:
                self.assertEqual(response.read(), b'jpeg')
                self.assertEqual(response.headers['Content-Type'], 'image/jpeg')
            for query in ('photo_id=2', 'photo_id=1&extra=1', 'photo_id=1&photo_id=1', 'photo_id=-1'):
                with self.assertRaises(HTTPError) as error:
                    urlopen(root + '/api/dock/reference.jpg?' + query, timeout=2)
                self.assertEqual(error.exception.code, 404)
            self.nav.dock['photo_id'] = None
            with self.assertRaises(HTTPError):
                urlopen(root + '/api/dock/reference.jpg?photo_id=1', timeout=2)
        finally:
            server.shutdown()
            server.server_close()
            worker.join(2)

    @unittest.skipUnless(HAVE_ARUCO, 'OpenCV AprilTag support unavailable')
    def test_real_camera_photo_drives_real_navigation_without_blocking(self):
        from v12_dock_camera import DockCamera
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
            actual_capture = cv2.VideoCapture
            class Capture:
                def __init__(self): self.capture = actual_capture(video)
                def isOpened(self): return self.capture.isOpened()
                def set(self, key, value): return True
                def get(self, key): return cv2.VideoWriter_fourcc(*'MJPG')
                def release(self): self.capture.release()
                def read(self):
                    time.sleep(.01)
                    ok, frame = self.capture.read()
                    if not ok:
                        self.capture.set(cv2.CAP_PROP_POS_FRAMES, 0)
                        return self.capture.read()
                    return ok, frame
            camera = DockCamera('/dev/video0', target_id=1, tag_size_m=.08,
                                mount_yaw_deg=0, mount_pitch_deg=0, mount_roll_deg=0,
                                camera_height_m=.14, verified=True)
            with patch.object(cv2, 'VideoCapture', side_effect=lambda *args: Capture()):
                self.enable()
                self.nav.camera = camera
                try:
                    camera.start()
                    self.nav.request_dock()
                    self.stop_proof()
                    deadline = time.monotonic() + 2.
                    while self.nav.dock['phase'] == 'capturing' and time.monotonic() < deadline:
                        self.nav._dock_cycle()
                        time.sleep(.01)
                    self.assertEqual(self.nav.dock['phase'], 'uploading')
                    self.assertFalse(camera.snapshot()['ready'])
                    photo_id = self.nav.dock['photo_id']
                    self.assertEqual(camera.photo_result(photo_id)['tag_id'], 1)
                    self.assertEqual(camera.reference_frame(photo_id)[:2], b'\xff\xd8')
                finally:
                    camera.close()

    def test_stop_error_fails_dock_without_claiming_proof(self):
        self.enable()
        self.nav.request_dock()
        closed, master = Event(), Mock(failed=False)
        master.discover.return_value = self.nav.latest
        master.history_since.return_value = dict(cursor=0, missed=0, exchanges=[], connected=True, error=None)
        def failed_stop(**unused):
            closed.set()
            raise ValueError('stop result unavailable')
        master.stop.side_effect = failed_stop
        with patch('v11_map_runtime.FeedbackWorker', return_value=master):
            self.nav.run('/dev/offline', closed)
        self.assertEqual(self.nav.dock['phase'], 'failed')
        self.assertIn('停车未确认', self.nav.dock['message'])
        self.assertFalse(self.nav.stop_verified)


if __name__ == '__main__':
    unittest.main()
