"""Real ROS adapter callbacks and session threads against memory-only ROS peers."""
from copy import deepcopy
import math
from pathlib import Path
from tempfile import TemporaryDirectory
from threading import Event, Lock, Thread
import time
from types import SimpleNamespace as Obj
import unittest
from unittest.mock import patch

from test_v11_ros import NS, OFFSET, Stamp, exchange
from v11_ros import OdomHistory, RosLocalizer, _AmclCandidates


def transform_message():
    return Obj(header=Obj(stamp=None, frame_id=''), child_frame_id='',
               transform=Obj(translation=Obj(x=0., y=0., z=0.), rotation=Obj(x=0., y=0., z=0., w=1.)))


def odometry_message():
    return Obj(header=Obj(stamp=None, frame_id=''), child_frame_id='',
               pose=Obj(pose=Obj(position=Obj(x=0., y=0., z=0.), orientation=None)),
               twist=Obj(twist=Obj(linear=Obj(x=0.), angular=Obj(z=0.))))


def mapped_pose(odom, alignment, stamp, variance=.0025):
    c, s = math.cos(alignment['yaw']), math.sin(alignment['yaw'])
    covariance = [0.]*36
    covariance[0] = covariance[7] = variance
    covariance[35] = math.radians(2.)**2
    return dict(x=alignment['x']+c*odom['x']-s*odom['y'],
                y=alignment['y']+s*odom['x']+c*odom['y'],
                yaw=math.atan2(math.sin(alignment['yaw']+odom['yaw']),
                               math.cos(alignment['yaw']+odom['yaw'])),
                stamp_ns=stamp, covariance=covariance)


class Handle:
    def __init__(self, topic='', callback=None):
        self.topic, self.callback, self.unregistered = topic, callback, False

    def unregister(self):
        self.unregistered = True


class Child:
    def __init__(self):
        self.code = None
        self.signals = []

    def poll(self):
        return self.code

    def send_signal(self, value):
        self.signals.append(value)
        self.code = 0

    def wait(self, timeout):
        return self.code

    def kill(self):
        self.code = -9


class RosPeer:
    def __init__(self):
        peer = self

        class Time:
            def __new__(cls, seconds, nanoseconds):
                return Stamp(seconds*NS+nanoseconds)

            @staticmethod
            def now():
                return Stamp(OFFSET+time.monotonic_ns())

        self.Time = Time
        self.core = Obj(is_initialized=lambda: True)
        self.handles, self.params, self.children, self.scans, self.requests = [], {}, [], [], []
        self.localizer = None
        self.auto_pose = self.emit_scans = True
        self.alignment = dict(x=1., y=2., yaw=.3)
        self.variance = .0025
        self.last_message = None
        self.pose_lock = Lock()

        class Publisher(Handle):
            def publish(self, message):
                if self.topic.endswith('/scan'):
                    peer.scans.append((time.monotonic(), message))
                    with peer.pose_lock:
                        if peer.auto_pose:
                            stamp = message.header.stamp.to_nsec()
                            odom = peer.localizer.odom.at(stamp, time.monotonic())
                            pose = mapped_pose(odom, peer.alignment, stamp, peer.variance)
                            q = Obj(x=0., y=0., z=math.sin(pose['yaw']/2), w=math.cos(pose['yaw']/2))
                            raw = Obj(header=Obj(stamp=Stamp(stamp), frame_id=peer.localizer.frames['map']),
                                      pose=Obj(pose=Obj(position=Obj(x=pose['x'], y=pose['y']), orientation=q),
                                               covariance=pose['covariance']))
                            peer.last_message = raw
                            for handle in tuple(peer.handles):
                                if not handle.unregistered and handle.topic.endswith('/amcl_pose'):
                                    handle.callback(raw)

        self.publisher_type = Publisher

    def Publisher(self, topic, message_type, **kwargs):
        handle = self.publisher_type(topic)
        self.handles.append(handle)
        return handle

    def Subscriber(self, topic, message_type, callback, **kwargs):
        handle = Handle(topic, callback)
        self.handles.append(handle)
        return handle

    def ServiceProxy(self, name, service_type):
        return lambda: self.requests.append((name, time.monotonic()))

    def wait_for_service(self, name, timeout):
        pass

    def set_param(self, name, value):
        self.params[name] = value

    def get_param(self, name, default):
        return self.params.get(name, default)

    def delete_param(self, name):
        for key in tuple(self.params):
            if key.startswith(name+'/'):
                del self.params[key]

    def is_shutdown(self):
        return False

    def child(self, *args, **kwargs):
        child = Child()
        self.children.append(child)
        return child


class CandidateTests(unittest.TestCase):
    def test_three_distinct_transforms_converge_despite_real_translation_and_yaw_wrap(self):
        gate = _AmclCandidates()
        alignment = dict(x=1., y=2., yaw=math.radians(179.))
        for index in range(3):
            odom = dict(x=index*.4, y=.2, yaw=index*.2)
            pose = mapped_pose(odom, alignment, index+1)
            result = gate.add(pose, odom, index*.2, index*.2+.1)
            if index < 2:
                self.assertIsNone(result)
        for key in alignment:
            self.assertAlmostEqual(result[key], alignment[key])
        self.assertIsNone(gate.add(pose, odom, .5, .6))
        self.assertEqual(len(gate.rows), 3)

    def test_uncertainty_transform_span_and_source_gap_require_new_candidates(self):
        odom = dict(x=0., y=0., yaw=0.)
        for failure in ('variance', 'translation', 'yaw', 'gap'):
            with self.subTest(failure=failure):
                gate = _AmclCandidates()
                result = None
                for index in range(3):
                    alignment = dict(x=.06 if failure == 'translation' and index == 2 else 0., y=0.,
                                     yaw=math.radians(4.) if failure == 'yaw' and index == 2 else 0.)
                    pose = mapped_pose(odom, alignment, index+1,
                                       .011 if failure == 'variance' and index == 2 else .0025)
                    now = 1. if failure == 'gap' and index == 2 else index*.2+.1
                    result = gate.add(pose, odom, now-.1, now)
                self.assertIsNone(result)

    def test_amcl_history_age_configuration_does_not_change_obstacle_default(self):
        source = exchange(1, 100, 10.021)
        source.received_at = 10.32
        self.assertIsNone(OdomHistory(OFFSET).add(source))
        odom = OdomHistory(OFFSET, max_age_s=.5)
        pose = odom.add(source)
        self.assertEqual(odom.at(pose['stamp_ns'], 10.499), pose)
        with self.assertRaises(ValueError):
            odom.at(pose['stamp_ns'], 10.5)


class TrackingTests(unittest.TestCase):
    def setUp(self):
        folder = TemporaryDirectory()
        self.addCleanup(folder.cleanup)
        path = Path(folder.name)/'map.yaml'
        path.write_text('image: map.pgm\n', encoding='utf8')
        self.peer = RosPeer()
        broadcaster = lambda: Obj(sendTransform=lambda value: None, pub_tf=Handle())
        modules = {'rospy': self.peer,
            'tf2_ros': Obj(TransformBroadcaster=broadcaster, StaticTransformBroadcaster=broadcaster),
            'geometry_msgs.msg': Obj(PoseWithCovarianceStamped=object, TransformStamped=transform_message),
            'nav_msgs.msg': Obj(Odometry=odometry_message), 'sensor_msgs.msg': Obj(LaserScan=object),
            'std_srvs.srv': Obj(Empty=object)}
        self.module_patch = patch.dict('sys.modules', modules)
        self.module_patch.start()
        self.addCleanup(self.module_patch.stop)
        self.process_patch = patch('v11_ros.subprocess.Popen', side_effect=self.peer.child)
        self.process_patch.start()
        self.addCleanup(self.process_patch.stop)
        self.path = path
        self.closed = Event()
        self.sequence = 0
        self.boot, self.epoch = 12, 1
        self.valid = True

    def launch(self, *, continuous=True, timeout=.9):
        localizer = RosLocalizer(self.path, '/raw_scan', continuous=continuous, timeout_s=timeout)
        self.peer.localizer = self.localizer = localizer
        self.addCleanup(localizer.close)
        self.feed = Thread(target=self.feed_sources)
        self.feed.start()
        self.addCleanup(self.stop_feeding)
        return localizer

    def stop_feeding(self):
        self.closed.set()
        self.feed.join(1.)

    def feed_sources(self):
        while not self.closed.wait(.05):
            now = time.monotonic()
            self.sequence += 1
            tick = round((now-.04)*1000) & 0xffffffff
            sample = exchange(self.sequence, tick, now-.019, epoch=self.epoch, valid=self.valid)
            sample.status.boot = self.boot
            self.localizer.publish(sample)
            if self.peer.emit_scans:
                scan = Obj(header=Obj(frame_id='raw_laser', seq=self.sequence,
                                      stamp=Stamp(self.localizer.odom.ros_offset_ns+round((now-.055)*NS))),
                           angle_min=-math.pi/2, angle_max=math.pi/2, angle_increment=math.pi/360,
                           range_min=.1, range_max=50., time_increment=0., ranges=[2.]*361, intensities=[])
                for handle in tuple(self.peer.handles):
                    if handle.topic == '/raw_scan' and not handle.unregistered:
                        handle.callback(scan)

    def wait_until(self, predicate, timeout=1.5):
        deadline = time.monotonic()+timeout
        while not predicate():
            if time.monotonic() >= deadline:
                self.fail('Timed out waiting for real localization thread')
            time.sleep(.01)

    def start_tracking(self):
        localizer = self.launch()
        result = localizer.localize(dict(x=1., y=2., yaw=.3))
        self.assertEqual(result[0]['stamp_ns'], result[1]['stamp_ns'])
        return localizer, result

    def test_continuous_real_callbacks_parameters_rate_and_fresh_snapshot(self):
        localizer, result = self.start_tracking()
        snapshot = localizer.tracking_snapshot()
        self.assertEqual(snapshot['status'], 'tracking')
        self.assertTrue(snapshot['enabled'] and snapshot['fresh'])
        self.assertEqual(snapshot['odom_pose']['boot'], 12)
        self.assertIn('source_monotonic_s', snapshot['odom_pose'])
        self.assertLess(snapshot['source_monotonic_s'], snapshot['received_monotonic_s'])
        self.assertAlmostEqual(snapshot['alignment']['yaw'], .3)
        self.assertAlmostEqual(snapshot['sigma_x_m'], .05)
        self.assertTrue(all(child.poll() is None for child in self.peer.children))
        self.assertGreaterEqual(len(self.peer.scans), 3)
        times = [t for t, _ in self.peer.scans]
        self.assertTrue(all(b-a >= .199 for a, b in zip(times, times[1:])))
        self.assertEqual(len(self.peer.requests), len(self.peer.scans))
        params = self.peer.params
        self.assertIn(60, [value for key, value in params.items() if key.endswith('/laser_max_beams')])
        self.assertIn(200, [value for key, value in params.items() if key.endswith('/min_particles')])
        amcl = next(value for key, value in params.items() if key.endswith('/amcl'))
        self.assertFalse(amcl['tf_broadcast'])
        self.assertEqual(amcl['resample_interval'], 1)

    def test_duplicate_and_out_of_order_poses_do_not_renew_and_recovery_reconverges(self):
        localizer, _ = self.start_tracking()
        with self.peer.pose_lock:
            self.peer.emit_scans = self.peer.auto_pose = False
            message = deepcopy(self.peer.last_message)
        source = (message.header.stamp.to_nsec()-localizer.odom.ros_offset_ns)/NS
        # The last normal callback may still await consumption when scans stop.
        self.wait_until(lambda: localizer.tracking_snapshot()['source_monotonic_s'] == source)
        latest = localizer.tracking_snapshot()
        callbacks = [h.callback for h in self.peer.handles if h.topic.endswith('/amcl_pose')]
        for _ in range(12):
            for callback in callbacks:
                callback(message)
                older = deepcopy(message)
                older.header.stamp = Stamp(message.header.stamp.to_nsec()-1)
                callback(older)
            time.sleep(.05)
        stale = localizer.tracking_snapshot()
        self.assertFalse(stale['fresh'])
        self.assertEqual(stale['status'], 'odom_fallback')
        self.assertEqual(stale['source_monotonic_s'], source)
        self.assertEqual(stale['alignment'], latest['alignment'])
        self.peer.auto_pose = self.peer.emit_scans = True
        self.wait_until(lambda: localizer.tracking_snapshot()['fresh'])
        self.assertGreater(localizer.tracking_snapshot()['source_monotonic_s'], source)

    def test_stop_is_nonblocking_old_callbacks_cannot_overwrite_restarted_session(self):
        localizer, _ = self.start_tracking()
        old = localizer.tracking_snapshot()
        old_callback = next(h.callback for h in self.peer.handles if h.topic.endswith('/amcl_pose'))
        message = deepcopy(self.peer.last_message)
        message.pose.pose.position.x = 999.
        before = time.monotonic()
        localizer.stop_tracking()
        self.assertLess(time.monotonic()-before, .05)
        self.assertFalse(localizer.tracking_snapshot()['fresh'])
        localizer.localize(dict(x=1., y=2., yaw=.3))
        current = localizer.tracking_snapshot()
        self.assertNotEqual(current['session_id'], old['session_id'])
        old_callback(message)
        time.sleep(.04)
        self.assertNotEqual(localizer.tracking_snapshot()['map_pose']['x'], 999.)

    def test_child_exit_falls_back_retaining_last_pair_and_cleans_children(self):
        localizer, _ = self.start_tracking()
        original = localizer.tracking_snapshot()['alignment']
        self.peer.children[0].code = 1
        self.wait_until(lambda: not localizer.tracking_snapshot()['fresh'])
        self.assertEqual(localizer.tracking_snapshot()['alignment'], original)
        self.assertIn('exited', localizer.tracking_snapshot()['reason'])
        self.wait_until(lambda: all(child.poll() is not None for child in self.peer.children))

    def test_initialization_cancel_and_timeout_release_sessions_and_callbacks(self):
        for cancelled in (True, False):
            with self.subTest(cancelled=cancelled):
                localizer = self.launch(timeout=.22)
                self.peer.auto_pose = False
                cancel, errors = Event(), []

                def localize():
                    try:
                        localizer.localize(dict(x=1., y=2., yaw=.3), cancelled=cancel.is_set)
                    except Exception as exc:
                        errors.append(exc)

                worker = Thread(target=localize)
                worker.start()
                self.wait_until(lambda: len(self.peer.children) == 2)
                if cancelled:
                    cancel.set()
                worker.join(.5)
                self.assertFalse(worker.is_alive())
                self.assertEqual(len(errors), 1)
                self.assertIsInstance(errors[0], RuntimeError if cancelled else TimeoutError)
                localizer.close()
                self.assertTrue(all(child.poll() is not None for child in self.peer.children))
                self.assertTrue(all(h.unregistered for h in self.peer.handles))
                self.assertEqual(self.peer.params, {})
                self.stop_feeding()
                self.closed.clear()
                self.peer.children.clear()
                self.peer.handles.clear()

    def test_boot_change_stops_adoption_even_after_new_samples_arrive(self):
        self.assert_source_cut('boot')

    def test_epoch_change_stops_adoption_even_after_new_samples_arrive(self):
        self.assert_source_cut('epoch')

    def test_real_odom_gap_stops_adoption_even_after_new_samples_arrive(self):
        self.assert_source_cut('gap')

    def test_invalid_odom_stops_adoption_even_after_new_samples_arrive(self):
        self.assert_source_cut('invalid')

    def assert_source_cut(self, cause):
        localizer, _ = self.start_tracking()
        previous = localizer.tracking_snapshot()['source_monotonic_s']
        if cause == 'boot':
            self.boot += 1
        elif cause == 'epoch':
            self.epoch += 1
        elif cause == 'gap':
            self.sequence += 4
        else:
            self.valid = False
        self.wait_until(lambda: not localizer.tracking_snapshot()['enabled'])
        time.sleep(.25)
        current = localizer.tracking_snapshot()
        self.assertFalse(current['fresh'])
        self.assertEqual(current['source_monotonic_s'], previous)
        self.assertEqual(current['status'], 'odom_fallback')

    def test_uncertain_results_retain_last_transform_until_three_good_scans_return(self):
        localizer, _ = self.start_tracking()
        original = localizer.tracking_snapshot()['alignment']
        self.peer.variance = .011
        self.wait_until(lambda: not localizer.tracking_snapshot()['fresh'])
        self.assertEqual(localizer.tracking_snapshot()['alignment'], original)
        self.assertIn('uncertainty', localizer.tracking_snapshot()['reason'])
        self.peer.variance = .0025
        before = len(self.peer.scans)
        self.wait_until(lambda: localizer.tracking_snapshot()['fresh'])
        self.assertGreaterEqual(len(self.peer.scans)-before, 3)

    def test_scan_without_future_odom_bracket_is_not_forwarded(self):
        localizer, _ = self.start_tracking()
        self.peer.emit_scans = False
        self.stop_feeding()
        count = len(self.peer.scans)
        unsupported = deepcopy(self.peer.scans[-1][1])
        unsupported.header.stamp = self.peer.Time.now()
        callback = next(h.callback for h in self.peer.handles if h.topic == '/raw_scan')
        callback(unsupported)
        time.sleep(.12)
        self.assertEqual(len(self.peer.scans), count)

    def test_default_single_accepts_first_legal_pair_and_stops_native_processes(self):
        localizer = self.launch(continuous=False)
        self.peer.variance = 1.
        localizer.localize(dict(x=1., y=2., yaw=.3))
        self.assertEqual(len(self.peer.scans), 1)
        self.assertEqual(self.peer.requests, [])
        self.assertTrue(all(child.poll() is not None for child in self.peer.children))
        self.assertFalse(localizer.tracking_snapshot()['enabled'])


if __name__ == '__main__':
    unittest.main()
