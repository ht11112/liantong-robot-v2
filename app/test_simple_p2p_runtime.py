"""Offline original-scan and body-clearance checks; no ROS/device imports."""
import math
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

from simple_p2p_runtime import DEFAULT_CONFIG, ScanGate, load_config, LiveStatusWriter, durable_json


SETTINGS = dict(stop_clearance_m=.2, body_front_m=.2, body_rear_m=.2, body_half_width_m=.25,
                laser_x_m=.13, laser_y_m=0., laser_yaw_rad=0., scan_timeout_s=.75)


def scan(stamp=100, obstacle=None):
    ranges = [10.] * 181
    if obstacle is not None:
        ranges[90] = obstacle
    return NS(header=NS(stamp=NS(secs=stamp, nsecs=0)), angle_min=-math.pi/2,
              angle_max=math.pi/2, angle_increment=math.pi/180,
              range_min=.1, range_max=50., ranges=ranges)


class ScanGateTests(unittest.TestCase):
    def test_clear_front_does_not_prove_rear_rotation_clearance(self):
        gate = ScanGate(SETTINGS)
        gate.accept(scan(), 100100000000, 10.)
        self.assertTrue(gate(10., 'drive')['ok'])
        self.assertFalse(gate(10., 'turn')['ok'])
        checked = ScanGate(SETTINGS, turn_area_clear=True)
        checked.accept(scan(), 100100000000, 10.)
        self.assertTrue(checked(10., 'turn')['ok'])

    def test_collision_clearance_uses_body_front_and_laser_offset(self):
        gate = ScanGate(SETTINGS, turn_area_clear=True)
        gate.accept(scan(obstacle=.25), 100100000000, 10.)
        self.assertFalse(gate(10., 'drive')['ok'])  # x=.38 -> body clearance .18
        gate.accept(scan(stamp=101, obstacle=.31), 101100000000, 11.)
        self.assertTrue(gate(11., 'drive')['ok'])   # x=.44 -> clearance .24

    def test_source_age_is_original_not_callback_or_duplicate_receipt(self):
        gate = ScanGate(SETTINGS)
        gate.accept(scan(), 100100000000, 10.)
        self.assertAlmostEqual(gate(10., 'drive')['source_monotonic_s'], 9.9)
        gate.accept(scan(), 100100000000, 10.6)
        self.assertAlmostEqual(gate(10.6, 'drive')['source_monotonic_s'], 9.9)
        self.assertFalse(gate(10.8, 'drive')['ok'])

    def test_bad_or_regressed_original_scan_never_clears_obstacles(self):
        gate = ScanGate(SETTINGS)
        gate.accept(scan(stamp=101), 101100000000, 10.)
        gate.accept(scan(stamp=100), 100100000000, 10.1)
        self.assertFalse(gate(10.1, 'drive')['ok'])
        broken = scan(stamp=102)
        broken.ranges[90] = float('nan')
        gate.accept(broken, 102100000000, 11.)
        self.assertFalse(gate(11., 'drive')['ok'])

    def test_absent_unknown_and_stale_scans_are_not_clear(self):
        gate = ScanGate(SETTINGS)
        self.assertFalse(gate(10., 'drive')['ok'])
        unknown = scan()
        unknown.ranges = [math.inf] * 181
        gate.accept(unknown, 100100000000, 10.)
        self.assertFalse(gate(10., 'drive')['ok'])
        gate.accept(scan(), 101000000000, 10.)
        self.assertFalse(gate(10., 'drive')['ok'])

    def test_turn_checks_corner_sweep_even_with_operator_rear_check(self):
        gate = ScanGate(SETTINGS, turn_area_clear=True)
        gate.accept(scan(obstacle=.32), 100100000000, 10.)
        self.assertTrue(gate(10., 'drive')['ok'])  # Static front gap is .25m.
        self.assertFalse(gate(10., 'turn')['ok'])  # Swept corner gap is only .13m.

    def test_sparse_front_sector_cannot_claim_obstacle_coverage(self):
        gate = ScanGate(SETTINGS, turn_area_clear=True)
        sparse = scan()
        sparse.ranges = [10.] * 11
        sparse.angle_increment = math.pi/10
        gate.accept(sparse, 100100000000, 10.)
        self.assertFalse(gate(10., 'drive')['ok'])

    def test_cli_default_configuration_exists_in_repo_and_standalone_package(self):
        controller, obstacle = load_config(DEFAULT_CONFIG)
        self.assertEqual(controller.max_linear_m_s, .03)
        self.assertEqual(obstacle['stop_clearance_m'], 0.)

    def test_zero_extra_margin_allows_outside_rectangle_but_rejects_inside_and_boundary(self):
        gate=ScanGate(dict(SETTINGS,stop_clearance_m=0.),turn_area_clear=True)
        for stamp,distance,allowed in ((100,.2501,True),(101,.2499,False),(102,.25,False)):
            sample=scan(stamp=stamp)
            sample.ranges[180]=distance  # +90deg, x=.13; body side is y=.25.
            gate.accept(sample,stamp*1000000000+100000000,float(stamp))
            self.assertEqual(gate(float(stamp),'drive')['ok'],allowed)

    def test_zero_extra_margin_uses_uninflated_turn_sweep(self):
        gate=ScanGate(dict(SETTINGS,stop_clearance_m=0.),turn_area_clear=True)
        gate.accept(scan(obstacle=.1),100100000000,10.)  # x=.23: outside body, inside turn radius.
        self.assertTrue(gate(10.,'drive')['ok'])
        self.assertFalse(gate(10.,'turn')['ok'])
        gate.accept(scan(stamp=101,obstacle=.21),101100000000,11.)  # x=.34 > radius .32016.
        self.assertTrue(gate(11.,'turn')['ok'])
        radius=math.hypot(SETTINGS['body_front_m'],SETTINGS['body_half_width_m'])
        gate.accept(scan(stamp=102,obstacle=radius-SETTINGS['laser_x_m']),102100000000,12.)
        self.assertFalse(gate(12.,'turn')['ok'])  # Exact uninflated sweep boundary remains blocked.

    def test_config_accepts_zero_margin_rejects_negative_and_keeps_geometry_source_positive(self):
        original=json.loads(DEFAULT_CONFIG.read_text(encoding='utf-8'))
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'config.json'
            path.write_text(json.dumps(original),encoding='utf-8')
            self.assertEqual(load_config(path)[1]['stop_clearance_m'],0.)
            for key,value in (('stop_clearance_m',-.001),('body_front_m',0.),
                              ('body_rear_m',0.),('body_half_width_m',0.),('scan_timeout_s',0.)):
                changed=json.loads(json.dumps(original))
                changed['obstacle'][key]=value
                path.write_text(json.dumps(changed),encoding='utf-8')
                with self.subTest(key=key),self.assertRaises(ValueError):load_config(path)

    def test_live_ram_submit_does_no_io_and_phase_does_not_refresh_source(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'live.json'
            writer=LiveStatusWriter(path,'boot',clock=lambda:20.)
            writer.submit(dict(event='odometry',phase='drive',event_monotonic_s=10.,
                source_monotonic_s=9.9,raw_packet=dict(position_left_raw=2,position_right_raw=3),
                pose=dict(x=.1,y=0.,yaw=0.)))
            writer.submit(dict(event='phase',phase='final_stop',event_monotonic_s=19.,
                source_monotonic_s=None,raw_packet=None,pose=None))
            self.assertFalse(path.exists())
            writer._write()
            import json
            status=json.loads(path.read_text())
            self.assertEqual(status['source_monotonic_s'],9.9)
            self.assertEqual(status['written_monotonic_s'],20.)
            self.assertEqual(status['event_monotonic_s'],19.)
            self.assertEqual(status['pose']['x'],.1)

    def test_live_writer_cleanup_failure_cannot_claim_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=LiveStatusWriter(Path(directory)/'missing'/'live.json','boot')
            writer.start()
            self.assertFalse(writer.close(dict(session_complete=False)))
            self.assertIsNotNone(writer.error)

    def test_raw_feedback_without_accumulated_pose_does_not_renew_pose_source(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=LiveStatusWriter(Path(directory)/'live.json','boot',clock=lambda:30.)
            writer.submit(dict(event='odometry',phase='drive',event_monotonic_s=10.,
                source_monotonic_s=9.9,raw_packet=dict(sample_sequence=1),
                measurement_monotonic_s=9.95,pose=dict(x=.4,y=0.,yaw=0.)))
            writer.submit(dict(event='odometry',phase='final_stop',event_monotonic_s=20.,
                source_monotonic_s=19.9,raw_packet=dict(sample_sequence=2),pose=None))
            writer._write()
            state=json.loads(writer.path.read_text(encoding='utf-8'))
            self.assertEqual(state['source_monotonic_s'],19.9)
            self.assertEqual(state['pose_source_monotonic_s'],9.9)
            self.assertEqual(state['pose_measurement_monotonic_s'],9.95)
            self.assertEqual(state['pose']['x'],.4)

    def test_durable_result_flushes_before_replace_and_rejects_nan_without_overwrite(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'result.json'
            path.write_text('{"previous":true}',encoding='utf-8')
            events=[]
            import os
            original_replace=os.replace
            def replace(source,target):
                events.append('replace')
                self.assertEqual(events[0],'fsync')
                original_replace(source,target)
            with patch('simple_p2p_runtime.os.fsync',side_effect=lambda fd:events.append('fsync')), patch('simple_p2p_runtime.os.replace',side_effect=replace):
                durable_json(path,dict(complete=True))
            self.assertEqual(json.loads(path.read_text(encoding='utf-8')),dict(complete=True))
            with self.assertRaises(ValueError):
                durable_json(path,dict(invalid=float('nan')))
            self.assertEqual(json.loads(path.read_text(encoding='utf-8')),dict(complete=True))

    def test_final_encoder_hold_keeps_sealed_pose_time_separate_from_stop_confirmation(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=LiveStatusWriter(Path(directory)/'live.json','boot',clock=lambda:12.)
            writer.submit(dict(event='odometry',phase='arrival_settling',event_monotonic_s=10.,
                source_monotonic_s=9.9,measurement_monotonic_s=9.95,
                raw_packet=dict(sample_sequence=1),pose=dict(x=.5,y=0.,yaw=0.)))
            final_packet=dict(sample_sequence=9,position_left_raw=100,position_right_raw=200)
            writer.submit(dict(event='odometry',phase='final_stop',event_monotonic_s=11.,
                source_monotonic_s=10.9,raw_packet=final_packet,pose=None))
            writer.start()
            self.assertTrue(writer.close(dict(session_complete=True,position_verified=True,
                latest_odometry=dict(x=.5,y=0.,yaw=0.,source_monotonic_s=9.9,measurement_monotonic_s=9.95),
                position_verification=dict(method='preSTOP_measured_zero_plus_unchanged_encoders'),
                final_stop_chain=dict(complete=True,zero_samples=[final_packet]))))
            state=json.loads(writer.path.read_text(encoding='utf-8'))
            self.assertTrue(state['position_verified'])
            self.assertEqual(state['raw_packet']['sample_sequence'],9)
            self.assertEqual(state['source_monotonic_s'],10.9)
            self.assertEqual(state['pose_source_monotonic_s'],9.9)
            self.assertEqual(state['pose_measurement_monotonic_s'],9.95)
            self.assertEqual(state['pose']['x'],.5)


if __name__ == '__main__':
    unittest.main()
