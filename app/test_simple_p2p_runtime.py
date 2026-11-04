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
            checked=gate(float(stamp),'drive')
            self.assertEqual(checked['ok'],allowed)
            if not allowed:
                hit=checked['diagnostic']['hits'][0]
                self.assertEqual(checked['diagnostic']['hit_rule'],'body_rectangle_clearance')
                self.assertEqual(hit['original_index'],180)
                self.assertAlmostEqual(hit['y_body_m'],distance)
                self.assertEqual(hit['body_clearance_m'],0.)

    def test_zero_extra_margin_uses_uninflated_turn_sweep(self):
        gate=ScanGate(dict(SETTINGS,stop_clearance_m=0.),turn_area_clear=True)
        gate.accept(scan(obstacle=.1),100100000000,10.)  # x=.23: outside body, inside turn radius.
        self.assertTrue(gate(10.,'drive')['ok'])
        self.assertFalse(gate(10.,'turn')['ok'])
        gate.accept(scan(stamp=101,obstacle=.21),101100000000,11.)  # x=.34 > radius .32016.
        self.assertTrue(gate(11.,'turn')['ok'])
        radius=math.hypot(SETTINGS['body_front_m'],SETTINGS['body_half_width_m'])
        gate.accept(scan(stamp=102,obstacle=radius-SETTINGS['laser_x_m']),102100000000,12.)
        checked=gate(12.,'turn')
        self.assertFalse(checked['ok'])  # Exact uninflated sweep boundary remains blocked.
        self.assertEqual(checked['diagnostic']['hit_rule'],'whole_body_turn_sweep_clearance')
        self.assertAlmostEqual(checked['diagnostic']['turn_sweep_limit_m'],radius)

    def test_rejection_snapshot_preserves_original_ray_neighbors_header_and_source(self):
        gate=ScanGate(dict(SETTINGS,stop_clearance_m=0.),turn_area_clear=True)
        sample=scan()
        sample.header.seq=42
        sample.header.frame_id='front_laser'
        sample.ranges[150]=.12  # 60 degrees: body x=.19, y=.103923...
        sample.ranges[148]=math.inf
        gate.accept(sample,100100000000,10.)
        with patch('simple_p2p_runtime.json.dumps',side_effect=AssertionError('no control-thread serialization')):
            diagnostic=gate(10.1,'drive')['diagnostic']
        self.assertEqual(diagnostic['header_ns'],100000000000)
        self.assertEqual(diagnostic['header_sequence'],42)
        self.assertEqual(diagnostic['frame_id'],'front_laser')
        self.assertEqual(diagnostic['checked_phase'],'drive')
        self.assertAlmostEqual(diagnostic['source_monotonic_s'],9.9)
        self.assertAlmostEqual(diagnostic['source_age_s'],.2)
        self.assertEqual(diagnostic['geometry']['stop_clearance_m'],0.)
        self.assertEqual(diagnostic['scan_geometry']['positive_infinity_rays'],1)
        self.assertEqual(diagnostic['scan_geometry']['front_ray_count'],181)
        self.assertEqual(diagnostic['hit_count'],1)
        self.assertEqual(diagnostic['return_identity'],'unclassified')
        hit=diagnostic['hits'][0]
        self.assertEqual(hit['original_index'],150)
        self.assertAlmostEqual(hit['range_m'],.12)
        self.assertAlmostEqual(hit['angle_laser_rad'],math.pi/3)
        self.assertAlmostEqual(hit['x_body_m'],.19)
        self.assertAlmostEqual(hit['y_body_m'],.12*math.sin(math.pi/3))
        self.assertEqual([ray['original_index'] for ray in diagnostic['first_hit_neighbors']],list(range(148,153)))
        neighbor=diagnostic['first_hit_neighbors'][0]
        self.assertEqual(neighbor['range_classification'],'positive_infinity_no_return')
        self.assertIsNone(neighbor['range_m'])
        self.assertIsNone(neighbor['x_body_m'])
        json.dumps(diagnostic,allow_nan=False)

    def test_duplicate_scan_cannot_replace_rejection_point_or_renew_source(self):
        gate=ScanGate(dict(SETTINGS,stop_clearance_m=0.),turn_area_clear=True)
        original=scan()
        original.header.frame_id='original_frame'
        original.ranges[180]=.2
        gate.accept(original,100100000000,10.)
        replacement=scan()
        replacement.header.frame_id='duplicate_changed_frame'
        gate.accept(replacement,100100000000,10.5)
        diagnostic=gate(10.5,'drive')['diagnostic']
        self.assertAlmostEqual(diagnostic['source_monotonic_s'],9.9)
        self.assertAlmostEqual(diagnostic['source_age_s'],.6)
        self.assertEqual(diagnostic['frame_id'],'original_frame')
        self.assertEqual(diagnostic['hits'][0]['original_index'],180)
        self.assertEqual(diagnostic['hits'][0]['range_m'],.2)
        diagnostic['hits'][0]['range_m']=999.
        diagnostic['geometry']['body_front_m']=999.
        unchanged=gate(10.5,'drive')['diagnostic']
        self.assertEqual(unchanged['hits'][0]['range_m'],.2)
        self.assertEqual(unchanged['geometry']['body_front_m'],.2)
        self.assertNotIn('diagnostic',gate(10.7,'drive'))  # Original source is now stale.

    def test_snapshot_uses_actual_extrinsics_without_classifying_self_returns(self):
        settings=dict(SETTINGS,stop_clearance_m=0.,laser_x_m=-.1,laser_y_m=.02,
                      laser_yaw_rad=math.pi/2,body_front_m=.3,body_rear_m=.17,body_half_width_m=.23)
        gate=ScanGate(settings,turn_area_clear=True)
        gate.accept(scan(obstacle=.15),100100000000,10.)
        diagnostic=gate(10.,'drive')['diagnostic']
        self.assertEqual(diagnostic['geometry'],settings)
        hit=diagnostic['hits'][0]
        self.assertEqual(hit['original_index'],90)
        self.assertEqual(hit['angle_laser_rad'],0.)
        self.assertAlmostEqual(hit['angle_body_rad'],math.pi/2)
        self.assertAlmostEqual(hit['x_body_m'],-.1)
        self.assertAlmostEqual(hit['y_body_m'],.17)
        self.assertEqual(diagnostic['return_identity'],'unclassified')

    def test_maximum_scan_rejection_evidence_has_bounded_hits_neighbors_and_frame(self):
        gate=ScanGate(dict(SETTINGS,stop_clearance_m=0.),turn_area_clear=True)
        sample=scan()
        sample.ranges=[.1]*20000
        sample.angle_increment=math.pi/(len(sample.ranges)-1)
        sample.header.frame_id='x'*2000
        gate.accept(sample,100100000000,10.)
        diagnostic=gate(10.,'drive')['diagnostic']
        self.assertGreater(diagnostic['hit_count'],8)
        self.assertEqual(len(diagnostic['hits']),8)
        self.assertTrue(diagnostic['hits_truncated'])
        self.assertEqual(len(diagnostic['first_hit_neighbors']),3)  # First measured ray is the hit.
        self.assertEqual(len(diagnostic['frame_id']),256)
        self.assertTrue(diagnostic['frame_id_truncated'])
        self.assertEqual(diagnostic['scan_geometry']['original_ray_count'],20000)
        encoded=json.dumps(diagnostic,allow_nan=False)
        self.assertLess(len(encoded),12000)

    def test_invalid_rays_still_reject_and_clear_scan_avoids_snapshot_work(self):
        gate=ScanGate(dict(SETTINGS,stop_clearance_m=0.),turn_area_clear=True)
        gate.accept(scan(),100100000000,10.)
        with patch.object(gate,'_rejection_diagnostic',side_effect=AssertionError('clear path snapshot')):
            self.assertTrue(gate(10.,'drive')['ok'])
        for stamp,value in enumerate((float('nan'),-math.inf,.01,51.),start=101):
            bad=scan(stamp=stamp)
            bad.ranges[180]=value
            gate.accept(bad,stamp*1000000000+100000000,float(stamp))
            checked=gate(float(stamp),'drive')
            self.assertFalse(checked['ok'])
            self.assertNotIn('diagnostic',checked)
            json.dumps(checked,allow_nan=False)

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

    def test_supplemental_stop_uses_new_raw_packet_without_renewing_failed_pose(self):
        with tempfile.TemporaryDirectory() as directory:
            writer=LiveStatusWriter(Path(directory)/'live.json','boot',clock=lambda:30.)
            writer.submit(dict(event='odometry',phase='drive',event_monotonic_s=10.,
                source_monotonic_s=9.9,measurement_monotonic_s=9.95,
                raw_packet=dict(sample_sequence=1),pose=dict(x=.39,y=0.,yaw=0.)))
            writer.start()
            final_packet=dict(sample_sequence=20,position_left_raw=100,position_right_raw=200)
            self.assertTrue(writer.close(dict(session_complete=False,position_verified=False,
                latest_odometry=dict(x=.39,y=0.,yaw=0.,source_monotonic_s=9.9,measurement_monotonic_s=9.95),
                final_stop_chain=dict(complete=False,zero_samples=[]),
                final_stop_recovery=dict(complete=True,zero_samples=[final_packet]))))
            state=json.loads(writer.path.read_text(encoding='utf-8'))
            self.assertEqual(state['raw_packet']['sample_sequence'],20)
            self.assertFalse(state['session_complete'])
            self.assertFalse(state['position_verified'])
            self.assertEqual(state['pose_source_monotonic_s'],9.9)
            self.assertEqual(state['pose_measurement_monotonic_s'],9.95)
            self.assertEqual(state['pose']['x'],.39)


if __name__ == '__main__':
    unittest.main()
