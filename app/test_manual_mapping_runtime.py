"""Pure offline hand-pushed mapping conversions; no ROS, serial, or devices."""
from copy import deepcopy
import importlib.util
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
import threading
import time
from types import SimpleNamespace as NS
import unittest

from manual_mapping_runtime import (LatestMapWriter, MainLoopTimings, SourceGate, crop_front_scan, gmapping_parameters,
    grid_snapshot, header_record, mapping_outcome, names_for, save_new_map,
    source_ros_ns, source_current, stamp_ns)


def scan():
    return NS(header=NS(seq=7, stamp=NS(secs=10, nsecs=999000000), frame_id='map'),
        angle_min=-math.pi, angle_max=math.pi, angle_increment=math.pi/4,
        range_min=.1, range_max=12., time_increment=.001, scan_time=.1,
        ranges=[1., 2., 3., 4., 5., 6., 7., 8., 9.], intensities=list(range(9)))


def grid():
    return NS(header=NS(seq=2, stamp=NS(secs=20, nsecs=123), frame_id='manual_test_map'),
        info=NS(width=3, height=2, resolution=.05,
            origin=NS(position=NS(x=-1., y=-2., z=0.), orientation=NS(x=0., y=0., z=0., w=1.))),
        data=[0, 100, -1, 25, 65, 50])


class ManualMappingTests(unittest.TestCase):
    def test_import_does_not_import_ROS_or_open_devices(self):
        before = set(sys.modules)
        spec = importlib.util.spec_from_file_location('manual_mapping_import_test',
            str(Path(__file__).with_name('manual_mapping_runtime.py')))
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        self.assertFalse({'rospy', 'tf2_ros', 'serial'} & (set(sys.modules)-before))

    def test_front_slice_has_only_measured_rays_and_derived_first_ray_time(self):
        raw = scan()
        cropped, evidence = crop_front_scan(raw, 'manual_test_laser')
        self.assertEqual(cropped.ranges, [3., 4., 5., 6., 7.])
        self.assertEqual(cropped.intensities, [2, 3, 4, 5, 6])
        self.assertAlmostEqual(cropped.angle_min, -math.pi/2)
        self.assertAlmostEqual(cropped.angle_max, math.pi/2)
        self.assertEqual((cropped.header.stamp.secs, cropped.header.stamp.nsecs), (11, 1000000))
        self.assertEqual(evidence['derived_first_ray_offset_ns'], 2000000)
        self.assertEqual(evidence['raw_header']['frame_id'], 'map')
        self.assertEqual(evidence['rear_rays_invented'], 0)
        self.assertEqual(raw.ranges, list(map(float, range(1, 10))))
        self.assertEqual(raw.header.stamp.nsecs, 999000000)
        self.assertEqual(raw.header.frame_id, 'map')

    def test_invalid_rays_are_skipped_not_invented_free_ranges(self):
        raw = scan()
        raw.ranges[2:7] = [float('nan'), -math.inf, .05, 13., math.inf]
        cropped, evidence = crop_front_scan(raw, 'manual_test_laser')
        self.assertTrue(all(x == math.inf for x in cropped.ranges))
        self.assertEqual(evidence['invalid_rays_skipped'], 4)
        self.assertEqual(evidence['positive_infinite_rays'], 1)
        self.assertEqual(evidence['raw_selected_ranges'], [None, '-inf', .05, 13., 'inf'])
        json.dumps(evidence, allow_nan=False)

    def test_zero_time_increment_preserves_exact_original_stamp(self):
        raw = scan()
        raw.time_increment = 0.
        cropped, _ = crop_front_scan(raw, 'manual_test_laser')
        self.assertEqual(header_record(cropped.header)['stamp_nsecs'], 999000000)
        self.assertEqual(cropped.header.stamp.secs, 10)

    def test_actual_float32_1200_beam_geometry_is_accepted(self):
        raw = scan()
        raw.angle_min, raw.angle_increment = -3.1415927410125732, .005235987715423107
        raw.angle_max = raw.angle_min+1199*raw.angle_increment
        raw.ranges, raw.intensities = [2.]*1200, []
        cropped, evidence = crop_front_scan(raw, 'manual_test_laser')
        self.assertEqual(len(cropped.ranges), 601)
        self.assertEqual(evidence['first_raw_index'], 300)

    def test_bad_layouts_and_unobserved_front_are_rejected(self):
        for change in (dict(angle_increment=-.1), dict(angle_min=0., angle_max=2*math.pi),
                       dict(range_min=13.), dict(time_increment=-.1), dict(angle_max=1.)):
            raw = scan()
            for key, value in change.items():
                setattr(raw, key, value)
            with self.subTest(change=change), self.assertRaises(ValueError):
                crop_front_scan(raw, 'manual_test_laser')
        raw = scan()
        raw.angle_min, raw.angle_max, raw.angle_increment = -math.pi/2, math.pi/2, math.pi/2047
        raw.ranges, raw.intensities = [1.]*2048, []
        with self.assertRaises(ValueError):
            crop_front_scan(raw, 'manual_test_laser')  # OpenSlam asserts beams < 2048.

    def test_source_gate_duplicates_and_old_samples_cannot_renew_age(self):
        gate = SourceGate(.75)
        h = header_record(scan().header)
        self.assertTrue(gate.accept(h, 11.1))
        original = gate.last_ns
        h['seq'] += 1
        self.assertFalse(gate.accept(h, 11.5))
        self.assertFalse(gate.accept(h, 11.749))  # exact original source + .75
        self.assertEqual(gate.last_ns, original)
        old = dict(h, stamp_secs=10, stamp_nsecs=998000000)
        self.assertFalse(gate.accept(old, 11.1))
        future = dict(h, stamp_secs=12)
        self.assertFalse(gate.accept(future, 11.1))
        self.assertEqual(gate.counts, dict(accepted=1, duplicate=1, reordered=1, future=1, stale=1))

    def test_source_bridge_preserves_original_age_and_is_not_relay_time(self):
        result = source_ros_ns(40., 40.25, 100000000000)
        self.assertEqual(result, 99750000000)
        self.assertEqual(result, source_ros_ns(40., 40.5, 100250000000))
        with self.assertRaises(ValueError):
            source_ros_ns(41., 40.25, 100000000000)

    def test_old_or_future_laser_cannot_prove_complete_recording_window(self):
        h = dict(stamp_secs=10, stamp_nsecs=0)
        self.assertTrue(source_current(h, 10.749))
        self.assertFalse(source_current(h, 10.75))
        self.assertFalse(source_current(h, 9.99))

    def test_clean_recording_with_gap_is_saved_but_never_complete_quality(self):
        query = dict(recording_ended_cleanly=True, odometry_continuity_validated=False,
                     metrics=dict(gap_count=2))
        result = mapping_outcome(query, 1, True)  # Includes a gap seen only by query drain.
        self.assertTrue(result['recording_ended_cleanly'])
        self.assertEqual(result['gap_count'], 2)
        self.assertFalse(result['odometry_continuity_validated'])
        self.assertFalse(result['mapping_quality_complete'])
        self.assertFalse(result['recording_complete'])
        query.update(odometry_continuity_validated=True, metrics=dict(gap_count=0))
        self.assertTrue(mapping_outcome(query, 0, True)['recording_complete'])
        self.assertFalse(mapping_outcome(query, 0, False)['recording_complete'])
        self.assertFalse(mapping_outcome(query, 1, True)['recording_complete'])

    def test_new_names_and_parameters_do_not_load_old_map(self):
        namespace, frames = names_for('test1234')
        self.assertEqual(namespace, '/manual_mapping_test1234')
        self.assertEqual(len(set(frames.values())), 4)
        parameters = gmapping_parameters(frames)
        self.assertEqual(parameters['map_frame'], 'manual_test1234_map')
        self.assertEqual(parameters['maxRange'], 50.)
        self.assertFalse(any('file' in name for name in parameters))
        for bad in ('', 'a/b/c', '../test', 'test-name'):
            with self.assertRaises(ValueError):
                names_for(bad)

    def test_map_output_has_ros_coordinate_row_order_and_unknown_cells(self):
        snapshot = grid_snapshot(grid(), 'manual_test_map')
        self.assertEqual(snapshot['known_cells'], 5)
        with tempfile.TemporaryDirectory() as directory:
            report = save_new_map(directory, snapshot, 'final')
            self.assertEqual(Path(report['pgm']).read_bytes(), b'P5\n3 2\n255\n'+bytes([254, 0, 205, 254, 0, 205]))
            self.assertIn('image: final.pgm', Path(report['yaml']).read_text())
            self.assertEqual(json.loads(Path(report['grid']).read_text())['data'], [0, 100, -1, 25, 65, 50])
            with self.assertRaises(ValueError):
                save_new_map(directory, snapshot, '../old')

    def test_periodic_map_writes_only_small_metadata_and_final_keeps_exact_grid(self):
        snapshot = grid_snapshot(grid(), 'manual_test_map')
        with tempfile.TemporaryDirectory() as directory:
            latest = save_new_map(directory, snapshot, 'latest')
            metadata = json.loads(Path(latest['metadata']).read_text())
            self.assertNotIn('data', metadata)
            self.assertEqual(set(metadata), {'header', 'width', 'height', 'resolution', 'origin',
                                             'known_cells', 'pgm_sha256'})
            self.assertEqual(metadata['header'], dict(snapshot['header']))
            self.assertEqual(metadata['origin'], [-1., -2., 0.])
            self.assertEqual(metadata['pgm_sha256'], hashlib.sha256(Path(latest['pgm']).read_bytes()).hexdigest())
            self.assertFalse((Path(directory)/'latest_grid.json').exists())
            self.assertIsNone(latest['grid'])
            final = save_new_map(directory, snapshot, 'final')
            self.assertEqual(json.loads(Path(final['grid']).read_text())['data'], list(snapshot['data']))
            self.assertEqual(Path(final['pgm']).read_bytes(), Path(latest['pgm']).read_bytes())

    def test_main_timing_ring_is_bounded_and_keeps_longest_and_failed_stage(self):
        now = [10.]
        timing = MainLoopTimings(limit=2, clock=lambda: now[0])
        timing.begin('grid_snapshot')
        now[0] += .4
        timing.finish(cells=215040)
        for _ in range(3):
            timing.begin('odom_batch_publish', batch_count=2)
            now[0] += .01
            timing.finish()
        timing.begin('status_write')
        now[0] += .2
        timing.finish(error_type='OSError', message='injected')
        report = timing.report()
        self.assertEqual(len(report['recent']), 2)
        self.assertAlmostEqual(report['longest']['grid_snapshot']['duration_s'], .4)
        self.assertEqual(report['counts']['odom_batch_publish'], 3)
        self.assertEqual(report['recent'][-1]['error_type'], 'OSError')
        self.assertIsNone(report['active'])

    def test_invalid_map_frame_cells_and_geometry_are_rejected(self):
        original = grid()
        for key in ('frame', 'cells', 'quaternion', 'resolution'):
            bad = deepcopy(original)
            if key == 'frame':
                bad.header.frame_id = 'old_map'
            elif key == 'cells':
                bad.data[0] = 101
            elif key == 'quaternion':
                bad.info.origin.orientation.w = .5
            else:
                bad.info.resolution = float('nan')
            with self.subTest(key=key), self.assertRaises(ValueError):
                grid_snapshot(bad, 'manual_test_map')

    def test_writer_blocked_io_does_not_block_submit_and_only_latest_pending_survives(self):
        entered, release = threading.Event(), threading.Event()
        seen = []
        def write(_directory, snapshot, _stem):
            seen.append(snapshot)
            if len(seen) == 1:
                entered.set()
                release.wait(2.)
            return dict(header=dict(snapshot['header']))
        original = grid()
        first = grid_snapshot(original, 'manual_test_map')
        original.data[0] = 100
        self.assertEqual(first['data'][0], 0)
        with self.assertRaises(TypeError):
            first['header']['seq'] = 99
        with self.assertRaises(TypeError):
            first['data'][0] = 99
        writer = LatestMapWriter('.', write)
        try:
            writer.submit(first)
            self.assertTrue(entered.wait(1.))
            second = grid_snapshot(grid(), 'manual_test_map')
            third = grid_snapshot(grid(), 'manual_test_map')
            began = time.monotonic()
            writer.submit(second)
            writer.submit(third)
            self.assertLess(time.monotonic()-began, .1)
            self.assertTrue(writer.diagnostics()['in_flight'])
            self.assertEqual(writer.diagnostics()['pending_replacements'], 1)
        finally:
            release.set()
            report = writer.close(2.)
        self.assertFalse(report['thread_still_alive'])
        self.assertIs(seen[0], first)
        self.assertIs(seen[1], third)  # Reference handoff, no second grid copy.
        self.assertEqual(report['written'], 2)

    def test_writer_error_is_latched_and_rejects_further_submission(self):
        def broken(_directory, _snapshot, _stem):
            raise OSError('injected disk failure')
        writer = LatestMapWriter('.', broken)
        writer.submit(grid_snapshot(grid(), 'manual_test_map'))
        report = writer.close(2.)
        self.assertEqual(report['failure']['message'], 'injected disk failure')
        self.assertFalse(report['thread_still_alive'])
        self.assertEqual(report['written'], 0)
        with self.assertRaises(RuntimeError):
            writer.submit(grid_snapshot(grid(), 'manual_test_map'))

    def test_writer_join_timeout_reports_live_writer_until_it_really_exits(self):
        entered, release = threading.Event(), threading.Event()
        def blocked(_directory, _snapshot, _stem):
            entered.set()
            release.wait(2.)
            return {}
        writer = LatestMapWriter('.', blocked)
        try:
            writer.submit(grid_snapshot(grid(), 'manual_test_map'))
            self.assertTrue(entered.wait(1.))
            report = writer.close(.01)
            self.assertTrue(report['thread_still_alive'])
            self.assertTrue(report['in_flight'])
            with self.assertRaises(RuntimeError):
                writer.submit(grid_snapshot(grid(), 'manual_test_map'))
        finally:
            release.set()
            self.assertFalse(writer.close(2.)['thread_still_alive'])


if __name__ == '__main__':
    unittest.main()
