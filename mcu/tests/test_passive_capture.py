"""Memory/file-only passive capture acceptance; no serial import or hardware."""
import contextlib
import importlib.util
import io
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest
from unittest import mock

PATH = Path(__file__).resolve().parents[1]/'tools'/'passive_capture.py'
SPEC = importlib.util.spec_from_file_location('passive_capture_tested', PATH)
capture = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = capture
SPEC.loader.exec_module(capture)


def packet(sequence, publication_ms=None, **changes):
    publication_ms = 1000+(sequence-1)*50 if publication_ms is None else publication_ms
    fields = dict(boot=0x1234567812345678, epoch=1,
        publication_sequence=sequence & 0xFFFFFFFF, publication_tick_ms=publication_ms & 0xFFFFFFFF,
        sample_sequence=sequence & 0xFFFFFFFF, measurement_tick_ms=(publication_ms-10) & 0xFFFFFFFF,
        task=0, ack_request=0, ack_result=0, state=0, flags=3, index=0, count=0, progress=0.,
        x=0., y=0., yaw=0., v=0., w=0., position_left=123, position_right=456,
        left_tenths_rpm=0, right_tenths_rpm=0, first_error=0, first_drive_error=0,
        first_error_tick_ms=0, grant_open_ms=14, grant_close_ms=38,
        accepted_request_watermark=0)
    fields.update(changes)
    value = capture.codec.Telemetry(**fields)
    return capture.codec.frame(128, struct.pack(capture.LAYOUT, *value.__dict__.values()))


def dataset(folder, frames=None, *, prefix=b'', suffix=b'', receipts=None, info_changes=None):
    folder.mkdir()
    frames = [packet(i+1) for i in range(1301)] if frames is None else frames
    reads = []
    if prefix:
        reads.append((prefix,100.01))
    for i, raw in enumerate(frames):
        reads.append((raw,100.025+i*.05 if receipts is None else receipts[i]))
    if suffix:
        reads.append((suffix,165.04))
    raw, chunks, previous = bytearray(), [], 100.
    for i,(data,receipt) in enumerate(reads):
        chunks.append(dict(read_index=i,read_start_monotonic_s=max(previous,receipt-.001),
            receipt_monotonic_s=receipt,receipt_wall_s=1800000000+receipt,
            offset=len(raw),length=len(data),raw_flushed=True))
        raw.extend(data)
        previous=receipt
    (folder/'raw.bin').write_bytes(raw)
    (folder/'chunks.jsonl').write_text(''.join(json.dumps(line)+'\n' for line in chunks),encoding='utf-8')
    info=dict(schema='mcu-v10-passive-capture-v1',source_label='offline generated complete frames',
        duration_requested_s=65.,start_monotonic_s=100.,end_monotonic_s=165.05,
        capture_complete=True,read_error=None,cleanup_errors=[],forbidden_attempts=[],outbound_operations=0,
        software_sha256=capture.software_identity(),expected_boot_id=0x1234567812345678,expected_epoch=1)
    if info_changes:
        info.update(info_changes)
    info.update(raw_sha256=capture.sha256(folder/'raw.bin'),chunks_sha256=capture.sha256(folder/'chunks.jsonl'))
    capture.save_json(folder/'capture.json',info)
    return info


class StaticAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)

    def tearDown(self):
        self.temp.cleanup()

    def analyze(self, name='data', frames=None, **kwargs):
        folder=self.root/name
        info=dataset(folder,frames,**kwargs)
        report,events=capture.analyze_files(folder/'raw.bin',folder/'chunks.jsonl',info)
        return folder,report,events

    def test_clean_65_seconds_passes_without_boot_STOP_confirmed(self):
        _,report,events=self.analyze()
        self.assertTrue(report['capture_complete'])
        self.assertTrue(report['data_valid'])
        self.assertTrue(report['acceptance_pass'],report['failure_checks'])
        self.assertEqual(report['timing']['MCU_publication']['rate_hz'],20.)
        self.assertAlmostEqual(report['timing']['host_receipt']['rate_hz'],20.)
        self.assertEqual(report['samples']['nominal_rate_hz'],20.)
        self.assertAlmostEqual(report['complete_frame_span_s'],65.)
        self.assertFalse(report['driver_guard_verified_by_tool'])
        self.assertFalse(report['physical_stop_verified_by_tool'])
        self.assertFalse(report['startup_STOP_CONFIRMED_required'])
        self.assertEqual(bytes.fromhex(events[0]['raw_hex']),packet(1))

    def test_bounded_initial_and_final_fragments_retained_separately(self):
        prefix=packet(0,950)[50:]
        suffix=packet(1302)[:37]
        folder,report,events=self.analyze(prefix=prefix,suffix=suffix)
        self.assertTrue(report['acceptance_pass'],report['failure_checks'])
        self.assertEqual(report['event_counts']['boundary_prefix'],1)
        self.assertEqual(report['event_counts']['boundary_suffix'],1)
        self.assertEqual(report['event_counts']['crc_error'],0)
        self.assertEqual(bytes.fromhex(events[0]['raw_hex']),prefix)
        self.assertEqual(bytes.fromhex(events[-1]['raw_hex']),suffix)
        self.assertTrue((folder/'raw.bin').read_bytes().startswith(prefix))

    def test_complete_bad_CRC_in_middle_or_tail_never_whitewashed(self):
        for index in (10,1300):
            frames=[packet(i+1) for i in range(1301)]
            bad=bytearray(frames[index]);bad[-1]^=1;frames[index]=bytes(bad)
            with self.subTest(index=index):
                _,report,events=self.analyze('crc'+str(index),frames)
                self.assertFalse(report['data_valid'])
                self.assertFalse(report['acceptance_pass'])
                self.assertGreaterEqual(report['event_counts']['crc_error'],1)
                self.assertTrue(any(item['kind']=='crc_error' and item['raw_hex']==frames[index].hex() for item in events))

    def test_middle_noise_and_valid_command_frame_are_not_dropped(self):
        for name,addition in (('noise',b'bad-noise'),('command',capture.codec.command(7,1,1,1,1))):
            frames=[packet(i+1) for i in range(1301)]
            frames[20]=frames[20]+addition
            _,report,events=self.analyze(name,frames)
            self.assertFalse(report['acceptance_pass'])
            self.assertFalse(report['data_valid'])
            self.assertTrue(any(item['raw_hex']==addition.hex() for item in events))

    def test_zero_data_repeated_publication_and_backward_sequence_fail(self):
        datasets=[('empty',[]),('repeat',[packet(1)]*1301),
                  ('backward',[packet(i+1 if i!=20 else 1) for i in range(1301)])]
        for name,frames in datasets:
            with self.subTest(name=name):
                _,report,_=self.analyze(name,frames)
                self.assertFalse(report['data_valid'])
                self.assertFalse(report['acceptance_pass'])

    def test_sample_repeat_is_original_and_skipped_observations_not_MCU_loss(self):
        frames=[packet(i+1) for i in range(1301)]
        frames[10]=packet(11,sample_sequence=10,measurement_tick_ms=1440)
        _,report,_=self.analyze(frames=frames)
        self.assertTrue(report['acceptance_pass'],report['failure_checks'])
        self.assertEqual(report['samples']['unchanged_repeats'],1)
        self.assertEqual(report['samples']['skipped_observations'],1)
        self.assertEqual(report['samples']['nominal_rate_hz'],20.)

    def test_sample_same_sequence_cannot_renew_time_or_mutate_pose(self):
        for field,value in (('measurement_tick_ms',1490),('x',.1),('position_left',124),('left_tenths_rpm',1)):
            frames=[packet(i+1) for i in range(1301)]
            changes=dict(sample_sequence=10,measurement_tick_ms=1440)
            changes[field]=value
            frames[10]=packet(11,**changes)
            with self.subTest(field=field):
                _,report,_=self.analyze(field,frames)
                self.assertFalse(report['data_valid'])
                self.assertEqual(report['samples']['changed_repeats'],1)

    def test_bad_sample_direction_or_source_age_or_ODOM_valid_flag_fails(self):
        variants=(dict(sample_sequence=9),dict(measurement_tick_ms=1440),
                  dict(measurement_tick_ms=1200),dict(flags=1))
        for number,changes in enumerate(variants):
            frames=[packet(i+1) for i in range(1301)]
            frames[10]=packet(11,**changes)
            _,report,_=self.analyze('sample'+str(number),frames)
            self.assertFalse(report['acceptance_pass'])
            self.assertFalse(report['data_valid'])

    def test_error_RPM_task_ACK_watermark_and_state_fail_health(self):
        for name,changes in (('rpm',dict(left_tenths_rpm=3)),('error',dict(first_error=7)),
             ('drive',dict(first_drive_error=0x10000)),('uart',dict(flags=35)),
             ('task',dict(task=1)),('ack',dict(ack_request=1,flags=259)),
             ('watermark',dict(accepted_request_watermark=1)),('state',dict(state=4))):
            frames=[packet(i+1) for i in range(1301)]
            frames[10]=packet(11,**changes)
            _,report,_=self.analyze(name,frames)
            self.assertFalse(report['acceptance_pass'],name)

    def test_old_fault_boot_new_boot_and_wrong_expected_epoch_never_ignored(self):
        frames=[packet(i+1) for i in range(1301)]
        frames[0]=packet(1,boot=99,state=9,first_error=8,flags=1)
        _,report,_=self.analyze('oldboot',frames)
        self.assertFalse(report['acceptance_pass'])
        self.assertEqual(report['event_counts']['telemetry'],1301)
        self.assertEqual(len(report['boots']),2)
        folder,_,_=self.analyze('wrongreference')
        report=capture.analyze_capture(folder,expected_boot_id=100,expected_epoch=2)
        self.assertFalse(report['acceptance_pass'])
        self.assertIn('capture_vs_analysis_expected_identity',report['failure_checks'])

    def test_short_complete_frame_span_and_host_USB_burst_fail(self):
        frames=[packet(i+1) for i in range(1199)]
        _,report,_=self.analyze('short',frames,info_changes=dict(end_monotonic_s=160.,duration_requested_s=60.))
        self.assertFalse(report['acceptance_pass'])
        self.assertLess(report['complete_frame_span_s'],60.)
        receipts=[100.025+(i//2)*.1 for i in range(1301)]
        _,report,_=self.analyze('burst',receipts=receipts)
        self.assertFalse(report['checks']['host_receipt_timing'])
        self.assertTrue(report['checks']['MCU_publication_timing'])

    def test_crc_valid_nonfinite_pose_retains_raw_and_reports_without_crash(self):
        frames=[packet(i+1) for i in range(1301)]
        frames[10]=packet(11,x=float('nan'))
        folder,report,events=self.analyze(frames=frames)
        self.assertFalse(report['data_valid'])
        self.assertIsNone(report['odometry_drift'])
        self.assertIsNone(events[10]['decoded']['x'])
        self.assertEqual(events[10]['raw_hex'],frames[10].hex())
        capture._write_analysis(folder,report,events)
        self.assertFalse(json.loads((folder/'report.json').read_text())['acceptance_pass'])

    def test_chunk_coverage_or_SHA_tamper_fails(self):
        folder,_,_=self.analyze('tamper')
        with (folder/'raw.bin').open('ab') as output:
            output.write(b'changed')
        report=capture.analyze_capture(folder)
        self.assertFalse(report['data_valid'])
        self.assertTrue(report['acquisition_errors'])

    def test_analyze_default_never_opens_serial_and_preserves_original_files(self):
        folder,_,_=self.analyze()
        before={name:capture.sha256(folder/name) for name in ('raw.bin','chunks.jsonl','capture.json')}
        with mock.patch.object(capture,'serial_reader',side_effect=AssertionError('No open permitted')):
            with contextlib.redirect_stdout(io.StringIO()):
                result=capture.main(['--input-dir',str(folder),'--output-dir',str(self.root/'analysis')])
        self.assertEqual(result,0)
        self.assertEqual(before,{name:capture.sha256(folder/name) for name in before})


class FakeClock:
    def __init__(self):
        self.ms=0
    def monotonic(self):
        return 100+self.ms/1000
    def wall(self):
        return 1800000000+self.monotonic()


class Reader:
    def __init__(self,clock):
        self.clock,self.reads,self.closed=clock,0,False
        self.inject=None
        self.writes=0
    def read(self,size):
        self.clock.ms+=50
        self.reads+=1
        if self.inject is not None:
            self.inject(self)
        return packet(self.reads)
    def write(self,data):
        self.writes+=1
        raise AssertionError('Underlying write must never be called')
    def close(self):
        self.closed=True


class AcquisitionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name)
    def tearDown(self):
        self.temp.cleanup()
    def run_capture(self,reader,clock,name='capture'):
        return capture.capture_stream(reader,self.root/name,65.,monotonic=clock.monotonic,wall=clock.wall,
            expected_boot_id=0x1234567812345678,expected_epoch=1)

    def test_capture_reads_only_and_each_raw_chunk_is_flush_recorded(self):
        clock=FakeClock();reader=Reader(clock)
        report=self.run_capture(reader,clock)
        self.assertTrue(report['acceptance_pass'],report['failure_checks'])
        self.assertEqual(reader.writes,0)
        self.assertTrue(reader.closed)
        chunks=[json.loads(line) for line in (self.root/'capture/chunks.jsonl').read_text().splitlines()]
        self.assertEqual(report['raw_bytes'],reader.reads*110)
        self.assertTrue(all(item['raw_flushed'] for item in chunks))
        self.assertEqual(chunks[0]['receipt_monotonic_s'],100.05)
        self.assertEqual(capture.analyze_capture(self.root/'capture')['acceptance_pass'],True)

    def test_read_exception_preserves_prior_raw_and_all_three_states_fail(self):
        clock=FakeClock();reader=Reader(clock)
        def fail(value):
            if value.reads==2:
                raise OSError('injected read failure')
        reader.inject=fail
        report=self.run_capture(reader,clock)
        self.assertFalse(report['capture_complete'])
        self.assertFalse(report['data_valid'])
        self.assertFalse(report['acceptance_pass'])
        self.assertEqual((self.root/'capture/raw.bin').read_bytes(),packet(1))
        self.assertIn('injected read failure',(self.root/'capture/chunks.jsonl').read_text())

    def test_forbidden_write_and_flush_injection_fail_without_underlying_action(self):
        for operation in ('write','flush','reset_input_buffer','reset_output_buffer','send_break'):
            clock=FakeClock();reader=Reader(clock);guard=capture.ReadOnlyReader(reader)
            def inject(value):
                if value.reads==2:
                    getattr(guard,operation)(b'unsafe')
            reader.inject=inject
            report=self.run_capture(guard,clock,operation)
            self.assertFalse(report['capture_complete'])
            self.assertFalse(report['data_valid'])
            self.assertFalse(report['acceptance_pass'])
            self.assertEqual(report['forbidden_attempts'],[operation])
            self.assertEqual(reader.writes,0)
            self.assertEqual((self.root/operation/'raw.bin').read_bytes(),packet(1))

    def test_capture_requires_execute_and_port_and_new_directory(self):
        with mock.patch.object(capture,'serial_reader',side_effect=AssertionError('No open permitted')):
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                capture.main(['capture','--port','NOT-A-REAL-PORT','--output-dir',str(self.root/'blocked')])
        self.assertFalse((self.root/'blocked').exists())
        clock=FakeClock();reader=Reader(clock)
        (self.root/'exists').mkdir()
        with self.assertRaises(FileExistsError):
            self.run_capture(reader,clock,'exists')
        self.assertEqual(reader.reads,0)


if __name__=='__main__':
    unittest.main()
