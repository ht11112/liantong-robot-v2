"""RPC path/process/identity boundaries without ROS, SSH, serial or subprocesses."""
from contextlib import nullcontext
import copy
import json
import io
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import local_p2p_remote as remote


BOOT = '029754c3-b24f-4360-a3f8-cfa24d2a36e2'
TASK = 'ef900db1-f2ec-408b-98ab-427ae8a04e30'
TASK2 = '74f99538-3c41-468c-a0b0-5dcd30fbce93'


class Child:
    pid = 123
    def send_signal(self,signal): raise AssertionError('Healthy path must never signal')
    def wait(self,timeout): raise AssertionError('Healthy path must never wait')


class RemoteTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.directory = Path(self.temporary.name).resolve()
        self.identity = dict(pid=123,start_ticks=456,state='S')
        self.starts,self.probes = [],[]
        def spawn(argv,**kwargs):
            self.starts.append((argv,kwargs))
            return Child()
        def probe(expected):
            self.probes.append(expected)
            return dict(boot_id=BOOT,session_complete=True,latest_feedback=dict(position_left_raw=2,
                position_right_raw=4294967293),query_counts={'2':1,'3':0,'5':1,'6':0,'7':0})
        self.service = remote.Remote(self.directory,boot_reader=lambda:BOOT,
            pid_reader=lambda pid:self.identity,popen=spawn,lock_factory=lambda path:nullcontext(),
            probe=probe,clock=lambda:100.)

    def request(self,**changes):
        result = dict(op='start',request_id=TASK,expected_boot_id=BOOT,goal_x=.5,goal_y=0.,
                      expected_start_encoders=dict(left_raw=2,right_raw=4294967293),area_clear=True)
        result.update(changes)
        return result

    def test_start_is_fixed_argv_environment_and_repeated_request_is_idempotent(self):
        result = self.service.handle(self.request())
        self.assertEqual(result['task_id'],TASK)
        self.assertTrue(result['active'])
        argv,kwargs = self.starts[0]
        self.assertNotIn('shell',kwargs)
        self.assertEqual(argv[2],str(self.directory/'simple_p2p_runtime.py'))
        self.assertEqual(argv[argv.index('--expected-start-encoders')+1:][:2],['2','4294967293'])
        self.assertEqual(argv[argv.index('--output-dir')+1],str(self.directory/'jobs'/TASK/'run'))
        self.assertIn('--execute',argv)
        self.assertEqual(kwargs['env']['PYTHONNOUSERSITE'],'1')
        self.service.handle(self.request())
        self.assertEqual(len(self.starts),1)
        with self.assertRaisesRegex(ValueError,'different task parameters'):
            self.service.handle(self.request(goal_x=.6))

    def test_reconnect_inventory_discovers_active_job_and_prevents_probe_or_second_start(self):
        self.service.handle(self.request())
        state = self.service.handle(dict(op='poll'))
        self.assertEqual(state['active_task_id'],TASK)
        self.assertEqual(state['active_jobs'][0]['start_ticks'],456)
        with self.assertRaisesRegex(RuntimeError,'probe is forbidden'):
            self.service.handle(dict(op='probe'))
        with self.assertRaisesRegex(RuntimeError,'Another local task'):
            self.service.handle(self.request(request_id=TASK2))
        self.assertEqual(self.probes,[])
        self.assertEqual(len(self.starts),1)

    def test_probe_has_no_process_and_cancel_only_writes_exact_job_file(self):
        result = self.service.handle(dict(op='probe',expected_boot_id=BOOT))
        self.assertTrue(result['session_complete'])
        self.assertEqual(self.starts,[])
        self.assertEqual(self.probes,[BOOT])
        self.service.handle(self.request())
        result = self.service.handle(dict(op='cancel',task_id=TASK))
        self.assertTrue(result['cancel_requested'])
        self.assertEqual((self.directory/'jobs'/TASK/'cancel.request').read_text(),'operator_cancel\n')
        self.assertEqual(len(self.starts),1)
        self.assertFalse((self.directory/'cancel.request').exists())

    def test_pid_reuse_zombie_or_old_boot_cannot_be_reported_active(self):
        self.service.handle(self.request())
        for identity in (None,dict(pid=123,start_ticks=999,state='S'),dict(pid=123,start_ticks=456,state='Z')):
            self.identity=identity
            result=self.service.handle(dict(op='poll',task_id=TASK))
            self.assertFalse(result['active'])
            self.assertTrue(result['terminal_without_result'])
        self.identity=dict(pid=123,start_ticks=456,state='S')
        self.service.boot=lambda:'6cbe467d-8332-4051-8bed-72641a5d41bf'
        self.assertFalse(self.service.handle(dict(op='poll',task_id=TASK))['active'])

    def test_poll_preserves_original_source_and_result_first_failure(self):
        self.service.handle(self.request())
        output=self.directory/'jobs'/TASK/'run';output.mkdir()
        original=dict(phase='drive',pose=dict(x=.1,y=0.,yaw=0.),source_monotonic_s=99.,
                      raw_packet=dict(position_left_raw=20,position_right_raw=4294967276))
        (output/'live_status.json').write_text(json.dumps(original))
        status=self.service.handle(dict(op='poll',task_id=TASK))
        self.assertEqual(status['live_status'],original)
        self.assertEqual(status['now_monotonic_s'],100.)
        self.identity=None
        first=dict(phase='drive',message='source expired')
        (output/'result.json').write_text(json.dumps(dict(first_failure=first,position_verified=False)))
        status=self.service.handle(dict(op='poll',task_id=TASK))
        self.assertEqual(status['result']['first_failure'],first)
        self.assertFalse(status['result']['position_verified'])

    def test_compact_separate_recovery_preserves_primary_count_scan_and_old_pose(self):
        primary=[dict(sample_sequence=i,position_left_raw=i) for i in range(1,4)]
        newer=[dict(sample_sequence=i,position_left_raw=i) for i in range(4,9)]
        failure=dict(phase='arrival_zero',message='Scan/clearance rejected: obstacle within body clearance')
        scan=dict(phase='arrival_zero',reason='obstacle within body clearance',
            diagnostic=dict(schema='simple-p2p-scan-rejection-v1',header_ns=123,
                hits=[dict(original_index=17,range_m=.15,x_body_m=.18,y_body_m=.14)]))
        pose=dict(x=.39,y=.002,yaw=.01,source_monotonic_s=99.,sample_sequence=3)
        result=dict(session_complete=False,motion_complete=False,position_verified=False,
            first_failure=failure,first_scan_rejection=scan,latest_odometry=pose,
            final_stop_chain=dict(complete=False,error='Original zero proof failed',
                position_verified=False,zero_samples=primary),
            final_stop_recovery=dict(complete=True,position_verified=False,zero_samples=newer),
            rows=[{'large':'raw rows stay on RK'}],raw_events=[{'large':'raw stream stays on RK'}])
        unchanged=copy.deepcopy(result)
        compact=remote.compact_result(result)
        self.assertEqual(compact['final_stop_chain']['zero_sample_count'],3)
        self.assertFalse(compact['final_stop_chain']['complete'])
        self.assertEqual(compact['final_stop_chain']['error'],'Original zero proof failed')
        self.assertEqual(compact['final_stop_recovery']['zero_sample_count'],5)
        self.assertTrue(compact['final_stop_recovery']['complete'])
        self.assertEqual(compact['latest_feedback'],newer[-1])
        self.assertEqual(compact['latest_odometry'],pose)
        self.assertEqual(compact['latest_odometry']['source_monotonic_s'],99.)
        self.assertEqual(compact['first_failure'],failure)
        self.assertEqual(compact['first_scan_rejection'],scan)
        self.assertNotIn('rows',compact)
        self.assertNotIn('raw_events',compact)
        self.assertTrue(compact['full_evidence_retained_on_RK'])
        self.assertEqual(result,unchanged)

    def test_compact_recovery_cannot_claim_original_position_or_task_success(self):
        result=dict(session_complete=True,motion_complete=True,position_verified=True,
            final_stop_chain=dict(complete=False,position_verified=False,zero_samples=[]),
            final_stop_recovery=dict(complete=True,position_verified=True,
                zero_samples=[dict(sample_sequence=i) for i in range(5)]))
        compact=remote.compact_result(result)
        for key in ('session_complete','motion_complete','position_verified'):
            self.assertFalse(compact[key])
            self.assertTrue(result[key])  # Compacting must not rewrite full evidence.
        self.assertFalse(compact['final_stop_chain']['complete'])
        self.assertFalse(compact['final_stop_recovery']['position_verified'])
        self.assertTrue(result['final_stop_recovery']['position_verified'])

    def test_incomplete_recovery_does_not_replace_primary_feedback_and_legacy_success_is_retained(self):
        old=dict(sample_sequence=3)
        result=dict(session_complete=False,motion_complete=False,position_verified=False,
            final_stop_chain=dict(complete=False,zero_samples=[old]),
            final_stop_recovery=dict(complete=False,error='No matching reply',
                zero_samples=[dict(sample_sequence=9)]))
        compact=remote.compact_result(result)
        self.assertEqual(compact['latest_feedback'],old)
        self.assertEqual(compact['final_stop_chain']['zero_sample_count'],1)
        self.assertFalse(compact['final_stop_recovery']['complete'])
        result=dict(session_complete=True,motion_complete=True,position_verified=True,
            final_stop_chain=dict(complete=True,position_verified=True,zero_samples=[old]))
        compact=remote.compact_result(result)
        self.assertEqual(compact['latest_feedback'],old)
        self.assertEqual(compact['final_stop_chain']['zero_sample_count'],1)
        self.assertTrue(compact['session_complete'])
        self.assertTrue(compact['motion_complete'])
        self.assertTrue(compact['position_verified'])

    def test_bad_boot_missing_operator_flag_nonfinite_encoder_bool_and_path_injection_never_spawn(self):
        changes = [dict(expected_boot_id=TASK),dict(area_clear=False),dict(goal_x=float('nan')),
            dict(goal_y=float('inf')),dict(expected_start_encoders=dict(left_raw=True,right_raw=0)),
            dict(expected_start_encoders=dict(left_raw=-1,right_raw=0)),dict(request_id='../escape'),
            dict(command='rm anything'),dict(output_dir='/tmp/escape')]
        for change in changes:
            with self.subTest(change=change),self.assertRaises((ValueError,RuntimeError)):
                self.service.handle(self.request(**change))
        self.assertEqual(self.starts,[])
        for task in ('../escape',TASK+'/extra','',TASK.upper()):
            with self.assertRaises(ValueError):
                self.service.handle(dict(op='cancel',task_id=task))

    def test_failed_or_completed_request_is_never_automatically_relaunched(self):
        self.service.handle(self.request())
        self.identity=None
        status=self.service.handle(self.request())
        self.assertFalse(status['active'])
        self.assertEqual(len(self.starts),1)
        path=self.directory/'jobs'/TASK/'job.json'
        record=json.loads(path.read_text());record.update(pid=None,launch_state='uncertain')
        path.write_text(json.dumps(record))
        with self.assertRaisesRegex(RuntimeError,'Unresolved'):
            self.service.handle(self.request(request_id=TASK2))

    def test_strict_json_rejects_duplicate_keys_and_nonfinite_literals(self):
        for text in ('{"op":"probe","op":"start"}','{"goal":NaN}','{"goal":Infinity}'):
            with self.assertRaises(ValueError):remote.strict_json(text)

    def test_corrupt_evidence_never_certifies_success_or_blocks_active_cancel(self):
        self.service.handle(self.request())
        output=self.directory/'jobs'/TASK/'run';output.mkdir()
        for text in ('', '{"session_complete":', '[]', '{"value":NaN}'):
            with self.subTest(text=text):
                (output/'result.json').write_text(text)
                (output/'live_status.json').write_text('')
                status=self.service.handle(dict(op='poll',task_id=TASK))
                self.assertTrue(status['active'])
                self.assertIsNone(status['result'])
                self.assertEqual(len(status['evidence_errors']),2)
                self.assertFalse(status['terminal_without_result'])
                self.assertTrue(self.service.handle(dict(op='cancel',task_id=TASK))['cancel_requested'])
        self.identity=None
        status=self.service.handle(dict(op='poll',task_id=TASK))
        self.assertTrue(status['terminal_without_result'])
        self.assertIsNone(status['result'])
        # Corrupt ownership metadata still blocks new owners rather than being
        # treated like an unavailable optional result.
        (self.directory/'jobs'/TASK/'job.json').write_text('')
        with self.assertRaises(ValueError):
            self.service.handle(self.request(request_id=TASK2))
        self.assertEqual(len(self.starts),1)

    def test_stdio_returns_one_json_per_request_and_never_prints_child_output(self):
        incoming=io.StringIO('{"op":"probe"}\n{"op":"unknown"}\n')
        outgoing=io.StringIO()
        with patch.object(remote,'Remote',return_value=self.service),patch.object(remote.sys,'stdin',incoming),patch.object(remote.sys,'stdout',outgoing):
            self.assertEqual(remote.main(['--stdio']),0)
        lines=outgoing.getvalue().splitlines()
        self.assertEqual(len(lines),2)
        first,last=map(json.loads,lines)
        self.assertTrue(first['ok'])
        self.assertFalse(last['ok'])
        self.assertEqual(self.starts,[])


if __name__=='__main__':unittest.main()
