"""Offline broker/HTTP checks. No SSH, ROS or physical serial access."""
import hashlib
import http.client
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
import uuid
from http.server import ThreadingHTTPServer

from local_p2p_server import Broker, RequestError, compose, relative_goal, handler_for


def rid():
    return str(uuid.uuid4())


class Remote:
    def __init__(self):
        self.requests = []
        self.boot = "029754c3-b24f-4360-a3f8-cfa24d2a36e2"
        self.packet = dict(position_left_raw=2, position_right_raw=4294967293)
        self.active = None
        self.response = None

    def close(self):
        pass

    def request(self, data):
        self.requests.append(data.copy())
        op = data["op"]
        if op == "poll" and "task_id" not in data:
            return dict(ok=True, boot_id=self.boot, active_task_id=self.active,
                        active_jobs=[] if self.active is None else [dict(task_id=self.active)])
        if op == "probe":
            return dict(ok=True, boot_id=self.boot, session_complete=True, now_monotonic_s=100.,
                        origin=dict(source_monotonic_s=99.95), latest_feedback=dict(self.packet),
                        complete_write_counts={"2":1,"5":1,"3":0,"6":0,"7":0})
        if op == "start":
            if set(data) != {"op","request_id","goal_x","goal_y","expected_boot_id","expected_start_encoders","area_clear"}:
                raise ValueError("RPC contract mismatch")
            self.active = data["request_id"]
            return dict(ok=True, boot_id=self.boot, task_id=self.active, active=True, live_status=None, result=None)
        return self.response


class BrokerTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.map = Path(self.tmp.name)/"final.pgm"
        self.map.write_bytes(b"P5\n2 2\n255\n\x00\xcd\xfe\xff")
        self.sha = hashlib.sha256(self.map.read_bytes()).hexdigest()
        self.remote = Remote()
        self.broker = Broker(self.remote, self.map, self.sha, .05, clock=lambda:100.)
        self.broker.cycle()

    def tearDown(self):
        self.tmp.cleanup()

    def reset(self):
        self.broker.submit("reset", dict(request_id=rid()))
        self.broker.cycle(self.broker.queue.get_nowait())
        self.broker.busy = False

    def command(self):
        epoch = self.broker.reference["epoch"]
        p = self.broker.submit("preview", dict(epoch=epoch,x=.5,y=.2))
        return dict(epoch=epoch,x=.5,y=.2,preview_id=p["preview_id"],area_clear=True,request_id=rid())

    def test_reset_and_preview_are_read_only_and_map_has_no_coordinate_control(self):
        self.reset()
        command = self.command()
        self.assertTrue(command["preview_id"])
        self.assertEqual(self.broker.reference["pose"],dict(x=0.,y=0.,yaw=0.))
        self.assertTrue(self.broker.map["reference_only"])
        self.assertEqual(self.broker.map["sha256"],self.sha)
        self.assertTrue(all(x["op"] in ("probe","poll") for x in self.remote.requests))

    def test_goal_uses_fixed_reset_frame_across_turned_task_start(self):
        pose = dict(x=1.,y=2.,yaw=math.pi/2)
        local = relative_goal(pose,1.,3.)
        self.assertAlmostEqual(local["x"],1.)
        self.assertAlmostEqual(local["y"],0.)
        back = compose(pose,dict(x=local["x"],y=local["y"],yaw=-math.pi/2))
        self.assertAlmostEqual(back["x"],1.)
        self.assertAlmostEqual(back["y"],3.)
        self.assertAlmostEqual(back["yaw"],0.)

    def test_idle_handpush_invalidates_epoch_and_preview(self):
        self.reset()
        command = self.command()
        self.remote.packet["position_left_raw"] += 1
        self.broker.cycle()
        self.assertFalse(self.broker.reference["valid"])
        with self.assertRaises(RequestError):
            self.broker.submit("execute",command)

    def test_execute_reprobes_then_passes_exact_uint32_anchor_once(self):
        self.reset()
        command = self.command()
        self.broker.submit("execute",command)
        self.broker.cycle(self.broker.queue.get_nowait())
        self.broker.busy = False
        response = self.broker.submit("execute",command)
        self.assertTrue(response["duplicate"])
        starts = [x for x in self.remote.requests if x["op"] == "start"]
        self.assertEqual(len(starts),1)
        self.assertEqual(starts[0]["expected_start_encoders"],dict(left_raw=2,right_raw=4294967293))
        self.assertTrue(starts[0]["area_clear"])
        self.assertTrue(self.broker.task["active"])
        with self.assertRaises(RequestError):
            self.broker.submit("reset",dict(request_id=rid()))

    def test_changed_encoder_between_preview_and_launch_never_starts(self):
        self.reset()
        command = self.command()
        self.broker.submit("execute",command)
        self.remote.packet["position_right_raw"] += 1
        with self.assertRaises(RuntimeError):
            self.broker.cycle(self.broker.queue.get_nowait())
        self.assertFalse(any(x["op"] == "start" for x in self.remote.requests))

    def test_preview_cannot_be_reused_after_goal_change_or_without_area_confirmation(self):
        self.reset()
        command = self.command()
        command["x"] = .6
        with self.assertRaises(RequestError):
            self.broker.submit("execute",command)
        command = self.command()
        command["area_clear"] = False
        with self.assertRaises(RequestError):
            self.broker.submit("execute",command)
        self.assertFalse(any(x["op"] == "start" for x in self.remote.requests))

    def test_final_anchor_is_last_stop_packet_not_stale_live_packet(self):
        self.reset()
        command = self.command()
        self.broker.submit("execute",command)
        self.broker.cycle(self.broker.queue.get_nowait())
        final = dict(position_left_raw=200,position_right_raw=4294967000)
        self.broker.update_task(dict(boot_id=self.remote.boot,task_id=command["request_id"],active=False,
            live_status=dict(boot_id=self.remote.boot,raw_packet=self.remote.packet),result=dict(boot_id=self.remote.boot,session_complete=True,
            position_verified=True,final_stop_chain=dict(complete=True),latest_feedback=final,
            latest_odometry=dict(x=.45,y=.2,yaw=.3))))
        self.assertEqual(self.broker.anchor,dict(left_raw=200,right_raw=4294967000))
        self.assertAlmostEqual(self.broker.reference["pose"]["x"],.45)
        self.assertTrue(self.broker.task["stop_confirmed"])

    def test_recovered_foreign_active_job_blocks_reset_and_never_fabricates_pose(self):
        self.remote.active = rid()
        self.broker.discover()
        self.assertTrue(self.broker.task["active"])
        self.assertFalse(self.broker.reference["valid"])
        with self.assertRaises(RequestError):
            self.broker.submit("reset",dict(request_id=rid()))

    def test_failed_parking_invalidates_reference(self):
        self.reset()
        self.broker.task.update(id=rid(),active=True)
        self.broker.task_start = dict(x=0,y=0,yaw=0)
        self.broker.update_task(dict(boot_id=self.remote.boot,task_id=self.broker.task["id"],active=False,
            result=dict(boot_id=self.remote.boot,session_complete=False,position_verified=False,final_stop_chain=dict(complete=True),
                        latest_odometry=dict(x=.3,y=0,yaw=0))))
        self.assertFalse(self.broker.reference["valid"])

    def test_map_hash_mismatch_refused(self):
        with self.assertRaises(ValueError):
            Broker(self.remote,self.map,"0"*64,.05)

    def test_ssh_round_trip_delay_is_included_in_original_source_age(self):
        current = [100.]
        broker = Broker(self.remote,self.map,self.sha,.05,clock=lambda:current[0])
        original = self.remote.request
        def delayed(payload):
            result = original(payload)
            current[0] += 2.
            return result
        self.remote.request = delayed
        broker.idle_probe()
        state = broker.snapshot()
        self.assertAlmostEqual(state["odometry_age_s"],2.05)
        self.assertAlmostEqual(state["snapshot_age_s"],2.)

    def test_inventory_response_without_original_odometry_does_not_renew_source_age(self):
        current = [100.]
        broker = Broker(self.remote,self.map,self.sha,.05,clock=lambda:current[0])
        broker.idle_probe()
        self.assertAlmostEqual(broker.snapshot()["odometry_age_s"],.05)
        current[0] += 10.
        self.assertAlmostEqual(broker.snapshot()["odometry_age_s"],10.05)
        broker.call(dict(op="poll"))
        self.assertAlmostEqual(broker.snapshot()["odometry_age_s"],10.05)
        self.assertAlmostEqual(broker.snapshot()["snapshot_age_s"],0.)

    def test_service_restart_does_not_restore_valid_old_origin(self):
        self.reset()
        self.broker.state_file = Path(self.tmp.name)/"local-state.json"
        self.broker.persist()
        restored = Broker(self.remote,self.map,self.sha,.05,self.broker.state_file,clock=lambda:100.)
        restored.cycle()
        self.assertFalse(restored.snapshot()["reference"]["valid"])
        self.assertIsNone(restored.reference["epoch"])

    def test_rk_reboot_does_not_apply_historical_completed_task_to_current_frame(self):
        self.reset()
        old_boot = self.remote.boot
        old_pose = dict(self.broker.reference["pose"])
        self.broker.task.update(id=rid(),active=True)
        self.broker.task_start = dict(x=0.,y=0.,yaw=0.)
        self.remote.boot = rid()
        self.remote.response = dict(boot_id=self.remote.boot,task_id=self.broker.task["id"],active=False,
            record=dict(boot_id=old_boot),result=dict(boot_id=old_boot,session_complete=True,
            position_verified=True,latest_odometry=dict(x=.5,y=.3,yaw=0),
            latest_feedback=dict(position_left_raw=100,position_right_raw=200),final_stop_chain=dict(complete=True)))
        self.broker.discover()
        self.assertFalse(self.broker.reference["valid"])
        self.assertFalse(self.broker.task["complete"])
        self.assertFalse(self.broker.task["stop_confirmed"])
        self.assertEqual(self.broker.task["phase"],"historical")
        self.assertEqual(self.broker.reference["pose"],old_pose)
        self.assertEqual(self.broker.boot_id,self.remote.boot)

    def test_slow_task_recovery_does_not_block_http_snapshot(self):
        self.broker.task.update(id=rid(),active=True)
        entered, release, snapshot_done = threading.Event(),threading.Event(),threading.Event()
        original = self.remote.request
        def blocked(payload):
            if payload["op"] == "poll" and "task_id" in payload:
                entered.set()
                release.wait(2.)
                return dict(boot_id=self.remote.boot,task_id=payload["task_id"],active=False)
            return original(payload)
        self.remote.request = blocked
        worker = threading.Thread(target=self.broker.discover)
        reader = threading.Thread(target=lambda:(self.broker.snapshot(),snapshot_done.set()))
        worker.start()
        try:
            self.assertTrue(entered.wait(1.))
            reader.start()
            self.assertTrue(snapshot_done.wait(.25),"HTTP state must not wait for SSH recovery")
        finally:
            release.set(); worker.join(2.)
            if reader.ident:
                reader.join(2.)

    def test_http_get_has_no_actions_and_post_requires_origin_host_and_token(self):
        server = ThreadingHTTPServer(("127.0.0.1",0),handler_for(self.broker))
        thread = threading.Thread(target=server.serve_forever,daemon=True)
        thread.start()
        try:
            before = len(self.remote.requests)
            conn = http.client.HTTPConnection("127.0.0.1",server.server_port,timeout=2)
            conn.request("GET","/api/state",headers={"Host":"127.0.0.1:8766"})
            reply = conn.getresponse()
            token = json.loads(reply.read())["csrf_token"]
            self.assertEqual(reply.status,200)
            self.assertEqual(len(self.remote.requests),before)
            body = json.dumps(dict(request_id=rid()))
            headers = {"Host":"127.0.0.1:8766","Content-Type":"application/json","X-CSRF-Token":token}
            conn.request("POST","/api/reset",body=body,headers=headers)
            reply = conn.getresponse(); reply.read()
            self.assertEqual(reply.status,403)
            headers["Origin"] = "http://127.0.0.1:8766"
            conn.request("POST","/api/reset",body=body,headers=headers)
            reply = conn.getresponse(); reply.read()
            self.assertEqual(reply.status,200)
            self.assertEqual(len(self.remote.requests),before)
            self.assertEqual(self.broker.queue.qsize(),1)
            conn.close()
        finally:
            server.shutdown(); server.server_close(); thread.join()


if __name__ == "__main__":
    unittest.main()
