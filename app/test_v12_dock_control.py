import json
import math
from pathlib import Path
import struct
from tempfile import TemporaryDirectory
from http.server import ThreadingHTTPServer
from threading import Event, Thread
from urllib.request import Request, urlopen
import unittest
from dataclasses import replace
from unittest.mock import Mock, patch

import test_v11_map_runtime as base
from mcu_v11 import EXECUTING, ODOM_INITIALIZED, ODOM_VALID, RX_FAULT, STOP_CONFIRMED, Rejected
from v11_map_runtime import Navigation, handler_for
from v12_dock_control import DockSettings, dock_segment, dock_sites, reverse_plan


class Camera:
    def __init__(self):
        self.sequence = 0
        self.results = {}
        self.cancelled = []

    def snapshot(self):
        return dict(fresh=False, ready=False, reason='预览过期')

    def request_photo(self):
        self.sequence += 1
        return self.sequence

    def photo_result(self, photo_id):
        return self.results.get(photo_id)

    def cancel_photo(self, photo_id):
        self.cancelled.append(photo_id)
        self.results.pop(photo_id, None)

    def finish(self, photo_id=None, **changes):
        photo_id = self.sequence if photo_id is None else photo_id
        result = dict(photo_id=photo_id, frame_seq=photo_id, ready=True, image_valid=True,
                      reason=None, tag_id=1, plane_x_m=-.70, plane_y_m=0., normal_x=1., normal_y=0.)
        result.update(changes)
        self.results[photo_id] = result


class GeometryTest(unittest.TestCase):
    def test_once_plan_reaches_axle_target_after_turn_and_reverse(self):
        for px, py, yaw in ((-.7, 0., 0.), (-.7, .15, .1), (-.7, -.15, -.1)):
            plan = reverse_plan(dict(plane_x_m=px, plane_y_m=py,
                                     normal_x=math.cos(yaw), normal_y=math.sin(yaw)))
            q, turn, distance = plan['target_body'], plan['turn_rad'], plan['reverse_m']
            self.assertAlmostEqual(-distance*math.cos(turn), q['x'])
            self.assertAlmostEqual(-distance*math.sin(turn), q['y'])
            self.assertAlmostEqual(math.hypot(q['x']-px, q['y']-py), .25)
            self.assertEqual(math.copysign(1, turn), -math.copysign(1, py) if py else -1.)

    def test_boundary_budget_and_wire_are_real_production(self):
        self.assertEqual(reverse_plan(dict(plane_x_m=-1.25, plane_y_m=0., normal_x=1., normal_y=0.))['budget_s'], 125)
        self.assertEqual(struct.unpack('<If', dock_segment('move', -1.).segments[0]), (3, -1.))
        self.assertEqual(struct.unpack('<If', dock_segment('normal_turn', math.pi/6).segments[0])[0], 2)
        for value in (-1.001, -.005, 0., .01, float('nan')):
            with self.assertRaises(ValueError):
                dock_segment('move', value)
        for px,py in ((.1,0.),(-1.26,0.),(-.2,0.),(-1.25,.1)):
            with self.assertRaises(ValueError):
                reverse_plan(dict(plane_x_m=px,plane_y_m=py,normal_x=1.,normal_y=0.))

    def test_config_persistence_and_map_binding(self):
        with TemporaryDirectory() as folder:
            path = Path(folder)/'dock.json'
            settings = DockSettings('a', path)
            sites = dock_sites(settings.value)
            self.assertAlmostEqual(sites['waiting']['x'], .2)
            self.assertAlmostEqual(sites['axle_target']['x'], -.25)
            settings.save(dict(map_id='a',x=1.,y=2.,yaw=math.pi/2))
            loaded = DockSettings('a', path)
            self.assertEqual(loaded.value, settings.value)
            self.assertAlmostEqual(dock_sites(loaded.value)['waiting']['y'], 2.7)
            self.assertEqual(DockSettings('b',path).value['x'], -.5)
            for bad in (dict(map_id='b',x=0.,y=0.,yaw=0.),dict(map_id='a',x=True,y=0.,yaw=0.)):
                with self.assertRaises(ValueError):
                    settings.save(bad)


class DockTest(unittest.TestCase):
    setUp = base.NavigationTest.setUp
    observe = base.NavigationTest.observe

    def enable(self):
        self.nav.mode = 'drive'
        self.nav.camera = Camera()
        self.observe(flags=self.sample.flags | STOP_CONFIRMED)
        self.nav.align(dict(x=1.7,y=1.,yaw=0.), 'manual')
        self.nav.configure_dock(dict(map_id=self.grid.metadata()['map_id'],x=1.,y=1.,yaw=0.))

    def stop_proof(self, verified=True):
        master = Mock()
        master.stop.return_value = dict(stop_verified=verified)
        self.nav._stop(master)
        self.nav.stop_requested = False
        if self.nav.work == 'stopping':
            self.nav.work = None

    def begin(self, single=False):
        self.enable()
        if single:
            self.nav.dock_single_step_test = True
            self.nav.request_dock_step('move', -.03)
        else:
            self.nav.request_dock()
        self.assertTrue(self.nav.stop_requested)
        self.stop_proof()
        self.assertEqual(self.nav.dock['phase'], 'capturing')

    def photo(self, **changes):
        self.nav.camera.finish(**changes)
        self.nav._dock_cycle()
        return self.nav.pending

    def tick(self, dt=.05, **changes):
        if 'flags' in changes:
            changes['flags'] |= ODOM_INITIALIZED | ODOM_VALID
        self.now += dt
        tick = self.sample.measurement_tick_ms+round(dt*1000)
        self.observe(sample_sequence=self.sample.sample_sequence+1,measurement_tick_ms=tick,
                     publication_tick_ms=tick+5, **changes)

    def start_motion(self):
        plan, self.nav.pending = self.nav.pending, None
        master = Mock()
        ready = replace(self.nav.latest,status=replace(self.sample,task=7,state=2))
        master.upload.side_effect = lambda plan,on_ack: (on_ack(ready),[ready])[1]
        started = replace(ready,sent_at=self.now-.02,received_at=self.now,
            status=replace(ready.status,state=4,flags=EXECUTING|ODOM_INITIALIZED|ODOM_VALID,lease_remaining_ms=30000))
        master.start.return_value = started
        self.nav._upload_dock(master, plan)
        self.sample = started.status
        self.nav.work = None
        return master

    def complete_motion(self, **changes):
        self.tick(state=8,flags=STOP_CONFIRMED|8,lease_remaining_ms=0,**changes)
        self.nav._dock_cycle()
        self.assertTrue(self.nav.stop_requested)
        self.stop_proof()

    def test_default_route_approach_preserves_manual_pose(self):
        self.enable()
        self.nav.align(dict(x=1.,y=1.,yaw=0.),'manual')
        old = dict(self.nav.origin)
        self.nav.request_dock()
        self.assertEqual(self.nav.dock['phase'], 'approaching')
        self.assertAlmostEqual(self.nav.pending['goal']['x'], 1.7)
        self.assertEqual(self.nav.origin, old)
        self.assertIsNone(self.nav.dock_plan)
        self.nav.work = None
        self.complete_motion()
        self.assertEqual(self.nav.camera.sequence,1)

    def test_first_target_plans_one_reverse_and_no_final_photo(self):
        self.begin()
        plan=self.photo()
        self.assertAlmostEqual(struct.unpack('<If',plan.segments[0])[1],-.45,places=6)
        self.start_motion()
        self.assertEqual(self.nav.dock['phase'],'backing')
        self.nav.camera.results.clear()
        self.complete_motion()
        self.assertEqual(self.nav.dock['phase'],'complete')
        self.assertEqual(self.nav.dock['message'],'倒车计划执行完成，已停稳')
        self.assertEqual(self.nav.camera.sequence,1)
        self.assertIsNone(self.nav.pending)

    def test_offset_turn_then_reverse_uses_same_photo(self):
        self.begin()
        self.photo(plane_y_m=.15)
        solution=dict(self.nav.dock_solution)
        self.start_motion()
        self.assertEqual(self.nav.dock['phase'],'turning')
        self.complete_motion(yaw=solution['turn_rad'])
        self.assertEqual(self.nav.dock_motion,'reverse')
        self.assertEqual(self.nav.camera.sequence,1)
        self.start_motion()
        self.complete_motion()
        self.assertEqual(self.nav.dock['phase'],'complete')
        self.assertEqual(self.nav.camera.sequence,1)

    def test_missing_or_wrong_id_searches_using_measured_angle(self):
        self.begin()
        self.photo(ready=False,tag_id=None)
        self.assertEqual(struct.unpack('<If',self.nav.pending.segments[0])[0],2)
        self.start_motion()
        self.complete_motion(yaw=math.radians(27.))
        self.photo(ready=False,tag_id=None)
        self.assertAlmostEqual(struct.unpack('<If',self.nav.pending.segments[0])[1],math.radians(33.),places=6)
        self.start_motion()
        self.complete_motion(yaw=math.radians(57.))
        self.photo()
        self.assertEqual(self.nav.dock_motion,'reverse')
        self.assertEqual(self.nav.dock_search_index,2)

    def test_one_search_circle_uses_wrapped_measured_turns_and_then_stops(self):
        self.begin()
        for i in range(1,13):
            self.photo(ready=False,tag_id=None)
            self.start_motion()
            measured=math.radians(i*30-2)
            self.complete_motion(yaw=math.atan2(math.sin(measured),math.cos(measured)))
        self.photo(ready=False,tag_id=None)
        self.assertEqual(self.nav.dock_stop_outcome,'failed')
        self.assertIsNone(self.nav.pending)
        self.assertAlmostEqual(self.nav.dock['search_angle_deg'],358.,places=4)

    def test_photo_has_no_raw_counter_equality_gate_but_pose_check_remains(self):
        self.begin()
        self.tick(position_left=12,position_right=9)
        self.photo()
        self.assertIsNotNone(self.nav.pending)
        self.tick(x=self.sample.x+.021)
        master=Mock()
        with self.assertRaises(ValueError):
            self.nav._upload_dock(master,self.nav.pending)
        master.start.assert_not_called()

    def test_capture_failure_stale_id_unverified_and_three_second_timeout_stop(self):
        for failure in ('camera','old','unverified','timeout'):
            with self.subTest(failure=failure):
                self.setUp();self.begin()
                if failure=='camera': self.photo(ready=False,image_valid=False,reason='no image')
                elif failure=='unverified': self.photo(ready=False)
                elif failure=='old':
                    self.photo(photo_id=99)
                    self.assertIsNone(self.nav.pending)
                    self.assertEqual(self.nav.dock['phase'],'capturing')
                    self.tick(3.01);self.nav._dock_cycle()
                else: self.tick(3.01);self.nav._dock_cycle()
                self.assertEqual(self.nav.dock_stop_outcome,'failed')
                self.assertIsNone(self.nav.pending)

    def test_cancelled_late_photo_cannot_resume(self):
        self.begin()
        self.nav.request_stop()
        self.nav.camera.finish()
        self.stop_proof()
        self.nav._dock_cycle()
        self.assertEqual(self.nav.dock['phase'],'cancelled')
        self.assertIsNone(self.nav.pending)

    def test_failed_stop_cannot_complete_or_schedule_reverse(self):
        self.begin();self.photo();self.start_motion()
        self.tick(state=8,flags=STOP_CONFIRMED|8,lease_remaining_ms=0)
        self.nav._dock_cycle();self.stop_proof(False)
        self.assertEqual(self.nav.dock['phase'],'failed')
        self.assertIsNone(self.nav.pending)

    def test_long_reverse_renews_beyond_thirty_seconds_and_ignores_preview_loss(self):
        self.begin();self.photo(plane_x_m=-1.25);master=self.start_motion()
        deadline=self.nav.dock_deadline
        master.renew.side_effect=lambda task: replace(self.nav.latest,status=replace(self.sample,lease_remaining_ms=30000))
        for _ in range(3):
            for _ in range(500):
                self.tick(state=4,flags=EXECUTING,lease_remaining_ms=4000)
            self.nav._dock_cycle();self.nav._renew_if_due(master,self.nav.latest)
        self.assertEqual(master.renew.call_count,3)
        self.assertFalse(self.nav.stop_requested)
        self.assertEqual(self.nav.camera.sequence,1)
        self.assertAlmostEqual(deadline,(5.-.02)+125)

    def test_budget_from_actual_start_blocks_renew_and_stops(self):
        self.begin();self.photo(plane_x_m=-1.25);master=self.start_motion()
        for _ in range(2500):
            self.tick(state=4,flags=EXECUTING,lease_remaining_ms=4000)
        self.nav._renew_if_due(master,self.nav.latest)
        master.renew.assert_not_called()
        self.assertTrue(self.nav.stop_requested)
        self.assertIsNone(self.nav.lease_task)
        self.assertIn('预算',self.nav.dock['message'])

    def test_old_short_step_and_search_turn_do_not_renew(self):
        for role in ('step','search','align'):
            with self.subTest(role=role):
                self.setUp();self.begin(single=role=='step')
                self.photo(ready=role!='search',tag_id=None if role=='search' else 1,
                           plane_y_m=.15 if role=='align' else 0.)
                master=self.start_motion()
                self.tick(1.,state=4,flags=EXECUTING,lease_remaining_ms=4000)
                self.nav._renew_if_due(master,self.nav.latest)
                master.renew.assert_not_called()

    def test_single_step_ends_after_stop_without_rephotograph(self):
        self.begin(single=True);self.photo();self.start_motion();self.complete_motion()
        self.assertEqual(self.nav.dock['phase'],'step_complete')
        self.assertEqual(self.nav.camera.sequence,1)
        self.assertIsNone(self.nav.pending)

    def test_fault_loss_cancel_and_boot_cannot_resume(self):
        for reason in ('stale','drive','rx','manual','boot'):
            with self.subTest(reason=reason):
                self.setUp();self.begin();self.photo();self.start_motion()
                if reason=='stale': self.now+=.6
                elif reason=='drive': self.tick(first_drive_error=72)
                elif reason=='rx': self.tick(flags=RX_FAULT)
                elif reason=='boot': self.tick(boot=43)
                else: self.nav.request_stop()
                self.nav._dock_cycle()
                self.assertTrue(self.nav.stop_requested)
                self.assertIsNone(self.nav.lease_task)

    def test_old_firmware_append_rejection_never_starts(self):
        self.begin();self.photo()
        master=Mock()
        master.upload.side_effect=Rejected('old firmware',self.nav.latest)
        with self.assertRaises(Rejected):
            self.nav._upload_dock(master,self.nav.pending)
        master.start.assert_not_called()

    def test_wrong_id_even_if_other_camera_target_was_verified_keeps_searching(self):
        self.begin();self.photo(tag_id=2)
        self.assertEqual(self.nav.dock_motion,'search')
        self.assertIsNone(self.nav.dock_solution)

    def test_cancel_during_start_reply_does_not_restore_lease(self):
        self.begin();plan=self.photo()
        master=Mock()
        master.stop_revision=0
        master.request_stop.side_effect=lambda:setattr(master,'stop_revision',master.stop_revision+1)
        master.history_since.return_value=dict(cursor=0,exchanges=[],missed=0,connected=True,error=None)
        master.upload.return_value=[self.nav.latest]
        entered,release=Event(),Event()
        def start(task,revision):
            self.assertEqual(revision,0)
            entered.set();release.wait(2)
            return self.nav.latest
        master.start.side_effect=start
        self.nav.feedback=master
        worker=Thread(target=self.nav._upload_dock,args=(master,plan))
        worker.start();self.assertTrue(entered.wait(2))
        self.nav.request_stop();release.set();worker.join(2)
        self.assertFalse(worker.is_alive())
        self.assertEqual(master.stop_revision,1)
        self.assertIsNone(self.nav.lease_task)
        self.assertIsNone(self.nav.pending)

    def test_real_config_and_reference_http_interfaces(self):
        self.enable()
        server=ThreadingHTTPServer(('127.0.0.1',0),handler_for(self.nav))
        worker=Thread(target=server.serve_forever);worker.start()
        try:
            body=dict(map_id=self.grid.metadata()['map_id'],x=1.2,y=1.,yaw=.1)
            request=Request('http://127.0.0.1:{}/api/dock/config'.format(server.server_port),
                data=json.dumps(body).encode(),headers={'Content-Type':'application/json','Host':'127.0.0.1:8767'})
            with urlopen(request,timeout=2) as response:
                state=json.load(response)
            self.assertEqual({k:state['dock']['config'][k] for k in ('map_id','x','y')},
                             {k:body[k] for k in ('map_id','x','y')})
            self.assertAlmostEqual(state['dock']['config']['yaw'],body['yaw'])
            self.nav.dock_single_step_test=True
            self.nav.request_dock_step('move',-.03);self.stop_proof();self.photo()
            self.nav.camera.reference_frame=lambda photo_id:b'jpeg'
            with urlopen('http://127.0.0.1:{}/api/dock/reference.jpg?photo_id=1'.format(server.server_port),timeout=2) as response:
                self.assertEqual(response.read(),b'jpeg')
        finally:
            server.shutdown();server.server_close();worker.join(2)

    def test_mutual_exclusion_with_normal_navigation_config_and_zero(self):
        self.begin()
        for action in (lambda:self.nav.align(dict(x=0.,y=0.,yaw=0.),'manual'),
                       lambda:self.nav.configure_dock(dict(map_id=self.grid.metadata()['map_id'],x=0.,y=0.,yaw=0.)),
                       lambda:self.nav.request_start('wrong')):
            with self.assertRaises(ValueError): action()


if __name__=='__main__':
    unittest.main()
