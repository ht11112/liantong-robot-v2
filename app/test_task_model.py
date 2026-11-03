"""Offline lifecycle/failure injection; no hardware or ROS dependency."""
import math
import unittest

from task_model import TaskError, TaskModel


def policy(action="continue_local_task", **changes):
    # These are synthetic test values, not site configuration or motion approval.
    result = dict(map_id="synthetic-map", connection_loss_action=action,
                  task_timeout_s=20.0, stop_timeout_s=3.0,
                  feedback_max_age_s=0.5, zero_speed_rpm=0.2,
                  zero_sample_count=3)
    result.update(changes)
    return result


def goal(**changes):
    result = dict(frame_id="map", map_id="synthetic-map", x=1.0, y=2.0)
    result.update(changes)
    return result


class TaskModelTest(unittest.TestCase):
    def running(self, action="continue_local_task", **changes):
        model = TaskModel(policy(action, **changes))
        task_id = model.submit("request-1", goal(), 0.0)["task_id"]
        model.prepared(task_id, 0.1)
        model.start(task_id, 0.2)
        model.drain_intents()
        return model, task_id

    def stopping(self, action="continue_local_task", **changes):
        model, task_id = self.running(action, **changes)
        stopped = model.complete(task_id, 1.0)
        return model, task_id, stopped["stop_cycle"]

    def confirm(self, model, task_id, cycle, base=1.0):
        model.stop_ack(task_id, cycle, base + 0.01)
        model.stop_status(task_id, cycle, True, 0, 0, base + 0.02, base + 0.03)
        for index in range(3):
            result = model.zero_sample(task_id, cycle, index + 1,
                                       base + 0.04 + index * 0.05,
                                       0.0, 0.0, base + 0.05 + index * 0.05)
        return result

    def test_explicit_config_has_no_implicit_connection_policy(self):
        for key in policy():
            config = policy()
            del config[key]
            with self.subTest(key=key), self.assertRaises(TaskError):
                TaskModel(config)
        for bad in ("unknown", [], None):
            with self.subTest(action=bad), self.assertRaises(TaskError):
                TaskModel(policy(bad))

    def test_goal_coordinates_frame_map_and_radians_are_validated(self):
        for bad in (goal(x=float("nan")), goal(y=float("inf")), goal(x=True),
                    goal(x=10 ** 1000),
                    goal(frame_id="odom"), goal(map_id="different"),
                    goal(yaw=math.pi + 0.1), goal(yaw=float("nan")),
                    dict(x=1, y=2), goal(z=0)):
            with self.subTest(goal=bad), self.assertRaises(TaskError):
                TaskModel(policy()).submit("id", bad, 0)

    def test_one_active_task_and_request_payload_idempotency(self):
        model = TaskModel(policy())
        first = model.submit("request-1", goal(), 0)
        repeat = model.submit("request-1", goal(yaw=None), 0.1)
        self.assertEqual(first["task_id"], repeat["task_id"])
        self.assertTrue(repeat["idempotent"])
        self.assertEqual(first["created_at"], repeat["created_at"])
        self.assertEqual(len(model.drain_intents()), 1)
        with self.assertRaises(TaskError):
            model.submit("request-1", goal(x=3), 0.2)
        with self.assertRaises(TaskError):
            model.submit("request-2", goal(), 0.3)

    def test_terminal_replay_does_not_reexecute(self):
        model, task_id, cycle = self.stopping()
        self.confirm(model, task_id, cycle)
        model.drain_intents()
        again = model.submit("request-1", goal(), 2)
        self.assertEqual(again["task_id"], task_id)
        self.assertEqual(again["state"], "COMPLETED")
        self.assertEqual(model.drain_intents(), [])
        self.assertNotEqual(model.submit("request-2", goal(x=3), 2.1)["task_id"], task_id)

    def test_preparation_never_implicitly_starts(self):
        model = TaskModel(policy())
        task_id = model.submit("id", goal(), 0)["task_id"]
        with self.assertRaises(TaskError):
            model.start(task_id, 0.1)
        self.assertEqual(model.prepared(task_id, 0.2)["state"], "READY")
        self.assertEqual([item["kind"] for item in model.drain_intents()], ["prepare"])
        model.start(task_id, 0.3)
        intents = model.drain_intents()
        self.assertEqual([item["kind"] for item in intents], ["execute_goal"])
        self.assertTrue(all(item["intent_only"] and not item["device_permit"]
                            for item in intents))

    def test_detaching_ui_continues_only_under_explicit_local_policy(self):
        model, task_id = self.running()
        result = model.ui_detached(task_id, 1)
        self.assertEqual(result["state"], "RUNNING")
        self.assertEqual(result["deadline"], 20.2)
        self.assertEqual(model.drain_intents(), [])

    def test_detach_pause_and_cancel_both_stop_first(self):
        for action, expected in (("pause", "PAUSED"), ("cancel", "CANCELLED")):
            with self.subTest(action=action):
                model, task_id = self.running(action)
                result = model.ui_detached(task_id, 1)
                self.assertEqual(result["state"], "STOPPING")
                self.assertFalse(result["stop_confirmed"])
                self.assertEqual(self.confirm(model, task_id, result["stop_cycle"])["state"], expected)

    def test_critical_fault_preserves_first_failure_during_stop(self):
        model, task_id = self.running()
        first = model.health_fault(task_id, "wheel_feedback_expired", 1)
        second = model.health_fault(task_id, "planner_exited", 1.01)
        model.cancel(task_id, 1.02)
        self.assertEqual(first["stop_cycle"], second["stop_cycle"])
        self.assertEqual(second["first_failure"], "wheel_feedback_expired")
        final = self.confirm(model, task_id, first["stop_cycle"], 1.1)
        self.assertEqual(final["state"], "FAILED")
        self.assertEqual(final["reason"], "wheel_feedback_expired")
        self.assertTrue(final["stop_confirmed"])
        self.assertEqual([item["kind"] for item in model.drain_intents()], ["cancel_goal", "stop"])

    def test_arrival_requires_stop_status_and_actual_zero_samples(self):
        model, task_id, cycle = self.stopping()
        self.assertEqual(model.stop_ack(task_id, cycle, 1.01)["state"], "STOPPING")
        for sample_id in range(3):
            result = model.zero_sample(task_id, cycle, sample_id, 1.02, 0, 0, 1.03)
            self.assertFalse(result["evidence_accepted"])
        self.assertEqual(model.snapshot(task_id)["state"], "STOPPING")
        final = self.confirm(model, task_id, cycle, 1.1)
        self.assertEqual(final["state"], "COMPLETED")
        self.assertTrue(final["stop_confirmed"])

    def test_cancellation_during_arrival_stop_takes_precedence(self):
        model, task_id, cycle = self.stopping()
        result = model.cancel(task_id, 1.005)
        self.assertEqual(result["stop_cycle"], cycle)
        self.assertEqual(result["stop_outcome"], "CANCELLED")
        self.assertEqual(result["reason"], "operator_cancelled")
        model.complete(task_id, 1.006)
        self.assertEqual(self.confirm(model, task_id, cycle, 1.1)["state"], "CANCELLED")

    def test_stop_outcome_priority_failure_cancel_pause_completion(self):
        model, task_id, cycle = self.stopping()
        self.assertEqual(model.pause(task_id, 1.001)["stop_outcome"], "PAUSED")
        self.assertEqual(model.cancel(task_id, 1.002)["stop_outcome"], "CANCELLED")
        result = model.health_fault(task_id, "critical_fault", 1.003)
        self.assertEqual(result["stop_outcome"], "FAILED")
        self.assertEqual(model.cancel(task_id, 1.004)["stop_outcome"], "FAILED")
        self.assertEqual(self.confirm(model, task_id, cycle, 1.1)["reason"], "critical_fault")

    def test_stop_cycle_ack_and_status_cannot_be_replayed(self):
        model, task_id, cycle = self.stopping()
        self.assertFalse(model.stop_ack(task_id, cycle - 1, 1.01)["evidence_accepted"])
        self.assertFalse(model.stop_status(task_id, cycle, True, 0, 0, 1.02, 1.03)["evidence_accepted"])
        model.stop_ack(task_id, cycle, 1.04)
        self.assertFalse(model.stop_status(task_id, cycle, True, 0, 0, 1.03, 1.05)["evidence_accepted"])
        self.assertEqual(model.snapshot(task_id)["stop_evidence"]["ack_at"], 1.04)
        model.stop_ack(task_id, cycle, 1.06)
        self.assertEqual(model.snapshot(task_id)["stop_evidence"]["ack_at"], 1.04)

    def test_stale_future_duplicate_and_reordered_samples_never_refresh(self):
        model, task_id, cycle = self.stopping()
        model.stop_ack(task_id, cycle, 1.01)
        model.stop_status(task_id, cycle, True, 0, 0, 1.02, 1.03)
        first = model.zero_sample(task_id, cycle, 1, 1.04, 0, 0, 1.05)
        self.assertTrue(first["evidence_accepted"])
        cases = ((1, 1.06, 1.07), (2, 1.035, 1.08), (3, 2.0, 1.09),
                 (4, 1.1, 1.7))
        for sample_id, stamp, now in cases:
            result = model.zero_sample(task_id, cycle, sample_id, stamp, 0, 0, now)
            self.assertFalse(result["evidence_accepted"])
        samples = model.snapshot(task_id)["stop_evidence"]["zero_samples"]
        self.assertEqual(len(samples), 1)
        self.assertEqual(samples[0]["source_stamp"], 1.04)
        self.assertEqual(samples[0]["received_at"], 1.05)

    def test_nonzero_speed_breaks_zero_streak(self):
        model, task_id, cycle = self.stopping()
        model.stop_ack(task_id, cycle, 1.01)
        model.stop_status(task_id, cycle, True, 0, 0, 1.02, 1.03)
        model.zero_sample(task_id, cycle, 1, 1.04, 0, 0, 1.05)
        result = model.zero_sample(task_id, cycle, 2, 1.06, 0.21, 0, 1.07)
        self.assertEqual(result["stop_evidence"]["zero_samples"], [])
        for index in range(2):
            result = model.zero_sample(task_id, cycle, 3 + index,
                                       1.08 + 0.02 * index, 0, 0, 1.09 + 0.02 * index)
        self.assertEqual(result["state"], "STOPPING")

    def test_bad_status_invalidates_zero_evidence_and_older_status_is_rejected(self):
        model, task_id, cycle = self.stopping()
        model.stop_ack(task_id, cycle, 1.01)
        model.stop_status(task_id, cycle, True, 0, 0, 1.02, 1.03)
        model.zero_sample(task_id, cycle, 1, 1.04, 0, 0, 1.05)
        bad = model.stop_status(task_id, cycle, False, 1, 0, 1.06, 1.07)
        self.assertIsNone(bad["stop_evidence"]["status"])
        self.assertEqual(bad["stop_evidence"]["zero_samples"], [])
        old = model.stop_status(task_id, cycle, True, 0, 0, 1.05, 1.08)
        self.assertFalse(old["evidence_accepted"])
        self.assertFalse(model.zero_sample(task_id, cycle, 2, 1.09, 0, 0, 1.1)["evidence_accepted"])

    def test_zero_evidence_must_be_fresh_at_final_confirmation(self):
        model, task_id, cycle = self.stopping(feedback_max_age_s=0.1)
        model.stop_ack(task_id, cycle, 1.01)
        model.stop_status(task_id, cycle, True, 0, 0, 1.02, 1.03)
        model.zero_sample(task_id, cycle, 1, 1.04, 0, 0, 1.05)
        model.zero_sample(task_id, cycle, 2, 1.06, 0, 0, 1.07)
        result = model.zero_sample(task_id, cycle, 3, 1.18, 0, 0, 1.19)
        self.assertEqual(result["state"], "STOPPING")
        self.assertEqual(len(result["stop_evidence"]["zero_samples"]), 1)

    def test_timeout_and_late_arrival_fail_only_after_stop_evidence(self):
        model, task_id = self.running(task_timeout_s=1)
        result = model.complete(task_id, 1.3)
        self.assertEqual(result["state"], "STOPPING")
        self.assertEqual(result["reason"], "task_timeout")
        self.assertEqual(self.confirm(model, task_id, result["stop_cycle"], 1.3)["state"], "FAILED")
        model, task_id = self.running(task_timeout_s=1)
        self.assertEqual(model.tick(1.2)["state"], "STOPPING")

    def test_stop_failure_and_timeout_never_claim_stopped_or_allow_new_task(self):
        for event in ("failure", "timeout"):
            with self.subTest(event=event):
                model, task_id, cycle = self.stopping()
                result = (model.stop_failed(task_id, cycle, "serial_error", 1.1)
                          if event == "failure" else model.tick(4.0))
                self.assertEqual(result["state"], "FAILED")
                self.assertFalse(result["stop_confirmed"])
                self.assertTrue(result["unconfirmed_stop_latched"])
                with self.assertRaises(TaskError):
                    model.submit("new", goal(), 4.1)
                repeat = model.submit("request-1", goal(), 4.2)
                self.assertEqual(repeat["task_id"], task_id)
                self.assertEqual(model.stop_ack(task_id, cycle, 4.3)["state"], "FAILED")

    def test_pause_resume_reprepares_and_requires_another_explicit_start(self):
        model, task_id = self.running("pause")
        result = model.pause(task_id, 1)
        cycle = result["stop_cycle"]
        with self.assertRaises(TaskError):
            model.resume(task_id, 1.005)
        self.confirm(model, task_id, cycle)
        with self.assertRaises(TaskError):
            model.submit("another", goal(), 1.5)
        model.drain_intents()
        result = model.resume(task_id, 1.6)
        self.assertEqual(result["state"], "PREPARING")
        self.assertEqual([item["kind"] for item in model.drain_intents()], ["prepare"])
        model.prepared(task_id, 1.7)
        self.assertEqual(model.snapshot(task_id)["state"], "READY")
        model.start(task_id, 1.8)
        self.assertEqual(model.snapshot(task_id)["deadline"], 20.2)
        result = model.cancel(task_id, 1.9)
        self.assertEqual(result["stop_cycle"], cycle + 1)
        self.assertFalse(model.stop_ack(task_id, cycle, 2.0)["evidence_accepted"])

    def test_invalid_clock_rejected_and_snapshot_does_not_refresh(self):
        model, task_id = self.running()
        before = model.snapshot(task_id)
        for bad in (float("nan"), float("inf"), True, 0.1, 10 ** 1000):
            with self.subTest(now=bad), self.assertRaises(TaskError):
                model.tick(bad)
        self.assertEqual(before, model.snapshot(task_id))
        before["goal"]["x"] = 999
        self.assertEqual(model.snapshot(task_id)["goal"]["x"], 1)


if __name__ == "__main__":
    unittest.main()
