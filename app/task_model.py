"""Offline point-to-point task lifecycle. Python 3.8+, standard library only.

Every timestamp is in one caller-supplied monotonic clock domain. Intents are
descriptions for a future adapter, never device permission or motor commands.
There is deliberately no ROS, network, serial, ARM or SETPOINT implementation.
"""
from copy import deepcopy
import math
import uuid


class TaskError(ValueError):
    """Invalid input or an operation inconsistent with this model's state."""


TERMINAL_STATES = frozenset(("COMPLETED", "CANCELLED", "FAILED"))
CONNECTION_ACTIONS = frozenset(("continue_local_task", "pause", "cancel"))


def _number(value, name, positive=False):
    if type(value) not in (int, float):
        raise TaskError("%s must be a finite number" % name)
    try:
        number = float(value)
    except (OverflowError, ValueError):
        raise TaskError("%s must be a finite number" % name)
    if not math.isfinite(number):
        raise TaskError("%s must be a finite number" % name)
    if positive and number <= 0:
        raise TaskError("%s must be positive" % name)
    return number


class TaskModel:
    """One active task; explicit preparation/start and evidence-based stopping.

    Required config keys: map_id, connection_loss_action, task_timeout_s,
    stop_timeout_s, feedback_max_age_s, zero_speed_rpm, zero_sample_count.
    Other keys belong to future adapters and are not interpreted here.
    """

    def __init__(self, config):
        if not isinstance(config, dict):
            raise TaskError("config must be a dictionary")
        required = ("map_id", "connection_loss_action", "task_timeout_s",
                    "stop_timeout_s", "feedback_max_age_s", "zero_speed_rpm",
                    "zero_sample_count")
        missing = [key for key in required if key not in config]
        if missing:
            raise TaskError("Missing explicit configuration: " + ", ".join(missing))
        if not isinstance(config["map_id"], str) or not config["map_id"].strip():
            raise TaskError("map_id must be a nonempty string")
        if (not isinstance(config["connection_loss_action"], str) or
                config["connection_loss_action"] not in CONNECTION_ACTIONS):
            raise TaskError("connection_loss_action must be explicit")
        count = config["zero_sample_count"]
        if type(count) is not int or not 1 <= count <= 1000:
            raise TaskError("zero_sample_count must be an integer from 1 to 1000")
        self.config = {"map_id": config["map_id"],
                       "connection_loss_action": config["connection_loss_action"],
                       "zero_sample_count": count}
        for key in ("task_timeout_s", "stop_timeout_s", "feedback_max_age_s"):
            self.config[key] = _number(config[key], key, positive=True)
        self.config["zero_speed_rpm"] = _number(config["zero_speed_rpm"], "zero_speed_rpm")
        if self.config["zero_speed_rpm"] < 0:
            raise TaskError("zero_speed_rpm cannot be negative")
        self._tasks = {}
        self._requests = {}
        self._active_id = None
        self._last_now = None
        self._intents = []
        self.unconfirmed_stop_latched = False

    def _clock(self, now):
        now = _number(now, "now")
        if self._last_now is not None and now < self._last_now:
            raise TaskError("Monotonic clock moved backwards")
        self._last_now = now
        return now

    def _task(self, task_id):
        if not isinstance(task_id, str) or task_id not in self._tasks:
            raise TaskError("Unknown task_id")
        return self._tasks[task_id]

    def _goal(self, goal):
        if not isinstance(goal, dict):
            raise TaskError("goal must be a dictionary")
        if set(goal) - {"frame_id", "map_id", "x", "y", "yaw"}:
            raise TaskError("Unknown goal fields")
        if goal.get("frame_id") != "map":
            raise TaskError("goal.frame_id must be map")
        if goal.get("map_id") != self.config["map_id"]:
            raise TaskError("goal.map_id does not match the loaded map")
        if "x" not in goal or "y" not in goal:
            raise TaskError("goal requires x and y")
        yaw = goal.get("yaw")
        if yaw is not None:
            yaw = _number(yaw, "goal.yaw")
            if not -math.pi <= yaw <= math.pi:
                raise TaskError("goal.yaw must be radians in [-pi, pi]")
        return {"frame_id": "map", "map_id": goal["map_id"],
                "x": _number(goal["x"], "goal.x"),
                "y": _number(goal["y"], "goal.y"), "yaw": yaw}

    def _intent(self, task, kind, now):
        self._intents.append({"kind": kind, "task_id": task["task_id"],
                              "stop_cycle": task["stop_cycle"], "at": now,
                              "intent_only": True, "device_permit": False})

    def _view(self, task, **extra):
        view = {key: deepcopy(value) for key, value in task.items()
                if not key.startswith("_")}
        view["unconfirmed_stop_latched"] = self.unconfirmed_stop_latched
        view.update(extra)
        return view

    def snapshot(self, task_id=None):
        """Read a copy without advancing clocks or renewing evidence."""
        task_id = self._active_id if task_id is None else task_id
        return None if task_id is None else self._view(self._task(task_id))

    def drain_intents(self):
        intents, self._intents = self._intents, []
        return deepcopy(intents)

    def submit(self, request_id, goal, now):
        now = self._clock(now)
        if not isinstance(request_id, str) or not request_id.strip() or len(request_id) > 128:
            raise TaskError("request_id must be a nonempty string of at most 128 characters")
        goal = self._goal(goal)
        if request_id in self._requests:
            task = self._tasks[self._requests[request_id]]
            if task["goal"] != goal:
                raise TaskError("request_id already belongs to a different payload")
            return self._view(task, idempotent=True)
        if self.unconfirmed_stop_latched:
            raise TaskError("Unconfirmed stop is latched; no new task may start")
        if self._active_id is not None:
            raise TaskError("Another task is active")
        task_id = uuid.uuid4().hex
        task = {"task_id": task_id, "request_id": request_id, "goal": goal,
                "state": "PREPARING", "created_at": now, "updated_at": now,
                "started_at": None, "deadline": None, "reason": None,
                "first_failure": None, "stop_cycle": 0, "stop_confirmed": False,
                "stop_requested_at": None, "stop_deadline": None,
                "stop_outcome": None,
                "stop_evidence": {"ack_at": None, "status": None,
                                  "zero_samples": [], "confirmed_at": None},
                "_seen_samples": set(), "_last_sample_stamp": None,
                "_status_anchor_at": None, "_last_status_stamp": None}
        self._tasks[task_id] = task
        self._requests[request_id] = task_id
        self._active_id = task_id
        self._intent(task, "prepare", now)
        return self._view(task, idempotent=False)

    def prepared(self, task_id, now):
        now = self._clock(now)
        task = self._task(task_id)
        if task["state"] != "PREPARING":
            raise TaskError("Only a PREPARING task can become READY")
        task.update(state="READY", updated_at=now)
        return self._view(task)

    def start(self, task_id, now):
        now = self._clock(now)
        task = self._task(task_id)
        if task["state"] != "READY":
            raise TaskError("Explicit start requires READY")
        # A resumed task keeps its original total running deadline.
        if task["deadline"] is not None and now >= task["deadline"]:
            self._request_stop(task, "FAILED", "task_timeout", now, failure=True)
            return self._view(task)
        if task["started_at"] is None:
            task["started_at"] = now
            task["deadline"] = now + self.config["task_timeout_s"]
        task.update(state="RUNNING", updated_at=now, reason=None,
                    stop_confirmed=False)
        self._intent(task, "execute_goal", now)
        return self._view(task)

    def _request_stop(self, task, outcome, reason, now, failure=False):
        if task["state"] in TERMINAL_STATES:
            return
        if (task["state"] != "STOPPING" and task["deadline"] is not None and
                now >= task["deadline"] and not failure):
            outcome, reason, failure = "FAILED", "task_timeout", True
        if failure and task["first_failure"] is None:
            task["first_failure"] = reason
        if task["state"] == "STOPPING":
            priority = {"COMPLETED": 0, "PAUSED": 1, "CANCELLED": 2, "FAILED": 3}
            if priority[outcome] > priority[task["stop_outcome"]]:
                task["stop_outcome"] = outcome
                task["reason"] = task["first_failure"] or reason
                task["updated_at"] = now
            return
        task.update(state="STOPPING", updated_at=now, reason=reason,
                    stop_cycle=task["stop_cycle"] + 1, stop_confirmed=False,
                    stop_requested_at=now,
                    stop_deadline=now + self.config["stop_timeout_s"],
                    stop_outcome=outcome,
                    stop_evidence={"ack_at": None, "status": None,
                                   "zero_samples": [], "confirmed_at": None},
                    _seen_samples=set(), _last_sample_stamp=None,
                    _status_anchor_at=None, _last_status_stamp=None)
        self._intent(task, "cancel_goal", now)
        self._intent(task, "stop", now)

    def complete(self, task_id, now):
        now = self._clock(now)
        task = self._task(task_id)
        if task["state"] not in ("RUNNING", "STOPPING"):
            raise TaskError("Arrival is only valid during RUNNING or STOPPING")
        self._request_stop(task, "COMPLETED", "goal_reached", now)
        return self._view(task)

    def cancel(self, task_id, now, reason="operator_cancelled"):
        now = self._clock(now)
        task = self._task(task_id)
        self._request_stop(task, "CANCELLED", self._reason(reason), now)
        return self._view(task)

    def pause(self, task_id, now, reason="operator_paused"):
        now = self._clock(now)
        task = self._task(task_id)
        if task["state"] == "PAUSED":
            return self._view(task)
        self._request_stop(task, "PAUSED", self._reason(reason), now)
        return self._view(task)

    def resume(self, task_id, now):
        now = self._clock(now)
        task = self._task(task_id)
        if task["state"] != "PAUSED" or not task["stop_confirmed"]:
            raise TaskError("Resume requires a PAUSED task with confirmed stop")
        if task["deadline"] is not None and now >= task["deadline"]:
            self._request_stop(task, "FAILED", "task_timeout", now, failure=True)
        else:
            task.update(state="PREPARING", updated_at=now, reason=None)
            self._intent(task, "prepare", now)
        return self._view(task)

    @staticmethod
    def _reason(reason):
        if not isinstance(reason, str) or not reason.strip():
            raise TaskError("reason must be a nonempty string")
        return reason

    def health_fault(self, task_id, reason, now):
        now = self._clock(now)
        task = self._task(task_id)
        self._request_stop(task, "FAILED", self._reason(reason), now, failure=True)
        return self._view(task)

    def ui_detached(self, task_id, now):
        now = self._clock(now)
        task = self._task(task_id)
        action = self.config["connection_loss_action"]
        if action != "continue_local_task":
            self._request_stop(task, "PAUSED" if action == "pause" else "CANCELLED",
                               "ui_detached", now)
        return self._view(task, connection_loss_action=action)

    def _stopping(self, task_id, stop_cycle, now):
        task = self._task(task_id)
        if type(stop_cycle) is not int:
            raise TaskError("stop_cycle must be an integer")
        if task["state"] != "STOPPING" or stop_cycle != task["stop_cycle"]:
            return task, False
        if now >= task["stop_deadline"]:
            self._stop_failure(task, "stop_timeout", now)
            return task, False
        return task, True

    def stop_ack(self, task_id, stop_cycle, now):
        now = self._clock(now)
        task, valid = self._stopping(task_id, stop_cycle, now)
        if valid and task["stop_evidence"]["ack_at"] is None:
            task["stop_evidence"]["ack_at"] = now
            task["updated_at"] = now
        return self._view(task, evidence_accepted=valid)

    def _source(self, source_stamp, now):
        source_stamp = _number(source_stamp, "source_stamp")
        return source_stamp, 0 <= now - source_stamp <= self.config["feedback_max_age_s"]

    def stop_status(self, task_id, stop_cycle, disarmed, target_left_rpm,
                    target_right_rpm, source_stamp, now):
        now = self._clock(now)
        stamp, fresh = self._source(source_stamp, now)
        left = _number(target_left_rpm, "target_left_rpm")
        right = _number(target_right_rpm, "target_right_rpm")
        if type(disarmed) is not bool:
            raise TaskError("disarmed must be a boolean")
        task, valid = self._stopping(task_id, stop_cycle, now)
        evidence = task["stop_evidence"]
        ack = evidence["ack_at"]
        last_stamp = task["_last_status_stamp"]
        valid = (valid and fresh and ack is not None and stamp > ack and
                 (last_stamp is None or stamp > last_stamp))
        if not valid:
            return self._view(task, evidence_accepted=False)
        task["updated_at"] = now
        task["_last_status_stamp"] = stamp
        if not disarmed or left != 0 or right != 0:
            evidence["status"] = None
            evidence["zero_samples"] = []
            task["_status_anchor_at"] = None
            return self._view(task, evidence_accepted=False)
        evidence["status"] = {"source_stamp": stamp, "received_at": now,
                              "disarmed": True, "target_left_rpm": left,
                              "target_right_rpm": right}
        if task["_status_anchor_at"] is None:
            task["_status_anchor_at"] = now
        return self._view(task, evidence_accepted=True)

    def zero_sample(self, task_id, stop_cycle, sample_id, source_stamp,
                    left_rpm, right_rpm, now):
        now = self._clock(now)
        stamp, fresh = self._source(source_stamp, now)
        left = _number(left_rpm, "left_rpm")
        right = _number(right_rpm, "right_rpm")
        if type(sample_id) is not int or sample_id < 0:
            raise TaskError("sample_id must be a nonnegative integer")
        task, valid = self._stopping(task_id, stop_cycle, now)
        evidence = task["stop_evidence"]
        anchor = task["_status_anchor_at"]
        last = task["_last_sample_stamp"]
        valid = (valid and fresh and anchor is not None and stamp > anchor and
                 sample_id not in task["_seen_samples"] and
                 (last is None or stamp > last))
        if not valid:
            return self._view(task, evidence_accepted=False)
        task["_seen_samples"].add(sample_id)
        task["_last_sample_stamp"] = stamp
        task["updated_at"] = now
        evidence["zero_samples"] = [sample for sample in evidence["zero_samples"]
                                    if now - sample["source_stamp"] <=
                                    self.config["feedback_max_age_s"]]
        if max(abs(left), abs(right)) > self.config["zero_speed_rpm"]:
            evidence["zero_samples"] = []
        else:
            evidence["zero_samples"].append({"sample_id": sample_id,
                                               "source_stamp": stamp,
                                               "received_at": now,
                                               "left_rpm": left,
                                               "right_rpm": right})
        status = evidence["status"]
        if (len(evidence["zero_samples"]) >= self.config["zero_sample_count"] and
                status is not None and
                now - status["source_stamp"] <= self.config["feedback_max_age_s"]):
            task.update(state=task["stop_outcome"], stop_confirmed=True)
            evidence["confirmed_at"] = now
            if task["state"] in TERMINAL_STATES:
                self._active_id = None
        return self._view(task, evidence_accepted=True)

    def _stop_failure(self, task, reason, now):
        if task["first_failure"] is None:
            task["first_failure"] = reason
        task.update(state="FAILED", reason=task["first_failure"], updated_at=now,
                    stop_outcome="FAILED", stop_confirmed=False)
        task["stop_failure"] = reason
        self.unconfirmed_stop_latched = True
        self._active_id = None

    def stop_failed(self, task_id, stop_cycle, reason, now):
        now = self._clock(now)
        task, valid = self._stopping(task_id, stop_cycle, now)
        if valid:
            self._stop_failure(task, self._reason(reason), now)
        return self._view(task)

    def tick(self, now):
        now = self._clock(now)
        if self._active_id is None:
            return None
        task = self._tasks[self._active_id]
        if task["state"] == "STOPPING":
            if now >= task["stop_deadline"]:
                self._stop_failure(task, "stop_timeout", now)
        elif task["deadline"] is not None and now >= task["deadline"]:
            self._request_stop(task, "FAILED", "task_timeout", now, failure=True)
        return self._view(task)
