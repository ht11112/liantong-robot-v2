"""Offline design tools only. There is no robot connection or motion command."""
import argparse
import json
import math
from pathlib import Path

from config import ConfigFormatError, load_config, validate_config
from task_model import TaskModel
from wheel_envelope import bounded_wheels_from_twist


def demo():
    # Synthetic fixture, intentionally independent of the incomplete site JSON.
    model = TaskModel(dict(map_id="simulation-only", connection_loss_action="continue_local_task",
                           task_timeout_s=30., stop_timeout_s=3., feedback_max_age_s=.75,
                           zero_speed_rpm=.2, zero_sample_count=3))
    rows = []

    def record(label, snapshot):
        rows.append(dict(event=label, state=snapshot["state"],
                         stop_confirmed=snapshot["stop_confirmed"]))

    task = model.submit("demo-request", dict(frame_id="map", map_id="simulation-only", x=.5, y=0.), 0.)
    task_id = task["task_id"]
    record("submit", task)
    record("prepared", model.prepared(task_id, .1))
    record("explicit_start", model.start(task_id, .2))
    record("PC_disconnected", model.ui_detached(task_id, 1.))
    task = model.complete(task_id, 2.)
    record("arrival_is_not_stopped", task)
    cycle = task["stop_cycle"]
    record("STOP_ACK_alone", model.stop_ack(task_id, cycle, 2.1))
    model.stop_status(task_id, cycle, True, 0, 0, 2.15, 2.2)
    for sample_id, stamp in enumerate((2.25, 2.3, 2.35), 1):
        task = model.zero_sample(task_id, cycle, sample_id, stamp, 0., 0., stamp + .01)
    record("fresh_stop_evidence", task)
    return dict(simulation_only=True, hardware_execution_authorized=False,
                site_configuration_used=False, events=rows,
                intents=model.drain_intents())


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    validate = sub.add_parser("validate", help="Check design inputs; never deploy")
    validate.add_argument("--config", default=str(Path(__file__).with_name("site_config.json")))
    validate.add_argument("--complete", action="store_true", help="Require all applicable choices")
    wheel = sub.add_parser("wheel-estimate", help="Estimate integer RPM; no device I/O")
    wheel.add_argument("--config", default=str(Path(__file__).with_name("site_config.json")))
    wheel.add_argument("--v", type=float, required=True, help="Requested m/s")
    wheel.add_argument("--w-deg", type=float, required=True, help="Requested degrees/s")
    sub.add_parser("demo", help="Run the synthetic in-memory task lifecycle")
    args = parser.parse_args(argv)
    try:
        if args.command == "demo":
            result, exit_code = demo(), 0
        else:
            config = load_config(args.config)
            result = validate_config(config, require_complete=getattr(args, "complete", False))
            exit_code = 2 if result["errors"] else 0
            if args.command == "wheel-estimate":
                if result["errors"]:
                    raise ValueError("Configuration has errors; validate it first")
                user, geometry = config["user"], config["geometry"]
                if user["max_linear_m_s"] is None or user["max_angular_deg_s"] is None:
                    raise ValueError("Speed limits must be explicitly filled before estimation")
                result = bounded_wheels_from_twist(
                    args.v, math.radians(args.w_deg), user["max_linear_m_s"],
                    user["max_angular_deg_s"], geometry["wheel_diameter_m"],
                    geometry["wheel_separation_m"])
    except (OSError, ValueError, ConfigFormatError) as exc:
        result, exit_code = dict(error=str(exc), hardware_execution_authorized=False), 2
    print(json.dumps(result, ensure_ascii=True, indent=2, allow_nan=False))
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
