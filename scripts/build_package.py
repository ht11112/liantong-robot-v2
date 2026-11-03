"""Build a standalone manual-mapping source package entirely offline.

No SSH, ROS, serial port, map loading, deployment or launch takes place here.
Default builds accept future source changes. Use --verify-r5-baseline only to
require the historical r5 source identity. Session metadata is optional and
requires every identity argument explicitly; it does not confirm an origin.
"""
import argparse
import ast
import datetime
import gzip
import hashlib
import io
import json
from pathlib import Path
import re
import tarfile
import uuid

from verify_source import ROOT, verify_r5_baseline


ENTRY_POINTS = ("manual_mapping_runtime.py", "manual_mapping_query.py",
                "test_manual_mapping_runtime.py", "test_manual_mapping_query.py")
DEPENDENCIES = ("control_client.py", "core.py", "rk_link.py", "transport.py")
FORBIDDEN = {"trial_navigation_runtime.py", "base_trial_adapter.py", "rk_localization_runtime.py",
             "rk_path_preview.py", "global_pose_proposal.py", "localization_ros_adapter.py"}


def sha256(raw):
    return hashlib.sha256(raw).hexdigest()


def selected_sources(app):
    """Follow local imports and add the four exact runtime-loaded host deps."""
    selected, pending = {}, list(ENTRY_POINTS)
    while pending:
        relative = pending.pop()
        if relative in selected:
            continue
        if relative in FORBIDDEN:
            raise ValueError("Manual mapping must not import a navigation lifecycle: " + relative)
        path = app / relative
        if path.is_symlink() or not path.is_file():
            raise ValueError("Required source is missing or symbolic: " + relative)
        raw = path.read_bytes()
        tree = ast.parse(raw.decode("utf-8-sig"), filename=relative, feature_version=(3, 8))
        selected[relative] = raw
        for node in ast.walk(tree):
            modules = ([node.module] if isinstance(node, ast.ImportFrom) and node.module
                       else [item.name for item in node.names] if isinstance(node, ast.Import) else [])
            for name in modules:
                local = name.split(".")[0] + ".py"
                if (app / local).is_file() and local not in selected:
                    pending.append(local)
    for name in DEPENDENCIES:
        relative = "deps/" + name
        path = app / relative
        if path.is_symlink() or not path.is_file():
            raise ValueError("Required host dependency is missing or symbolic: " + relative)
        selected[relative] = path.read_bytes()
        ast.parse(selected[relative].decode("utf-8-sig"), filename=relative, feature_version=(3, 8))
    return dict(sorted(selected.items()))


def archive_bytes(files):
    stream = io.BytesIO()
    with gzip.GzipFile(fileobj=stream, mode="wb", mtime=0) as compressed:
        with tarfile.open(fileobj=compressed, mode="w", format=tarfile.PAX_FORMAT) as archive:
            for relative, raw in sorted(files.items()):
                member = tarfile.TarInfo(relative)
                member.size, member.mode, member.mtime = len(raw), 0o644, 0
                archive.addfile(member, io.BytesIO(raw))
    return stream.getvalue()


def session_record(args):
    values = (args.session_nonce, args.expected_boot_id, args.remote_directory, args.duration_s)
    if all(value is None for value in values):
        return None
    if any(value is None for value in values):
        raise ValueError("Session metadata requires --session-nonce, --expected-boot-id, "
                         "--remote-directory and --duration-s together")
    if not re.fullmatch(r"[0-9a-f]{8}", args.session_nonce):
        raise ValueError("Session nonce must be eight lowercase hexadecimal characters")
    if str(uuid.UUID(args.expected_boot_id)) != args.expected_boot_id:
        raise ValueError("Expected boot ID must be a canonical UUID")
    if not re.fullmatch(r"/home/cat/p2p_navigation_20261103_manual_mapping_[A-Za-z0-9_]+", args.remote_directory):
        raise ValueError("Remote directory must match the recorder/viewer dedicated-session format")
    if not 1 <= args.duration_s <= 1200:
        raise ValueError("Duration must be 1..1200 seconds")
    return dict(schema="manual-mapping-session-input-v1", nonce=args.session_nonce,
                expected_boot_id=args.expected_boot_id, remote_directory=args.remote_directory,
                duration_s=args.duration_s, motion_requested=False, new_blank_map=True,
                serial_allowed_kinds=[2, 5], origin_recorded=False,
                origin_user_confirmation="not_confirmed_by_packaging", deployed=False,
                launched=False, device_state_checked=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True, help="New unique directory name beneath dist/")
    parser.add_argument("--verify-r5-baseline", action="store_true")
    parser.add_argument("--session-nonce")
    parser.add_argument("--expected-boot-id")
    parser.add_argument("--remote-directory")
    parser.add_argument("--duration-s", type=float)
    args = parser.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", args.name):
        parser.error("--name must be a single safe ASCII directory name")
    record = session_record(args)
    selected = selected_sources(ROOT / "app")
    if args.verify_r5_baseline:
        result = verify_r5_baseline()
        if result["failures"]:
            raise ValueError("r5 source baseline differs: " + "; ".join(result["failures"]))
        historical = json.loads((ROOT / "baselines/manual-mapping-r5.json").read_text(encoding="utf-8"))
        if set(selected) != set(historical["original_manifest"]["files"]):
            raise ValueError("Current manual-mapping import closure differs from the nine r5 files")
    manifest = dict(schema="manual-new-map-package-v1",
                    created_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    serial_allowed_kinds=[2, 5], motion_capability=False, new_blank_map=True,
                    geometry=dict(wheel_diameter_m=.2, wheel_separation_m=.3,
                                  laser_x_m=.13, laser_y_m=0., laser_z_m=.2, laser_yaw_rad=0., calibrated=False),
                    files={relative: sha256(raw) for relative, raw in selected.items()})
    package = dict(selected)
    package["manual_mapping_manifest.json"] = (json.dumps(manifest, indent=2) + "\n").encode("utf-8")
    raw_archive = archive_bytes(package)
    target = ROOT / "dist" / args.name
    if target.parent.is_symlink():
        raise ValueError("dist/ must not be a symbolic link")
    target.resolve().relative_to(ROOT.resolve())
    target.mkdir(parents=True, exist_ok=False)  # Refuse to overwrite any previous build.
    source_output = target / "package"
    source_output.mkdir()
    for relative, raw in package.items():
        path = source_output / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)
    archive_path = target / "manual-mapping.tar.gz"
    archive_path.write_bytes(raw_archive)
    result = dict(schema="manual-mapping-offline-build-v1", source_files=len(selected),
                  package=str(source_output.relative_to(ROOT)), archive=str(archive_path.relative_to(ROOT)),
                  sha256=sha256(raw_archive), verified_r5_sources=args.verify_r5_baseline,
                  launched=False, deployed=False, robot_connected=False)
    (target / "build-record.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    if record is not None:
        (target / "session-input.json").write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        result["session_input"] = str((target / "session-input.json").relative_to(ROOT))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
