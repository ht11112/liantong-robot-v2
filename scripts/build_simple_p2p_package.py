"""Build the independent simple point-to-point source package offline.

This reads source/configuration files and writes a new dist directory. It never
imports the robot runtime, connects to a robot, deploys or launches anything.
"""
import argparse
import ast
import datetime
import json
from pathlib import Path
import re

from build_package import archive_bytes, sha256
from verify_source import ROOT


ENTRY_POINTS = ("simple_p2p_runtime.py", "simple_p2p.py", "simple_p2p_base.py",
                "local_p2p_remote.py")
REMOTE_TEST = "test_local_p2p_remote.py"
DEPENDENCIES = tuple("deps/" + name + ".py" for name in
                     ("control_client", "core", "rk_link", "transport"))
CONFIGURATION = "config/simple-p2p.json"
FORBIDDEN_MODULES = {
    "trial_navigation_runtime", "base_trial_adapter", "rk_localization_runtime",
    "rk_path_preview", "global_pose_proposal", "localization_ros_adapter",
    "localization_anchor", "maploc_core", "amcl", "map_server", "firmware",
    "local_p2p_server", "test_local_p2p_server",
}


def allowed_source(relative):
    """Keep this package separate from mapping and historical navigation."""
    return (relative in DEPENDENCIES or relative in
            ("base_static_adapter.py", "local_p2p_remote.py", REMOTE_TEST)
            or "/" not in relative and relative.endswith(".py")
            and (relative.startswith("simple_p2p") or relative.startswith("test_simple")))


def local_modules(node):
    if isinstance(node, ast.Import):
        return [item.name for item in node.names]
    if isinstance(node, ast.ImportFrom):
        if node.level:
            raise ValueError("Relative imports are not supported by the flat source package")
        if node.module:
            return [node.module] + [node.module + "." + item.name for item in node.names
                                    if item.name != "*"]
    return []


def selected_sources(app):
    """Read the AST import closure, including runtime-loaded reviewed deps."""
    app = Path(app)
    tests = sorted(path.name for path in app.glob("test_simple*.py"))
    if not tests:
        raise ValueError("Simple point-to-point offline tests are missing")
    selected, pending = {}, list(ENTRY_POINTS) + list(DEPENDENCIES) + tests + [REMOTE_TEST]
    while pending:
        relative = pending.pop()
        if relative in selected:
            continue
        if not allowed_source(relative):
            raise ValueError("Source is outside the simple point-to-point package: " + relative)
        path = app / relative
        path.resolve().relative_to(app.resolve())
        if path.is_symlink() or not path.is_file():
            raise ValueError("Required source is missing or symbolic: " + relative)
        raw = path.read_bytes()
        tree = ast.parse(raw.decode("utf-8-sig"), filename=relative, feature_version=(3, 8))
        selected[relative] = raw
        for node in ast.walk(tree):
            for module in local_modules(node):
                module_root = module.split(".")[0]
                if (module_root in FORBIDDEN_MODULES
                        or module_root.startswith(("manual_mapping", "test_manual_mapping"))
                        or module_root == "test_mapping_viewer"):
                    raise ValueError("Historical mapping/navigation/firmware import is forbidden: " + module)
                name = module.replace(".", "/") + ".py"
                candidates = [name]
                if "/" not in name:
                    candidates.append("deps/" + name)
                for imported in candidates:
                    if (app / imported).is_file():
                        if imported not in selected:
                            pending.append(imported)
                        break
    return dict(sorted(selected.items()))


def configuration_bytes(root):
    path = root / CONFIGURATION
    path.resolve().relative_to(root.resolve())
    if path.is_symlink() or not path.is_file():
        raise ValueError("Simple point-to-point example configuration is missing or symbolic")
    raw = path.read_bytes()
    def invalid_constant(value):
        raise ValueError("Non-finite configuration value: " + value)
    document = json.loads(raw.decode("utf-8-sig"), parse_constant=invalid_constant)
    if not isinstance(document, dict):
        raise ValueError("Example configuration must be a JSON object")
    return raw


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", required=True, help="New unique directory name beneath dist/")
    args = parser.parse_args(argv)
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,79}", args.name):
        parser.error("--name must be a single safe ASCII directory name")
    selected = selected_sources(ROOT / "app")
    files = dict(selected)
    files[CONFIGURATION] = configuration_bytes(ROOT)
    manifest = dict(schema="simple-p2p-package-v1",
                    created_at=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                    entry_point="simple_p2p_runtime.py", configuration=CONFIGURATION,
                    remote_entry_point="local_p2p_remote.py",
                    motion_capability=True, launched=False, deployed=False,
                    robot_connected=False,
                    files={name: sha256(raw) for name, raw in sorted(files.items())})
    package = dict(files)
    package["simple_p2p_manifest.json"] = (json.dumps(manifest, indent=2) + "\n").encode("utf-8")
    raw_archive = archive_bytes(package)
    target = ROOT / "dist" / args.name
    if target.parent.is_symlink():
        raise ValueError("dist/ must not be a symbolic link")
    target.resolve().relative_to(ROOT.resolve())
    target.mkdir(parents=True, exist_ok=False)
    source_output = target / "package"
    source_output.mkdir()
    for relative, raw in package.items():
        path = source_output / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(raw)
    archive_path = target / "simple-p2p.tar.gz"
    archive_path.write_bytes(raw_archive)
    result = dict(schema="simple-p2p-offline-build-v1", source_files=len(selected),
                  files=len(files), configuration=CONFIGURATION,
                  package=str(source_output.relative_to(ROOT)),
                  archive=str(archive_path.relative_to(ROOT)), sha256=sha256(raw_archive),
                  motion_capability=True, launched=False, deployed=False, robot_connected=False)
    (target / "build-record.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
