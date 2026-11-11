"""Build an RK V12 navigation package from source and an external saved map."""

import argparse
import ast
import json
from pathlib import Path
import re
import sys

from build_package import archive_bytes, sha256
from verify_source import ROOT

sys.path.insert(0, str(ROOT / 'app'))
sys.path.insert(0, str(ROOT / 'mcu' / 'tools'))
from v11_map_plan import load_map


SOURCES = (
    'app/v12_feedback_worker.py', 'scripts/benchmark_v12_feedback.py',
    'app/v11_map_runtime.py', 'app/v11_map_plan.py', 'app/v11_ros.py', 'app/v11_obstacles.py',
    'app/v11_ekf.py', 'app/v11_ekf_worker.py',
    'app/v12_dock_camera.py', 'app/v12_dock_control.py',
    'app/manual_mapping_runtime.py', 'app/simple_p2p_history.py',
    'mcu/tools/mcu_v10.py', 'mcu/tools/mcu_v11.py',
)


def assemble(args):
    grid = load_map(args.map_pgm, args.map_yaml, args.map_sha256, args.yaml_sha256)
    files = {name: (ROOT / name).read_bytes() for name in SOURCES}
    for name, raw in files.items():
        ast.parse(raw.decode('utf-8-sig'), filename=name, feature_version=(3, 8))
    files['app/mf200_640x480.yaml'] = (ROOT / 'app/mf200_640x480.yaml').read_bytes()
    pgm, yaml = 'map/' + args.map_pgm.name, 'map/' + args.map_yaml.name
    files[pgm], files[yaml] = grid.pgm_bytes, args.map_yaml.read_bytes()
    metadata = grid.metadata()
    if sha256(files[yaml]) != metadata['yaml_sha256']:
        raise ValueError('Map YAML changed during packaging')
    manifest = dict(schema='v12-map-rk-package-1', entry_point='app/v11_map_runtime.py',
                    map=metadata, map_pgm=pgm, map_yaml=yaml, default_mode='ready',
                    dock_firmware_candidate_sha256=(sha256(args.dock_firmware_bin.read_bytes())
                                                    if getattr(args, 'dock_firmware_bin', None) else None),
                    files={name: sha256(raw) for name, raw in sorted(files.items())})
    files['manifest.json'] = (json.dumps(manifest, indent=2) + '\n').encode('utf-8')
    return files, manifest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', required=True)
    parser.add_argument('--map-yaml', required=True, type=Path)
    parser.add_argument('--map-pgm', required=True, type=Path)
    parser.add_argument('--map-sha256', required=True)
    parser.add_argument('--yaml-sha256', required=True)
    parser.add_argument('--dock-firmware-bin', type=Path, help='Offline docking firmware candidate to identify in the manifest')
    args = parser.parse_args(argv)
    if not re.fullmatch(r'[A-Za-z0-9][A-Za-z0-9_.-]{0,79}', args.name):
        parser.error('--name must be a single safe ASCII directory name')
    target = ROOT / 'dist' / args.name
    target.resolve().relative_to(ROOT.resolve())
    files, manifest = assemble(args)
    raw = archive_bytes(files)
    target.mkdir(parents=True, exist_ok=False)
    archive = target / 'v12-map-rk.tar.gz'
    archive.write_bytes(raw)
    (target / 'manifest.json').write_bytes(files['manifest.json'])
    print(json.dumps(dict(archive=str(archive), sha256=sha256(raw),
                          files=len(files), map_id=manifest['map']['map_id'])))


if __name__ == '__main__':
    main()
