"""Offline package smoke check: synthetic map, archive contents and imports."""
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path, PurePosixPath
import subprocess
import sys
import tarfile
import tempfile
from types import SimpleNamespace
import unittest


ROOT = Path(__file__).resolve().parents[1]


def package_builder():
    previous = list(sys.path)
    try:
        sys.path.insert(0, str(ROOT / 'scripts'))
        spec = importlib.util.spec_from_file_location(
            '_v12_package_smoke_builder', ROOT / 'scripts' / 'build_v11_map_package.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path[:] = previous


class MapPackageTests(unittest.TestCase):
    def test_archive_is_complete_and_imports_without_checkout_or_devices(self):
        builder = package_builder()
        with tempfile.TemporaryDirectory(prefix='v12-package-') as temporary:
            folder = Path(temporary)
            pgm, yaml = folder / 'floor.pgm', folder / 'floor.yaml'
            pgm.write_bytes(b'P5\n20 20\n255\n' + bytes([254])*400)
            yaml.write_text('image: floor.pgm\nresolution: 0.1\norigin: [0, 0, 0]\n'
                            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n',
                            encoding='utf-8')
            sha = lambda raw: hashlib.sha256(raw).hexdigest()
            args = SimpleNamespace(map_pgm=pgm, map_yaml=yaml,
                                   map_sha256=sha(pgm.read_bytes()),
                                   yaml_sha256=sha(yaml.read_bytes()))
            files, manifest = builder.assemble(args)
            self.assertEqual(manifest['wire_version'], 12)
            self.assertIn('app/v11_obstacles.py', files)
            self.assertIn('app/v12_feedback_worker.py', files)
            self.assertIn('scripts/benchmark_v12_feedback.py', files)
            self.assertEqual(set(manifest['files']), set(files)-{'manifest.json'})
            for name, digest in manifest['files'].items():
                self.assertEqual(sha(files[name]), digest, name)
            self.assertEqual(manifest, json.loads(files['manifest.json']))

            destination = folder / 'unpacked'
            destination.mkdir()
            archive = builder.archive_bytes(files)
            with tarfile.open(fileobj=io.BytesIO(archive), mode='r:gz') as tar:
                members = tar.getmembers()
                self.assertEqual({member.name for member in members}, set(files))
                for member in members:
                    self.assertTrue(member.isfile(), member.name)
                    path = PurePosixPath(member.name)
                    self.assertFalse(path.is_absolute())
                    self.assertNotIn('..', path.parts)
                    output = destination.joinpath(*path.parts)
                    output.parent.mkdir(parents=True, exist_ok=True)
                    with tar.extractfile(member) as source:
                        raw = source.read()
                    self.assertEqual(raw, files[member.name])
                    output.write_bytes(raw)

            # Ignore inherited PYTHONPATH and user site packages. The runtime
            # must find its dependencies inside the extracted package itself.
            env = dict(os.environ, TMP=str(folder), TEMP=str(folder))
            (destination / 'spawn_smoke.py').write_text('''import sys
sys.path.insert(0, 'app')
from v12_feedback_worker import FeedbackWorker
from mcu_v11 import LinkError

def offline_port(device, timeout):
    raise OSError('offline package smoke; no device opened')

if __name__ == '__main__':
    worker = FeedbackWorker('unused', port_factory=offline_port)
    try:
        worker.launch()
        try:
            worker.discover()
        except LinkError as exc:
            assert 'offline package smoke' in str(exc), str(exc)
        else:
            raise AssertionError('Expected offline fixture failure')
        assert not worker.snapshot()['connected']
        print('packaged feedback spawn OK')
    finally:
        worker.close()
''', encoding='utf-8')
            commands = (
                [sys.executable, '-E', '-s', '-B', manifest['entry_point'], '--help'],
                [sys.executable, '-E', '-s', '-B', '-c',
                 "import sys; from pathlib import Path; sys.path.insert(0, 'app'); "
                 "import v11_ros, v11_obstacles, manual_mapping_runtime, simple_p2p_history, v12_feedback_worker; "
                 "modules=(v11_ros,v11_obstacles,manual_mapping_runtime,simple_p2p_history,v12_feedback_worker); "
                 "[Path(m.__file__).resolve().relative_to(Path.cwd()) for m in modules]; "
                 "assert 'rospy' not in sys.modules; print('offline imports OK')"],
                [sys.executable, '-E', '-s', '-B', 'spawn_smoke.py'],
                [sys.executable, '-E', '-s', '-B', 'scripts/benchmark_v12_feedback.py',
                 '--port', 'unused', '--output', 'must-not-exist'],
            )
            outputs = []
            for command in commands:
                result = subprocess.run(command, cwd=destination, env=env,
                                        capture_output=True, text=True,
                                        encoding='utf-8', errors='replace', timeout=30)
                self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
                outputs.append(result.stdout)
            self.assertIn('--obstacle-scan-timeout', outputs[0])
            self.assertIn('--continuous-amcl', outputs[0])
            self.assertIn('offline imports OK', outputs[1])
            self.assertIn('packaged feedback spawn OK', outputs[2])
            self.assertFalse((destination / 'must-not-exist').exists())


if __name__ == '__main__':
    unittest.main()
