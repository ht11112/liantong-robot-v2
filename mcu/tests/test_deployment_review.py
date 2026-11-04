"""Independent temporary-file deployment fault probes; no hardware tool is run."""
from datetime import datetime, timezone
import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import types
import unittest
from unittest.mock import patch

MODULE = Path(__file__).resolve().parents[1] / 'tools' / 'deploy_v10.py'
spec = importlib.util.spec_from_file_location('reviewed_deploy_v10', MODULE)
deploy = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = deploy
spec.loader.exec_module(deploy)


def candidate_fixture(folder):
    """Synthetic vectors exercise pinning without depending on an ARM SDK/build."""
    image = bytearray((index * 17) & 255 for index in range(deploy.IMAGE_SIZE))
    struct.pack_into('<II', image, 0, deploy.RAM_BASE + deploy.RAM_SIZE, deploy.FLASH_BASE + 0x101)
    image_path, elf_path = folder/'candidate.bin', folder/'candidate.elf'
    image_path.write_bytes(image)
    elf_path.write_bytes(b'LOCAL MOCK ELF; never passed to GDB or hardware')
    root = folder/'source';root.mkdir()
    sources = {}
    for index in range(17):
        name = 'source_{}.c'.format(index)
        path = root/name;path.write_bytes(('source {}'.format(index)).encode())
        sources[name] = deploy.sha(path.read_bytes())
    expected_image, expected_elf = deploy.sha(image), deploy.sha(elf_path.read_bytes())
    receipt = dict(status='OFFLINE_ONLY_NOT_FLASHED', passed=True, source_sha256=sources,
                   firmware=dict(binary_sha256=expected_image, elf_sha256=expected_elf,
                                 binary_bytes=deploy.IMAGE_SIZE))
    receipt_path = folder/'build.json';receipt_path.write_text(json.dumps(receipt), encoding='utf-8')
    return image_path, elf_path, receipt_path, root, expected_image, expected_elf


def layout_fixture():
    fields = {}
    for obj, names in deploy.FIELDS.items():
        for name in names:
            key = obj+'.'+name
            width = (8 if name in ('boot_id', 'task_id') else 1 if name in (
                'view.state', 'view.stop_verified', 'view.guard_valid', 'sdo_ambiguous',
                'uart_failed', 'tx_suppressed', 'state', 'committed', 'has_started',
                'critical_latched', 'odom.initialized', 'odom.valid', 'transport_fault', 'has_request')
                else 2 if name in ('view.applied_left', 'view.applied_right', 'count', 'received', 'index',
                                  'odom.left_tenths_rpm', 'odom.right_tenths_rpm', 'rx_write', 'rx_read') else 4)
            fields[key] = dict(address=deploy.RAM_BASE+0x1000+len(fields)*8, size=width)
    for key, width in (('uwTick', 4), ('uart_error', 1)):
        fields[key] = dict(address=deploy.RAM_BASE+0x1000+len(fields)*8, size=width)
    address = min(row['address'] for row in fields.values())
    end = (max(row['address']+row['size'] for row in fields.values())+3)&~3
    return dict(fields=fields, address=address, size=end-address)


class MemoryTarget:
    """Duck-typed transport: models full files and command outcomes only."""
    def __init__(self, folder, plan, case='ok', mode='healthy', layout=None):
        self.folder, self.plan, self.case, self.mode = folder, plan, case, mode
        self.layout = layout or layout_fixture()
        self.calls, self.cleans, self.boot, self.tick = [], 0, 0x1020304050607080, 10000
        self.flash = bytes((index*13+7)&255 for index in range(deploy.FLASH_SIZE))
        self.uid = '1234567890abcdef12345678'
        self.options = {name: 0 for name in deploy.OPTION_NAMES}
        self.options.update(deploy.REQUIRED_OPTIONS)
        self.optr = 0xAA | 1<<16 | 1<<22 | 1<<27

    def ram(self):
        self.tick += 100
        values = {name: 0 for name in self.layout['fields']}
        values.update({'queue.boot_id': self.boot, 'queue.odom.epoch': 1,
                       'uwTick': self.tick, 'queue.odom.measurement_tick_ms': self.tick-20,
                       'queue.odom.sample_sequence': self.tick//50,
                       'drive.sample_sequence': self.tick//50, 'drive.speed_tick': self.tick-20,
                       'wire.pub_sequence': self.tick//50, 'wire.pub_tick_ms': self.tick-10})
        if self.mode == 'healthy':
            values.update({'drive.view.state': 1, 'drive.view.stop_verified': 1,
                           'drive.view.guard_valid': 1, 'queue.odom.initialized': 1,
                           'queue.odom.valid': 1})
        else:
            values.update({'drive.view.state': 5, 'drive.view.error': 72,
                           'drive.sdo_ambiguous': 1, 'drive.tx_suppressed': 1,
                           'queue.first_drive_error': 72})
        raw = bytearray(self.layout['size'])
        for name, row in self.layout['fields'].items():
            offset, value = row['address']-self.layout['address'], values[name]
            if name in ('queue.odom.x', 'queue.odom.y', 'queue.odom.yaw', 'queue.odom.travel'):
                struct.pack_into('<f', raw, offset, float(value))
            else:
                raw[offset:offset+row['size']] = int(value).to_bytes(row['size'], 'little')
        return bytes(raw)

    def metadata(self, label, full=False, ram=None):
        self.calls.append(dict(label=label, read_only=True))
        if self.case == 'read-error' and label == 'postwrite_first':
            raise RuntimeError('Independent readback failed')
        flash, uid, options, optr = self.flash, self.uid, dict(self.options), self.optr
        if label == 'backup_second' and self.case == 'unequal-backups':
            flash = bytes([flash[0]^1])+flash[1:]
        if label == 'prewrite_full' and self.case == 'changed-before-write':
            flash = flash[:40000]+bytes([flash[40000]^1])+flash[40001:]
        if label == 'postwrite_first':
            at = {'bad-prefix': 100, 'bad-padding': 32000, 'bad-tail': 40000}.get(self.case)
            if at is not None:
                flash = flash[:at]+bytes([flash[at]^1])+flash[at+1:]
            if self.case == 'uid-swap':uid = 'abcdef1234567890abcdef12'
            if self.case == 'options-swap':options['BOR_LEV'] ^= 1
            if self.case == 'optr-swap':optr ^= 0x100
        data = dict(uid=bytes.fromhex(uid), debug=struct.pack('<I', 0x1A55))
        if full:data['flash'] = flash
        if ram:data['ram'] = self.ram()
        files = {}
        for key, value in data.items():
            path = self.folder/(label+'_'+key+'.bin');path.write_bytes(value);files[key] = str(path)
        return dict(uid=uid, options=options, optr=optr, page_bytes=2048, files=files, data=data)

    def run(self, label, operations):
        self.calls.append(dict(label=label, argv=list(map(str, operations))))
        if label == 'erase_program_verify':
            begin, end = operations.index('-e')+1, operations.index('-d')
            assert list(map(int, operations[begin:end])) == list(range(16))
            padded = Path(operations[end+1]).read_bytes()
            assert len(padded) == 32768
            self.flash = padded+self.flash[32768:]
            return b'not a verification' if self.case == 'missing-verify' else b'Download verified successfully'
        if label == 'explicit_static_reset':
            if self.case != 'reset-noop':self.boot += 1
            self.tick = 2500
            return b'reset requested'
        raise AssertionError('Unexpected operation '+label)

    def cleanup(self):
        self.cleans += 1;self.calls.append(dict(label='cleanup'))
        if self.case == 'cleanup-error':raise RuntimeError('clear bit12 readback failed')
        return dict(cleared=True, after_value=0xA55, other_bits_preserved=True)


class DeploymentReview(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory();self.root = Path(self.temp.name)
        image, elf, receipt, sources, image_sha, elf_sha = candidate_fixture(self.root)
        self.patch_image = patch.object(deploy, 'IMAGE_SHA', image_sha)
        self.patch_elf = patch.object(deploy, 'ELF_SHA', elf_sha)
        pinned_sources = json.loads(receipt.read_text(encoding='utf-8'))['source_sha256']
        self.patch_sources = patch.object(deploy, 'SOURCE_SET_SHA', deploy.sha(
            json.dumps(pinned_sources, sort_keys=True, separators=(',', ':')).encode()))
        self.patch_image.start();self.patch_elf.start();self.patch_sources.start()
        self.plan = deploy.local_plan(image, elf, receipt, sources)

    def tearDown(self):
        self.patch_image.stop();self.patch_elf.stop();self.patch_sources.stop();self.temp.cleanup()

    def report(self):
        return dict(schema='mcu-v10-deployment-1', created_at=datetime.now(timezone.utc).isoformat(),
                    session_id='00000000-1111-2222-3333-444444444444',
                    stlink_serial='090038000500004E4E30594E', candidate=self.plan)

    def folder(self, name):
        result = self.root/name;result.mkdir();return result

    def backed(self, case='ok'):
        target = MemoryTarget(self.folder('backup'), self.plan)
        result = deploy.perform(target, 'inspect-backup', self.report(), self.plan)
        self.assertEqual(result['status'], 'BACKUP_VERIFIED')
        return target.folder/'receipt.json', target.flash

    def test_deploy_independent_full_readbacks_and_failure_boundaries(self):
        # Each failed deployment still performs one cleanup and never resets/retries.
        for case in ('ok', 'changed-before-write', 'bad-prefix', 'bad-padding',
                'bad-tail', 'uid-swap', 'options-swap', 'optr-swap', 'missing-verify', 'read-error', 'cleanup-error'):
            with self.subTest(case=case), tempfile.TemporaryDirectory() as directory:
                parent = Path(directory)
                backup_folder = parent/'backup';backup_folder.mkdir()
                backup_target = MemoryTarget(backup_folder, self.plan)
                deploy.perform(backup_target, 'inspect-backup', self.report(), self.plan)
                folder = parent/'deployment';folder.mkdir()
                target = MemoryTarget(folder, self.plan, case)
                result = deploy.perform(target, 'deploy', self.report(), self.plan, backup_path=backup_folder/'receipt.json')
                self.assertEqual(target.cleans, 1)
                labels = [call['label'] for call in target.calls]
                self.assertNotIn('explicit_static_reset', labels)
                self.assertLessEqual(labels.count('erase_program_verify'), 1)
                if case == 'ok':
                    self.assertEqual(result['status'], 'DEPLOYED_VERIFIED_NO_RESET')
                    self.assertTrue(result['outside_32KB_unchanged'])
                    self.assertFalse((folder/'FAILED_LOCK.json').exists())
                else:
                    self.assertEqual(result['status'], 'FAILED_KEEP_SITE_STATE')
                    self.assertTrue((folder/'FAILED_LOCK.json').is_file())
                    if case == 'changed-before-write':self.assertNotIn('erase_program_verify', labels)

    def test_unequal_dual_backups_are_not_accepted(self):
        target = MemoryTarget(self.folder('backup-fail'), self.plan, 'unequal-backups')
        result = deploy.perform(target, 'inspect-backup', self.report(), self.plan)
        self.assertEqual(result['status'], 'FAILED_KEEP_SITE_STATE')
        self.assertEqual(target.cleans, 1)
        self.assertFalse(any('argv' in call for call in target.calls))

    def test_noop_reset_cannot_certify_a_fresh_boot(self):
        backup_path, _ = self.backed()
        target = MemoryTarget(self.folder('program'), self.plan)
        deploy.perform(target, 'deploy', self.report(), self.plan, backup_path=backup_path)
        prior = target.folder/'receipt.json'
        for mode in ('healthy', 'isolated'):
            with self.subTest(mode=mode):
                fixture = MemoryTarget(self.folder('noop-'+mode), self.plan, 'reset-noop', mode)
                fixture.flash = target.flash
                result = deploy.perform(fixture, 'reset-audit', self.report(), self.plan,
                    deployment_path=prior, mode=mode, layout=fixture.layout, wait=lambda _: None)
                self.assertEqual(result['status'], 'FAILED_KEEP_SITE_STATE')
                self.assertIn('new MCU boot identity', result['error'])
                self.assertEqual([c['label'] for c in fixture.calls].count('explicit_static_reset'), 1)
                self.assertEqual(fixture.cleans, 1)

    def test_real_new_boot_static_audit_passes_in_both_physical_modes(self):
        backup_path, _ = self.backed()
        target = MemoryTarget(self.folder('program'), self.plan)
        deploy.perform(target, 'deploy', self.report(), self.plan, backup_path=backup_path)
        for mode in ('healthy', 'isolated'):
            with self.subTest(mode=mode):
                fixture = MemoryTarget(self.folder('new-boot-'+mode), self.plan, mode=mode)
                fixture.flash = target.flash
                result = deploy.perform(fixture, 'reset-audit', self.report(), self.plan,
                    deployment_path=target.folder/'receipt.json', mode=mode, layout=fixture.layout, wait=lambda _: None)
                self.assertEqual(result['status'], 'STATIC_AUDIT_'+mode.upper(), result['error'])
                self.assertNotEqual(result['previous_boot'], result['frames'][0]['fields']['queue.boot_id'])
                self.assertEqual(result['STOP_proof'], mode=='healthy')
                self.assertEqual([c['label'] for c in fixture.calls].count('explicit_static_reset'), 1)

    def test_local_pin_failure_precedes_hardware_constructor(self):
        raw = Path(self.plan['image_path']).read_bytes()
        Path(self.plan['image_path']).write_bytes(bytes([raw[0]^1])+raw[1:])
        with patch.object(deploy, 'Programmer', side_effect=AssertionError('hardware constructor')):
            with self.assertRaises(ValueError):
                deploy.local_plan(self.plan['image_path'], self.plan['elf_path'], self.plan['build_receipt'], self.root/'source')

    def test_elf_fields_are_derived_not_legacy_fixed_offsets(self):
        layout = layout_fixture()
        lines = ['FIELD {} {} {}'.format(name, row['address'], row['size']) for name, row in layout['fields'].items()]
        runner = lambda *args, **kwargs: types.SimpleNamespace(returncode=0, stdout=('\n'.join(lines)+'\n').encode(), stderr=b'')
        result = deploy.elf_layout(self.plan['elf_path'], 'NOT_EXECUTED_GDB', runner=runner)
        self.assertEqual(result, layout)
        self.assertEqual(result['fields']['drive.view.state']['size'], 1)
        self.assertGreater(result['address'], 0x20000400)

    def test_cleanup_only_clears_bit12_and_verifies_in_same_last_invocation(self):
        for corrupt in (False, True):
            with self.subTest(corrupt_readback=corrupt):
                calls = []
                sn = '090038000500004E4E30594E'
                def runner(command, **kwargs):
                    calls.append(command)
                    at = command.index('-w32')
                    self.assertEqual(command[at:at+3], ['-w32', hex(deploy.DBG_FREEZE), '0xa55'])
                    self.assertEqual(command[at+3:at+6], ['-u', hex(deploy.DBG_FREEZE), '4'])
                    self.assertFalse(any(value in command for value in ('-rst', '-halt', '-d', '-e')))
                    Path(command[at+6]).write_bytes(struct.pack('<I', 0xA54 if corrupt else 0xA55))
                    return types.SimpleNamespace(returncode=0, stdout=(sn+'\nDevice ID : 0x469\n').encode(), stderr=b'')
                target = deploy.Programmer(self.folder('cleanup-'+str(corrupt)), 'NOT_EXECUTED_PROGRAMMER', sn, runner=runner)
                target.verified_uid = '1234567890abcdef12345678'
                target.identity_read = lambda label: dict(data={'debug': struct.pack('<I', 0x1A55)})
                if corrupt:
                    with self.assertRaises(ValueError):target.cleanup()
                else:
                    result = target.cleanup()
                    self.assertTrue(result['cleared'] and result['other_bits_preserved'])
                    self.assertEqual(result['after_value'], 0xA55)
                self.assertEqual(len(calls), 1)  # No reconnect or retry after final clear/read.

    def test_real_multi_upload_log_shape_and_option_failure_cleanup(self):
        # Matches the real CLI's repeated Address/Bank preambles, without live data.
        options = {name: 0 for name in deploy.OPTION_NAMES}
        options.update(deploy.REQUIRED_OPTIONS)
        option_text = '\n'.join('     {} : {}'.format(name, hex(value)) for name, value in sorted(options.items()))
        preambles = ('\n  Address : 0x8000000\n  Bank : 0x00\n'
                     '\n  Address : 0x40022020\n  Bank : 0x01\n'
                     '\n  Address : 0x40022044\n  Bank : 0x02\n')
        parsed = deploy.parse_options((preambles+'OPTION BYTES BANK: 0\n'+option_text).encode()+b'\n\xff\xfe\n')
        self.assertEqual(parsed, options)
        self.assertNotIn('Address', parsed)
        with self.assertRaises(ValueError):deploy.parse_options(option_text+'\n     RDP : 0xBB\n')
        for bad_options in (False, True):
            with self.subTest(failed_option_parser=bad_options):
                calls, debug = [], [0x1800]
                uid = bytes.fromhex('1234567890abcdef12345678')
                sn = 'AB'*12
                memory = {0x1FFF7590: uid, 0x1FFF75E0: struct.pack('<H', 512),
                          0xE0042000: struct.pack('<I', 0x10000469),
                          0x40022020: struct.pack('<I', 0xAA | 1<<16 | 1<<22 | 1<<27)}
                def runner(command, **kwargs):
                    calls.append(command)
                    self.assertFalse(any(value in command for value in ('-rst', '-halt', '-d', '-e')))
                    for index, value in enumerate(command):
                        if value == '-w32':
                            self.assertEqual(int(command[index+1], 0), deploy.DBG_FREEZE)
                            debug[0] = int(command[index+2], 0)
                        elif value == '-u':
                            address, size = int(command[index+1], 0), int(command[index+2], 0)
                            raw = struct.pack('<I', debug[0]) if address == deploy.DBG_FREEZE else memory[address]
                            self.assertEqual(len(raw), size)
                            Path(command[index+3]).write_bytes(raw)
                    output = sn+'\nDevice ID : 0x469\n'+preambles+option_text
                    if bad_options and '-ob' in command:output += '\n     RDP : 0xBB\n'
                    return types.SimpleNamespace(returncode=0, stdout=output.encode(), stderr=b'')
                target = deploy.Programmer(self.folder('raw-identity-'+str(bad_options)),
                    'NOT_EXECUTED_PROGRAMMER', sn, expected_uid=uid.hex(), runner=runner)
                if bad_options:
                    with self.assertRaises(ValueError):target.metadata('initial')
                else:
                    self.assertEqual(target.metadata('initial')['uid'], uid.hex())
                self.assertEqual(target.verified_uid, uid.hex())
                result = target.cleanup()
                self.assertTrue(result['cleared'])
                self.assertEqual(debug[0], 0x800)
                self.assertEqual(sum('-w32' in command for command in calls), 1)
                self.assertIn('-w32', calls[-1])
                self.assertNotIn('-ob', calls[-2])  # Cleanup identity cannot fail on the OB parser again.


if __name__ == '__main__':
    unittest.main()
