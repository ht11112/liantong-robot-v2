"""Memory-only real deployment-tool tests; never invoke Programmer or devices."""
from datetime import datetime, timedelta, timezone
import importlib.util
import json
from pathlib import Path
import struct
import subprocess
import tempfile
import unittest
from unittest.mock import patch
import uuid

MODULE = Path(__file__).resolve().parents[1]/'tools/deploy_v10.py'
spec = importlib.util.spec_from_file_location('deployment_author_tests', MODULE)
d = importlib.util.module_from_spec(spec)
spec.loader.exec_module(d)
SN = '090038000500004E4E30594E'
UID = '03001d000451353433353535'
OPTR = 0xAA | (1 << 16) | (1 << 22) | (1 << 27)


def layout():
    fields, address = {}, d.RAM_BASE+16
    for obj, names in d.FIELDS.items():
        for name in names:
            key = obj+'.'+name
            size = 8 if name in ('boot_id', 'task_id') else 1 if key in ('drive.view.state', 'queue.state') else 4
            fields[key] = dict(address=address, size=size)
            address += size
    for name in ('uwTick', 'uart_error'):
        fields[name] = dict(address=address, size=4)
        address += 4
    return dict(address=d.RAM_BASE+16, size=address-(d.RAM_BASE+16), fields=fields)


class MemoryProgrammer:
    def __init__(self, folder, candidate):
        self.folder, self.candidate = Path(folder), candidate
        self.flash = bytes((i*17+29) & 255 for i in range(d.FLASH_SIZE))
        self.original = self.flash
        self.debug, self.optr, self.uid, self.idcode, self.capacity = 0x1801, OPTR, UID, 0x10000469, 512
        self.options = {key: d.REQUIRED_OPTIONS.get(key, 0) for key in d.OPTION_NAMES}
        self.calls, self.fail_label, self.tamper_label, self.tamper_kind = [], None, None, None
        self.current_label, self.mode, self.boot, self.tick = '', 'isolated', 1000, 5000
        self.lay = layout()
        self.reset_noop, self.reset_count, self.erase_count = False, 0, 0
        self.bad_empty, self.cleanup_reboot = False, False

    def fields(self):
        self.tick += 100
        values = {key: 0 for key in self.lay['fields']}
        values.update({'queue.boot_id': self.boot, 'queue.odom.epoch': 1, 'uwTick': self.tick,
                       'queue.odom.measurement_tick_ms': self.tick-10, 'queue.odom.sample_sequence': self.tick//50,
                       'wire.pub_sequence': self.tick//50, 'drive.view.state': 1 if self.mode == 'healthy' else 5})
        if self.mode == 'healthy':
            values.update({'drive.view.stop_verified': 1, 'drive.view.guard_valid': 1,
                           'queue.odom.initialized': 1, 'queue.odom.valid': 1})
        else:
            values.update({'drive.view.error': 72, 'queue.first_drive_error': 72,
                           'drive.sdo_ambiguous': 1, 'drive.tx_suppressed': 1})
        if self.bad_empty:
            values['queue.has_started'] = 1
        return values

    def ram(self):
        raw = bytearray(self.lay['size'])
        for key, value in self.fields().items():
            entry = self.lay['fields'][key]
            offset = entry['address']-self.lay['address']
            if key in ('queue.odom.x', 'queue.odom.y', 'queue.odom.yaw', 'queue.odom.travel'):
                raw[offset:offset+4] = struct.pack('<f', float(value))
            else:
                raw[offset:offset+entry['size']] = int(value).to_bytes(entry['size'], 'little')
        return bytes(raw)

    def __call__(self, command, **kwargs):
        label = next((p.stem.rsplit('_', 1)[0] for p in map(Path, command) if p.suffix == '.bin'), '')
        self.calls.append(command)
        if '-halt' in command:
            assert (self.folder/'program_in_progress.json').is_file()
        if '-e' in command:
            start, stop = command.index('-e')+1, command.index('-d')
            assert command[start:stop] == [str(n) for n in range(16)]
            assert (self.folder/'program_in_progress.json').is_file()
            self.flash = b'\xff'*d.APP_SIZE+self.flash[d.APP_SIZE:]
            self.erase_count += 1
        if '-d' in command:
            binary = Path(command[command.index('-d')+1]).read_bytes()
            assert len(binary) == d.APP_SIZE
            self.flash = binary+self.flash[len(binary):]
        if '-rst' in command:
            self.reset_count += 1
            if not self.reset_noop:
                self.boot += 1
        if '-w32' in command:
            pos = command.index('-w32')
            assert int(command[pos+1], 0) == d.DBG_FREEZE
            self.debug = int(command[pos+2], 0)
            if self.cleanup_reboot and not self.debug & d.IWDG_BIT:
                self.boot += 1
        options = self.options.copy()
        for index, item in enumerate(command):
            if item != '-u':
                continue
            address, size, path = int(command[index+1], 0), int(command[index+2], 0), Path(command[index+3])
            label = path.stem.rsplit('_', 1)[0]
            if address == d.FLASH_BASE:
                raw = self.flash
            elif address == 0x1FFF7590:
                raw = bytes.fromhex(self.uid)
            elif address == 0x1FFF75E0:
                raw = self.capacity.to_bytes(2, 'little')
            elif address == 0xE0042000:
                raw = self.idcode.to_bytes(4, 'little')
            elif address == 0x40022020:
                raw = self.optr.to_bytes(4, 'little')
            elif address == d.DBG_FREEZE:
                raw = self.debug.to_bytes(4, 'little')
            elif address == self.lay['address']:
                raw = self.ram()
            else:
                raise AssertionError('Unexpected target address')
            if self.tamper_label and path.name.startswith(self.tamper_label):
                if self.tamper_kind in ('prefix', 'padding', 'outside') and address == d.FLASH_BASE:
                    offset = {'prefix': 64, 'padding': d.APP_SIZE-1, 'outside': d.APP_SIZE+17}[self.tamper_kind]
                    changed = bytearray(raw);changed[offset] ^= 1;raw = bytes(changed)
                elif self.tamper_kind == 'uid' and address == 0x1FFF7590:
                    raw = b'\x12'*12
                elif self.tamper_kind == 'option':
                    options['BOR_LEV'] = 1
            path.write_bytes(raw[:size])
        output = ('ST-LINK SN : '+SN+'\nDevice ID : 0x469\n' +
                  '  Address : 0x08000000\n  Size : 0x80000\n  Bank : 0x00\n'
                  '  Address : 0x1FFF7590\n  Size : 0xC\n  Bank : 0x01\n' +
                  ''.join('    {} : 0x{:X}\n'.format(k, v) for k, v in options.items()) +
                  ('Download verified successfully\n' if '-d' in command else '')).encode()
        fail = self.fail_label and any(self.fail_label in str(c) for c in command)
        return subprocess.CompletedProcess(command, 1 if fail else 0, output, b'')


class DeploymentTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.base = Path(self.temp.name)
        self.image = self.base/'candidate.bin'
        raw = bytearray(b'\xa5'*d.IMAGE_SIZE)
        struct.pack_into('<II', raw, 0, d.RAM_BASE+d.RAM_SIZE, d.FLASH_BASE+0x101)
        self.image.write_bytes(raw)
        self.elf = self.base/'candidate.elf';self.elf.write_bytes(b'OFFLINE ELF FIXTURE')
        self.source_root = self.base/'sources';self.source_root.mkdir()
        sources = {}
        for i in range(17):
            name = 'source{}.c'.format(i);path = self.source_root/name;path.write_text('fixture '+str(i))
            sources[name] = d.sha(path.read_bytes())
        self.pins = patch.multiple(d, IMAGE_SHA=d.sha(raw), ELF_SHA=d.sha(self.elf.read_bytes()),
                                  SOURCE_SET_SHA=d.sha(json.dumps(sources, sort_keys=True, separators=(',', ':')).encode()))
        self.pins.start();self.addCleanup(self.pins.stop)
        self.build_receipt = self.base/'build.json'
        self.build_receipt.write_text(json.dumps(dict(passed=True, status='OFFLINE_ONLY_NOT_FLASHED', source_sha256=sources,
            firmware=dict(binary_sha256=d.IMAGE_SHA, elf_sha256=d.ELF_SHA, binary_bytes=d.IMAGE_SIZE))))
        self.plan = dict(image_path=str(self.image), image_sha256=d.IMAGE_SHA)
        self.session = str(uuid.uuid4())

    def tearDown(self):
        self.temp.cleanup()

    def action(self, name, peer=None, backup=None, deployment=None, mode='isolated'):
        folder = self.base/(name+'_'+str(len(list(self.base.iterdir()))))
        folder.mkdir()
        peer = peer or MemoryProgrammer(folder, self.image.read_bytes())
        peer.folder = folder
        target = d.Programmer(folder, 'not-a-real-programmer', SN, UID, runner=peer)
        report = dict(session_id=self.session, stlink_serial=SN, candidate=self.plan)
        result = d.perform(target, name, report, self.plan, backup, deployment, mode, peer.lay, wait=lambda _: None)
        return result, folder/'receipt.json', peer, target

    def test_backup_deploy_full_region_and_no_reset(self):
        saved, path, peer, _ = self.action('inspect-backup')
        self.assertEqual(saved['status'], 'BACKUP_VERIFIED')
        deployed, _, peer, target = self.action('deploy', peer, backup=path)
        self.assertEqual(deployed['status'], 'DEPLOYED_VERIFIED_NO_RESET')
        self.assertEqual(peer.flash[:d.IMAGE_SIZE], self.image.read_bytes())
        self.assertEqual(peer.flash[d.IMAGE_SIZE:d.APP_SIZE], b'\xff'*(d.APP_SIZE-d.IMAGE_SIZE))
        self.assertEqual(peer.flash[d.APP_SIZE:], peer.original[d.APP_SIZE:])
        self.assertEqual(peer.reset_count, 0)
        self.assertEqual(peer.erase_count, 1)
        last = target.calls[-1]['argv']
        self.assertIn('-w32', last);self.assertIn('-u', last)
        self.assertEqual(peer.debug, 0x801)

    def test_each_readback_corruption_locks_without_reset(self):
        for kind in ('prefix', 'padding', 'outside', 'uid', 'option'):
            with self.subTest(kind=kind):
                _, path, peer, _ = self.action('inspect-backup')
                peer.tamper_label, peer.tamper_kind = 'postwrite_first', kind
                report, receipt, peer, _ = self.action('deploy', peer, backup=path)
                self.assertIsNotNone(report['error'])
                self.assertEqual(peer.reset_count, 0)
                self.assertEqual(peer.erase_count, 1)
                self.assertTrue((receipt.parent/'FAILED_LOCK.json').is_file())

    def test_changed_third_read_never_erases(self):
        _, path, peer, _ = self.action('inspect-backup')
        peer.flash = b'\0'+peer.flash[1:]
        report, _, peer, _ = self.action('deploy', peer, backup=path)
        self.assertIsNotNone(report['error']);self.assertEqual(peer.erase_count, 0)

    def test_bad_identity_or_bank_never_writes(self):
        for key, value in (('uid', 'ff'*12), ('idcode', 0x450), ('capacity', 256), ('optr', OPTR & ~(1 << 22))):
            with self.subTest(key=key):
                folder = self.base/str(uuid.uuid4());folder.mkdir()
                peer = MemoryProgrammer(folder, self.image.read_bytes());setattr(peer, key, value)
                target = d.Programmer(folder, 'fake', SN, UID, runner=peer)
                report = d.perform(target, 'inspect-backup', dict(session_id=self.session, stlink_serial=SN), self.plan)
                self.assertIsNotNone(report['error'])
                if key != 'optr':
                    self.assertFalse(any('-w32' in call for call in peer.calls))
                else:
                    self.assertTrue(report['cleanup']['cleared'])
                    self.assertEqual(peer.erase_count, 0)

    def test_isolated_reset_then_healthy_reset_and_final_boot_binding(self):
        _, path, peer, _ = self.action('inspect-backup')
        _, deployed, peer, _ = self.action('deploy', peer, backup=path)
        isolated, _, peer, _ = self.action('reset-audit', peer, deployment=deployed)
        self.assertEqual(isolated['status'], 'STATIC_AUDIT_ISOLATED')
        peer.mode = 'healthy'
        healthy, _, peer, target = self.action('reset-audit', peer, deployment=deployed, mode='healthy')
        self.assertEqual(healthy['status'], 'STATIC_AUDIT_HEALTHY')
        self.assertEqual(healthy['cleanup']['final_fields']['queue.boot_id'], peer.boot)
        self.assertIn('cleanup_final_ram.bin', ' '.join(target.calls[-1]['argv']))

    def test_noop_reset_busy_queue_and_cleanup_reboot_refuse(self):
        for kind in ('noop', 'busy', 'cleanup_boot'):
            with self.subTest(kind=kind):
                _, path, peer, _ = self.action('inspect-backup')
                _, deployed, peer, _ = self.action('deploy', peer, backup=path)
                peer.mode = 'healthy'
                peer.reset_noop = kind == 'noop';peer.bad_empty = kind == 'busy';peer.cleanup_reboot = kind == 'cleanup_boot'
                report, _, peer, _ = self.action('reset-audit', peer, deployment=deployed, mode='healthy')
                self.assertIsNotNone(report['error'])
                self.assertEqual(peer.reset_count, 0 if kind == 'busy' else 1)

    def test_fresh_site_receipt_and_no_new_facts(self):
        value = dict(schema='mcu-v10-site-1', session_id=self.session, confirmed_at=datetime.now(timezone.utc).isoformat(),
                     operator_statement='Current stopped robot; power, CAN and RS485 isolated.', chassis_stopped=True,
                     no_rider=True, motor_power_disconnected=True, can_disconnected=True, rs485_disconnected=True, controller_safe_power=True)
        path = self.base/'site.json'
        path.write_text(json.dumps(value))
        self.assertEqual(d.site_receipt(path, 'isolated')[0]['session_id'], self.session)
        for delta in (-1900, 60):
            value['confirmed_at'] = (datetime.now(timezone.utc)+timedelta(seconds=delta)).isoformat();path.write_text(json.dumps(value))
            with self.assertRaises(ValueError):d.site_receipt(path, 'isolated')

    def test_local_pinned_release_and_default_never_programmer(self):
        args = ['inspect-backup', '--image', str(self.image), '--elf', str(self.elf), '--source-root', str(self.source_root),
                '--build-receipt', str(self.build_receipt), '--output-dir', str(self.base/'no_created'), '--stlink-serial', SN]
        with patch.object(d, 'Programmer', side_effect=AssertionError('Dry run must never construct Programmer')), patch('builtins.print'):
            self.assertEqual(d.main(args), 0)
        self.assertFalse((self.base/'no_created').exists())

    def test_source_and_binary_pin_changes_reject(self):
        d.local_plan(self.image, self.elf, self.build_receipt, self.source_root)
        (self.source_root/'source0.c').write_text('changed')
        with self.assertRaises(ValueError):
            d.local_plan(self.image, self.elf, self.build_receipt, self.source_root)
        self.image.write_bytes(b'wrong')
        with self.assertRaises(ValueError):
            d.local_plan(self.image, self.elf, self.build_receipt, self.source_root)

    def test_option_parser_ignores_upload_metadata_but_rejects_real_conflicts(self):
        options = ''.join('    {} : 0x{:X}\n'.format(k, d.REQUIRED_OPTIONS.get(k, 0)) for k in d.OPTION_NAMES)
        output = ' Address : 0x08000000\n Size : 0x80000\n Bank : 0x0\n Address : 0x20000000\n Size : 0x88C\n Bank : 0x1\n'+options
        self.assertEqual(len(d.parse_options(output)), 34)
        with self.assertRaises(ValueError):
            d.parse_options(output+' RDP : 0xBB\n')

    def test_option_failure_does_not_block_identity_only_freeze_cleanup(self):
        folder = self.base/'option_failure';folder.mkdir()
        peer = MemoryProgrammer(folder, self.image.read_bytes())
        peer.options.pop('RDP')
        target = d.Programmer(folder, 'fake', SN, UID, runner=peer)
        report = d.perform(target, 'inspect-backup', dict(session_id=self.session, stlink_serial=SN), self.plan)
        self.assertIsNotNone(report['error'])
        self.assertEqual(target.verified_uid, UID)
        self.assertTrue(report['cleanup']['cleared'])
        self.assertEqual(peer.debug, 0x801)
        self.assertEqual(peer.erase_count, 0)


if __name__ == '__main__':
    unittest.main()
