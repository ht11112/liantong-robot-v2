"""Pinned v10 SWD deployment. Import/default CLI never opens a device.

Only the root operator uses --execute after recording this session's physical
facts. No automatic retry, rollback, reset, option-byte or application-RAM write.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import subprocess
import time
import uuid

REPO = Path(__file__).resolve().parents[2]
IMAGE_SHA = '820d0735b2ee43c4add6a1435532c4f9e2e1dab735cb5c9b54a846e50a0cf878'
ELF_SHA = '725b523e2746b6e0444305779ef4571de8172f8804ac765ff3f74948248a7f4b'
SOURCE_SET_SHA = '14f18a6e8399387923bce4d2da506d8f0f6a3c48766920019ec40719574025df'
IMAGE_SIZE, APP_SIZE, FLASH_SIZE = 29292, 32768, 524288
FLASH_BASE, RAM_BASE, RAM_SIZE = 0x08000000, 0x20000000, 98304
DBG_FREEZE, IWDG_BIT = 0xE0042008, 0x1000
ARM_BIN = Path('F:/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin')
PROGRAMMER = Path('F:/stm32cube/bin/STM32_Programmer_CLI.exe')
REQUIRED_OPTIONS = {'RDP': 0xAA, 'DBANK': 1, 'BFB2': 0, 'nSWBOOT0': 0, 'nBOOT0': 1, 'IWDG_SW': 1}
OPTION_NAMES = set(('RDP BOR_LEV nRST_STOP nRST_STDBY nRST_SHDW IWDG_SW IWDG_STOP IWDG_STDBY WWDG_SW BFB2 DBANK nBOOT1 SRAM_PE CCMSRAM_RST nSWBOOT0 nBOOT0 NRST_MODE IRHEN PCROP1_STRT PCROP1_END PCROP_RDP WRP1A_STRT WRP1A_END WRP1B_STRT WRP1B_END PCROP2_STRT PCROP2_END WRP2A_STRT WRP2A_END WRP2B_STRT WRP2B_END SEC_SIZE1 BOOT_LOCK SEC_SIZE2').split())
FIELDS = {
    'drive': 'view.state view.error view.applied_generation view.applied_tick_ms view.applied_left view.applied_right view.stop_verified view.guard_valid sdo_ambiguous uart_failed tx_suppressed first_can_error sample_sequence speed_tick'.split(),
    'queue': 'boot_id task_id state count received index committed has_started first_error first_drive_error critical_latched odom.initialized odom.valid odom.epoch odom.measurement_tick_ms odom.sample_sequence odom.left_tenths_rpm odom.right_tenths_rpm odom.x odom.y odom.yaw odom.travel'.split(),
    'wire': 'transport_fault rx_write rx_read last_request has_request pub_sequence pub_tick_ms'.split(),
}


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_name(path.name + '.' + uuid.uuid4().hex + '.tmp')
    raw = (json.dumps(value, indent=2, ensure_ascii=False, allow_nan=False) + '\n').encode('utf-8')
    with temporary.open('xb') as stream:
        stream.write(raw)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(str(temporary), str(path))


def load_json(path):
    raw = Path(path).read_bytes()
    if len(raw) > 4 * 1024 * 1024:
        raise ValueError('Receipt exceeds bounded length')
    return json.loads(raw.decode('utf-8-sig')), sha(raw)


def local_plan(image, elf, build_receipt, source_root=REPO):
    image, elf, build_receipt = map(Path, (image, elf, build_receipt))
    raw, executable = image.read_bytes(), elf.read_bytes()
    if len(raw) != IMAGE_SIZE or sha(raw) != IMAGE_SHA or sha(executable) != ELF_SHA:
        raise ValueError('Pinned v10 BIN/ELF differs')
    receipt, digest = load_json(build_receipt)
    if receipt.get('passed') is not True or receipt.get('status') != 'OFFLINE_ONLY_NOT_FLASHED':
        raise ValueError('Successful offline build receipt required')
    firmware = receipt.get('firmware', {})
    if firmware.get('binary_sha256') != IMAGE_SHA or firmware.get('elf_sha256') != ELF_SHA or firmware.get('binary_bytes') != IMAGE_SIZE:
        raise ValueError('Build receipt does not bind pinned BIN/ELF')
    sources = receipt.get('source_sha256', {})
    if len(sources) != 17 or sha(json.dumps(sources, sort_keys=True, separators=(',', ':')).encode()) != SOURCE_SET_SHA:
        raise ValueError('Original 17-source release identity required')
    for name, expected in sources.items():
        path = (Path(source_root) / name).resolve()
        if Path(source_root).resolve() not in path.parents or sha(path.read_bytes()) != expected:
            raise ValueError('Release source changed: ' + name)
    sp, pc = struct.unpack_from('<II', raw)
    if not RAM_BASE < sp <= RAM_BASE + RAM_SIZE or not pc & 1 or not FLASH_BASE <= pc - 1 < FLASH_BASE + IMAGE_SIZE:
        raise ValueError('Unexpected initial vector table')
    return dict(image_path=str(image.resolve()), elf_path=str(elf.resolve()), image_sha256=IMAGE_SHA,
                elf_sha256=ELF_SHA, image_bytes=len(raw), build_receipt=str(build_receipt.resolve()),
                build_receipt_sha256=digest, source_sha256=sources, application_bytes=APP_SIZE,
                erase_pages=list(range(16)), supported_device_id='0x469', supported_flash_kb=512,
                supported_DBANK=1, page_bytes=2048, padded_sha256=sha(raw + b'\xff' * (APP_SIZE-len(raw))))


def site_receipt(path, mode, now=None):
    value, digest = load_json(path)
    if value.get('schema') != 'mcu-v10-site-1':
        raise ValueError('Fresh mcu-v10-site-1 receipt required')
    uuid.UUID(value['session_id'])
    at = datetime.fromisoformat(value['confirmed_at'].replace('Z', '+00:00'))
    if at.tzinfo is None:
        raise ValueError('Confirmation must have a timezone')
    age = ((now or datetime.now(timezone.utc)) - at).total_seconds()
    if not 0 <= age <= 1800:
        raise ValueError('Site confirmation is future or older than 30 minutes')
    if not isinstance(value.get('operator_statement'), str) or not value['operator_statement'].strip():
        raise ValueError('Actual operator statement required')
    required = ['chassis_stopped', 'no_rider']
    required += (['motor_power_disconnected', 'can_disconnected', 'rs485_disconnected', 'controller_safe_power']
                 if mode == 'isolated' else ['can_connected', 'driver_power_stable', 'rs485_connected', 'no_control_owner'])
    if any(value.get(key) is not True for key in required):
        raise ValueError('Missing explicit current site fact: ' + ', '.join(required))
    return value, digest


def parse_options(output):
    text = output.decode('utf-8', errors='replace') if isinstance(output, bytes) else output
    options = {}
    for name, value in re.findall(r'^\s+([A-Za-z][A-Za-z0-9_]*)\s*:\s*(0x[0-9a-fA-F]+)\b', text, re.M):
        if name not in OPTION_NAMES:
            continue  # Multi-upload CLI output repeats Address/Size/Bank metadata.
        if name in options and options[name] != int(value, 16):
            raise ValueError('Conflicting option-byte fields')
        options[name] = int(value, 16)
    if not OPTION_NAMES.issubset(options) or any(options.get(k) != v for k, v in REQUIRED_OPTIONS.items()):
        raise ValueError('Incomplete or unsupported bank/boot/security/watchdog options')
    return options


class Programmer:
    """Injected runner permits full offline tests. No subprocess is run at import."""
    def __init__(self, folder, programmer, serial, expected_uid=None, runner=None):
        if not re.fullmatch(r'[0-9A-Fa-f]{24}', serial):
            raise ValueError('Explicit 24-digit ST-LINK serial required')
        self.folder, self.programmer, self.serial = Path(folder), Path(programmer), serial.upper()
        self.expected_uid = expected_uid.lower() if expected_uid else None
        if self.expected_uid and not re.fullmatch('[0-9a-f]{24}', self.expected_uid):
            raise ValueError('UID must contain 12 bytes')
        self.runner = runner or subprocess.run
        self.calls, self.verified_uid = [], None
        self.final_layout = self.final_expected = self.final_mode = None

    def run(self, label, operations):
        command = [str(self.programmer), '-c', 'port=SWD', 'mode=HOTPLUG', 'freq=400', 'sn='+self.serial] + list(map(str, operations))
        self.calls.append(dict(label=label, argv=command))
        result = self.runner(command, capture_output=True, timeout=90)
        output = result.stdout + result.stderr
        (self.folder/(label+'.log')).write_bytes(output)
        if result.returncode or self.serial.encode() not in output or not re.search(rb'Device ID\s*:\s*0x469\b', output):
            raise RuntimeError('Programmer/target identity failed: ' + label)
        return output

    def metadata(self, label, full=False, ram=None):
        names = ['uid', 'flashsize', 'idcode', 'optr', 'debug'] + (['flash'] if full else []) + (['ram'] if ram else [])
        paths = {n: self.folder/(label+'_'+n+'.bin') for n in names}
        specs = [('uid', 0x1FFF7590, 12), ('flashsize', 0x1FFF75E0, 2), ('idcode', 0xE0042000, 4),
                 ('optr', 0x40022020, 4), ('debug', DBG_FREEZE, 4)]
        if full:
            specs.insert(0, ('flash', FLASH_BASE, FLASH_SIZE))
        if ram:
            specs.insert(0, ('ram', ram[0], ram[1]))
        operations = []
        for name, address, size in specs:
            operations += ['-u', hex(address), hex(size), str(paths[name])]
        output = self.run(label, operations + ['-ob', 'displ'])
        data = {name: path.read_bytes() for name, path in paths.items()}
        if any(len(data[n]) != count for n, _, count in specs):
            raise ValueError('Incomplete target readback')
        uid = data['uid'].hex()
        if uid in ('00'*12, 'ff'*12) or (self.expected_uid and uid != self.expected_uid) or (self.verified_uid and uid != self.verified_uid):
            raise ValueError('UID changed or differs from the intended target')
        if int.from_bytes(data['idcode'], 'little') & 0xFFF != 0x469 or int.from_bytes(data['flashsize'], 'little') != 512:
            raise ValueError('Unexpected raw MCU ID/capacity')
        # Identity permits only watchdog-debug cleanup, even when a later
        # option parser/configuration check fails. It never permits Flash writes.
        self.verified_uid = uid
        options = parse_options(output)
        optr = int.from_bytes(data['optr'], 'little')
        decoded = dict(RDP=optr & 255, DBANK=(optr >> 22) & 1, BFB2=(optr >> 20) & 1,
                       nSWBOOT0=(optr >> 26) & 1, nBOOT0=(optr >> 27) & 1, IWDG_SW=(optr >> 16) & 1)
        if decoded != REQUIRED_OPTIONS:
            raise ValueError('Raw OPTR differs from required page/boot/security configuration')
        self.verified_uid = uid
        return dict(uid=uid, options=options, optr=optr, page_bytes=2048,
                    files={n: str(p.resolve()) for n, p in paths.items()}, data=data)

    def identity_read(self, label='identity_only'):
        """Minimal fresh identity+DBG read; deliberately independent of OB parsing."""
        specs = [('uid', 0x1FFF7590, 12), ('flashsize', 0x1FFF75E0, 2),
                 ('idcode', 0xE0042000, 4), ('debug', DBG_FREEZE, 4)]
        paths, operations = {}, []
        for name, address, count in specs:
            paths[name] = self.folder/(label+'_'+name+'.bin')
            operations += ['-u', hex(address), hex(count), str(paths[name])]
        self.run(label, operations)
        data = {name: path.read_bytes() for name, path in paths.items()}
        if any(len(data[name]) != count for name, _, count in specs):
            raise ValueError('Incomplete cleanup identity read')
        uid = data['uid'].hex()
        if (uid in ('00'*12, 'ff'*12) or self.expected_uid and uid != self.expected_uid or
                self.verified_uid and uid != self.verified_uid):
            raise ValueError('Cleanup UID differs from verified target')
        if int.from_bytes(data['idcode'], 'little') & 0xFFF != 0x469 or int.from_bytes(data['flashsize'], 'little') != 512:
            raise ValueError('Cleanup raw MCU ID/capacity differs')
        self.verified_uid = uid
        return dict(uid=uid, data=data, files={n: str(p.resolve()) for n, p in paths.items()})

    def cleanup(self):
        before = self.identity_read('cleanup_identity')
        value = int.from_bytes(before['data']['debug'], 'little') & ~IWDG_BIT
        after = self.folder/'cleanup_debug_after.bin'
        operations = ['-w32', hex(DBG_FREEZE), hex(value), '-u', hex(DBG_FREEZE), '4', str(after)]
        final_ram = self.folder/'cleanup_final_ram.bin'
        if self.final_layout:
            operations += ['-u', hex(self.final_layout['address']), hex(self.final_layout['size']), str(final_ram)]
        self.run('cleanup_clear_and_readback', operations)
        raw = after.read_bytes()
        if len(raw) != 4 or int.from_bytes(raw, 'little') != value or value & IWDG_BIT:
            raise ValueError('IWDG debug freeze was not cleared and read back')
        result = dict(cleared=True, after_value=value, other_bits_preserved=True, no_later_target_connection=True)
        if self.final_layout:
            fields = decode_ram(final_ram.read_bytes(), self.final_layout)
            audit_fields(fields, self.final_mode)
            if any(fields[k] != self.final_expected[k] for k in ('queue.boot_id', 'queue.odom.epoch')):
                raise ValueError('Boot/epoch changed during final freeze cleanup; re-assess without automatic reset')
            result.update(final_fields=fields, final_ram_file=str(final_ram.resolve()))
        return result


def public_snapshot(snapshot):
    return {k: v for k, v in snapshot.items() if k != 'data'}


def same_identity(first, second):
    if any(first[k] != second[k] for k in ('uid', 'options', 'optr', 'page_bytes')):
        raise ValueError('Target identity/options changed')


def backup_action(target, report):
    first, second = target.metadata('backup_first', full=True), target.metadata('backup_second', full=True)
    same_identity(first, second)
    if first['data']['flash'] != second['data']['flash']:
        raise ValueError('Fresh independent full backups differ')
    report.update(status='BACKUP_VERIFIED', snapshots=[public_snapshot(first), public_snapshot(second)],
                  uid=first['uid'], options=first['options'], optr=first['optr'], page_bytes=first['page_bytes'],
                  full_flash_sha256=sha(first['data']['flash']), independent_full_reads_equal=True,
                  previous_application_32KB_sha256=sha(first['data']['flash'][:APP_SIZE]),
                  matches_historical_v9_diagnostic_prefix=(sha(first['data']['flash'][:19876]) ==
                      'd624f76fd507475e52f41ae3bc843f28663994e717d84c055e0662069540fc77'))


def backup_evidence(path, report, plan):
    saved, digest = load_json(path)
    if saved.get('status') != 'BACKUP_VERIFIED' or saved.get('action') != 'inspect-backup' or saved.get('error') is not None:
        raise ValueError('Successful fresh backup receipt required')
    if saved.get('session_id') != report['session_id'] or saved.get('stlink_serial') != report['stlink_serial']:
        raise ValueError('Backup belongs to another site session or programmer')
    if saved.get('candidate', {}).get('image_sha256') != plan['image_sha256'] or saved.get('cleanup', {}).get('cleared') is not True:
        raise ValueError('Backup candidate/cleanup identity differs')
    first, second = saved['snapshots']
    one, two = Path(first['files']['flash']).read_bytes(), Path(second['files']['flash']).read_bytes()
    if len(one) != FLASH_SIZE or one != two or sha(one) != saved['full_flash_sha256']:
        raise ValueError('Fresh backup files changed')
    return saved, digest, one


def deploy_action(target, report, plan, backup_path):
    backup, digest, original = backup_evidence(backup_path, report, plan)
    report.update(backup_receipt=str(Path(backup_path).resolve()), backup_receipt_sha256=digest)
    before = target.metadata('prewrite_full', full=True)
    if before['uid'] != backup['uid'] or before['options'] != backup['options'] or before['optr'] != backup['optr'] or before['data']['flash'] != original:
        raise ValueError('Target differs from this round\'s fresh independent backup')
    raw = Path(plan['image_path']).read_bytes()
    if sha(raw) != IMAGE_SHA:
        raise ValueError('Candidate changed immediately before programming')
    padded = raw + b'\xff' * (APP_SIZE-len(raw))
    binary = target.folder/'candidate_padded_32768.bin'
    binary.write_bytes(padded)
    expected = padded + original[APP_SIZE:]
    report.update(status='IN_PROGRESS_KEEP_ISOLATED', flash_write_attempted=True,
                  erase_pages=list(range(16)), programmed_range=[hex(FLASH_BASE), hex(FLASH_BASE+APP_SIZE)],
                  padded_sha256=sha(padded), expected_full_sha256=sha(expected), operation_counts_final=False)
    atomic_json(target.folder/'program_in_progress.json', report)
    freeze = int.from_bytes(before['data']['debug'], 'little') | IWDG_BIT
    output = target.run('erase_program_verify', ['-w32', hex(DBG_FREEZE), hex(freeze), '-halt', '-e'] +
                        [str(i) for i in range(16)] + ['-d', str(binary), hex(FLASH_BASE), '-v'])
    if b'Download verified successfully' not in output:
        raise ValueError('Programmer verify marker missing')
    reads = []
    for label in ('postwrite_first', 'postwrite_second'):
        snapshot = target.metadata(label, full=True)
        same_identity(before, snapshot)
        if snapshot['data']['flash'] != expected:
            raise ValueError('Independent readback differs in prefix, erased padding or preserved region: ' + label)
        reads.append(public_snapshot(snapshot))
    report.update(status='DEPLOYED_VERIFIED_NO_RESET', uid=before['uid'], options=before['options'], optr=before['optr'],
                  verified_full_sha256=sha(expected), outside_32KB_unchanged=True, reset_commands=0,
                  postwrite_reads=reads, operation_counts_final=True)


def elf_layout(elf, gdb=ARM_BIN/'arm-none-eabi-gdb.exe', runner=subprocess.run):
    command = [str(gdb), '-nx', '-batch', '-ex', 'set auto-load off', '-ex', 'file "'+Path(elf).resolve().as_posix()+'"']
    specifications = []
    for obj, fields in FIELDS.items():
        for field in fields:
            key = obj+'.'+field
            expression = 'printf "FIELD '+key+' %lu %lu\\n", (unsigned long)&'+key+', (unsigned long)sizeof('+key+')'
            command += ['-ex', expression]
            specifications.append(key)
    for key in ('uwTick', 'uart_error'):
        command += ['-ex', 'printf "FIELD '+key+' %lu %lu\\n", (unsigned long)&'+key+', (unsigned long)sizeof('+key+')']
        specifications.append(key)
    result = runner(command, capture_output=True, timeout=30)
    text = (result.stdout + result.stderr).decode('utf-8', errors='replace').replace('\r', '')
    fields = {key: dict(address=int(address), size=int(size)) for key, address, size in re.findall(r'^FIELD (\S+) (\d+) (\d+)$', text, re.M)}
    if result.returncode or set(fields) != set(specifications):
        raise ValueError('Offline GDB could not derive every new-ELF field')
    if any(v['size'] not in (1, 2, 4, 8) or not RAM_BASE <= v['address'] < v['address']+v['size'] <= RAM_BASE+RAM_SIZE for v in fields.values()):
        raise ValueError('Unexpected ELF field layout')
    start = min(v['address'] for v in fields.values()) & ~3
    end = (max(v['address']+v['size'] for v in fields.values())+3) & ~3
    return dict(fields=fields, address=start, size=end-start)


def decode_ram(raw, layout):
    if len(raw) != layout['size']:
        raise ValueError('Incomplete bounded RAM snapshot')
    result = {}
    for name, spec in layout['fields'].items():
        offset = spec['address']-layout['address']
        value = raw[offset:offset+spec['size']]
        if name in ('queue.odom.x', 'queue.odom.y', 'queue.odom.yaw', 'queue.odom.travel'):
            if spec['size'] != 4:
                raise ValueError('Unexpected float layout')
            result[name] = struct.unpack('<f', value)[0]
            if not math.isfinite(result[name]):
                raise ValueError('Invalid static odometry value')
        else:
            result[name] = int.from_bytes(value, 'little', signed=('tenths_rpm' in name or name.endswith(('applied_left', 'applied_right'))))
    return result


def empty_queue(fields):
    zero = 'queue.task_id queue.count queue.received queue.index queue.committed queue.has_started wire.last_request wire.has_request'.split()
    if any(fields[k] for k in zero) or fields['drive.view.state'] in (2, 3):
        raise ValueError('Expected empty, never-started queue and no enable state')


def audit_fields(fields, mode):
    empty_queue(fields)
    if not fields['queue.boot_id'] or fields['queue.critical_latched'] or fields['wire.transport_fault'] or fields['uart_error'] or fields['drive.uart_failed']:
        raise ValueError('Invalid boot or latched UART/transport fault')
    if fields['drive.view.applied_left'] or fields['drive.view.applied_right']:
        raise ValueError('Nonzero applied target in static audit')
    if fields['queue.state'] != 0 or fields['queue.first_error'] or fields['queue.odom.epoch'] != 1:
        raise ValueError('Expected clean boot IDLE queue with initial odometry epoch')
    if mode == 'healthy':
        if fields['drive.view.state'] != 1 or fields['drive.view.error'] or fields['queue.first_drive_error'] or not fields['drive.view.stop_verified'] or not fields['drive.view.guard_valid']:
            raise ValueError('Healthy automatic boot STOP/guard not proved')
        if fields['drive.sdo_ambiguous'] or not fields['queue.odom.valid'] or not fields['queue.odom.initialized']:
            raise ValueError('Healthy odometry missing or ambiguous')
        if (fields['uwTick']-fields['queue.odom.measurement_tick_ms']) & 0xFFFFFFFF >= 250:
            raise ValueError('Raw MCU measurement source too old')
        if max(abs(fields['queue.odom.left_tenths_rpm']), abs(fields['queue.odom.right_tenths_rpm'])) > 2:
            raise ValueError('Static measured speed exceeds zero bound')
    elif (fields['drive.view.state'] != 5 or fields['drive.view.error'] != 72 or fields['queue.first_drive_error'] != 72 or
          not fields['drive.sdo_ambiguous'] or not fields['drive.tx_suppressed'] or fields['drive.view.stop_verified'] or fields['queue.odom.valid']):
        raise ValueError('Expected isolated no-CAN failure, not a stop proof')


def reset_action(target, report, plan, deployment_path, mode, layout, wait=time.sleep):
    deployment, digest = load_json(deployment_path)
    if deployment.get('status') != 'DEPLOYED_VERIFIED_NO_RESET' or deployment.get('error') is not None or deployment.get('cleanup', {}).get('cleared') is not True:
        raise ValueError('Verified same-target deployment receipt required')
    before = target.metadata('prereset_full', full=True, ram=(layout['address'], layout['size']))
    if (sha(before['data']['flash']) != deployment['verified_full_sha256'] or before['uid'] != deployment['uid'] or
            before['options'] != deployment['options'] or deployment.get('candidate', {}).get('image_sha256') != plan['image_sha256']):
        raise ValueError('Current target no longer matches verified deployment')
    if mode == 'healthy':
        empty_queue(decode_ram(before['data']['ram'], layout))
    boot_spec = layout['fields']['queue.boot_id']
    offset = boot_spec['address']-layout['address']
    previous_boot = int.from_bytes(before['data']['ram'][offset:offset+boot_spec['size']], 'little')
    report.update(deployment_receipt=str(Path(deployment_path).resolve()), deployment_receipt_sha256=digest,
                  uid=before['uid'], options=before['options'], reset_commands=1, mode=mode, layout=layout, previous_boot=previous_boot)
    atomic_json(target.folder/'reset_in_progress.json', report)
    target.run('explicit_static_reset', ['-rst'])
    wait(2.5)
    frames = []
    for label in ('static_ram_first', 'static_ram_second'):
        snapshot = target.metadata(label, ram=(layout['address'], layout['size']))
        same_identity(before, snapshot)
        fields = decode_ram(snapshot['data']['ram'], layout)
        audit_fields(fields, mode)
        frames.append(dict(snapshot=public_snapshot(snapshot), fields=fields))
        if len(frames) == 1:
            wait(.3)
    one, two = frames[0]['fields'], frames[1]['fields']
    if previous_boot and one['queue.boot_id'] == previous_boot:
        raise ValueError('Reset did not establish a new MCU boot identity')
    if one['queue.boot_id'] != two['queue.boot_id'] or one['queue.odom.epoch'] != two['queue.odom.epoch'] or not 0 < (two['uwTick']-one['uwTick']) & 0xFFFFFFFF < 10000:
        raise ValueError('Static boot/epoch changed or MCU tick failed to progress')
    if mode == 'healthy' and not 0 < (two['queue.odom.sample_sequence']-one['queue.odom.sample_sequence']) & 0xFFFFFFFF < 10000:
        raise ValueError('Healthy different real wheel samples did not advance')
    if not 0 < (two['wire.pub_sequence']-one['wire.pub_sequence']) & 0xFFFFFFFF < 10000:
        raise ValueError('Autonomous telemetry publisher failed to progress')
    report.update(status='STATIC_AUDIT_'+mode.upper(), frames=frames, motion_authority=False,
                  STOP_proof=(mode == 'healthy'), requires_reboot_after_CAN_reconnection=(mode == 'isolated'))
    target.final_layout, target.final_expected, target.final_mode = layout, two, mode


def perform(target, action, report, plan, backup_path=None, deployment_path=None, mode='isolated', layout=None, wait=time.sleep):
    """One action, one final cleanup. Failures never run a reset/retry/rollback."""
    report.update(action=action, error=None, reset_commands=0, flash_write_attempted=False,
                  STOP_proof=False, motion_authority=False, automatic_retry=False, automatic_rollback=False)
    try:
        if action == 'inspect-backup':
            backup_action(target, report)
        elif action == 'deploy':
            deploy_action(target, report, plan, backup_path)
        else:
            reset_action(target, report, plan, deployment_path, mode, layout, wait)
    except BaseException as error:
        report['status'] = 'FAILED_KEEP_SITE_STATE'
        report['error'] = type(error).__name__ + ': ' + str(error)
    finally:
        try:
            report['cleanup'] = target.cleanup()
        except BaseException as error:
            report['cleanup_error'] = type(error).__name__ + ': ' + str(error)
            report['status'] = 'FAILED_KEEP_SITE_STATE'
            report['error'] = report['error'] or 'Debug-freeze cleanup unverified'
        report['programmer_calls'] = target.calls
        report['finished_at'] = datetime.now(timezone.utc).isoformat()
        atomic_json(target.folder/'receipt.json', report)
        if report['error']:
            atomic_json(target.folder/'FAILED_LOCK.json', dict(receipt_sha256=sha((target.folder/'receipt.json').read_bytes()), automatic_retry_reset_rollback=False))
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=('inspect-backup', 'deploy', 'reset-audit'))
    for name in ('image', 'elf', 'build-receipt', 'output-dir', 'stlink-serial'):
        parser.add_argument('--'+name, required=True)
    parser.add_argument('--programmer', default=str(PROGRAMMER))
    parser.add_argument('--gdb', default=str(ARM_BIN/'arm-none-eabi-gdb.exe'))
    parser.add_argument('--source-root', default=str(REPO))
    parser.add_argument('--expected-uid')
    parser.add_argument('--site-receipt')
    parser.add_argument('--backup-receipt')
    parser.add_argument('--deployment-receipt')
    parser.add_argument('--mode', choices=('isolated', 'healthy'), default='isolated')
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args(argv)
    plan = local_plan(args.image, args.elf, args.build_receipt, args.source_root)
    layout = elf_layout(args.elf, args.gdb) if args.action == 'reset-audit' else None
    if not args.execute:
        print(json.dumps(dict(dry_run=True, hardware_accessed=False, action=args.action, candidate=plan, layout=layout), indent=2))
        return 0
    if not args.site_receipt or args.action == 'deploy' and not args.backup_receipt or args.action == 'reset-audit' and not args.deployment_receipt:
        parser.error('Execute requires current site receipt and the action\'s backup/deployment receipt')
    mode = args.mode if args.action == 'reset-audit' else 'isolated'
    site, site_sha = site_receipt(args.site_receipt, mode)
    folder = Path(args.output_dir).resolve()
    folder.mkdir(parents=True, exist_ok=False)
    target = Programmer(folder, args.programmer, args.stlink_serial, args.expected_uid)
    report = dict(schema='mcu-v10-deployment-1', created_at=datetime.now(timezone.utc).isoformat(),
                  session_id=site['session_id'], site_receipt_sha256=site_sha, site_receipt=str(Path(args.site_receipt).resolve()),
                  stlink_serial=target.serial, candidate=plan, programmer_sha256=sha(Path(args.programmer).read_bytes()))
    result = perform(target, args.action, report, plan, args.backup_receipt, args.deployment_receipt, args.mode, layout)
    print(json.dumps(dict(receipt=str(folder/'receipt.json'), status=result['status'], error=result['error']), ensure_ascii=False))
    return 2 if result['error'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
