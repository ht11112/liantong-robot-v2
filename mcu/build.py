"""Offline-only C tests and STM32G474 production build; never accesses devices."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
REPO = ROOT.parent
UNITY = Path('C:/Program Files/Unity/Hub/Editor/6000.6.2f1/Editor/Data/PlaybackEngines/WebGLSupport/BuildTools/Emscripten')
SDK = Path('C:/Users/Administrator/STM32Cube/Repository/STM32Cube_FW_G4_V1.6.3')
ARM = Path('F:/STM32CubeIDE_2.2.0/STM32CubeIDE/plugins/com.st.stm32cube.ide.mcu.externaltools.gnu-tools-for-stm32.14.3.rel1.win32_1.0.100.202602081740/tools/bin')


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def identities():
    return {str(p.relative_to(REPO)).replace('\\', '/'): sha(p)
            for p in sorted(ROOT.rglob('*'))
            if p.is_file() and p.suffix in ('.c', '.h', '.py', '.ld')
            and '__pycache__' not in p.parts}


def run(command, destination, env=None):
    r = subprocess.run(list(map(str, command)), cwd=REPO, env=env, capture_output=True,
                       text=True, encoding='utf-8', errors='replace', timeout=180)
    destination.write_text(r.stdout+r.stderr, encoding='utf-8')
    if r.returncode:
        raise RuntimeError('Offline process failed: '+str(command[0])+'\n'+r.stdout+r.stderr)
    return r.stdout+r.stderr


def host_tests(args, out):
    sources = [ROOT/'src'/n for n in ('mq.c', 'drive.c', 'wire.c') if (ROOT/'src'/n).is_file()]
    native = shutil.which('cc') or shutil.which('gcc') or shutil.which('clang')
    common = ['-std=c11', '-Wall', '-Wextra', '-Werror', '-DMQ_HOST_TEST=1',
              '-I'+str(ROOT/'include'), '-I'+str(ROOT/'tests')]
    env = dict(os.environ)
    if native:
        compiler = [native]
    else:
        compiler = [str(args.emscripten/'emscripten/emcc.bat')]
        if not Path(compiler[0]).is_file():
            raise FileNotFoundError('A native C compiler or --emscripten is required')
        config = out/'emscripten_config.py'
        config.write_text('LLVM_ROOT='+repr(str(args.emscripten/'llvm'))+'\n'
            +'BINARYEN_ROOT='+repr(str(args.emscripten/'binaryen'))+'\n'
            +'NODE_JS=['+repr(str(args.emscripten/'node/node.exe'))+']\nFROZEN_CACHE=True\n', encoding='utf-8')
        env['EM_CONFIG'] = str(config)
    cases = []
    for test in sorted((ROOT/'tests').glob('test_*.c')):
        binary = out/(test.stem+('.exe' if os.name=='nt' and native else '' if native else '.js'))
        flags = [] if native else ['-sENVIRONMENT=node', '-sEXIT_RUNTIME=1', '-sSINGLE_FILE=1']
        run(compiler+common+list(map(str, sources))+[str(test), '-lm']+flags+['-o', str(binary)],
            out/(test.stem+'_compile.log'), env)
        command = [str(binary)] if native else [str(args.emscripten/'node/node.exe'), str(binary)]
        output = run(command, out/(test.stem+'.log'), env)
        cases.append(dict(name=test.stem, assertions='C assert enabled',
                          pass_lines=[x for x in output.splitlines() if x.startswith(('PASS ', 'REVIEW PASS '))],
                          compiler=native or compiler[0]))
    if not cases:
        raise RuntimeError('No C production-code tests found')
    py_env = dict(os.environ, MQ_V11_CODEC_GOLDEN_LOG=str(out/'test_wire.log'))
    py = run([sys.executable, '-B', '-m', 'unittest', 'discover', '-s', str(ROOT/'tests'),
              '-p', 'test_*.py', '-v'], out/'python_tests.log', py_env)
    return dict(C_suites=cases, Python_summary=py.strip().splitlines()[-5:])


def firmware(args, out):
    suffix = '.exe' if os.name == 'nt' else ''
    def tool(name):
        if args.compiler_dir:
            return args.compiler_dir/(name+suffix)
        found = shutil.which(name)
        if not found:
            raise FileNotFoundError(name)
        return Path(found)
    gcc = tool('arm-none-eabi-gcc')
    hal = args.sdk/'Drivers/STM32G4xx_HAL_Driver'
    cmsis = args.sdk/'Drivers/CMSIS'
    device = cmsis/'Device/ST/STM32G4xx'
    sources = [ROOT/'src'/n for n in ('board_g474.c', 'mq.c', 'drive.c', 'wire.c')]
    sources += [device/'Source/Templates/system_stm32g4xx.c',
                device/'Source/Templates/gcc/startup_stm32g474xx.s']
    modules = ('hal', 'hal_cortex', 'hal_rcc', 'hal_rcc_ex', 'hal_gpio', 'hal_dma',
               'hal_dma_ex', 'hal_flash', 'hal_flash_ex', 'hal_pwr', 'hal_pwr_ex',
               'hal_uart', 'hal_uart_ex', 'hal_fdcan', 'hal_rng')
    sources += [hal/('Src/stm32g4xx_'+m+'.c') for m in modules]
    flags = ['-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', '-Os', '-g3',
             '-ffunction-sections', '-fdata-sections', '-Wall', '-Wextra', '-Werror',
             '-DMQ_PRODUCTION=1', '-DSTM32G474xx', '-DUSE_HAL_DRIVER',
             '-DUSER_VECT_TAB_ADDRESS', '-I'+str(ROOT/'include'), '-I'+str(hal/'Inc'),
             '-I'+str(device/'Include'), '-I'+str(cmsis/'Include')]
    objects = []
    for source in sources:
        if not source.is_file():
            raise FileNotFoundError(source)
        obj = out/(source.name+'.o')
        run([gcc]+flags+['-c', str(source), '-o', str(obj)], out/(source.name+'.compile.log'))
        objects.append(obj)
    elf, binary, mapping = (out/('mcu-v11.'+s) for s in ('elf', 'bin', 'map'))
    run([gcc]+flags+['-nostartfiles', '--specs=nano.specs', '--specs=nosys.specs']+objects+
        ['-T'+str(ROOT/'flash.ld'), '-Wl,--gc-sections', '-Wl,-Map='+str(mapping),
         '-Wl,--start-group', '-lc', '-lm', '-Wl,--end-group', '-o', elf], out/'link.log')
    run([tool('arm-none-eabi-objcopy'), '-O', 'binary', elf, binary], out/'objcopy.log')
    size = run([tool('arm-none-eabi-size'), elf], out/'size.log')
    symbols = run([tool('arm-none-eabi-nm'), '-n', '-S', elf], out/'symbols.log')
    if binary.stat().st_size > 32768:
        raise AssertionError('32KB application budget exceeded; no expanded image is permitted')
    addresses = {}
    for line in symbols.splitlines():
        parts = line.split()
        if len(parts) in (3, 4):
            try:
                addresses[parts[-1]] = int(parts[0], 16)
            except ValueError:
                pass
    if addresses['_ebss']+4096 >= addresses['_estack']:
        raise AssertionError('4KB stack reservation exceeded')
    forbidden = ('stop_speed_diagnostics', 'bench_control', 'failsafe_diagnostics',
                 'bench_arm', 'bench_setpoint', 'test_core', 'test_review')
    if any(name in addresses for name in forbidden):
        raise AssertionError('Legacy/test symbol present in production')
    return dict(binary_bytes=binary.stat().st_size, flash_limit_bytes=32768,
        binary_sha256=sha(binary), elf_sha256=sha(elf), map_sha256=sha(mapping),
        compiler_sha256=sha(gcc), compiler_flags=flags, size_output=size,
        RAM_static_bytes=addresses['_ebss']-0x20000000,
        stack_reserved_bytes=4096,
        RAM_remaining_after_stack=addresses['_estack']-addresses['_ebss']-4096,
        forbidden_symbols_absent=list(forbidden),
        SDK_sources_sha256={str(p):sha(p) for p in sources if ROOT not in p.parents})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host-tests', action='store_true')
    parser.add_argument('--firmware', action='store_true')
    parser.add_argument('--name', default='mcu-v11-offline')
    parser.add_argument('--emscripten', type=Path, default=UNITY)
    parser.add_argument('--sdk', type=Path, default=SDK)
    parser.add_argument('--compiler-dir', type=Path, default=ARM if os.name=='nt' else None)
    args = parser.parse_args()
    if not args.host_tests and not args.firmware:
        parser.error('Choose --host-tests and/or --firmware; no hardware operation exists')
    if not args.name or any(c not in 'abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-.' for c in args.name) or args.name in ('.', '..'):
        parser.error('--name must be a single safe output directory name')
    out = REPO/'build'/args.name
    out.mkdir(parents=True, exist_ok=False)
    started = time.monotonic()
    report = dict(status='OFFLINE_ONLY_NOT_FLASHED', hardware_accessed=False,
                  flashed=False, source_sha256=identities())
    try:
        if args.host_tests:
            report['tests'] = host_tests(args, out)
        if args.firmware:
            report['firmware'] = firmware(args, out)
        if report['source_sha256'] != identities():
            raise RuntimeError('Source changed during build; rerun against a frozen source set')
        report['passed'] = True
    except Exception as exc:
        report.update(passed=False, error=str(exc))
        raise
    finally:
        report['elapsed_s'] = time.monotonic()-started
        (out/'receipt.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
        print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'}, indent=2))


if __name__ == '__main__':
    main()
