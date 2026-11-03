"""Runtime-only strict external dependency loader and termios-preserving owner."""
from contextlib import contextmanager
import importlib.util
import os
from pathlib import Path
import sys


def load_dependency(directory, module_name):
    path = Path(directory).resolve() / (module_name + '.py')
    if not path.is_file():
        raise RuntimeError('Required reviewed external dependency is missing: ' + str(path))
    # Load exact files, never an incidental same-name module on ROS/PYTHONPATH.
    spec = importlib.util.spec_from_file_location('_green_external_' + module_name, str(path))
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    if module_name == 'control_client' and (module.VERSION != 9 or module.IDENTITY != 0x47344339
                                             or module.LEASE_MS != 500 or module.MAX_RPM != 5):
        raise RuntimeError('Unexpected external control_client identity/envelope')
    return module


@contextmanager
def exclusive_port(name):
    import serial
    import termios
    fd = os.open(name, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        before = termios.tcgetattr(fd)
    finally:
        os.close(fd)
    port = serial.Serial(port=None, baudrate=115200, timeout=.01, write_timeout=.10,
                         exclusive=True, rtscts=False, dsrdtr=False, xonxoff=False)
    port.dtr = port.rts = False
    port.port = name
    try:
        port.open()
        port.reset_input_buffer()
        yield port
    finally:
        port.close()
        fd = os.open(name, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            termios.tcsetattr(fd, termios.TCSANOW, before)
            if termios.tcgetattr(fd) != before:
                raise RuntimeError('Host serial settings did not restore')
        finally:
            os.close(fd)
