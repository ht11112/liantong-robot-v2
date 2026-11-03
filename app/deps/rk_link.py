"""Bounded RK3588 RS485 probe; only fixed PING can be transmitted explicitly."""
import argparse
import datetime
import json
import os
from pathlib import Path
import re
import socket
import subprocess
import termios
import time

def locate():
    if socket.gethostname()!='ltzx3-robot' or Path('/sys/class/net/eth0/address').read_text().strip().lower()!='f6:91:b4:0a:da:22':
        raise RuntimeError('Unexpected RK identity; no serial port opened')
    matches=[]
    for tty in Path('/sys/class/tty').glob('ttyUSB*'):
        for parent in (tty/'device').resolve().parents:
            try:identity={k:(parent/k).read_text().strip() for k in ['idVendor','idProduct','bcdDevice']}
            except (FileNotFoundError,NotADirectoryError):continue
            if identity=={'idVendor':'1a86','idProduct':'7523','bcdDevice':'0264'}:
                matches.append({'port':'/dev/'+tty.name,'usb_path':str(parent),'identity':identity})
            break
    if len(matches)!=1:raise RuntimeError('Expected exactly one known RS485 REV0264 adapter')
    target=matches[0]
    result=subprocess.run(['fuser',target['port']],capture_output=True,text=True,timeout=3)
    target['occupancy']={'code':result.returncode,'stdout':result.stdout,'stderr':result.stderr}
    if result.returncode!=1 or result.stderr.strip():
        raise RuntimeError('RS485 port is occupied or occupancy uncertain')
    return target

