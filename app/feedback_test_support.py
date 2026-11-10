"""Byte-level serial fixture for offline feedback-process tests only."""

from dataclasses import asdict
import struct
import time

from mcu_v11 import (ACK_VALID, APPEND, BEGIN, COMMIT, COMMITTED, EXECUTING, HEADER,
                    ODOM_INITIALIZED, ODOM_VALID, QUERY, RENEW, START, STARTED,
                    STATUS, STOP, STOP_CONFIRMED, Telemetry, crc16)


class SimulatedPort:
    """Generate wire replies at wall-clock speed; no hardware or navigation logic."""

    def __init__(self, profile='normal', reply_timeout=.1):
        self.profile = profile
        self.started = time.monotonic()
        self.delay = .010
        self.pending = bytearray()
        self.ready_at = 0.
        self.counter = self.task = self.count = 0
        self.state = 0
        self.extra_flags = 0
        self.watermark = 0
        self.stopped_at = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def close(self):
        pass

    def write(self, packet):
        if crc16(packet[2:-2]) != struct.unpack_from('<H', packet, len(packet)-2)[0]:
            raise AssertionError('Host sent an invalid command CRC')
        opcode = packet[3]
        request = struct.unpack_from('<I', packet, 18)[0]
        task = struct.unpack_from('<Q', packet, 26)[0]
        if self.profile == 'write_failure' and self.counter >= 5:
            raise OSError('simulated device removal')
        if opcode == BEGIN:
            self.task, self.state, self.extra_flags = task, 1, 0
            self.count = struct.unpack_from('<H', packet, 34)[0]
        elif opcode == COMMIT:
            self.state, self.extra_flags = 2, COMMITTED
        elif opcode == START:
            self.state, self.extra_flags = 4, COMMITTED | STARTED | EXECUTING
        elif opcode == STOP:
            self.state, self.extra_flags = 9, (0 if self.profile == 'stop_no_proof' else STOP_CONFIRMED)
            self.stopped_at = time.monotonic()
        elif opcode not in (QUERY, APPEND, RENEW):
            raise AssertionError('Unexpected simulated opcode: {}'.format(opcode))
        self.counter += 1
        self.watermark = request
        now = time.monotonic()
        elapsed_ms = int((now-self.started)*1000)
        sample_step = elapsed_ms // 50
        tick_base = 0xffffff00 if self.profile == 'wrap' else 1000
        seq_base = 0xfffffff0 if self.profile == 'wrap' else 0
        source_base = 0xfffffffa if self.profile == 'wrap' else 20
        valid = not (self.profile == 'invalid' and 6 <= sample_step <= 8)
        flags = ACK_VALID | ODOM_INITIALIZED | (ODOM_VALID if valid else 0) | self.extra_flags
        boot = 43 if self.profile == 'boot_change' and self.counter >= 15 else 42
        epoch = 2 if self.profile == 'epoch_change' and self.counter >= 15 else 1
        values = dict(boot=boot, epoch=epoch,
                      publication_sequence=(seq_base+self.counter) & 0xffffffff,
                      publication_tick_ms=(tick_base+elapsed_ms) & 0xffffffff,
                      sample_sequence=(source_base+sample_step) & 0xffffffff,
                      measurement_tick_ms=(tick_base+sample_step*50) & 0xffffffff,
                      task=self.task, ack_request=request, ack_result=0, state=self.state,
                      flags=flags, index=0, count=self.count, progress=0., x=0., y=0., yaw=0.,
                      v=0., w=0., position_left=0, position_right=0,
                      left_tenths_rpm=0, right_tenths_rpm=0, first_error=0,
                      first_drive_error=0, first_error_tick_ms=0,
                      lease_remaining_ms=30000 if self.state == 4 else 0,
                      accepted_request_watermark=self.watermark)
        status = Telemetry(**values)
        body = HEADER[2:] + STATUS.pack(*asdict(status).values())
        self.pending.extend(b'\xa5\x5a' + body + struct.pack('<H', crc16(body)))
        self.ready_at = now + self.delay
        return len(packet)

    def read(self, size):
        delay = max(0., self.ready_at-time.monotonic())
        if delay:
            time.sleep(min(.002, delay))
        if time.monotonic() < self.ready_at:
            return b''
        data = bytes(self.pending[:size])
        del self.pending[:size]
        return data


def simulated_port(device, reply_timeout):
    return SimulatedPort(device, reply_timeout)
