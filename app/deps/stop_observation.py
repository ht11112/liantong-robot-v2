"""Bounded serial STOP observation; status progress never renews motion authority."""
import json
import time


def report_progress(publish,errors):
    """A failed ROS/display notification must never prevent the serial STOP."""
    try:publish();return True
    except Exception as error:
        errors.append(type(error).__name__+': '+str(error));return False


def stopped(packet):
    return (packet.get('state') == 1 and packet.get('stop_verified') == 1
        and packet.get('guard_flags', 0) & 7 == 5
        and all(packet.get(k) == 0 for k in
                ('status', 'reserved', 'error', 'session', 'target_pack',
                 'uart_critical_short', 'uart_critical_total')))


def verify_stop(client, observe, clock=time.monotonic, sleep=time.sleep):
    client.stop()  # ACK is not evidence of completed asynchronous cleanup.
    deadline = clock() + 1.5
    packet = None
    while clock() < deadline:
        packet = client.stopping_status()
        observe(packet)  # Publish real STATUS while CAN/odometry is suspended.
        if stopped(packet):
            return packet
        sleep(.02)
    raise RuntimeError('STOP not verified DISARMED within 1.5s; no further ARM: '
                       + json.dumps(packet))
