"""Strict v9 client. Importing never opens a port; ARM is always explicit.

Targets are integer driver RPM (speed feedback is tenths of RPM). Calling
exchange(1/2/5/8) never extends a motion lease. No reconnection/automatic ARM.
"""
import math
import os
import secrets
import struct
import threading
import time

VERSION=9
IDENTITY=0x47344339
LEASE_MS=500
MAX_RPM=5
FEEDBACK_STOP_RPM=15
STATUS_FIELDS='tick_ms state error session last_rx_seq last_applied_seq target_pack stop_verified guard_flags identity setpoint_age_ms challenge uart_critical_total'.split()
FEEDBACK_FIELDS='tick_ms sample_sequence sample_tick_ms valid_mask position_left_raw position_right_raw speed_raw left_tick_ms right_tick_ms speed_tick_ms status_raw fault_raw flags'.split()
DIAG_FIELDS='tick_ms critical ore fe ne pe unknown ring_full rearm_busy rearm_error tx_error bad_crc bad_frames'.split()

def crc16(data):
    crc=0xffff
    for byte in data:
        crc^=byte
        for _ in range(8):
            crc=(crc>>1)^(0xa001 if crc&1 else 0)
    return crc

def request(kind,seq,left_rpm=0,right_rpm=0,session=0,challenge=0):
    if kind not in (1,2,3,5,6,7,8) or type(seq) is not int or not 1<=seq<=65535:
        raise ValueError('Invalid kind/sequence')
    if any(type(x) is not int for x in (left_rpm,right_rpm,session,challenge)):
        raise ValueError('Integer fields required; do not confuse RPM with tenths of RPM')
    if not 0<=session<=0xffffffff or not 0<=challenge<=0xffffffff:
        raise ValueError('Invalid session/challenge')
    if kind!=7 and (left_rpm or right_rpm):
        raise ValueError('Only SETPOINT carries targets')
    if kind==7 and (not session or challenge or max(abs(left_rpm),abs(right_rpm))>MAX_RPM):
        raise ValueError('SETPOINT requires a session and targets within +/-5 integer RPM')
    if kind==6 and (not session or not challenge):
        raise ValueError('ARM requires explicit nonzero session and current challenge')
    if kind not in (3,6,7) and session:
        raise ValueError('Unexpected session')
    if kind not in (6,8) and challenge or kind==8 and challenge not in (0,1,2,3,4,5):
        raise ValueError('Unexpected challenge/diagnostic page')
    body=struct.pack('<BBBBHhhII',0xaa,0x55,VERSION,kind,seq,left_rpm,right_rpm,session,challenge)
    return body+struct.pack('<H',crc16(body))

def decode(frame):
    if len(frame)!=64 or frame[:3]!=b'\xaa\x55\x09' or frame[3] not in (0x81,0x82,0x83,0x85,0x86,0x87,0x88):
        raise ValueError('Unexpected v9 response')
    if crc16(frame[:62])!=struct.unpack_from('<H',frame,62)[0]:
        raise ValueError('Response CRC mismatch')
    p=dict(type=frame[3],sequence=struct.unpack_from('<H',frame,4)[0],reserved=frame[6],status=frame[7],
           uart_critical_short=struct.unpack_from('<H',frame,60)[0])
    words=struct.unpack_from('<13I',frame,8)
    if frame[3]==0x85:
        p.update(zip(FEEDBACK_FIELDS,words))
        p['position_left'],p['position_right']=struct.unpack_from('<ii',frame,24)
        l,r=struct.unpack('<hh',struct.pack('<I',p['speed_raw']))
        p.update(speed_left_rpm=l/10.,speed_right_rpm=r/10.,controller_state=(p['flags']>>8)&255,controller_error=p['flags']>>16)
    elif frame[3]==0x88:
        p.update(zip(DIAG_FIELDS,words))
        # Page 1 words have a separate meaning, selected by the request.
        p['words']=list(words)
    else:
        p.update(zip(STATUS_FIELDS,words))
        p['target_left_rpm'],p['target_right_rpm']=struct.unpack('<hh',struct.pack('<I',p['target_pack']))
    return p

def fresh(packet):
    if packet['type']!=0x85 or packet['reserved'] or packet['status'] or packet['uart_critical_short']:
        raise ValueError('Invalid/errored feedback')
    if packet['valid_mask']!=31 or not packet['sample_sequence'] or packet['fault_raw'] or packet['controller_error']:
        raise ValueError('Incomplete CAN feedback or controller/driver fault')
    state=packet['controller_state']
    if state not in (1,5,10) or packet['flags']&3!=1:
        raise ValueError('Controller state/guard does not permit feedback use')
    expected=0 if state==1 else 7
    if packet['status_raw']&15!=expected or (packet['status_raw']>>16)&15!=expected:
        raise ValueError('Driver enable state disagrees with MCU state')
    if bool(packet['flags']&4)!=(state==1):
        raise ValueError('Stopped-state assertion disagrees with control state')
    for key in ('sample_tick_ms','left_tick_ms','right_tick_ms','speed_tick_ms'):
        if (packet['tick_ms']-packet[key])&0xffffffff>250:
            raise ValueError('CAN sample older than 250ms')
    if max(abs(packet['speed_left_rpm']),abs(packet['speed_right_rpm']))>FEEDBACK_STOP_RPM:
        raise ValueError('Feedback speed exceeds 15RPM stop threshold')

class Client:
    def __init__(self,port):
        self.port=port
        self.sequence=0
        self.buffer=bytearray()
        self.session=0
        self.trace=[]
        self.last_tick=None
        self.needs_stop=False
        self._inflight=None
        self._transaction_id=0
        self._wire_port=port
        self._wire_owner=(os.getpid(),threading.get_ident())
        self._nonblocking=False
        self._revocation_generation=0
        self.last_exchange_failure=None
        self.last_exchange_completion=None
    def revoke(self):
        self.session=0
        self.needs_stop=True
        self._revocation_generation+=1
    def _assert_wire_owner(self):
        if self.port is not self._wire_port or (os.getpid(),threading.get_ident())!=self._wire_owner:
            self.revoke()
            raise RuntimeError('The original Client/port owner PID and thread are required')
    def _assert_nonblocking(self):
        Client._assert_wire_owner(self)
        if not self._nonblocking:
            raise RuntimeError('Explicit nonblocking port setup is required')
        for name in ('timeout','write_timeout'):
            value=getattr(self.port,name,None)
            if type(value) not in (int,float) or value!=0:
                self.revoke()
                raise RuntimeError('Nonblocking port '+name+' must remain zero')
    def enable_nonblocking(self):
        """Owner-only setup outside a transaction; no port reopen or extra reader.

        The adapter must honor pyserial timeout=0/write_timeout=0 semantics.
        Read-back checks are configuration evidence, not a hard realtime proof.
        """
        Client._assert_wire_owner(self)
        if self._inflight is not None:
            raise RuntimeError('Cannot change port mode during an exchange')
        try:
            self.port.timeout=0
            self.port.write_timeout=0
            self._nonblocking=True
            Client._assert_nonblocking(self)
        except Exception:
            self._nonblocking=False
            self.revoke()
            raise
        return dict(timeout=0,write_timeout=0,owner_pid=self._wire_owner[0],owner_thread=self._wire_owner[1])
    @property
    def pending_exchange(self):
        tx=self._inflight
        return None if tx is None else Client._ticket(self,tx)
    def _ticket(self,tx):
        return {key:tx[key] for key in ('id','kind','sequence','started_monotonic_s',
            'deadline_monotonic_s','request_hex','bytes_accepted','write_attempted')}
    def _fail_wire(self,error,*,reason):
        tx=self._inflight
        if tx is not None:
            self.last_exchange_failure=dict(Client._ticket(self,tx),reason=reason,
                error=type(error).__name__+': '+str(error),stop_confirmed=False,
                completed_monotonic_s=time.monotonic())
            self._inflight=None
            self.buffer.clear()
        self.revoke()
    def _check_wire_clock(self,tx):
        now=time.monotonic()
        if tx['kind']!=3 and tx['revocation_generation']!=self._revocation_generation:
            raise RuntimeError('Exchange authority was revoked while the reply was pending')
        if not math.isfinite(now) or now<tx['last_poll_monotonic_s']:
            raise RuntimeError('Exchange monotonic clock moved backwards or is invalid')
        tx['last_poll_monotonic_s']=now
        if now>=tx['deadline_monotonic_s']:
            raise TimeoutError('No matching CRC-verified v9 response within 300ms')
        return now
    def _begin_wire(self,kind,left_rpm=0,right_rpm=0,session=0,challenge=0,*,_stopping_observation=False,nonblocking=False):
        Client._assert_wire_owner(self)
        if self._inflight is not None:
            raise RuntimeError('Exactly one exchange may be in flight')
        if nonblocking or self._nonblocking:
            Client._assert_nonblocking(self)
        if _stopping_observation and (kind!=2 or self.session):
            raise RuntimeError('Stopping observation is an unarmed STATUS read only')
        if kind==7 and (not self.session or session!=self.session):
            raise RuntimeError('Client is not explicitly armed for this session')
        if kind in (6,7) and self.needs_stop:
            raise RuntimeError('A previous failure requires explicit STOP before motion')
        self.sequence=self.sequence%65535+1
        try:
            raw=request(kind,self.sequence,left_rpm,right_rpm,session,challenge)
            start=time.monotonic()
            if not math.isfinite(start):
                raise RuntimeError('Invalid monotonic clock')
            self._transaction_id+=1
            tx=dict(id=self._transaction_id,kind=kind,sequence=self.sequence,raw=raw,
                request_hex=raw.hex(),started_monotonic_s=start,deadline_monotonic_s=start+.3,
                last_poll_monotonic_s=start,bytes_accepted=None,write_attempted=False,
                revocation_generation=self._revocation_generation,
                stopping_observation=bool(_stopping_observation))
            self._inflight=tx
            Client._check_wire_clock(self,tx)
            tx['write_attempted']=True
            accepted=self.port.write(raw)
            # Exceptions/None mean an unknown number may already be on the wire.
            tx['bytes_accepted']=accepted if type(accepted) is int and 0<=accepted<=len(raw) else None
            if type(accepted) is not int or accepted!=len(raw):
                raise OSError('Short or unknown request write; old frame will not be retried or completed')
            Client._check_wire_clock(self,tx)
            return Client._ticket(self,tx)
        except Exception as error:
            Client._fail_wire(self,error,reason='begin_failed')
            raise
    def begin_exchange(self,kind,left_rpm=0,right_rpm=0,session=0,challenge=0,*,_stopping_observation=False):
        """Send one complete request once; return its immutable-by-copy identity."""
        return Client._begin_wire(self,kind,left_rpm,right_rpm,session,challenge,
            _stopping_observation=_stopping_observation,nonblocking=True)
    def _poll_wire(self,ticket_id=None,*,nonblocking=False):
        Client._assert_wire_owner(self)
        tx=self._inflight
        if tx is None:
            raise RuntimeError('No exchange is in flight')
        if ticket_id is not None and (type(ticket_id) is not int or ticket_id!=tx['id']):
            raise RuntimeError('Exchange ticket does not identify the active transaction')
        try:
            if nonblocking or self._nonblocking:
                Client._assert_nonblocking(self)
            Client._check_wire_clock(self,tx)
            chunk=self.port.read(64)
            Client._check_wire_clock(self,tx)
            if not isinstance(chunk,bytes) or len(chunk)>64:
                raise ValueError('Port read must return at most the requested 64 bytes')
            self.buffer.extend(chunk)
            # At most one 64-byte read and 128 bounded parse steps per call.
            for _ in range(128):
                if len(self.buffer)<64:
                    break
                if self.buffer[:3]!=b'\xaa\x55\x09':
                    del self.buffer[0]
                    continue
                frame=bytes(self.buffer[:64])
                try:p=decode(frame)
                except ValueError:
                    del self.buffer[0]
                    continue
                del self.buffer[:64]
                if p['type']!=(tx['kind']|0x80) or p['sequence']!=tx['sequence']:
                    continue
                kind=tx['kind']
                self.trace.append(dict(request_hex=tx['request_hex'],reply_hex=frame.hex()))
                if p['reserved'] or p['status'] or (p['uart_critical_short'] and kind!=3):
                    raise ValueError('MCU rejected request or UART critical count is nonzero')
                if kind!=3 and self.last_tick is not None and (p['tick_ms']-self.last_tick)&0xffffffff>0x7fffffff:
                    raise ValueError('MCU tick restarted or moved backwards')
                if kind==5:
                    fresh(p)
                    if self.session and p['controller_state']==1:
                        raise ValueError('MCU became disarmed; no automatic re-ARM')
                if kind not in (5,8):
                    if p['identity']!=IDENTITY:
                        raise ValueError('Unexpected v9 identity')
                    if kind!=3 and p['error'] and not tx['stopping_observation']:
                        raise ValueError('Latched MCU error; explicit STOP recovery required')
                    if self.session and kind!=3 and p['session']!=self.session:
                        raise ValueError('MCU session lost; no automatic re-ARM')
                finished=Client._check_wire_clock(self,tx)
                self.last_tick=p['tick_ms']
                self.last_exchange_completion=dict(Client._ticket(self,tx),completed_monotonic_s=finished)
                self._inflight=None
                return p
            Client._check_wire_clock(self,tx)
            return None
        except Exception as error:
            Client._fail_wire(self,error,reason='poll_failed')
            raise
    def poll_exchange(self,ticket_id=None):
        return Client._poll_wire(self,ticket_id,nonblocking=True)
    def cancel_for_stop(self):
        """Invalidate local wait only; accepted bytes cannot be withdrawn.

        A partial/unknown old frame can prevent a single new STOP from framing.
        No retry, padding, buffer flush of the device, or reconnect is attempted.
        Only a matching STOP reply and the existing separate zero proof suffice.
        """
        Client._assert_wire_owner(self)
        tx=self._inflight
        if tx is not None and tx['kind']==3:
            return dict(retained_stop=True,cancelled_ticket=None,pending_stop=Client._ticket(self,tx),stop_confirmed=False)
        cancelled=None if tx is None else Client._ticket(self,tx)
        if tx is not None:
            Client._fail_wire(self,RuntimeError('Exchange cancelled for explicit STOP'),reason='cancelled_for_stop')
        else:
            self.revoke()
            self.buffer.clear()
        return dict(retained_stop=False,cancelled_ticket=cancelled,pending_stop=None,stop_confirmed=False)
    def exchange(self,kind,left_rpm=0,right_rpm=0,session=0,challenge=0,*,_stopping_observation=False):
        """Synchronous compatibility for preparation/STOP, never the run loop.

        Private non-virtual wire calls avoid reentering subclass split-phase gates.
        """
        ticket=Client._begin_wire(self,kind,left_rpm,right_rpm,session,challenge,
            _stopping_observation=_stopping_observation)
        while True:
            packet=Client._poll_wire(self,ticket['id'])
            if packet is not None:return packet
            if self._nonblocking:time.sleep(.001)
    def arm(self,session=None):
        """Explicit user action only. Starts at zero RPM and does not command motion."""
        if self.needs_stop:
            raise RuntimeError('A previous failure requires explicit STOP before a new ARM')
        self.session=0
        state=self.exchange(2)
        if state['state']!=1 or state['error'] or not state['stop_verified'] or state['guard_flags']&7!=5:
            raise RuntimeError('Explicit stopped and guarded state required before ARM')
        session=secrets.randbelow(0xffffffff)+1 if session is None else session
        self.exchange(6,session=session,challenge=state['challenge'])
        deadline=time.monotonic()+4
        while time.monotonic()<deadline:
            state=self.exchange(2)
            if state['state']==10 and state['session']==session:
                self.session=session
                return state
            if state['state'] not in (2,4):
                raise RuntimeError('ARM did not reach READY at zero RPM')
            time.sleep(.02)
        raise TimeoutError('ARM did not complete; target lease cannot be renewed by status')
    def setpoint(self,left_rpm,right_rpm):
        """Call at 20Hz; only new SETPOINTs extend the fixed MCU 500ms lease."""
        return self.exchange(7,left_rpm,right_rpm,session=self.session)
    def stop(self):
        cancelled=self.cancel_for_stop()
        if cancelled['retained_stop']:
            raise RuntimeError('STOP already in flight; poll existing transaction')
        state=self.exchange(3)
        self.needs_stop=False
        return state  # ACK only; ARM independently requires stopped/guarded IDLE.
    def stopping_status(self):
        """Observe asynchronous STOP cleanup, including a still-latched error.

        Read-only, with no active client session. CRC, identity, UART counters,
        sequence and clock checks remain strict. An error is data to wait on,
        never a successful STOP or permission to ARM.
        """
        return self.exchange(2,_stopping_observation=True)

def wheels_from_twist(linear_x,angular_z,diameter=.2,separation=.3):
    """Integer-RPM conversion; scale both wheels together to preserve curvature."""
    if not all(math.isfinite(v) for v in (linear_x,angular_z,diameter,separation)) or diameter<=0 or separation<=0:
        raise ValueError('Invalid geometry/velocity')
    factor=60/(math.pi*diameter)
    l=(linear_x-angular_z*separation/2)*factor
    r=-(linear_x+angular_z*separation/2)*factor
    scale=max(1,max(abs(l),abs(r))/MAX_RPM)
    return int(round(l/scale)),int(round(r/scale))
