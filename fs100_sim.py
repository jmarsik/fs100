#!/usr/bin/env python3
#
# Minimal simulator of the YRC1000 / FS100 High Speed Ethernet Server, for offline tests
# and demos. It answers the robot control commands (UDP 10040) and the file commands
# (UDP 10041) with plausible data; it does not move anything.
#
# Usage: python3 fs100_sim.py [--host 127.0.0.2]
#
# Command layouts follow the YRC1000 Ethernet function manual HW1483358 rev 11, chapter 3.
# Licensed under the MIT license (see LICENSE).
#

import argparse
import math
import socket
import struct
import threading
import time

HDR = 32
LAST = 0x80000000
BLOCK = 479

ST_OK = 0x00
ST_UNDEFINED = 0x08
ST_VENDOR = 0x1f
ST_NO_INSTANCE = 0x28


def _pad(text, size, encoding='utf-8'):
    raw = text.encode(encoding)[:size]
    return raw + bytes(size - len(raw))


def _answer(req, service, status=ST_OK, added=0, data=b'', block=LAST):
    h = bytearray(HDR)
    h[0:4] = b'YERC'
    struct.pack_into('<HH', h, 4, HDR, len(data))
    h[8] = 3
    h[9] = req[9]
    h[10] = 1
    h[11] = req[11]
    struct.pack_into('<I', h, 12, block)
    h[16:24] = b'99999999'
    h[24] = service | 0x80
    h[25] = status
    h[26] = 1 if added else 0
    struct.pack_into('<H', h, 28, added)
    return bytes(h) + data


class SimState:
    """Controller data the simulator serves. Tests may change it directly."""

    def __init__(self):
        self.lock = threading.Lock()
        self.var_count = 100
        self.B = [i % 256 for i in range(self.var_count)]
        self.I = [i * 10 - 500 for i in range(self.var_count)]
        self.D = [i * 1000 for i in range(self.var_count)]
        self.R = [i / 4 for i in range(self.var_count)]
        self.S = ['' for _ in range(self.var_count)]
        self.S[10] = '0003'
        self.S[11] = '6V2514708880001'
        self.S[12] = 'ERAR-1000-06VX'
        self.P = [{'data_type': 0, 'form': 0, 'tool_no': 0, 'user_coor_no': 0, 'extended_form': 0,
                   'pos': [i * 100, -i * 100, i, 0, 0, 0, 0, 0]} for i in range(128)]
        self.BP = [[0] + [i * 10] + [0] * 7 for i in range(128)]
        self.EX = [[0] * 9 for _ in range(128)]
        self.IO = {}
        self.REG = [0] * 1000
        self.remote = True        # command remote; file commands need it
        self.write_protect = False  # emulates S2C541=1 during play
        self.status = (0x40 | 0x04 | 0x80, 0x40)  # play, auto, command remote / servo on
        self.job = ('IO_START2', 12, 3, 10000)  # override in 0.01 % (live: 10000)
        self.alarms = [
            (4107, 1, 1, '2026/10/10 08:12', 'OUT OF RANGE(ABSO DATA)', '', '[SLURBT]'),
            (1325, 0, 0, '2026/10/10 08:11', 'COMMUNICATION ERROR(ENCODER)', '[SV#1]', '[1]'),
        ]
        self.history = {
            1: [(1325, 0, 0, '2026/10/09 21:00', 'COMMUNICATION ERROR(ENCODER)', '[SV#1]', '')],
            1001: [(4107, 1, 3, '2026/10/10 08:12', 'OUT OF RANGE(ABSO DATA)', '', '[SLURBT]'),
                   (4100, 2, 1, '2026/10/08 10:00', 'OVERRUN', '', '[2]')],
            2001: [], 3001: [], 4001: [(9999, 0, 0, '2026/10/01 07:00', 'OFFLINE TEST', '', '')],
        }
        self.axis_names = ['S', 'L', 'U', 'R', 'B', 'T', '', '']
        self.cart_names = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz', '', '']
        self.base_names = ['1', '', '', '', '', '', '', '']
        self.firmware = 'YAS5.10.00A(EN/CZ)-00'
        self.files = {
            'TRANSPORT_POSITION.JBI': b'/JOB\r\n//NAME TRANSPORT_POSITION\r\n//POS\r\n///NPOS 1,1,0,0,0,0\r\n'
                                      b'//INST\r\n///DATE 2024/05/24 10:00\r\n///ATTR SC,RW\r\n'
                                      b'///GROUP1 RB1,BS1\r\nNOP\r\nMOVJ C00000 BC00000 VJ=5.00\r\nEND\r\n',
            'IO_START2.JBI': b'/JOB\r\n//NAME IO_START2\r\n//POS\r\n///NPOS 0,0,0,0,0,0\r\n//INST\r\n'
                             b'///DATE 2024/09/04 12:00\r\n///ATTR SC,RW\r\nNOP\r\nDOUT OT#(1) ON\r\nEND\r\n',
            'VARNAME.DAT': (b'//VARNAME\r\n///SHARE 2000,2000,2000,2000,2000,10000,10000,10000\r\n///B\r\n'
                            b'0003 1,0,GRIPPER STATE\r\n///I\r\n///D\r\n0005 1,0,LUB DIST M\r\n'
                            b'0006 1,0,CURR DIST M\r\n///R\r\n///S\r\n0010 1,0,MACHINE SERIAL\r\n'
                            b'0011 1,0,ROBOT ARM SERIAL\r\n0012 1,0,ROBOT CTRL SERIA\r\n///P\r\n///BP\r\n///EX\r\n'),
            'LOGDATA.DAT': (b'//LOGDATA\r\n///INDEX 1\r\nDATE              :2026/10/10 08:10:00\r\n'
                            b'EVENT             :POWER ON\r\nLOGIN NAME        :\r\n\r\n///INDEX 2\r\n'
                            b'DATE              :2026/10/10 08:09:00\r\nEVENT             :START\r\n'
                            b'LOGIN NAME        :\r\nTASK              :#08\r\nJOB NAME          :IO_START2\r\n'
                            b'LINE              :0000\r\n\r\n'),
            'SYSTEM.SYS': b'//SYSTEM\r\n///DATE 2026/10/10 08:00\r\n//SYSTEM NO\r\nYAS5.10.00A(EN/CZ)-00\r\n',
            'ALL.PRM': b'//ALL\r\n' + b'x' * 3000 + b'\r\n',
        }
        self.t0 = time.monotonic()


class FS100Simulator:
    """UDP server pair emulating a controller's HSE endpoint."""

    def __init__(self, host='127.0.0.2', port_robot=10040, port_file=10041, state=None):
        self.state = state or SimState()
        self.stale_reply = False  # when True, every answer is preceded by one with a wrong request ID
        self.drop_next = 0        # drop this many requests without answering
        self.delay = 0.0          # seconds to wait before answering (slow link)
        self.sock_robot = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_robot.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_robot.bind((host, port_robot))
        self.sock_file = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock_file.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock_file.bind((host, port_file))
        self._stop = threading.Event()
        self._threads = []
        self._file_sessions = {}  # addr -> remaining blocks (list of bytes)
        self.received_files = {}

    # ---------------------------------------------------------------- lifecycle

    def start(self):
        for sock, handler in ((self.sock_robot, self._robot), (self.sock_file, self._file)):
            t = threading.Thread(target=self._loop, args=(sock, handler), daemon=True)
            t.start()
            self._threads.append(t)
        return self

    def stop(self):
        self._stop.set()
        for s in (self.sock_robot, self.sock_file):
            try:
                s.close()
            except OSError:
                pass

    def __enter__(self):
        return self.start()

    def __exit__(self, *exc):
        self.stop()

    def _loop(self, sock, handler):
        sock.settimeout(0.2)
        while not self._stop.is_set():
            try:
                req, addr = sock.recvfrom(2048)
            except socket.timeout:
                continue
            except OSError:
                return
            if len(req) < HDR or req[0:4] != b'YERC':
                continue
            if self.drop_next:
                self.drop_next -= 1
                continue
            try:
                answers = handler(req, addr)
            except Exception as e:  # keep serving; report as system error
                answers = [_answer(req, req[29] if len(req) > 29 else 0, ST_VENDOR, 0xCFFF)]
                print('simulator error:', repr(e))
            if self.delay:
                time.sleep(self.delay)
            for ans in answers or []:
                if self.stale_reply:
                    bad = bytearray(ans)
                    bad[11] = (bad[11] + 7) & 0xff
                    sock.sendto(bytes(bad), addr)
                sock.sendto(ans, addr)

    # ---------------------------------------------------------------- robot control

    def _robot(self, req, addr):
        cmd, inst, attr, service = struct.unpack_from('<HHBB', req, 24)
        data = req[HDR:HDR + struct.unpack_from('<H', req, 6)[0]]
        st = self.state
        with st.lock:
            try:
                status, added, payload = self._dispatch(cmd, inst, attr, service, data)
            except IndexError:
                status, added, payload = ST_VENDOR, 0xA001, b''
        return [_answer(req, service, status, added, payload)]

    def _dispatch(self, cmd, inst, attr, service, data):
        st = self.state
        t = time.monotonic() - st.t0
        ok = (ST_OK, 0)
        if cmd in (0x70, 0x30A, 0x71, 0x30B):
            if cmd in (0x70, 0x30A):
                if not 1 <= inst <= 4:
                    return ST_NO_INSTANCE, 0, b''
                items = st.alarms
                idx = inst - 1
            else:
                base = next((b for b in (1, 1001, 2001, 3001, 4001) if b <= inst < b + 100), None)
                if base is None:
                    return ST_VENDOR, 0xA001, b''
                items = st.history[base]
                idx = inst - base
            a = items[idx] if idx < len(items) else (0, 0, 0, '', '', '', '')
            payload = struct.pack('<III', a[0], a[1], a[2]) + _pad(a[3], 16) + _pad(a[4], 32)
            if cmd in (0x30A, 0x30B):
                payload += _pad(a[5], 16) + _pad(a[6], 96) + bytes(96)
            return ST_OK, 0, payload
        if cmd == 0x72:
            return ST_OK, 0, struct.pack('<II', *st.status)
        if cmd == 0x73:
            if inst != 1:
                return ST_OK, 0, bytes(32) + struct.pack('<III', 0, 0, 0)
            name, line, step, ovr = st.job
            return ST_OK, 0, _pad(name, 32) + struct.pack('<III', line, step, ovr)
        if cmd == 0x74:
            names = st.cart_names if 101 <= inst <= 108 else st.base_names if 11 <= inst <= 18 else st.axis_names
            if inst not in (1, 11, 101):
                return ST_VENDOR, 0xA001, b''
            return ST_OK, 0, b''.join(_pad(n, 4) for n in names)
        if cmd == 0x75:
            if inst == 1:
                pos = [int(10000 * math.sin(t / 3 + k)) for k in range(6)] + [0, 0]
                head = (0, 0, 0, 0, 0)
            elif inst == 101:
                pos = [1500000, 20000, 1200000, 1800000, 0, 900000, 0, 0]
                head = (16, 4, 0, 0, 0)
            elif inst == 11:
                pos = [11641, 0, 0, 0, 0, 0, 0, 0]
                head = (0, 0, 0, 0, 0)
            else:
                return ST_VENDOR, 0xA001, b''
            return ST_OK, 0, struct.pack('<5I8i', *head, *pos)
        if cmd in (0x76, 0x77, 0x0411):
            if inst not in (1, 11):
                return ST_VENDOR, 0xA001, b''
            n = 6 if inst == 1 else 1
            if cmd == 0x76:
                vals = [int(5 * math.sin(t + k)) for k in range(n)]
            elif cmd == 0x77:
                vals = [int(3000 * math.sin(t / 2 + k)) for k in range(n)]
            else:
                vals = [35 + k + int(2 * math.sin(t / 10)) for k in range(n)]
            if cmd == 0x0411:  # live YAS5.10: 8 values, unused axes 0
                return ST_OK, 0, struct.pack('<8i', *(vals + [0] * (8 - n)))
            # live YAS5.10: torque / position error carry only the existing axes (6 robot, 1 base)
            return ST_OK, 0, struct.pack('<{}i'.format(n), *vals)
        if cmd == 0x0412:
            boards = struct.pack('<3i', int(t * 10) % 1000, 812, 123456)
            return ST_OK, 0, struct.pack('<IBBBB', 0x01, 1, 1, 0, 0) + boards + bytes(7 * 12)
        if cmd == 0x0413:
            if inst != 1:
                return ST_VENDOR, 0xA001, b''
            return ST_OK, 0, struct.pack('<i5i', 41, 0, 0, 0, 0, 0)
        if cmd == 0x88:
            texts = {1: ('2024/09/04 12:00', "012345:12'34"), 10: ('2024/09/04 12:05', "004321:00'10"),
                     110: ('2024/09/05 08:00', "001234:56'00"), 210: ('2024/09/05 08:00', "000987:01'02")}
            if inst not in texts and not (11 <= inst <= 18 or 111 <= inst <= 118 or 211 <= inst <= 218):
                return ST_VENDOR, 0xA001, b''
            start, elapse = texts.get(inst, ('2024/09/04 12:00', "000100:00'00"))
            return ST_OK, 0, _pad(start, 16) + _pad(elapse, 12)
        if cmd == 0x89:
            if inst == 11:
                return ST_OK, 0, _pad(st.firmware, 24) + _pad('1-06VX180-120-A0', 16) + _pad('4.30', 8)
            if inst == 101:
                return ST_OK, 0, _pad(st.firmware, 24) + _pad('GENERAL', 16) + bytes(8)
            if 12 <= inst <= 44:
                return ST_OK, 0, bytes(48)
            return ST_VENDOR, 0xA001, b''
        if cmd in (0x82, 0x83, 0x84, 0x85, 0x86, 0x87, 0x8A, 0x8B):
            return ST_VENDOR, 0x2100 if not st.remote else 0x2080, b''  # simulator never moves
        if 0x78 <= cmd <= 0x81 or cmd == 0x8C:
            return self._var_single(cmd, inst, service, data)
        if 0x300 <= cmd <= 0x309 or cmd == 0x30C:
            return self._var_plural(cmd, inst, service, data)
        return ST_UNDEFINED, 0, b''

    # variables ------------------------------------------------------

    _SCALAR = {0x7A: ('B', 'B', 1), 0x7B: ('I', '<h', 2), 0x7C: ('D', '<i', 4), 0x7D: ('R', '<f', 4)}

    def _get(self, cmd, n):
        st = self.state
        if cmd in self._SCALAR:
            name, fmt, size = self._SCALAR[cmd]
            arr = getattr(st, name)
            if n >= len(arr):
                raise IndexError
            return struct.pack(fmt, arr[n])
        if cmd in (0x7E, 0x8C):
            if n >= len(st.S):
                raise IndexError
            return _pad(st.S[n], 16 if cmd == 0x7E else 32)
        if cmd == 0x78:
            return struct.pack('B', st.IO.get(n, 0))
        if cmd == 0x79:
            return struct.pack('<H', st.REG[n])
        if cmd == 0x7F:
            p = st.P[n]
            return struct.pack('<5I8i', p['data_type'], p['form'], p['tool_no'], p['user_coor_no'],
                               p['extended_form'], *p['pos'])
        if cmd == 0x80:
            return struct.pack('<I8i', *st.BP[n])
        if cmd == 0x81:
            if not st.EX:
                raise IndexError
            return struct.pack('<I8i', *st.EX[n])
        raise IndexError

    def _put(self, cmd, n, raw):
        st = self.state
        if st.write_protect:
            return ST_VENDOR, 0xC800
        if cmd in self._SCALAR:
            name, fmt, size = self._SCALAR[cmd]
            arr = getattr(st, name)
            if n >= len(arr):
                raise IndexError
            if len(raw) != size:
                return ST_VENDOR, 0xB003
            arr[n] = struct.unpack(fmt, raw)[0]
            return ok_status()
        if cmd in (0x7E, 0x8C):
            size = 16 if cmd == 0x7E else 32
            if len(raw) != size:
                return ST_VENDOR, 0xB003
            st.S[n] = raw.split(b'\x00', 1)[0].decode('utf-8', errors='replace')
            return ok_status()
        if cmd == 0x78:
            if not 2701 <= n <= 2956:
                return ST_VENDOR, 0xB002
            st.IO[n] = raw[0]
            return ok_status()
        if cmd == 0x79:
            if n >= 600:
                return ST_VENDOR, 0xB002
            st.REG[n] = struct.unpack('<H', raw[:2])[0]
            return ok_status()
        if cmd == 0x7F:
            if len(raw) != 52:
                return ST_VENDOR, 0xB003
            v = struct.unpack('<5I8i', raw)
            st.P[n] = {'data_type': v[0], 'form': v[1], 'tool_no': v[2], 'user_coor_no': v[3],
                       'extended_form': v[4], 'pos': list(v[5:])}
            return ok_status()
        if cmd in (0x80, 0x81):
            if len(raw) != 36:
                return ST_VENDOR, 0xB003
            (st.BP if cmd == 0x80 else st.EX)[n] = list(struct.unpack('<I8i', raw))
            return ok_status()
        raise IndexError

    def _var_single(self, cmd, inst, service, data):
        if service in (0x0E, 0x01):
            return ST_OK, 0, self._get(cmd, inst)
        if service in (0x10, 0x02):
            status, added = self._put(cmd, inst, data)
            return status, added, b''
        return ST_UNDEFINED, 0, b''

    _PLURAL = {0x300: (0x78, 1), 0x301: (0x79, 2), 0x302: (0x7A, 1), 0x303: (0x7B, 2), 0x304: (0x7C, 4),
               0x305: (0x7D, 4), 0x306: (0x7E, 16), 0x307: (0x7F, 52), 0x308: (0x80, 36), 0x309: (0x81, 36),
               0x30C: (0x8C, 32)}

    def _var_plural(self, cmd, inst, service, data):
        single, size = self._PLURAL[cmd]
        count = struct.unpack('<I', data[:4])[0]
        if count == 0 or (size == 1 and count % 2) or 4 + count * size > BLOCK:
            return ST_VENDOR, 0xB003, b''
        if service == 0x33:
            return ST_OK, 0, struct.pack('<I', count) + b''.join(self._get(single, inst + k) for k in range(count))
        if service == 0x34:
            if len(data) != 4 + count * size:
                return ST_VENDOR, 0xB003, b''
            for k in range(count):
                status, added = self._put(single, inst + k, data[4 + k * size:4 + (k + 1) * size])
                if status:
                    return status, added, b''
            return ST_OK, 0, b''
        return ST_UNDEFINED, 0, b''

    # ---------------------------------------------------------------- file control

    def _file(self, req, addr):
        service = req[29]
        ack = req[10]
        block = struct.unpack_from('<I', req, 12)[0]
        data = req[HDR:HDR + struct.unpack_from('<H', req, 6)[0]]
        st = self.state
        if ack == 0:
            if not st.remote:
                return [_answer(req, service, ST_VENDOR, 0xE2B2)]
            if service == 0x32:
                pattern = data.decode('ascii') or '*.JBI'
                ext = pattern.rsplit('.', 1)[-1].upper()
                if ext == '*':
                    ext = 'JBI'
                content = ''.join(n + '\r\n' for n in sorted(st.files) if n.upper().endswith('.' + ext))
                return self._start_stream(req, addr, service, content.encode())
            if service == 0x16:
                name = data.decode('utf-8')
                if name not in st.files:
                    return [_answer(req, service, ST_VENDOR, 0xE2B3)]
                return self._start_stream(req, addr, service, st.files[name])
            if service == 0x09:
                return [_answer(req, service, ST_VENDOR, 0xE2AF)]
            if service == 0x15:
                self._file_sessions[addr] = ('load', data.decode('utf-8'), bytearray())
                return [_answer(req, service, block=0)]
            return [_answer(req, service, ST_UNDEFINED)]
        # acknowledgements / data from the PC
        session = self._file_sessions.get(addr)
        if session is None:
            return []
        if session[0] == 'save':
            blocks = session[1]
            if block & LAST or not blocks:
                del self._file_sessions[addr]
                return []
            return [self._next_block(req, addr, service)]
        if session[0] == 'load':
            session[2].extend(data)
            if block & LAST:
                st.files[session[1]] = bytes(session[2])
                self.received_files[session[1]] = bytes(session[2])
                del self._file_sessions[addr]
            return [_answer(req, service, block=block)]
        return []

    def _start_stream(self, req, addr, service, content):
        blocks = [content[i:i + BLOCK] for i in range(0, len(content), BLOCK)] or [b'']
        self._file_sessions[addr] = ('save', blocks, 0)
        return [self._next_block(req, addr, service)]

    def _next_block(self, req, addr, service):
        kind, blocks, sent = self._file_sessions[addr]
        chunk = blocks.pop(0)
        sent += 1
        self._file_sessions[addr] = (kind, blocks, sent)
        block = sent | (LAST if not blocks else 0)
        return _answer(req, service, data=chunk, block=block)


def ok_status():
    return ST_OK, 0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--host', default='127.0.0.2', help='address to bind (default 127.0.0.2)')
    a = ap.parse_args()
    sim = FS100Simulator(a.host).start()
    print('FS100/YRC1000 HSE simulator on {}:10040/10041, Ctrl-C to stop'.format(a.host))
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sim.stop()


if __name__ == '__main__':
    main()
