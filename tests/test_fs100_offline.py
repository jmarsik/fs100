#!/usr/bin/env python3
#
# Offline tests: FS100 against the bundled simulator (fs100_sim.py), no controller needed.
# Run: python3 tests/test_fs100_offline.py
#

import os
import socket
import struct
import sys
import tempfile
import unittest

test_dir = os.path.dirname(os.path.realpath(__file__))
module_dir = os.path.dirname(test_dir)
sys.path.append(module_dir)

from fs100 import FS100, FS100AnsPacket  # noqa: E402
from fs100_sim import FS100Simulator  # noqa: E402

HOST = '127.0.0.1'


def free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((HOST, 0))
    port = s.getsockname()[1]
    s.close()
    return port


class TestFS100Offline(unittest.TestCase):

    def setUp(self):
        self.pr, self.pf = free_udp_port(), free_udp_port()
        self.sim = FS100Simulator(HOST, self.pr, self.pf).start()
        self.robot = FS100(HOST, timeout=0.5, port_robot_control=self.pr, port_file_control=self.pf)

    def tearDown(self):
        self.robot.disconnect()
        self.sim.stop()

    # ------------------------------------------------------------ status, info, alarms

    def test_status(self):
        st = {}
        self.assertEqual(self.robot.get_status(st), FS100.ERROR_SUCCESS)
        self.assertTrue(st['play'])
        self.assertTrue(st['servo_on'])
        self.assertTrue(st['cmd_remote'])
        self.assertFalse(st['teach'])

    def test_system_info_strips_nul(self):
        info = {}
        self.assertEqual(self.robot.acquire_system_info(FS100.SystemInfoType.R1, info), FS100.ERROR_SUCCESS)
        self.assertEqual(info['software_version'], 'YAS5.10.00A(EN/CZ)-00')
        self.assertEqual(info['model'], '1-06VX180-120-A0')
        self.assertEqual(info['parameter_version'], '4.30')

    def test_management_time(self):
        t = {}
        self.assertEqual(self.robot.acquire_management_time(FS100.ManagementTimeType.CONTROL_POWER_ON, t), 0)
        self.assertEqual(t['start'], '2024/09/04 12:00')
        self.assertNotIn('\x00', t['elapse'])

    def test_current_alarms(self):
        alarms = []
        self.assertEqual(self.robot.read_alarms(alarms), FS100.ERROR_SUCCESS)
        self.assertEqual([a['code'] for a in alarms], [4107, 1325])
        self.assertEqual(alarms[0]['subcode_text'], '[SLURBT]')
        self.assertEqual(alarms[1]['subcode_info'], '[SV#1]')
        last = {}
        self.assertEqual(self.robot.get_last_alarm(last), 0)
        self.assertEqual(last['name'], 'OUT OF RANGE(ABSO DATA)')

    def test_alarm_history(self):
        hist = []
        self.assertEqual(self.robot.read_alarm_history(FS100.AlarmHistoryType.MINOR, hist), 0)
        self.assertEqual([(h['num'], h['code']) for h in hist], [(1001, 4107), (1002, 4100)])
        hist = []
        self.assertEqual(self.robot.read_alarm_history(FS100.AlarmHistoryType.USER, hist, extended=False), 0)
        self.assertEqual(hist, [])

    def test_job_info(self):
        info = {}
        self.assertEqual(self.robot.read_executing_job_info(info), 0)
        self.assertEqual((info['job_name'], info['line_num']), ('IO_START2', 12))

    # ------------------------------------------------------------ axes and monitoring

    def test_axis_names_8(self):
        names = {}
        self.assertEqual(self.robot.read_axis_name(names, robot_no=1), 0)
        self.assertEqual(names['1st_axis'], 'S')
        self.assertEqual(names['8th_axis'], '')

    def test_position(self):
        pos = {}
        self.assertEqual(self.robot.read_position(pos, robot_no=101), 0)
        self.assertEqual(pos['data_type'], 16)
        self.assertEqual(len(pos['pos']), 8)

    def test_position_error_signed(self):
        self.sim.state.t0 -= 4.0  # sin() negative for some axes
        err = {}
        self.assertEqual(self.robot.read_position_error(err), 0)
        self.assertEqual(len(err), 6)  # existing axes only, as a live YAS5.10 answers
        self.assertTrue(all(-10 < v < 10 for v in err.values()), err)

    def test_torque_default_keys(self):
        tq = {}
        self.assertEqual(self.robot.read_torque(tq), 0)
        # the answer carries the 6 existing axes only (as a live YAS5.10 does)
        self.assertEqual(list(tq), ['1st_axis', '2nd_axis', '3rd_axis', '4th_axis', '5th_axis', '6th_axis'])
        base = {}
        self.assertEqual(self.robot.read_torque(base, robot_no=11), 0)  # 1-axis base: 4 bytes
        self.assertEqual(list(base), ['1st_axis'])

    def test_temperatures_and_regen(self):
        temp = {}
        self.assertEqual(self.robot.read_encoder_temperature(temp), 0)
        self.assertTrue(30 < temp['1st_axis'] < 45)
        status, conv = self.robot.read_converter_temperature(1)
        self.assertEqual((status, conv), (0, 41))
        regen = {}
        self.assertEqual(self.robot.read_regenerative_power(regen), 0)
        self.assertEqual(regen['enabled_boards'], 1)
        self.assertEqual(regen['boards'][0]['total'], 123456)

    # ------------------------------------------------------------ variables

    def test_scalar_read_write(self):
        v = FS100.Variable(FS100.VarType.INTEGER, 7)
        self.assertEqual(self.robot.read_variable(v), 0)
        self.assertEqual(v.val, 7 * 10 - 500)
        v.val = -1234
        self.assertEqual(self.robot.write_variable(v), 0)
        self.assertEqual(self.sim.state.I[7], -1234)
        r = FS100.Variable(FS100.VarType.REAL, 2, 1.5)
        self.assertEqual(self.robot.write_variable(r), 0)
        self.assertEqual(self.sim.state.R[2], 1.5)

    def test_string_padded_and_stripped(self):
        s = FS100.Variable(FS100.VarType.STRING, 10)
        self.assertEqual(self.robot.read_variable(s), 0)
        self.assertEqual(s.val, '0003')  # no trailing NULs
        s.val = 'ABC'
        self.assertEqual(len(s.val_to_bytes()), 16)
        self.assertEqual(self.robot.write_variable(s), 0)
        self.assertEqual(self.sim.state.S[10], 'ABC')
        s32 = FS100.Variable(FS100.VarType.STRING32, 11)
        self.assertEqual(self.robot.read_variable(s32), 0)
        self.assertEqual(s32.val, '6V2514708880001')
        with self.assertRaises(ValueError):
            FS100.Variable(FS100.VarType.STRING, 0, 'x' * 17).val_to_bytes()

    def test_plural_read_chunks_and_types(self):
        ints = [FS100.Variable(FS100.VarType.INTEGER, n) for n in range(100)]
        bytes_ = [FS100.Variable(FS100.VarType.BYTE, n) for n in (1, 2, 3, 50)]  # odd run + single
        strings = [FS100.Variable(FS100.VarType.STRING, n) for n in range(40)]  # > 29 per packet
        self.assertEqual(self.robot.read_variables(ints + bytes_ + strings), 0)
        self.assertEqual([v.val for v in ints], self.sim.state.I)
        self.assertEqual([v.val for v in bytes_], [1, 2, 3, 50])
        self.assertEqual(strings[12].val, 'ERAR-1000-06VX')

    def test_plural_read_large_double(self):
        self.sim.state.D = list(range(0, 300 * 7, 7))
        ds = [FS100.Variable(FS100.VarType.DOUBLE, n) for n in range(300)]  # > 118 per packet
        self.assertEqual(self.robot.read_variables(ds), 0)
        self.assertEqual(ds[299].val, 299 * 7)

    def test_plural_write(self):
        vs = [FS100.Variable(FS100.VarType.BYTE, n, n + 100) for n in range(5)]  # odd count
        self.assertEqual(self.robot.write_variables(vs), 0)
        self.assertEqual(self.sim.state.B[:5], [100, 101, 102, 103, 104])
        with self.assertRaises(ValueError):
            self.robot.write_variables([FS100.Variable(FS100.VarType.BYTE, 0, 1),
                                        FS100.Variable(FS100.VarType.BYTE, 2, 1)])

    def test_position_variables_8_axes(self):
        p = FS100.Variable(FS100.VarType.ROBOT_POSITION, 3)
        self.assertEqual(self.robot.read_variable(p), 0)
        self.assertEqual(len(p.val['pos']), 8)
        self.assertEqual(p.val['pos'][:2], (300, -300))
        p.val['pos'] = (1, 2, 3, 4, 5, 6, 7)  # 7-tuple still accepted, padded to 8
        self.assertEqual(self.robot.write_variable(p), 0)
        self.assertEqual(self.sim.state.P[3]['pos'], [1, 2, 3, 4, 5, 6, 7, 0])
        bp = FS100.Variable(FS100.VarType.BASE_POSITION, 2)
        self.assertEqual(self.robot.read_variable(bp), 0)
        self.assertEqual(bp.val['1st_axis'], 20)
        self.assertIn('8th_axis', bp.val)
        self.assertEqual(self.robot.write_variable(bp), 0)

    def test_write_protect_error_text(self):
        self.sim.state.write_protect = True
        v = FS100.Variable(FS100.VarType.DOUBLE, 1, 5)
        self.assertNotEqual(self.robot.write_variable(v), 0)
        self.assertEqual(self.robot.errno, 0xC800)
        self.assertIn('S2C541', FS100.describe_error(0x1f, self.robot.errno))

    def test_bad_instance(self):
        v = FS100.Variable(FS100.VarType.INTEGER, 500)
        self.assertEqual(self.robot.read_variable(v), 0x1f)
        self.assertEqual(self.robot.errno, 0xA001)

    # ------------------------------------------------------------ transport behaviour

    def test_stale_answers_are_ignored(self):
        self.sim.stale_reply = True
        self.robot.connect()
        for n in (1, 2, 3):
            v = FS100.Variable(FS100.VarType.DOUBLE, n)
            self.assertEqual(self.robot.read_variable(v), 0)
            self.assertEqual(v.val, n * 1000)

    def test_timeout_and_retry(self):
        self.robot.timeout = 0.2
        self.sim.drop_next = 1
        st = {}
        self.assertEqual(self.robot.get_status(st), FS100.ERROR_CONNECTION)
        self.robot.retries = 1
        self.sim.drop_next = 1
        self.assertEqual(self.robot.get_status(st), FS100.ERROR_SUCCESS)

    def test_no_controller(self):
        robot = FS100(HOST, timeout=0.2, port_robot_control=free_udp_port())
        st = {}
        self.assertEqual(robot.get_status(st), FS100.ERROR_CONNECTION)
        # nothing listens: either a timeout or an ICMP port unreachable (ECONNREFUSED)
        self.assertRegex(FS100.describe_error(FS100.ERROR_CONNECTION, robot.errno), 'no answer|connection error')

    def test_short_answer_packet(self):
        with self.assertRaises(ValueError):
            FS100AnsPacket(b'YERC')

    # ------------------------------------------------------------ files

    def test_file_list_and_read(self):
        files = []
        self.assertEqual(self.robot.get_file_list('*.JBI', files), 0)
        self.assertEqual(files, ['IO_START2.JBI', 'TRANSPORT_POSITION.JBI'])
        status, data = self.robot.read_file('ALL.PRM')  # several 479-byte blocks
        self.assertEqual(status, 0)
        self.assertEqual(data, self.sim.state.files['ALL.PRM'])

    def test_read_file_abort_and_sink(self):
        calls = []

        def progress(count):
            calls.append(count)
            return count < 1000  # abort in the third block

        status, data = self.robot.read_file('ALL.PRM', progress)
        self.assertEqual(status, FS100.ERROR_CANCELLED)
        self.assertEqual(len(calls), 3)
        self.assertEqual(len(data), calls[-1])
        # the connection is usable afterwards
        status, data = self.robot.read_file('ALL.PRM')
        self.assertEqual((status, data), (0, self.sim.state.files['ALL.PRM']))
        chunks = []
        status, data = self.robot.read_file('ALL.PRM', sink=chunks.append)
        self.assertEqual((status, data), (0, b''))
        self.assertEqual(b''.join(chunks), self.sim.state.files['ALL.PRM'])

    def test_slow_first_block_waits_file_timeout(self):
        # big files: the controller takes longer than the normal timeout to send the first block
        self.sim.file_start_delay = 0.8
        try:
            status, data = self.robot.read_file('ALL.PRM')
        finally:
            self.sim.file_start_delay = 0.0
        self.assertEqual(status, 0)
        self.assertGreater(len(data), 479)

    def test_file_commands_use_file_port_when_connected(self):
        self.robot.connect()  # persistent robot control socket must not be reused for files
        files = []
        self.assertEqual(self.robot.get_file_list('*.DAT', files), 0)
        self.assertIn('VARNAME.DAT', files)
        self.assertTrue(self.robot.connected())
        st = {}
        self.assertEqual(self.robot.get_status(st), 0)

    def test_recv_file_with_path(self):
        self.sim.state.files['/SPDRV/CMOSBK.BIN'] = b'$$Y1.CMOSTYPE' + bytes(1000)
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(self.robot.recv_file('/SPDRV/CMOSBK.BIN', d), 0)
            with open(os.path.join(d, 'CMOSBK.BIN'), 'rb') as f:
                self.assertEqual(f.read()[:13], b'$$Y1.CMOSTYPE')

    def test_missing_file(self):
        status, data = self.robot.read_file('NOPE.JBI')
        self.assertNotEqual(status, 0)
        self.assertEqual(self.robot.errno, 0xE2B3)

    def test_file_needs_remote(self):
        self.sim.state.remote = False
        files = []
        self.assertNotEqual(self.robot.get_file_list('*.JBI', files), 0)
        self.assertEqual(self.robot.errno, 0xE2B2)

    def test_send_file(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, 'UP.JBI')
            content = b'/JOB\r\n' + b'NOP\r\n' * 200
            with open(path, 'wb') as f:
                f.write(content)
            self.assertEqual(self.robot.send_file(path), 0)
        self.assertEqual(self.sim.received_files['UP.JBI'], content)

    def test_request_raw(self):
        ans = self.robot.request(0x72, 1, 0, 0x01)
        self.assertEqual(ans.status, 0)
        self.assertEqual(len(ans.data), 8)
        d1, d2 = struct.unpack('<II', ans.data)
        self.assertTrue(d2 & 0x40)


if __name__ == '__main__':
    unittest.main(verbosity=2)
