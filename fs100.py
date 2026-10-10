#!/usr/bin/env python
#
# YASKAWA FS100 High Speed Ethernet Server Functions
#
# Copyright (c) 2019-2022 FIH Mobile Limited
#
# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.
#
# Authors:
#    Hsinko Yu <hsinkoyu@fih-foxconn.com>
#
# Also works with the YRC1000. Command layouts were checked against the
# YRC1000 Ethernet function manual HW1483358 rev 11, chapter 3 ("High-Speed
# Ethernet Server Function"); page references below are PDF pages of that
# manual.
#

import logging
import ntpath
import os
import socket
import struct
import threading
import time
from enum import IntEnum

log = logging.getLogger('fs100')


# common header excluding sub-header
class FS100PacketHeader:
    HEADER_IDENTIFIER = 'YERC'
    HEADER_SIZE = 0x20
    HEADER_RESERVED_1 = 3
    HEADER_DIVISION_ROBOT_CONTROL = 1
    HEADER_DIVISION_FILE_CONTROL = 2
    HEADER_ACK_REQUEST = 0
    HEADER_ACK_NOT_REQUEST = 1
    HEADER_BLOCK_NUMBER_REQ = 0
    HEADER_RESERVED_2 = '99999999'
    HEADER_PADDING = 0

    def __init__(self, packet=None, data_size=None, division=None, ack=None, req_id=None, block_no=None):
        if packet is not None:
            self.data_size = struct.unpack('<H', packet[6:8])[0]
            self.division = struct.unpack('B', packet[9:10])[0]
            self.ack = struct.unpack('B', packet[10:11])[0]
            self.req_id = struct.unpack('B', packet[11:12])[0]
            self.block_no = struct.unpack('<I', packet[12:16])[0]
        else:
            self.data_size = data_size
            self.division = division
            self.ack = ack
            self.req_id = req_id
            self.block_no = block_no

    def to_bytes(self):
        h = FS100PacketHeader.HEADER_IDENTIFIER.encode(encoding='ascii')
        h += struct.pack('<H', FS100PacketHeader.HEADER_SIZE)
        h += struct.pack('<H', self.data_size)
        h += struct.pack('B', FS100PacketHeader.HEADER_RESERVED_1)
        h += struct.pack('B', self.division)
        h += struct.pack('B', self.ack)
        h += struct.pack('B', self.req_id)
        h += struct.pack('<I', self.block_no)
        h += FS100PacketHeader.HEADER_RESERVED_2.encode(encoding='ascii')
        return h


class FS100ReqPacket(FS100PacketHeader):

    def __init__(self, division, req_id, cmd_no, inst, attr, service, data, data_size):
        FS100PacketHeader.__init__(self, None, data_size, division, FS100PacketHeader.HEADER_ACK_REQUEST, req_id,
                                   FS100PacketHeader.HEADER_BLOCK_NUMBER_REQ)
        self.cmd_no = cmd_no
        self.inst = inst
        self.attr = attr
        self.service = service
        self.data = data

    def to_bytes(self):
        h = FS100PacketHeader.to_bytes(self)
        h += struct.pack('<H', self.cmd_no)
        h += struct.pack('<H', self.inst)
        h += struct.pack('B', self.attr)
        h += struct.pack('B', self.service)
        h += struct.pack('<H', FS100PacketHeader.HEADER_PADDING)
        h += self.data
        return h

    def clone(self, data=None):
        if data is None:
            data = self.data
        data_size = len(data)
        return FS100ReqPacket(self.division, self.req_id, self.cmd_no, self.inst, self.attr, self.service, data,
                              data_size)


class FS100AnsPacket(FS100PacketHeader):

    def __init__(self, packet):
        if len(packet) < FS100PacketHeader.HEADER_SIZE:
            raise ValueError('answer packet too short ({} bytes)'.format(len(packet)))
        FS100PacketHeader.__init__(self, packet)
        self.service = struct.unpack('B', packet[24:25])[0]
        self.status = struct.unpack('B', packet[25:26])[0]
        self.added_status_size = struct.unpack('B', packet[26:27])[0]
        self.added_status = struct.unpack('<H', packet[28:30])[0]
        self.data = packet[FS100PacketHeader.HEADER_SIZE:FS100PacketHeader.HEADER_SIZE + self.data_size]

    # for debug purpose
    def to_bytes(self):
        h = FS100PacketHeader.to_bytes(self)
        h += struct.pack('B', self.service)
        h += struct.pack('B', self.status)
        h += struct.pack('B', self.added_status_size)
        h += struct.pack('B', FS100PacketHeader.HEADER_PADDING)
        h += struct.pack('<H', self.added_status)
        h += struct.pack('<H', FS100PacketHeader.HEADER_PADDING)
        h += self.data
        return h


def _cstr(raw, encoding=None):
    """Decode a NUL-padded controller string."""
    return bytes(raw).split(b'\x00', 1)[0].decode(encoding or FS100.ENCODING, errors='replace')


_AXIS_KEYS = ('1st_axis', '2nd_axis', '3rd_axis', '4th_axis', '5th_axis', '6th_axis', '7th_axis', '8th_axis')


def _ints(raw, signed=True, count=None):
    """Unpack consecutive little-endian 32-bit integers."""
    n = len(raw) // 4 if count is None else min(count, len(raw) // 4)
    return list(struct.unpack('<{}{}'.format(n, 'i' if signed else 'I'), bytes(raw[:n * 4])))


class FS100:
    """YASKAWA FS100 High Speed Ethernet Server Functions

    This class implements most of YASKAWA FS100 High Speed Ethernet Server Functions.
    https://www.motoman.com/getmedia/16B5CD92-BD0B-4DE0-9DC9-B71D0B6FE264/160766-1CD.pdf.aspx?ext=.pdf
    It works with the YRC1000 too (manual HW1483358, chapter 3).

    Attributes:
        ip (str): IP address of the controller
        timeout (int, optional): Communication timeout value in second between PC and Controller.
            Defaults to 0.8.
        retries (int, optional): How many times a robot control request is resent after a timeout.
            Defaults to 0. File transfers are never retried.
        port_robot_control, port_file_control (int, optional): UDP ports, default 10040 / 10041
            (for simulators and relays).
        local_port_robot_control, local_port_file_control (int, optional): fixed local (source) UDP
            ports; default is an ephemeral port. Needed behind udp-over-tcp.
        errno (int): Number of last error (the added status code of the last answer)

    Methods:
        switch_power(): Turn on/off the power supply
        mov(): Make robot move to a specified position
        pmov(): Make robot move to a specified pulse position
        select_cycle(): Select the way a job in pendant plays
        select_job(): Select a job in pendant for later playing
        play_job(): Start playing a job in pendant
        read_executing_job_info(): Read the info of executing job
        read_axis_name(): Read the name of each axis
        read_position(): Read the robot position
        read_position_error(): Read the robot position error data
        read_torque(): Read the robot torque data of each axis
        read_variable(): Read a robot variable
        read_variables(): Read multiple robot variable with plural commands
        write_variable(): Write a robot variable
        write_variables(): Write multiple robot variables with plural commands
        get_status(): Retrieve various status of the robot
        read_alarm_info(): Retrieve info of the specified alarm (alarm history)
        read_alarm_history(): Retrieve all entries of one alarm history category
        get_last_alarm(): Retrieve info of the latest alarm
        read_alarm(): Retrieve info of one of the up to four current alarms
        read_alarms(): Retrieve info of all current alarms
        read_alarm_ex(): Like read_alarm(), with the sub code character strings
        read_alarm_info_ex(): Like read_alarm_info(), with the sub code character strings
        reset_alarm(): To reset alarms or cancel errors
        acquire_system_info(): Acquire system information
        acquire_management_time(): Acquire usage time of an action
        read_encoder_temperature(): Read the encoder temperature of each axis (YAS4.81 and later)
        read_regenerative_power(): Read the amount of regenerative power (YAS4.90 and later)
        read_converter_temperature(): Read the converter temperature (YAS5.20 and later)
        show_text_on_pendant(): Show text on pendant
        get_file_list(): Retrieve list of files ended with extension in pendant
        send_file(): Send a local file to pendant
        recv_file(): Receive a file from pendant
        read_file(): Receive a file from pendant into memory
        delete_file(): Delete a file in pendant
        request(): Send any robot control command and return the raw answer
        describe_error(): Human readable text for a status / added status pair
    """
    DEBUG = False

    # Encoding of strings coming from the controller (alarm names, job names, S variables).
    # The controller sends them in the language selected on the programming pendant
    # (manual p. 76), so this has to match that setting for non-ASCII text.
    ENCODING = 'utf-8'

    UDP_PORT_ROBOT_CONTROL = 10040
    UDP_PORT_FILE_CONTROL = 10041

    TRANSMISSION_SEND = 1
    TRANSMISSION_SEND_AND_RECV = 2

    ERROR_SUCCESS = 0
    ERROR_CONNECTION = 1
    ERROR_NO_SUCH_FILE_OR_DIRECTORY = 2

    # power supply command
    POWER_TYPE_HOLD = 1
    POWER_TYPE_SERVO = 2
    POWER_TYPE_HLOCK = 3
    POWER_SWITCH_ON = 1
    POWER_SWITCH_OFF = 2

    # cycle selection command
    CYCLE_TYPE_STEP = 1
    CYCLE_TYPE_ONE_CYCLE = 2
    CYCLE_TYPE_CONTINUOUS = 3

    # move command
    MOVE_TYPE_JOINT_ABSOLUTE_POS = 1
    MOVE_TYPE_LINEAR_ABSOLUTE_POS = 2
    MOVE_TYPE_LINEAR_INCREMENTAL_POS = 3
    MOVE_SPEED_CLASS_PERCENT = 0  # for joint operation
    MOVE_SPEED_CLASS_MILLIMETER = 1
    MOVE_SPEED_CLASS_DEGREE = 2
    MOVE_COORDINATE_SYSTEM_BASE = 16
    MOVE_COORDINATE_SYSTEM_ROBOT = 17
    MOVE_COORDINATE_SYSTEM_USER = 18
    MOVE_COORDINATE_SYSTEM_TOOL = 19

    # reset alarm command
    RESET_ALARM_TYPE_ALARM = 1
    RESET_ALARM_TYPE_ERROR = 2

    # status codes of the answer sub-header (manual p. 168)
    STATUS_TEXT = {
        0x00: 'success',
        0x08: 'requested command is not defined',
        0x09: 'invalid element number',
        0x1f: 'vendor specific error (see added status)',
        0x28: 'requested instance does not exist',
    }

    # added status codes (manual pp. 169-172)
    ADDED_STATUS_TEXT = {
        0x1010: 'Command error', 0x1011: 'Error in number of command operands',
        0x1012: 'Command operand value range over', 0x1013: 'Command operand length error',
        0x1020: 'Disk full of files', 0x2010: 'Manipulator operating', 0x2020: 'Hold by programming pendant',
        0x2030: 'Hold by playback panel', 0x2040: 'External hold', 0x2050: 'Command hold',
        0x2060: 'Error/alarm occurring', 0x2070: 'Servo OFF', 0x2080: 'Incorrect mode',
        0x2090: 'File accessing by other function', 0x2100: 'Command remote not set',
        0x2110: 'This data cannot be accessed', 0x2120: 'This data cannot be loaded', 0x2130: 'Editing',
        0x2150: 'Running the coordinate conversion function', 0x3010: 'Turn ON the servo power',
        0x3040: 'Perform home positioning', 0x3050: 'Confirm positions', 0x3070: 'Current value not made',
        0x3220: 'Panel lock; mode/cycle prohibit signal is ON', 0x3230: 'Panel lock; start prohibit signal is ON',
        0x3350: 'User coordinate is not taught', 0x3360: 'User coordinate is destroyed',
        0x3370: 'Incorrect control group', 0x3380: 'Incorrect base axis data',
        0x3390: 'Relative job conversion prohibited (at CVTRJ)', 0x3400: 'Master job call prohibited (parameter)',
        0x3410: 'Master job call prohibited (lamp ON during operation)',
        0x3420: 'Master job call prohibited (teach lock)', 0x3430: 'Robot calibration data not defined',
        0x3450: 'Servo power cannot be turned ON', 0x3460: 'Coordinate system cannot be set',
        0x4010: 'Insufficient memory capacity (job registered memory)',
        0x4012: 'Insufficient memory capacity (position data registered memory)',
        0x4020: 'Job editing prohibited', 0x4030: 'Same job name exists', 0x4040: 'No specified job',
        0x4060: 'Set an execution job', 0x4120: 'Position data is destroyed', 0x4130: 'Position data not exist',
        0x4140: 'Incorrect position variable type', 0x4150: 'END instruction for job which is not master job',
        0x4170: 'Instruction data is destroyed', 0x4190: 'Invalid character in job name',
        0x4200: 'Invalid character in the label name', 0x4230: 'Invalid instruction in this system',
        0x4420: 'No step in job to be converted', 0x4430: 'Already converted', 0x4480: 'Teach user coordinate',
        0x4490: 'Relative job/ independent control function not permitted',
        0x5110: 'Syntax error (syntax of instruction)', 0x5120: 'Position data error', 0x5130: 'No NOP or END',
        0x5170: 'Format error (incorrect format)', 0x5180: 'Incorrect number of data', 0x5200: 'Data range over',
        0x5310: 'Syntax error (except instruction)', 0x5340: 'Error in pseudo instruction specification',
        0x5370: 'Error in condition file data record', 0x5390: 'Error in JOB data record',
        0x5430: 'System data not same', 0x5480: 'Incorrect welding function type',
        0x6010: 'The robot/station is under the operation', 0x6020: 'Not enough memory of the specified device',
        0x6030: 'Cannot be accessed to the specified device', 0x6040: 'Unexpected auto backup request',
        0x6050: 'CMOS size is over the RAM area', 0x6060: 'No memory allocation at the power supply on',
        0x6070: 'Accessing error to backup file information', 0x6080: 'Failed in sorting backup file (Remove)',
        0x6090: 'Failed in sorting backup file (Rename)', 0x6100: 'Drive name exceeds the specified values',
        0x6110: 'Incorrect device', 0x6120: 'System error', 0x6130: 'Auto backup is not available',
        0x6140: 'Cannot be backed up under the auto backup', 0xA000: 'Undefined command', 0xA001: 'Instance error',
        0xA002: 'Attribute error', 0xA100: 'Replying data part size error (hardware limit)',
        0xA101: 'Replying data part size error (software limit)', 0xB001: 'Undefined position variable',
        0xB002: 'Data use prohibited', 0xB003: 'Requiring data size error', 0xB004: 'Out of range the data',
        0xB005: 'Data undefined', 0xB006: 'Specified application unregistered',
        0xB007: 'Specified type unregistered', 0xB008: 'Control group setting error', 0xB009: 'Speed setting error',
        0xB00A: 'Operating speed is not setting', 0xB00B: 'Operation coordinate system setting error',
        0xB00C: 'Type setting error', 0xB00D: 'Tool No. setting error', 0xB00E: 'User No. setting error',
        0xC001: 'System error (data area setting processing error)',
        0xC002: 'System error (over the replying data area)',
        0xC003: 'System error (size of the data element not same)',
        0xC800: 'System error (customize API processing error), e.g. write during play with S2C541=1',
        0xCFFF: 'Other error', 0xD8FA: 'Transmission exclusive error (BUSY or Semaphore error)',
        0xD8F1: 'Processing the another command (BUSY condition)',
        0xE24F: 'Wrong parameter setting for the system backup',
        0xE250: 'System backup file creating error (confirm if the mode is the remote mode)',
        0xE289: 'System error', 0xE28A: 'System error', 0xE28B: 'Disconnect the communication due to receive timeout',
        0xE28C: 'Cannot over write the target file', 0xE29C: 'The requested file does not exist or the file size is 0',
        0xE29D: 'System error', 0xE29E: 'System error', 0xE29F: 'System error', 0xE2A0: 'The wrong required pass',
        0xE2A7: 'The relevant file is not in the requested file list', 0xE2AA: 'System error',
        0xE2AF: 'Receive the deletion request of the file that cannot to delete', 0xE2B0: 'System error',
        0xE2B1: 'The directory cannot to be deleted',
        0xE2B2: 'Receive the request of the sending/receiving file at the remote OFF state',
        0xE2B3: 'File not found', 0xE2B4: 'The requested pass is too long',
        0xE444: 'Processing the another command (BUSY condition)', 0xE49D: 'Format error (data size 0)',
        0xE49E: 'Format error (frame size over)', 0xE49F: 'Format error (frame size 0)',
        0xE4A1: 'Format error (block number error)', 0xE4A2: 'Format error (ACK error)',
        0xE4A3: 'Format error (processing category error)', 0xE4A4: 'Format error (access level error)',
        0xE4A5: 'Format error (header size error)', 0xE4A6: 'Format error (identifier error)',
        0xE4A7: 'Format error (the size of the requested command and received frame are different)',
        0xE4A8: 'System error', 0xE4A9: 'System error', 0xFFF0: 'System error', 0xFFF2: 'System error',
        0xFFF3: 'System error', 0xFFF4: 'System error', 0xFFF5: 'System error',
        0xFFF6: 'Too many request and unable to process (BUSY condition)', 0xFFF7: 'System error',
        0xFFF8: 'System error', 0xFFFE: 'The remote mode is detected, and disconnect the communication',
    }

    # threading safe lock
    # It is shared by all instances on purpose (existing behaviour): it also keeps different
    # FS100 objects of one process from talking to the controller at the same time.
    transmission_lock = threading.RLock()

    def __init__(self, ip, timeout=0.8, retries=0, port_robot_control=None, port_file_control=None,
                 local_port_robot_control=None, local_port_file_control=None):
        self.ip = ip
        self.timeout = timeout
        self.retries = retries
        self.port_robot_control = port_robot_control or FS100.UDP_PORT_ROBOT_CONTROL
        self.port_file_control = port_file_control or FS100.UDP_PORT_FILE_CONTROL
        self.local_port_robot_control = local_port_robot_control
        self.local_port_file_control = local_port_file_control
        self.sock = None
        self._req_id = 0

        # number of last error
        self.errno = 0

    # ------------------------------------------------------------------ transport

    def _open_socket(self, port):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(self.timeout)
        local_port = self.local_port_file_control if port == self.port_file_control else self.local_port_robot_control
        if local_port:
            # fixed source port, e.g. for udp-over-tcp, which sends answers to one fixed port.
            # No SO_REUSEADDR: a second client on the same port must fail ("address in use")
            # instead of silently receiving some of the other client's answers.
            try:
                s.bind(('', local_port))
            except OSError:
                s.close()
                raise
        s.connect((self.ip, port))
        return s

    def connect(self, port=None):
        """Open the robot control socket and keep it open until disconnect().

        Without connect(), every request opens and closes its own socket.
        File control commands always use their own socket on the file control port.
        """
        if self.sock is None:
            self.sock = self._open_socket(port or self.port_robot_control)

    def disconnect(self):
        if self.sock is not None:
            self.sock.close()
            self.sock = None

    def connected(self):
        return self.sock is not None

    def _next_req_id(self):
        # The manual asks to increment the request ID for every new command (p. 57). The answer
        # echoes it, which lets transmit() drop late answers to earlier, timed-out requests.
        with FS100.transmission_lock:
            self._req_id = (self._req_id + 1) & 0xff
            return self._req_id

    def generate_error_ans_packet(self, result, errno):
        # when error, result and error number are what callers care about
        p = bytearray(25)
        p += struct.pack('B', result)
        p += bytearray(2)
        p += struct.pack('<H', errno & 0xffff)
        p += bytearray(2)
        return p

    def transmit(self, packet, direction=TRANSMISSION_SEND_AND_RECV, sock=None, retries=None):
        """Send one request packet and return the matching answer (FS100AnsPacket).

        Answers whose request ID or processing division do not match the request are
        discarded (they belong to earlier requests that timed out). On a timeout or a
        socket error an answer with status ERROR_CONNECTION is returned.
        `sock` is used by the file control commands, which run on their own socket.
        """
        # transmission_lock is a reentrant lock. Owning thread may acquire it again without blocking.
        # This prevents sending robot control command during file control transmission.
        with FS100.transmission_lock:
            to_disc = False
            if sock is None:
                if not self.connected():
                    self.connect()
                    to_disc = True
                sock = self.sock
            try:
                retries = self.retries if retries is None else retries
                for attempt in range(retries + 1):
                    ans = self._send_recv(sock, bytes(packet), direction)
                    if ans is None or ans.status != FS100.ERROR_CONNECTION or ans.added_status != FS100.ERROR_CONNECTION:
                        return ans
                    if attempt < retries:
                        log.warning("timeout, resending request (attempt %d/%d)", attempt + 2, retries + 1)
                return ans
            finally:
                if to_disc:
                    self.disconnect()

    def _send_recv(self, sock, packet, direction):
        division = packet[9]
        req_id = packet[11]
        try:
            sock.send(packet)
            if FS100.DEBUG:
                print("PC -> FS100: {}, Len={}".format(packet, len(packet)))
            log.debug("PC -> FS100: %s", packet.hex())
            if direction != FS100.TRANSMISSION_SEND_AND_RECV:
                return None
            deadline = time.monotonic() + self.timeout
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise socket.timeout('timed out')
                sock.settimeout(remaining)
                ans_packet = sock.recv(2048)
                if FS100.DEBUG:
                    print("PC <- FS100: {}, Len={}".format(ans_packet, len(ans_packet)))
                log.debug("PC <- FS100: %s", ans_packet.hex())
                if len(ans_packet) < FS100PacketHeader.HEADER_SIZE or ans_packet[0:4] != b'YERC':
                    log.warning("ignoring malformed answer (%d bytes)", len(ans_packet))
                    continue
                if ans_packet[11] != req_id or ans_packet[9] != division:
                    log.info("ignoring stale answer (request id %d, expected %d)", ans_packet[11], req_id)
                    continue
                return FS100AnsPacket(ans_packet)
        except socket.error as error:
            log.warning("ConnectionError: %s", error)
            errno = error.errno
            if errno is None:
                errno = FS100.ERROR_CONNECTION
            return FS100AnsPacket(self.generate_error_ans_packet(FS100.ERROR_CONNECTION, errno))

    @staticmethod
    def describe_error(status, added_status=0):
        """Return a human readable text for an answer status and added status."""
        if status == FS100.ERROR_SUCCESS:
            return FS100.STATUS_TEXT[0]
        if status == FS100.ERROR_CONNECTION:
            # not an HSE status: set by transmit() when no answer arrived
            if added_status in (0, FS100.ERROR_CONNECTION):
                return 'no answer (timeout or connection error)'
            return 'connection error: {} (errno {})'.format(os.strerror(added_status), added_status)
        text = FS100.STATUS_TEXT.get(status, 'status 0x{:02x}'.format(status))
        if added_status:
            text += ': {} (0x{:04X})'.format(FS100.ADDED_STATUS_TEXT.get(added_status, 'unknown'), added_status)
        return text

    def request(self, cmd_no, inst, attr, service, data=b''):
        """Send a robot control command and return the answer packet (FS100AnsPacket).

        Sets errno. The caller checks ans.status and parses ans.data.
        """
        data = bytes(data)
        req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_ROBOT_CONTROL, self._next_req_id(), cmd_no, inst,
                             attr, service, data, len(data))
        ans = self.transmit(req.to_bytes())
        self.errno = ans.added_status
        return ans

    def _robot_cmd(self, what, cmd_no, inst, attr, service, data=b''):
        ans = self.request(cmd_no, inst, attr, service, data)
        if ans.status != FS100.ERROR_SUCCESS:
            log.warning("failed %s, err=%s (%s)", what, hex(ans.added_status),
                        FS100.describe_error(ans.status, ans.added_status))
        return ans

    # ------------------------------------------------------------------ control commands

    def switch_power(self, power_type, switch):
        """Turn on/off the power supply

        Args:
            power_type (int): Type of power supply. One of following:
                FS100.POWER_TYPE_HOLD,
                FS100.POWER_TYPE_SERVO,
                FS100.POWER_TYPE_HLOCK
            switch (int): Type of switch. One of following:
                FS100.POWER_SWITCH_ON,
                FS100.POWER_SWITCH_OFF

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        return self._robot_cmd("switching power supply", 0x83, power_type, 0x01, 0x10,
                               struct.pack('<I', switch)).status

    def select_cycle(self, cycle_type):
        """Select the way a job in pendant plays

        Args:
            cycle_type (int): Type of playing. One of following:
                FS100.CYCLE_TYPE_STEP,
                FS100.CYCLE_TYPE_ONE_CYCLE,
                FS100.CYCLE_TYPE_CONTINUOUS

        Note:
            If the robot is in hold, FS100.CYCLE_TYPE_CONTINUOUS can be selected
            to resume playing.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        return self._robot_cmd("to select cycle", 0x84, 2, 0x01, 0x10, struct.pack('<I', cycle_type)).status

    def mov(self, move_type, coordinate, speed_class, speed, pos, form=0, extended_form=0, robot_no=1,
                 station_no=0, tool_no=0, user_coor_no=0):
        """Make robot move to a specified position

        Args:
            move_type (int): Type of move path. One of following:
                FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS,
                FS100.MOVE_TYPE_LINEAR_ABSOLUTE_POS,
                FS100.MOVE_TYPE_LINEAR_INCREMENTAL_POS
            coordinate (int): Coordinate system. One of following:
                FS100.MOVE_COORDINATE_SYSTEM_BASE,
                FS100.MOVE_COORDINATE_SYSTEM_ROBOT,
                FS100.MOVE_COORDINATE_SYSTEM_USER,
                FS100.MOVE_COORDINATE_SYSTEM_TOOL
            speed_class (int): Type of move speed. One of following:
                FS100.MOVE_SPEED_CLASS_PERCENT,
                FS100.MOVE_SPEED_CLASS_MILLIMETER,
                FS100.MOVE_SPEED_CLASS_DEGREE
            speed (int): Move speed.
                in 0.01 % for speed type FS100.MOVE_SPEED_CLASS_PERCENT,
                in 0.1 mm/s for speed type FS100.MOVE_SPEED_CLASS_MILLIMETER,
                in 0.1 degree/s for speed type FS100.MOVE_SPEED_CLASS_DEGREE
            pos (tuple): Target position in tuple (x, y, z, Rx, Ry, Rz, Re). x, y, z are in 0.000001 m,
                whereas Rx, Ry, Rz, Re are in 0.0001 degree
            form (int, optional): Robot pose. Defaults to 0.
            extended_form (int, optional): Robot extended pose. Defaults to 0.
            robot_no (int, optional): Robot number (1 to 2). Defaults to 1.
            station_no (int, optional): Station number. Defaults to 0.
            tool_no (int, optional): Tool number (0 to 63). Defaults to 0.
            user_coor_no (int, optional): User coordinate number (0 to 63). Defaults to 0.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        data = struct.pack('<I', robot_no)
        data += struct.pack('<I', station_no)
        data += struct.pack('<I', speed_class)
        data += struct.pack('<I', speed)
        data += struct.pack('<I', coordinate)
        data += struct.pack('<iiiiiii', pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6])
        data += struct.pack('<I', 0)  # reserved
        data += struct.pack('<I', form)
        data += struct.pack('<I', extended_form)
        data += struct.pack('<I', tool_no)
        data += struct.pack('<I', user_coor_no)
        data += bytearray(36)
        return self._robot_cmd("moving to the target position", 0x8a, move_type, 0x01, 0x02, data).status

    def pmov(self, move_type, speed_class, speed, pulse, robot_no=1, station_no=0, tool_no=0):
        """Make robot move to a specified pulse position

        Args:
            move_type (int): Type of move path. One of following:
                FS100.MOVE_TYPE_JOINT_ABSOLUTE_POS,
                FS100.MOVE_TYPE_LINEAR_ABSOLUTE_POS
            speed_class (int): Type of move speed. One of following:
                FS100.MOVE_SPEED_CLASS_PERCENT,
                FS100.MOVE_SPEED_CLASS_MILLIMETER,
                FS100.MOVE_SPEED_CLASS_DEGREE
            speed (int): Move speed.
                in 0.01 % for speed type FS100.MOVE_SPEED_CLASS_PERCENT,
                in 0.1 mm/s for speed type FS100.MOVE_SPEED_CLASS_MILLIMETER,
                in 0.1 degree/s for speed type FS100.MOVE_SPEED_CLASS_DEGREE
            pulse (tuple): Target position in tuple (S, L, U, R, B, T, E).
            robot_no (int, optional): Robot number (1 to 2). Defaults to 1.
            station_no (int, optional): Station number. Defaults to 0.
            tool_no (int, optional): Tool number (0 to 63). Defaults to 0.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        data = struct.pack('<I', robot_no)
        data += struct.pack('<I', station_no)
        data += struct.pack('<I', speed_class)
        data += struct.pack('<I', speed)
        data += struct.pack('<iiiiiii', pulse[0], pulse[1], pulse[2], pulse[3], pulse[4], pulse[5], pulse[6])
        data += struct.pack('<I', 0)  # reserved
        data += struct.pack('<I', tool_no)
        data += bytearray(36)
        return self._robot_cmd("moving to the target position", 0x8b, move_type, 0x01, 0x02, data).status

    # ------------------------------------------------------------------ alarms

    def _parse_alarm(self, data, alarm, extended=False):
        alarm['code'] = struct.unpack('<I', data[0:4])[0]
        alarm['data'] = struct.unpack('<I', data[4:8])[0]
        alarm['type'] = struct.unpack('<I', data[8:12])[0]
        alarm['time'] = _cstr(data[12:28], 'ascii')
        alarm['name'] = _cstr(data[28:60])
        if extended:
            # 0x30A / 0x30B answers (manual pp. 130, 133)
            alarm['subcode_info'] = _cstr(data[60:76])
            alarm['subcode_text'] = _cstr(data[76:172])
            alarm['subcode_reverse'] = _cstr(data[172:268])

    def read_alarm(self, index, alarm):
        """Retrieve info of one of the current alarms (up to four are shown on the pendant)

        Args:
            index (int): 1 for the latest alarm, 2 to 4 for the older ones
            alarm (dict): Where the retrieved info is written to (code, data, type, time, name)

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ans = self._robot_cmd("getting alarm {}".format(index), 0x70, index, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            self._parse_alarm(ans.data, alarm)
        return ans.status

    def get_last_alarm(self, alarm):
        """Retrieve info of the latest alarm

        Args:
            alarm (dict): Where the retrieved info is written to

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        return self.read_alarm(1, alarm)

    def read_alarm_ex(self, index, alarm):
        """Like read_alarm(), but also returns the sub code character strings (command 0x30A)

        Adds keys subcode_info (e.g. '[SV#1]'), subcode_text (e.g. '[SLURBT]') and
        subcode_reverse (which characters the pendant shows reversed).
        """
        ans = self._robot_cmd("getting alarm {}".format(index), 0x30a, index, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            self._parse_alarm(ans.data, alarm, extended=True)
        return ans.status

    def read_alarms(self, alarms, extended=True):
        """Retrieve all current alarms (up to four)

        Args:
            alarms (list): Where the alarms (dicts) are appended, latest first. Empty slots
                (alarm code 0) are left out.
            extended (bool, optional): Use command 0x30A with the sub code strings. Defaults to True.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise the status of the first failed read.
        """
        for index in range(1, 5):
            alarm = dict()
            ret = self.read_alarm_ex(index, alarm) if extended else self.read_alarm(index, alarm)
            if ret != FS100.ERROR_SUCCESS:
                return ret
            if alarm['code'] == 0:
                break
            alarms.append(alarm)
        return FS100.ERROR_SUCCESS

    class AlarmHistoryType(IntEnum):
        MAJOR = 1
        MINOR = 1001
        USER_SYSTEM = 2001
        USER = 3001
        OFFLINE = 4001

    def read_alarm_info(self, alarm_num, alarm_info):
        """Retrieve info of the specified alarm

        Args:
            alarm_num (int): 1 to 100    : Major failure
                             1001 to 1100: Monitor alarm
                             2001 to 2100: User alarm (system)
                             3001 to 3100: User alarm (user)
                             4001 to 4100: Off line alarm
            alarm_info (dict): Where the retrieved info is written to

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ans = self._robot_cmd("to read the alarm info", 0x71, alarm_num, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            self._parse_alarm(ans.data, alarm_info)
        return ans.status

    def read_alarm_info_ex(self, alarm_num, alarm_info):
        """Like read_alarm_info(), but also returns the sub code character strings (command 0x30B)"""
        ans = self._robot_cmd("to read the alarm info", 0x30b, alarm_num, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            self._parse_alarm(ans.data, alarm_info, extended=True)
        return ans.status

    def read_alarm_history(self, history_type, history, extended=True, max_count=100):
        """Retrieve the entries of one alarm history category

        Args:
            history_type (FS100.AlarmHistoryType): Category (its first alarm number)
            history (list): Where the entries (dicts, with an extra key 'num') are appended,
                newest first. Reading stops at the first empty entry (alarm code 0).
            extended (bool, optional): Use command 0x30B with the sub code strings. Defaults to True.
            max_count (int, optional): How many entries to read at most (1 to 100). Defaults to 100.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise the status of the failed read.
        """
        for num in range(int(history_type), int(history_type) + max_count):
            entry = {'num': num}
            ret = self.read_alarm_info_ex(num, entry) if extended else self.read_alarm_info(num, entry)
            if ret != FS100.ERROR_SUCCESS:
                return ret
            if entry['code'] == 0:
                break
            history.append(entry)
        return FS100.ERROR_SUCCESS

    def reset_alarm(self, alarm_type):
        """To reset alarms or cancel errors

        Args:
            alarm_type (int): Type of alarm. One of following:
                FS100.RESET_ALARM_TYPE_ALARM,
                FS100.RESET_ALARM_TYPE_ERROR

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        return self._robot_cmd("resetting alarms or cancelling errors", 0x82, alarm_type, 1, 0x10,
                               struct.pack('<I', 1)).status

    # ------------------------------------------------------------------ status

    def get_status(self, status):
        """Retrieve various status of the robot

        Args:
            status (dict): Where the retrieved status is written to

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ans = self._robot_cmd("getting the status", 0x72, 1, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            data_1 = struct.unpack('<I', ans.data[0:4])[0]
            data_2 = struct.unpack('<I', ans.data[4:8])[0]
            status['step'] = bool(data_1 & 0x01)
            status['one_cycle'] = bool(data_1 & 0x02)
            status['auto_and_cont'] = bool(data_1 & 0x04)
            status['running'] = bool(data_1 & 0x08)
            # The YRC1000 manual calls bit 4 "Speed Limit" (p. 80); kept the old key too.
            status['guard_safe'] = bool(data_1 & 0x10)
            status['speed_limit'] = bool(data_1 & 0x10)
            status['teach'] = bool(data_1 & 0x20)
            status['play'] = bool(data_1 & 0x40)
            status['cmd_remote'] = bool(data_1 & 0x80)
            status['hold_by_pendant'] = bool(data_2 & 0x02)
            # Data 2 bit 2 is blank in the YRC1000 manual (p. 80); FS100 named it external hold.
            status['hold_externally'] = bool(data_2 & 0x04)
            status['hold_by_cmd'] = bool(data_2 & 0x08)
            status['alarming'] = bool(data_2 & 0x10)
            status['error_occurring'] = bool(data_2 & 0x20)
            status['servo_on'] = bool(data_2 & 0x40)
            status['raw'] = (data_1, data_2)
        return ans.status

    def read_executing_job_info(self, info, task=0):
        """Read the info of executing job

        Args:
            info (dict): Where the job info is stored
            task (int, optional): 0 for the master task, 1 to 15 for the sub tasks. Defaults to 0.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ans = self._robot_cmd("to read the info of executing job", 0x73, task + 1, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            info['job_name'] = _cstr(ans.data[0:32])
            info['line_num'] = struct.unpack('<I', ans.data[32:36])[0]
            info['step_num'] = struct.unpack('<I', ans.data[36:40])[0]
            info['speed_override_value'] = struct.unpack('<I', ans.data[40:44])[0]
        return ans.status

    def play_job(self):
        """Start playing a job in pendant

        Note:
            select_job() should be performed before this method.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        return self._robot_cmd("playing job", 0x86, 1, 1, 0x10, struct.pack('<I', 1)).status

    def select_job(self, job_name, line_num=0):
        """Select a job in pendant for later playing

        Args:
            job_name (str): Name of the job file
            line_num (int, optional): The beginning line number when playing. Defaults to 0.

        Raises:
            ValueError: Length of the job name exceeds the maximum 32 characters.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        if job_name.upper().endswith('.JBI'):
            job_name = job_name[:-4]
        data = job_name.encode(encoding=FS100.ENCODING)
        if len(data) > 32:
            raise ValueError('Job name is too long')
        data += bytearray(32 - len(data))
        data += struct.pack('<I', line_num)
        # NOTE: the YRC1000 manual says attribute "fixed to 1" (p. 109); this sends 0 as it always did.
        return self._robot_cmd("selecting the job", 0x87, 1, 0, 0x02, data).status

    # ------------------------------------------------------------------ file control

    def _file_transfer(self, what, service, name_or_filter):
        """Run a file control session that streams data from the controller (save, list).

        Returns (status, data_bytes).
        """
        with FS100.transmission_lock:
            try:
                sock = self._open_socket(self.port_file_control)
            except OSError as error:
                log.warning("ConnectionError: %s", error)
                self.errno = FS100.ERROR_CONNECTION
                return FS100.ERROR_CONNECTION, b''
            try:
                req_id = self._next_req_id()
                content = bytearray()
                data = name_or_filter
                req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, req_id, 0, 0, 0, service,
                                     data, len(data))
                ans = self.transmit(req.to_bytes(), sock=sock, retries=0)
                while ans.status == FS100.ERROR_SUCCESS:
                    content += ans.data
                    req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, req_id, 0, 0, 0, service,
                                         bytearray(0), 0)
                    # not a request but an ack
                    req.ack = FS100PacketHeader.HEADER_ACK_NOT_REQUEST
                    req.block_no = ans.block_no
                    if ans.block_no & 0x80000000 != 0:
                        # The last piece arrived. Send the final ack.
                        self.transmit(req.to_bytes(), FS100.TRANSMISSION_SEND, sock=sock, retries=0)
                        break
                    else:
                        ans = self.transmit(req.to_bytes(), sock=sock, retries=0)
                self.errno = ans.added_status
                if ans.status != FS100.ERROR_SUCCESS:
                    log.warning("failed %s, err=%s (%s)", what, hex(ans.added_status),
                                FS100.describe_error(ans.status, ans.added_status))
                return ans.status, bytes(content)
            finally:
                sock.close()

    def delete_file(self, file_name):
        """Delete a file in pendant

        Args:
            file_name (str): Name of the file to be deleted

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        with FS100.transmission_lock:
            sock = self._open_socket(self.port_file_control)
            try:
                data = file_name.encode(encoding=FS100.ENCODING)
                req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, self._next_req_id(), 0, 0, 0,
                                     0x09, data, len(data))
                ans = self.transmit(req.to_bytes(), sock=sock, retries=0)
                self.errno = ans.added_status
                if ans.status != FS100.ERROR_SUCCESS:
                    log.warning("failed deleting the file, err=%s", hex(ans.added_status))
                return ans.status
            finally:
                sock.close()

    def get_file_list(self, extension, list):
        """Retrieve list of files ended with `extension` in pendant

        Args:
            extension (str): Should be '*.JBI', '*.DAT', '*.CND', '*.PRM', '*.SYS' or '*.LST'
            list (list): Where to store the retrieved list

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        status, raw = self._file_transfer("getting the file list", 0x32, extension.encode(encoding='ascii'))
        if status == FS100.ERROR_SUCCESS:
            # decode once: a multi-byte character may be split across two blocks
            list.extend(raw.decode(FS100.ENCODING, errors='replace').splitlines())
        return status

    def send_file(self, filename):
        """Send a local file to pendant

        Args:
            filename (str): Path of the local file

        Raises:
            ValueError: Empty content of the local file

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        try:
            with open(filename, 'rb') as f:
                context = f.read()
        except FileNotFoundError:
            log.warning("file not found: '%s'", filename)
            self.errno = 0xe2b3
            return FS100.ERROR_NO_SUCH_FILE_OR_DIRECTORY

        if len(context) == 0:
            raise ValueError('An empty file')

        with FS100.transmission_lock:
            sock = self._open_socket(self.port_file_control)
            try:
                CHUNK_SIZE = 400
                block_no = 0
                req_id = self._next_req_id()
                data = ntpath.basename(filename).encode(encoding=FS100.ENCODING)
                req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, req_id, 0, 0, 0, 0x15, data,
                                     len(data))
                ans = self.transmit(req.to_bytes(), sock=sock, retries=0)
                while ans.status == FS100.ERROR_SUCCESS:
                    if ans.block_no & 0x80000000 != 0:
                        # we have sent the last piece of data and this is the final ack from controller
                        break
                    block_no += 1
                    if CHUNK_SIZE * block_no >= len(context):
                        data = context[(block_no - 1) * CHUNK_SIZE:]
                        block_no |= 0x80000000
                    else:
                        data = context[(block_no - 1) * CHUNK_SIZE:block_no * CHUNK_SIZE]
                    req = FS100ReqPacket(FS100PacketHeader.HEADER_DIVISION_FILE_CONTROL, req_id, 0, 0, 0, 0x15, data,
                                         len(data))
                    req.ack = FS100PacketHeader.HEADER_ACK_NOT_REQUEST
                    req.block_no = block_no
                    ans = self.transmit(req.to_bytes(), sock=sock, retries=0)
                self.errno = ans.added_status
                if ans.status != FS100.ERROR_SUCCESS:
                    log.warning("failed sending the file, err=%s", hex(ans.added_status))
                return ans.status
            finally:
                sock.close()

    def read_file(self, filename):
        """Receive a file from pendant into memory

        Args:
            filename (str): Name of the file in pendant, e.g. 'TEST.JBI', 'ALMHIST.DAT' or
                '/SPDRV/CMOSBK.BIN' for the batch data backup

        Returns:
            tuple: (status, content bytes). status is FS100.ERROR_SUCCESS for success, otherwise
                failure and errno attribute indicates the error code.
        """
        return self._file_transfer("receiving the file", 0x16, filename.encode(encoding=FS100.ENCODING))

    def recv_file(self, filename, local_dir):
        """Receive a file from pendant

        Args:
            filename (str): Name of the file in pendant
            local_dir (str): Where in local to save the file

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        if not os.path.isdir(local_dir):
            log.warning("directory not found: '%s'", local_dir)
            self.errno = 0xe2b3
            return FS100.ERROR_NO_SUCH_FILE_OR_DIRECTORY

        status, context = self.read_file(filename)
        if status == FS100.ERROR_SUCCESS:
            # basename: the controller name may carry a path such as /SPDRV/CMOSBK.BIN
            with open(os.path.join(local_dir, ntpath.basename(filename)), 'wb') as f:
                f.write(context)
        return status

    # ------------------------------------------------------------------ axes, position, torque

    def read_axis_name(self, axis_name, robot_no=101):
        """Read the name of each axis

        Args:
            axis_name (dict): Where the name of each axis is stored ('1st_axis' to '8th_axis')
            robot_no (int, optional): Robot number. Defaults to 101 (R1 in cartesian coordinate).

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ans = self._robot_cmd("to read the name of each axis", 0x74, robot_no, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            for n, key in enumerate(_AXIS_KEYS[:len(ans.data) // 4]):
                axis_name[key] = _cstr(ans.data[n * 4:n * 4 + 4])
        return ans.status

    def read_position(self, pos_info, robot_no=101):
        """Read the robot position

        Args:
            pos_info (dict): Where the robot position data is stored
            robot_no (int, optional): Robot number. Defaults to 101 (R1 in cartesian coordinate).

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ans = self._robot_cmd("reading the position info", 0x75, robot_no, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            pos_info['data_type'] = struct.unpack('<I', ans.data[0:4])[0]
            pos_info['form'] = struct.unpack('<I', ans.data[4:8])[0]
            pos_info['tool_no'] = struct.unpack('<I', ans.data[8:12])[0]
            pos_info['user_coor_no'] = struct.unpack('<I', ans.data[12:16])[0]
            pos_info['extended_form'] = struct.unpack('<I', ans.data[16:20])[0]
            pos_info['pos'] = tuple(_ints(ans.data[20:]))
        return ans.status

    def _read_axis_values(self, what, cmd_no, robot_no, data, axes_count):
        ans = self._robot_cmd(what, cmd_no, robot_no, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            for key, value in zip(_AXIS_KEYS[:axes_count], _ints(ans.data)):
                data[key] = value
        return ans.status

    def read_position_error(self, axis_data, robot_no=1, axes_count=8):
        """Read the robot position error data

        Args:
            axis_data (dict): Where the robot position error data is stored (signed pulses)
            robot_no (int, optional): Robot number. Defaults to 1.
            axes_count (int, optional): How many axes to store. Defaults to 8.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        return self._read_axis_values("reading the position error info", 0x76, robot_no, axis_data, axes_count)

    def read_torque(self, torque_data, axes_count=7, robot_no=1):
        """Read the robot torque data of each axis

        Args:
            torque_data (dict): Where the robot torque data is stored
            axes_count (int, optional): How many axes the robot have. Defaults to 7 for backwards compatibility.
            robot_no (int, optional): Robot number. Defaults to 1.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ans = self._robot_cmd("to read the torque data", 0x77, robot_no, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            # The answer carries only the existing axes (6 for a 6-axis robot, 1 for a 1-axis
            # base: checked on a YRC1000 YAS5.10), so map just what arrived.
            for key, value in zip(_AXIS_KEYS[:axes_count], _ints(ans.data)):
                torque_data[key] = value
        return ans.status

    def read_encoder_temperature(self, temperature, group=1, axes_count=8):
        """Read the encoder temperature of each axis (command 0x0411, YAS4.81 and later)

        Args:
            temperature (dict): Where the values are stored ('1st_axis' ...)
            group (int, optional): 1-8 robot R1-R8, 11-18 base B1-B8, 21-44 station S1-S24.
                Defaults to 1.
            axes_count (int, optional): How many axes to store. Defaults to 8.

        Note:
            The manual does not state the unit (degrees Celsius assumed). Attribute is
            sent as 1 ("fixed to 1", p. 144) with Get_Attribute_All.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ans = self._robot_cmd("to read the encoder temperature", 0x0411, group, 1, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            for key, value in zip(_AXIS_KEYS[:axes_count], _ints(ans.data)):
                temperature[key] = value
        return ans.status

    def read_regenerative_power(self, info):
        """Read the amount of regenerative power (command 0x0412, YAS4.90 and later)

        Args:
            info (dict): Filled with 'enabled_boards' (bit mask), 'status' (0 servo off,
                1 consuming, 2 regenerating), 'unit' (0 kWh, 1 10 Wh) and 'boards', a list of
                dicts with 'board', 'today', 'yesterday', 'total' for every enabled board.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ans = self._robot_cmd("to read the regenerative power", 0x0412, 1, 1, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            data = ans.data
            mask = struct.unpack('<I', data[0:4])[0]
            info['enabled_boards'] = mask
            info['status'] = data[4]
            info['unit'] = data[5]
            values = _ints(data[8:])
            info['boards'] = []
            for board in range(8):
                if mask & (1 << board) and len(values) >= board * 3 + 3:
                    today, yesterday, total = values[board * 3:board * 3 + 3]
                    info['boards'].append({'board': board + 1, 'today': today, 'yesterday': yesterday,
                                           'total': total})
        return ans.status

    def read_converter_temperature(self, board=1):
        """Read the converter temperature of one servo board (command 0x0413, YAS5.20 and later)

        Args:
            board (int, optional): Servo board number 1 to 8. Defaults to 1.

        Note:
            The manual does not state the unit (degrees Celsius assumed).

        Returns:
            tuple: (status, temperature or None)
        """
        ans = self._robot_cmd("to read the converter temperature", 0x0413, board, 1, 0x01)
        if ans.status == FS100.ERROR_SUCCESS and len(ans.data) >= 4:
            return ans.status, struct.unpack('<i', ans.data[0:4])[0]
        return ans.status, None

    # ------------------------------------------------------------------ variables

    class VarType(IntEnum):
        IO = 0x78  # 1 byte
        REGISTER = 0x79  # 2 bytes
        BYTE = 0x7a  # 1 byte
        INTEGER = 0x7b  # 2 bytes
        DOUBLE = 0x7c  # 4 bytes
        REAL = 0x7d  # 4 bytes
        STRING = 0x7e  # 16 bytes
        ROBOT_POSITION = 0x7f
        BASE_POSITION = 0x80
        EXTERNAL_AXIS = 0x81
        STRING32 = 0x8c  # 32 bytes, the same S variables with their full 32-byte length

    # bytes per value on the wire
    VAR_SIZE = {
        VarType.IO: 1, VarType.REGISTER: 2, VarType.BYTE: 1, VarType.INTEGER: 2, VarType.DOUBLE: 4,
        VarType.REAL: 4, VarType.STRING: 16, VarType.STRING32: 32, VarType.ROBOT_POSITION: 52,
        VarType.BASE_POSITION: 36, VarType.EXTERNAL_AXIS: 36,
    }

    # plural commands (manual pp. 113-127, 142)
    VAR_PLURAL_CMD = {
        VarType.IO: 0x300, VarType.REGISTER: 0x301, VarType.BYTE: 0x302, VarType.INTEGER: 0x303,
        VarType.DOUBLE: 0x304, VarType.REAL: 0x305, VarType.STRING: 0x306, VarType.ROBOT_POSITION: 0x307,
        VarType.BASE_POSITION: 0x308, VarType.EXTERNAL_AXIS: 0x309, VarType.STRING32: 0x30c,
    }

    # values per plural command: (479 bytes data part - 4 bytes count) / value size,
    # even for the 1-byte types. Matches the maxima in the manual tables (29 S, 14 S32, 9 P, 13 BP/EX).
    VAR_PLURAL_MAX = {
        VarType.IO: 474, VarType.REGISTER: 237, VarType.BYTE: 474, VarType.INTEGER: 237, VarType.DOUBLE: 118,
        VarType.REAL: 118, VarType.STRING: 29, VarType.STRING32: 14, VarType.ROBOT_POSITION: 9,
        VarType.BASE_POSITION: 13, VarType.EXTERNAL_AXIS: 13,
    }

    POSITION_TYPES = (VarType.ROBOT_POSITION, VarType.BASE_POSITION, VarType.EXTERNAL_AXIS)

    class Variable:
        """FS100 Variable

        Attributes:
            type (FS100.VarType): Variable type
            num (int): Variable number
            val (*, optional): Variable value
        """
        def __init__(self, type, num, val=None):
            self.type = type
            self.num = num
            self.val = val

        def set_val(self, raw_bytes):
            if self.type == FS100.VarType.IO:
                self.val = struct.unpack('B', raw_bytes[0:1])[0]
            elif self.type == FS100.VarType.REGISTER:
                self.val = struct.unpack('<H', raw_bytes[0:2])[0]
            elif self.type == FS100.VarType.BYTE:
                self.val = struct.unpack('B', raw_bytes[0:1])[0]
            elif self.type == FS100.VarType.INTEGER:
                self.val = struct.unpack('<h', raw_bytes[0:2])[0]
            elif self.type == FS100.VarType.DOUBLE:
                self.val = struct.unpack('<i', raw_bytes[0:4])[0]
            elif self.type == FS100.VarType.REAL:
                self.val = struct.unpack('<f', raw_bytes[0:4])[0]
            elif self.type in (FS100.VarType.STRING, FS100.VarType.STRING32):
                self.val = _cstr(raw_bytes)
            elif self.type == FS100.VarType.ROBOT_POSITION:
                if self.val is None: self.val = {}
                self.val['data_type'] = struct.unpack('<I', raw_bytes[0:4])[0]
                self.val['form'] = struct.unpack('<I', raw_bytes[4:8])[0]
                self.val['tool_no'] = struct.unpack('<I', raw_bytes[8:12])[0]
                self.val['user_coor_no'] = struct.unpack('<I', raw_bytes[12:16])[0]
                self.val['extended_form'] = struct.unpack('<I', raw_bytes[16:20])[0]
                # 8 axes on the YRC1000 (13 words, p. 99)
                self.val['pos'] = tuple(_ints(raw_bytes[20:52]))
            elif self.type in (FS100.VarType.BASE_POSITION, FS100.VarType.EXTERNAL_AXIS):
                if self.val is None: self.val = {}
                self.val['data_type'] = struct.unpack('<I', raw_bytes[0:4])[0]
                # 8 axes on the YRC1000 (9 words, pp. 101, 103)
                for key, value in zip(_AXIS_KEYS, _ints(raw_bytes[4:36])):
                    self.val[key] = value

        def val_to_bytes(self):
            ret = None
            if self.type == FS100.VarType.IO:
                ret = struct.pack('B', self.val)
            elif self.type == FS100.VarType.REGISTER:
                ret = struct.pack('<H', self.val)
            elif self.type == FS100.VarType.BYTE:
                ret = struct.pack('B', self.val)
            elif self.type == FS100.VarType.INTEGER:
                ret = struct.pack('<h', self.val)
            elif self.type == FS100.VarType.DOUBLE:
                ret = struct.pack('<i', self.val)
            elif self.type == FS100.VarType.REAL:
                ret = struct.pack('<f', self.val)
            elif self.type in (FS100.VarType.STRING, FS100.VarType.STRING32):
                size = FS100.VAR_SIZE[self.type]
                ret = self.val.encode(encoding=FS100.ENCODING)
                if len(ret) > size:
                    raise ValueError('String is longer than {} bytes'.format(size))
                # the controller expects the full fixed-size field (4 or 8 words)
                ret += bytes(size - len(ret))
            elif self.type == FS100.VarType.ROBOT_POSITION:
                pos = (list(self.val['pos']) + [0] * 8)[:8]
                ret = struct.pack('<I', self.val['data_type'])
                ret += struct.pack('<I', self.val['form'])
                ret += struct.pack('<I', self.val['tool_no'])
                ret += struct.pack('<I', self.val['user_coor_no'])
                ret += struct.pack('<I', self.val['extended_form'])
                ret += struct.pack('<8i', *pos)
            elif self.type in (FS100.VarType.BASE_POSITION, FS100.VarType.EXTERNAL_AXIS):
                ret = struct.pack('<I', self.val['data_type'])
                for key in _AXIS_KEYS:
                    ret += struct.pack('<i', self.val.get(key, 0))
            return ret

    def read_variable(self, var):
        """Read a robot variable

        Args:
            var (FS100.Variable): The variable being read

        Note:
            Value of the variable is stored in `val` attribute of var.

        Examples:
            >>> robot = FS100('10.0.0.2')
            >>> var_b0 = FS100.Variable(FS100.VarType.BYTE, 0)
            >>> robot.read_variable(var_b0)
            >>> print("var_b0={}".format(var_b0.val))

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        log.debug("FS100.read_variable(VarType=%s, %s)", var.type, var.num)

        attr = 1
        service = 0x0e
        if var.type in FS100.POSITION_TYPES:
            # get all attributes
            attr = 0
            service = 0x01
        ans = self._robot_cmd("reading the variable", var.type, var.num, attr, service)
        if ans.status == FS100.ERROR_SUCCESS:
            var.set_val(ans.data)
        return ans.status

    def _group_nums(self, list):
        list2 = sorted(list)
        sublist = []

        while list2:
            v = list2.pop(0)

            if not sublist or sublist[-1] in [v, v-1]:
                sublist.append(v)
            else:
                yield sublist
                sublist = [v]

        if sublist:
            yield sublist

    def _plural_chunks(self, vars):
        """Split consecutive vars of one type into chunks a plural command can carry.

        1-byte types need an even count; an odd tail is returned as a single variable.
        """
        size_max = FS100.VAR_PLURAL_MAX[vars[0].type]
        for start in range(0, len(vars), size_max):
            chunk = vars[start:start + size_max]
            if FS100.VAR_SIZE[chunk[0].type] == 1 and len(chunk) % 2 == 1:
                if len(chunk) > 1:
                    yield chunk[:-1]
                yield chunk[-1:]
            else:
                yield chunk

    def _read_consecutive_variables(self, vars):
        """Read multiple consecutive robot variables (of the same type)

        Args:
            vars (list[FS100.Variable]): The variables being read

        Raises:
            ValueError: Empty list given or doesn't contain var objects with the same type or
            var objects numbers not consecutive

        Note:
            Value of each variable is stored in `val` attribute of each var object.
            All var objects in vars must have the same type (i.e., INTEGER) and must have
            consecutive numbers, otherwise failure is immediately returned.
            Lists longer than one plural command can carry are split.

        Examples:
            >>> robot = FS100('10.0.0.2')
            >>> vars = [FS100.Variable(FS100.VarType.INTEGER, 0), FS100.Variable(FS100.VarType.INTEGER, 1)]
            >>> robot.read_variables(vars)
            >>> print([(v.num, v.val) for v in vars])

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        if len(vars) == 0:
            raise ValueError('Input list cannot be empty')
        log.debug("FS100._read_consecutive_variables(VarType=%s, list=%s)", vars[0].type, [v.num for v in vars])

        var = vars[0]
        if any([v.type != var.type for v in vars]):
            raise ValueError('Input list must contain var objects of the same type')

        nums = [v.num for v in vars]
        if sorted(nums) != list(range(min(nums), max(nums) + 1)):
            raise ValueError('Input list must contain var objects with consecutive numbers')

        vars = sorted(vars, key=lambda v: v.num)
        var_type_size = FS100.VAR_SIZE[var.type]
        ret = FS100.ERROR_SUCCESS
        for chunk in self._plural_chunks(vars):
            if len(chunk) == 1:
                ret |= self.read_variable(chunk[0])
                continue
            # data contains number of variables to be read from the starting number
            data = struct.pack('<I', len(chunk))
            ans = self._robot_cmd("reading variables", FS100.VAR_PLURAL_CMD[var.type], chunk[0].num, 0, 0x33, data)
            if ans.status != FS100.ERROR_SUCCESS:
                ret |= ans.status
                continue
            for ix, v in enumerate(chunk):
                # returned data start with 4 bytes with number of variables, then individual consecutive values follow
                v.set_val(ans.data[4 + var_type_size * ix:4 + var_type_size * (ix + 1)])
        return ret

    def read_variables(self, vars):
        """Read multiple robot variables

        Args:
            vars (list[FS100.Variable]): The variables being read

        Note:
            Internally it groups vars by type and divides them into sublists containing var
            objects with consecutive numbers. Then it executes `_read_consecutive_variables`
            to read each sublist with HSE plural commands (as many as needed).

            Value of each variable is stored in `val` attribute of each var object.

        Examples:
            >>> robot = FS100('10.0.0.2')
            >>> vars = [FS100.Variable(FS100.VarType.INTEGER, 3), FS100.Variable(FS100.VarType.INTEGER, 5), FS100.Variable(FS100.VarType.INTEGER, 6)]
            >>> robot.read_variables(vars)
            >>> print([(v.num, v.val) for v in vars])

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ret = FS100.ERROR_SUCCESS
        by_type = {}
        for v in vars:
            by_type.setdefault(v.type, []).append(v)
        for type_vars in by_type.values():
            # duplicates of one number are read once and copied
            vars_dict = {}
            for v in type_vars:
                vars_dict.setdefault(v.num, []).append(v)
            for keys_consecutive in self._group_nums(list(vars_dict.keys())):
                firsts = [vars_dict[k][0] for k in keys_consecutive]
                if len(firsts) == 1:
                    ret |= self.read_variable(firsts[0])
                else:
                    ret |= self._read_consecutive_variables(firsts)
                for k in keys_consecutive:
                    for dup in vars_dict[k][1:]:
                        dup.val = vars_dict[k][0].val
        return ret

    def write_variable(self, var):
        """Write a robot variable

        Args:
            var (FS100.Variable): The variable being written

        Note:
            Value for the variable is specified in `val` attribute.

        Examples:
            >>> robot = FS100('10.0.0.2')
            >>> var_s1 = FS100.Variable(FS100.VarType.STRING, 1, 'Hello, World!')
            >>> robot.write_variable(var_s1)

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        attr = 1
        service = 0x10
        if var.type in FS100.POSITION_TYPES:
            # set all attributes
            attr = 0
            service = 0x02
        data = var.val_to_bytes()
        return self._robot_cmd("writing the variable", var.type, var.num, attr, service, data).status

    def write_variables(self, vars):
        """Write multiple consecutive robot variables of one type with plural commands

        Args:
            vars (list[FS100.Variable]): The variables, all of one type and with consecutive numbers

        Raises:
            ValueError: Empty list, mixed types or numbers not consecutive

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        if len(vars) == 0:
            raise ValueError('Input list cannot be empty')
        if any(v.type != vars[0].type for v in vars):
            raise ValueError('Input list must contain var objects of the same type')
        vars = sorted(vars, key=lambda v: v.num)
        nums = [v.num for v in vars]
        if nums != list(range(nums[0], nums[0] + len(nums))):
            raise ValueError('Input list must contain var objects with consecutive numbers')
        ret = FS100.ERROR_SUCCESS
        for chunk in self._plural_chunks(vars):
            if len(chunk) == 1:
                ret |= self.write_variable(chunk[0])
                continue
            data = struct.pack('<I', len(chunk)) + b''.join(v.val_to_bytes() for v in chunk)
            ret |= self._robot_cmd("writing variables", FS100.VAR_PLURAL_CMD[chunk[0].type], chunk[0].num, 0, 0x34,
                                   data).status
        return ret

    # ------------------------------------------------------------------ system information

    class SystemInfoType(IntEnum):
        R1 = 11
        R2 = 12
        S1 = 21
        S2 = 22
        S3 = 23
        APPLICATION = 101

    def acquire_system_info(self, type, info):
        """Acquire system information

        Args:
            type (FS100.SystemInfoType): Type of system (11-18 R1-R8, 21-44 S1-S24, 101-108 application 1-8)
            info (dict): Where acquired information is stored

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ans = self._robot_cmd("acquiring system info", 0x89, type, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            info['software_version'] = _cstr(ans.data[0:24])
            info['model'] = _cstr(ans.data[24:40])
            info['parameter_version'] = _cstr(ans.data[40:48])
        return ans.status

    class ManagementTimeType(IntEnum):
        CONTROL_POWER_ON = 1
        SERVO_POWER_ON_TOTAL = 10
        SERVO_POWER_ON_R1 = 11
        SERVO_POWER_ON_R2 = 12
        SERVO_POWER_ON_S1 = 21
        SERVO_POWER_ON_S2 = 22
        SERVO_POWER_ON_S3 = 23
        PLAYBACK_TOTAL = 110
        PLAYBACK_R1 = 111
        PLAYBACK_R2 = 112
        PLAYBACK_S1 = 121
        PLAYBACK_S2 = 122
        PLAYBACK_S3 = 123
        MOTION_TOTAL = 210
        MOTION_R1 = 211
        MOTION_R2 = 212
        MOTION_S1 = 221
        MOTION_S2 = 222
        MOTION_S3 = 223
        OPERATION = 301

    def acquire_management_time(self, type, time):
        """Acquire usage time of an action

        Args:
            type (FS100.ManagementTimeType): Type of action
            time (dict): Where acquired usage time is stored ('start' like '2011/10/10 15:49',
                'elapse' like '000000:00'00')

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        ans = self._robot_cmd("acquiring management time", 0x88, type, 0, 0x01)
        if ans.status == FS100.ERROR_SUCCESS:
            time['start'] = _cstr(ans.data[0:16])
            time['elapse'] = _cstr(ans.data[16:28])
        return ans.status

    def show_text_on_pendant(self, text):
        """Show text on pendant

        Args:
            text (str): Text to be shown on pendant

        Raises:
            ValueError: Length of the text exceeds the maximum 30 characters.

        Returns:
            int: FS100.ERROR_SUCCESS for success, otherwise failure and errno attribute
                indicates the error code.
        """
        data = text.encode(encoding=FS100.ENCODING)
        if len(data) > 30:
            raise ValueError('Text is too long')
        data += bytearray(32 - len(data))
        return self._robot_cmd("to show text on pendant", 0x85, 1, 1, 0x10, data).status
