#!/usr/bin/env python3
"""RH-P12-RN gripper driver node.

TCP/DRL 통합 아키텍처:
- Doosan DRL에서 표준 파이썬 socket 모듈로 TCP 서버를 열고,
  이 노드(PC)가 클라이언트로 접속해 Modbus RTU 명령을 전달한다.
- Doosan 내장 server_socket_* API에 버그가 있어 DRL 내부 socket 사용.
- RViz 연동 시 gripper_command_action_enabled: false로 설정 → action 서버 생략.
- tcp_protocol: json | binary  (binary가 레이턴시 낮음)
"""
from __future__ import annotations

import json
import logging
import socket
import struct
import threading
import time
import queue

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import qos_profile_sensor_data

from dsr_msgs2.srv import DrlStart
from sensor_msgs.msg import JointState
from std_msgs.msg import String

try:
    from rclpy.action import ActionServer, CancelResponse, GoalResponse
    from interfaces.action import Grasp as GripperCommand
    _ACTION_AVAILABLE = True
except ImportError:
    _ACTION_AVAILABLE = False
    GripperCommand = None

try:
    from interfaces.srv import GripperSetPosition
    _SET_POS_AVAILABLE = True
except ImportError:
    _SET_POS_AVAILABLE = False
    GripperSetPosition = None

logger = logging.getLogger(__name__)

TCP_MAGIC = b"GP"
TCP_VERSION = 1
TCP_HDR = struct.Struct(">2sBBHH")  # magic(2) ver(1) type(1) seq(2) payload_len(2)

TCP_T_PING  = 1
TCP_T_PONG  = 2
TCP_T_CMD   = 3
TCP_T_ACK   = 4
TCP_T_STATE = 5
TCP_T_STOP  = 6

# binary STATE payload format: cur(i16) pos(i32) gcur(i16) gpos_lo(u16) gpos_hi(u16)
_STATE_FMT = ">hihHH"
_STATE_SIZE = struct.calcsize(_STATE_FMT)


class ModbusRTU:
    @staticmethod
    def crc16(data: bytes) -> bytes:
        crc = 0xFFFF
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        return struct.pack("<H", crc)

    @classmethod
    def fc06(cls, slave_id: int, start: int, value: int) -> bytes:
        body = bytes([slave_id, 0x06]) + struct.pack(">HH", start, value)
        return body + cls.crc16(body)

    @classmethod
    def fc16(cls, slave_id: int, start: int, num_regs: int, values: list) -> bytes:
        body = bytes([slave_id, 0x10]) + struct.pack(">HHB", start, num_regs, len(values) * 2)
        for val in values:
            body += struct.pack(">H", val)
        return body + cls.crc16(body)

    @classmethod
    def fc03(cls, slave_id: int, start: int, count: int) -> bytes:
        body = bytes([slave_id, 0x03]) + struct.pack(">HH", start, count)
        return body + cls.crc16(body)


def build_drl_write_packets(packets: list[bytes], wait_between: float = 0.1, tail_wait: float = 0.2) -> str:
    lines = [
        "flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)",
        "wait(0.1)",
        "def _flush():",
        "    flange_serial_read(0.05)",
        "_flush()",
    ]
    for pkt in packets:
        byte_str = "b'" + "".join([f"\\x{x:02x}" for x in pkt]) + "'"
        lines.append(f"flange_serial_write({byte_str})")
        lines.append(f"wait({wait_between})")
        lines.append("_flush()")
    if tail_wait > 0:
        lines.append(f"wait({tail_wait})")
    lines.append("flange_serial_close()")
    return "\n".join(lines) + "\n"


def build_drl_move_and_poll(
    slave_id: int,
    target_pulse: int,
    target_current: int,
    grip_current_threshold: int,
    pos_tolerance: int = 20,
    max_loops: int = 50,
) -> str:
    cur_move_pkt = "b'" + "".join([f"\\x{x:02x}" for x in ModbusRTU.fc06(slave_id, 275, target_current)]) + "'"
    pos_move_pkt = "b'" + "".join([f"\\x{x:02x}" for x in ModbusRTU.fc16(slave_id, 282, 2, [target_pulse, 0])]) + "'"
    pos_read_pkt = "b'" + "".join([f"\\x{x:02x}" for x in ModbusRTU.fc03(slave_id, 284, 2)]) + "'"
    cur_read_pkt = "b'" + "".join([f"\\x{x:02x}" for x in ModbusRTU.fc03(slave_id, 287, 1)]) + "'"

    code = (
        "flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)\n"
        "wait(0.1)\n"
        "def _flush():\n"
        "    flange_serial_read(0.05)\n"
        "def _read_cur():\n"
        "    for _i in range(3):\n"
        "        _flush()\n"
        f"        flange_serial_write({cur_read_pkt})\n"
        "        wait(0.05)\n"
        "        _sz, _val = flange_serial_read(0.3)\n"
        "        if _sz >= 7 and _val[1] == 3:\n"
        "            _v = (_val[3] << 8) | _val[4]\n"
        "            if _v > 32767:\n"
        "                _v = _v - 65536\n"
        "            return _v\n"
        "    return -99999\n"
        "def _read_pos():\n"
        "    for _i in range(3):\n"
        "        _flush()\n"
        f"        flange_serial_write({pos_read_pkt})\n"
        "        wait(0.05)\n"
        "        _sz, _val = flange_serial_read(0.3)\n"
        "        if _sz >= 9 and _val[1] == 3:\n"
        "            _hi = (_val[3] << 8) | _val[4]\n"
        "            _lo = (_val[5] << 8) | _val[6]\n"
        "            return (_hi << 16) | _lo\n"
        "    return -99999\n"
        "_flush()\n"
        f"flange_serial_write({cur_move_pkt})\n"
        "wait(0.3)\n"
        "_flush()\n"
        f"flange_serial_write({pos_move_pkt})\n"
        "wait(0.5)\n"
        "_flush()\n"
        "__done = False\n"
        f"__loop = {max_loops}\n"
        "while not __done and __loop > 0:\n"
        "    __loop = __loop - 1\n"
        "    __cur = _read_cur()\n"
        "    if __cur != -99999:\n"
        f"        if abs(__cur) > {int(grip_current_threshold)}:\n"
        "            __done = True\n"
        "            break\n"
        "    __pos = _read_pos()\n"
        "    if __pos != -99999:\n"
        f"        if __pos >= {int(target_pulse - pos_tolerance)} and __pos <= {int(target_pulse + pos_tolerance)}:\n"
        "            __done = True\n"
        "            break\n"
        "    wait(0.15)\n"
        "flange_serial_close()\n"
    )
    return code


# DRL 서버 코드 — 로봇 내장 DRL 환경에서 실행됨
# Doosan 내장 server_socket_* API 버그로 인해 표준 파이썬 socket 사용
# PROTOCOL="json" | "binary" 로 프로토콜 선택 가능
DRL_SERVER_CODE = """\
import socket
import json
import select
import time
import struct

SLAVE_ID = __SLAVE_ID__
TCP_PORT = __TCP_PORT__
PROTOCOL = "__TCP_PROTOCOL__"
STATE_PERIOD_SEC = __STATE_PERIOD_SEC__
TCP_STATE_STREAM_ENABLED = __TCP_STATE_STREAM_ENABLED__
TCP_CMD_FRAME_WAIT_SEC = __TCP_CMD_FRAME_WAIT_SEC__
PRESENT_CURRENT_REG = __PRESENT_CURRENT_REG__
PRESENT_POSITION_REG = __PRESENT_POSITION_REG__
PRESENT_POSITION_REGS = __PRESENT_POSITION_REGS__
GOAL_POS_REG = __GOAL_POS_REG__
GOAL_CUR_REG = __GOAL_CUR_REG__
SNAP_ENABLED = __SNAP_ENABLED__
PLC_FEEDBACK_ENABLED = __PLC_FEEDBACK_ENABLED__
PLC_ADDR_POS = __PLC_ADDR_POS__
PLC_ADDR_CUR = __PLC_ADDR_CUR__
PLC_ADDR_CODE = __PLC_ADDR_CODE__

flange_serial_open(baudrate=57600, bytesize=DR_EIGHTBITS, parity=DR_PARITY_NONE, stopbits=DR_STOPBITS_ONE)
wait(0.1)

def _flush():
    for _k in range(10):
        _sz, _val = flange_serial_read(0.01)
        if _sz <= 0:
            break

def _crc16(_data):
    _crc = 0xFFFF
    for _b in _data:
        _crc = _crc ^ _b
        for _i in range(8):
            if (_crc & 1) != 0:
                _crc = (_crc >> 1) ^ 0xA001
            else:
                _crc = (_crc >> 1)
    return _crc

def _fc03(_addr, _count):
    _pkt = bytes([SLAVE_ID, 0x03, (_addr >> 8) & 0xff, _addr & 0xff, (_count >> 8) & 0xff, _count & 0xff])
    _c = _crc16(_pkt)
    return _pkt + bytes([_c & 0xff, (_c >> 8) & 0xff])

def _fc06(_addr, _val):
    _pkt = bytes([SLAVE_ID, 0x06, (_addr >> 8) & 0xff, _addr & 0xff, (_val >> 8) & 0xff, _val & 0xff])
    _c = _crc16(_pkt)
    return _pkt + bytes([_c & 0xff, (_c >> 8) & 0xff])

def _read_cur():
    for _i in range(3):
        _flush()
        flange_serial_write(_fc03(PRESENT_CURRENT_REG, 1))
        wait(0.02)
        _sz, _val = flange_serial_read(0.1)
        if _sz >= 7 and _val[1] == 3 and _val[2] == 2:
            _v = (_val[3] << 8) | _val[4]
            if _v > 32767:
                _v = _v - 65536
            return _v
    return -99999

def _read_reg16(_addr):
    for _i in range(2):
        _flush()
        flange_serial_write(_fc03(_addr, 1))
        wait(0.02)
        _sz, _val = flange_serial_read(0.1)
        if _sz >= 7 and _val[1] == 3 and _val[2] == 2:
            return ((_val[3] << 8) | _val[4])
    return -99999

def _read_pos():
    for _i in range(3):
        _flush()
        flange_serial_write(_fc03(PRESENT_POSITION_REG, PRESENT_POSITION_REGS))
        wait(0.02)
        _sz, _val = flange_serial_read(0.1)
        if PRESENT_POSITION_REGS == 1:
            if _sz >= 7 and _val[1] == 3 and _val[2] == 2:
                return ((_val[3] << 8) | _val[4])
        else:
            if _sz >= 9 and _val[1] == 3 and _val[2] == 4:
                _low = ((_val[3] << 8) | _val[4])
                _high = ((_val[5] << 8) | _val[6])
                _v = _low + (_high << 16)
                if _v >= 2147483648:
                    _v = _v - 4294967296
                return _v
    return -99999

def _read_cur_pos_bulk():
    try:
        _pos_regs = int(PRESENT_POSITION_REGS)
        if _pos_regs < 1:
            _pos_regs = 1
        _cur_reg = int(PRESENT_CURRENT_REG)
        _pos_reg = int(PRESENT_POSITION_REG)
        _start = _cur_reg if _cur_reg < _pos_reg else _pos_reg
        _end = _cur_reg if _cur_reg > (_pos_reg + _pos_regs - 1) else (_pos_reg + _pos_regs - 1)
        _count = (_end - _start) + 1
        if _count <= 0 or _count > 16:
            return _read_cur(), _read_pos()
        for _i in range(3):
            _flush()
            flange_serial_write(_fc03(_start, _count))
            wait(0.02)
            _sz, _val = flange_serial_read(0.1)
            if _sz < (5 + 2*_count) or _val[1] != 3:
                continue
            _data = _val[3:3+2*_count]
            def _reg_u16(_addr):
                _idx = _addr - _start
                if _idx < 0 or _idx >= _count:
                    return 0
                return (_data[2*_idx] << 8) | _data[2*_idx + 1]
            _cur_u = _reg_u16(_cur_reg)
            _cur = _cur_u - 65536 if _cur_u > 32767 else _cur_u
            if _pos_regs == 1:
                _pos = _reg_u16(_pos_reg)
            else:
                _low = _reg_u16(_pos_reg)
                _high = _reg_u16(_pos_reg + 1)
                _pos = _low + (_high << 16)
                if _pos >= 2147483648:
                    _pos = _pos - 4294967296
            return _cur, _pos
    except:
        pass
    return _read_cur(), _read_pos()

def _plc_write_int(_addr, _val):
    try:
        if not PLC_FEEDBACK_ENABLED:
            return
        _a = max(0, min(23, int(_addr)))
        set_output_register_int(_a, int(_val))
    except:
        pass

_flush()
flange_serial_write(_fc06(256, 1))
wait(0.2)
_flush()

MAGIC = b"GP"
VERSION = 1
HDR = struct.Struct(">2sBBHH")
T_PING = 1
T_PONG = 2
T_CMD  = 3
T_ACK  = 4
T_STATE= 5
T_STOP = 6

_srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
_srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    _srv.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
except Exception:
    pass
_srv.bind(('0.0.0.0', TCP_PORT))
_srv.listen(1)
_srv.settimeout(30.0)
tp_log("GP TCP Server port=" + str(TCP_PORT) + " proto=" + PROTOCOL)

_conn = None
try:
    _conn, _addr = _srv.accept()
    _conn.settimeout(0.05)
    try:
        _conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception:
        pass
except Exception:
    pass

if _conn:
    def _sendall(_b):
        try:
            _off = 0
            while _off < len(_b):
                _sent = _conn.send(_b[_off:])
                if _sent <= 0:
                    break
                _off = _off + _sent
        except:
            pass

    def _send_obj(_obj):
        try:
            _payload = json.dumps(_obj).encode('utf-8')
            _sendall(bytes([(len(_payload) >> 8) & 0xff, len(_payload) & 0xff]) + _payload)
        except:
            pass

    def _send_bin(_type, _seq, _payload):
        try:
            _hdr = HDR.pack(MAGIC, VERSION, int(_type), int(_seq) & 0xFFFF, len(_payload))
            _sendall(_hdr + _payload)
        except:
            pass

    if PROTOCOL == "binary":
        _send_bin(T_PONG, 0, b"")
    else:
        _send_obj({"type": "hello", "slave_id": SLAVE_ID, "tcp_port": TCP_PORT})
    _plc_write_int(PLC_ADDR_CODE, 0)

    try:
        _conn.setblocking(False)
    except Exception:
        pass
    _rxbuf = b""
    _last_state_t = 0.0

    def _recv_bin_packets():
        global _rxbuf
        packets = []
        while True:
            try:
                r, _, _ = select.select([_conn], [], [], 0)
                if not r:
                    break
                _raw = _conn.recv(2048)
                if _raw == b"":
                    return None
                if _raw:
                    if b"STOP" in _raw:
                        return "STOP"
                    _rxbuf = _rxbuf + _raw
            except Exception:
                break
        while True:
            if len(_rxbuf) < HDR.size:
                break
            try:
                _magic, _ver, _type, _seq, _plen = HDR.unpack(_rxbuf[:HDR.size])
            except:
                _rxbuf = _rxbuf[1:]
                continue
            if _magic != MAGIC or _ver != VERSION or _plen < 0 or _plen > 65535:
                _rxbuf = _rxbuf[1:]
                continue
            if len(_rxbuf) < HDR.size + _plen:
                break
            _payload = _rxbuf[HDR.size:HDR.size+_plen]
            _rxbuf = _rxbuf[HDR.size+_plen:]
            packets.append((_type, _seq, _payload))
        return packets

    def _drain_rx_json():
        global _rxbuf
        msgs = []
        while True:
            try:
                r, _, _ = select.select([_conn], [], [], 0)
                if not r:
                    break
                _raw = _conn.recv(2048)
                if _raw == b"":
                    return None
                if _raw:
                    if b"STOP" in _raw:
                        return "STOP"
                    _rxbuf = _rxbuf + _raw
            except Exception:
                break
        while len(_rxbuf) >= 2:
            _n = (_rxbuf[0] << 8) | _rxbuf[1]
            if len(_rxbuf) < 2 + _n:
                break
            _payload = _rxbuf[2:2+_n]
            _rxbuf = _rxbuf[2+_n:]
            try:
                msgs.append(json.loads(_payload.decode('utf-8', errors='ignore')))
            except:
                pass
        return msgs

    while True:
        if PROTOCOL == "binary":
            _pkts = _recv_bin_packets()
            if _pkts is None or _pkts == "STOP":
                break
            for _type, _seq, _payload in (_pkts or []):
                if _type == T_PING:
                    _send_bin(T_PONG, _seq, b"")
                    continue
                if _type == T_STOP:
                    break
                if _type != T_CMD:
                    continue
                _ok = True
                _err = ""
                try:
                    if len(_payload) < 2:
                        _ok = False
                    else:
                        _n = (_payload[0] << 8) | _payload[1]
                        _off = 2
                        _flush()
                        for _k in range(_n):
                            if _off + 2 > len(_payload):
                                _ok = False
                                break
                            _flen = (_payload[_off] << 8) | _payload[_off+1]
                            _off = _off + 2
                            if _off + _flen > len(_payload):
                                _ok = False
                                break
                            _pkt = _payload[_off:_off+_flen]
                            _off = _off + _flen
                            flange_serial_write(_pkt)
                            wait(TCP_CMD_FRAME_WAIT_SEC)
                            _flush()
                except Exception as _e:
                    _ok = False
                    _err = str(_e)
                try:
                    _eb = (_err or "").encode("utf-8") if (not _ok and _err) else b""
                    if len(_eb) > 200:
                        _eb = _eb[:200]
                    _ap = bytes([1 if _ok else 0]) + bytes([(len(_eb) >> 8) & 0xFF, len(_eb) & 0xFF]) + _eb
                    _send_bin(T_ACK, _seq, _ap)
                except:
                    pass
                _plc_write_int(PLC_ADDR_CODE, 1 if _ok else -1)
        else:
            _cmd_msgs = _drain_rx_json()
            if _cmd_msgs is None or _cmd_msgs == "STOP":
                break
            if _cmd_msgs:
                for _m in _cmd_msgs:
                    if _m.get("type", "") == "ping":
                        _send_obj({"type": "pong", "t": 0})
                        continue
                    _cmd_id = _m.get("id", 0)
                    _frames = _m.get("frames", [])
                    _flush()
                    _ok = True
                    _err = ""
                    try:
                        for _hx in _frames:
                            _pkt = bytes.fromhex(_hx)
                            flange_serial_write(_pkt)
                            wait(TCP_CMD_FRAME_WAIT_SEC)
                            _flush()
                    except Exception as _e:
                        _ok = False
                        _err = str(_e)
                    _send_obj({"type": "ack", "id": _cmd_id, "ok": _ok, "err": _err})
                    _plc_write_int(PLC_ADDR_CODE, 1 if _ok else -1)

                    if SNAP_ENABLED:
                        try:
                            _snap = {}
                            for _addr in range(270, 291):
                                _snap[str(_addr)] = _read_reg16(_addr)
                            _send_obj({"type": "snap", "id": _cmd_id, "snap": _snap})
                        except:
                            pass

        _now = time.time()
        if TCP_STATE_STREAM_ENABLED and (_now - _last_state_t) >= STATE_PERIOD_SEC:
            _last_state_t = _now
            cur_val, pos_val = _read_cur_pos_bulk()
            if cur_val != -99999 and pos_val != -99999:
                _plc_write_int(PLC_ADDR_CUR, cur_val)
                _plc_write_int(PLC_ADDR_POS, pos_val)
                gcur = _read_reg16(GOAL_CUR_REG)
                gpos_lo = _read_reg16(GOAL_POS_REG)
                gpos_hi = _read_reg16(GOAL_POS_REG + 1)
                if PROTOCOL == "binary":
                    try:
                        _p = struct.pack(">hihHH", int(cur_val), int(pos_val), int(gcur), int(gpos_lo) & 0xFFFF, int(gpos_hi) & 0xFFFF)
                    except Exception:
                        _p = b""
                    _send_bin(T_STATE, 0, _p)
                else:
                    _send_obj({
                        "type": "state",
                        "cur": cur_val,
                        "pos": pos_val,
                        "pcur_reg": PRESENT_CURRENT_REG,
                        "gcur_reg": GOAL_CUR_REG,
                        "gpos_reg": GOAL_POS_REG,
                        "gcur": gcur,
                        "gpos_lo": gpos_lo,
                        "gpos_hi": gpos_hi,
                    })

        wait(0.005)

    _conn.close()

_srv.close()
flange_serial_close()
tp_log("GP TCP Server closed.")
"""


class GripperNode(Node):
    def __init__(self) -> None:
        super().__init__("gripper_node")
        self._cb = ReentrantCallbackGroup()

        # ---- 파라미터 선언 ----
        self.declare_parameter("robot_ns", "dsr01")
        self.declare_parameter("robot_ip", "110.120.1.40")
        self.declare_parameter("robot_port", 9000)
        self.declare_parameter("tcp_external_server", False)
        self.declare_parameter("state_hz", 10.0)
        self.declare_parameter("grip_current_threshold", 50)
        self.declare_parameter("position_scale", 1.0)
        self.declare_parameter("position_use_low_word", False)
        self.declare_parameter("position_word_order", "hi_lo")
        self.declare_parameter("goal_position_scale", 1.0)
        self.declare_parameter("command_transport", "drl")
        self.declare_parameter("slave_id", 1)
        self.declare_parameter("present_current_reg", 287)
        self.declare_parameter("present_position_reg", 290)
        self.declare_parameter("present_position_regs", 2)
        self.declare_parameter("goal_current_reg", 275)
        self.declare_parameter("goal_position_reg", 282)
        self.declare_parameter("goal_position_write_mode", "fc16")  # auto|fc06|fc16
        self.declare_parameter("goal_position_regs", 2)
        self.declare_parameter("drl_snap_enabled", False)
        self.declare_parameter("plc_feedback_enabled", False)
        self.declare_parameter("plc_addr_pos", 0)
        self.declare_parameter("plc_addr_cur", 2)
        self.declare_parameter("plc_addr_code", 1)
        self.declare_parameter("tcp_ack_timeout_sec", 3.0)
        self.declare_parameter("tcp_state_hz", 20.0)       # 20Hz 기본 (binary 모드 효과 극대화)
        self.declare_parameter("tcp_state_stream_enabled", True)
        self.declare_parameter("tcp_cmd_frame_wait_sec", 0.02)
        self.declare_parameter("tcp_protocol", "json")     # json | binary
        self.declare_parameter("tcp_watchdog_enabled", True)
        self.declare_parameter("tcp_watchdog_period_sec", 1.0)
        self.declare_parameter("tcp_watchdog_stale_sec", 2.5)
        self.declare_parameter("tcp_rx_buf_max_bytes", 65536)
        self.declare_parameter("tcp_rx_buf_keep_bytes", 8192)
        self.declare_parameter("done_pos_tolerance", 20)
        self.declare_parameter("done_min_motion", 10)
        self.declare_parameter("done_require_reached", True)
        self.declare_parameter("grip_detect_enabled", True)
        self.declare_parameter("action_max_wait_sec", 20.0)
        self.declare_parameter("pulse_open", 100)
        self.declare_parameter("pulse_closed_preset", 420)
        self.declare_parameter("init_current", 400)
        self.declare_parameter("grip_current", 300)
        self.declare_parameter("gripper_command_action_enabled", False)
        self.declare_parameter("direct_cmd_topic_enabled", False)
        self.declare_parameter("direct_cmd_topic", "/gripper/cmd_direct")
        self.declare_parameter("direct_cmd_latest_only", True)
        self.declare_parameter("direct_cmd_streaming", True)
        self.declare_parameter("direct_cmd_streaming_ack_timeout_sec", 0.5)
        self.declare_parameter("direct_cmd_streaming_no_ack", True)
        self.declare_parameter("mode", "real")
        self.declare_parameter("set_position_service_enabled", True)
        self.declare_parameter("set_position_service", "/gripper/set_position")

        # ---- 파라미터 읽기 ----
        ns = str(self.get_parameter("robot_ns").value).strip()
        self._prefix = f"/{ns}" if ns else ""
        self._robot_ip = str(self.get_parameter("robot_ip").value).strip()
        self._tcp_port = int(self.get_parameter("robot_port").value)
        self._tcp_external = bool(self.get_parameter("tcp_external_server").value)
        self._state_hz = float(self.get_parameter("state_hz").value)
        self._grip_threshold = float(self.get_parameter("grip_current_threshold").value)
        self._pos_scale = float(self.get_parameter("position_scale").value)
        self._pos_use_low = bool(self.get_parameter("position_use_low_word").value)
        self._pos_word_order = str(self.get_parameter("position_word_order").value).lower()
        self._goal_pos_scale = float(self.get_parameter("goal_position_scale").value)
        self._cmd_transport = str(self.get_parameter("command_transport").value).lower()
        self._slave_id = int(self.get_parameter("slave_id").value)
        self._present_current_reg = int(self.get_parameter("present_current_reg").value)
        self._present_position_reg = int(self.get_parameter("present_position_reg").value)
        self._present_position_regs = int(self.get_parameter("present_position_regs").value)
        self._goal_cur_reg = int(self.get_parameter("goal_current_reg").value)
        self._goal_pos_reg = int(self.get_parameter("goal_position_reg").value)
        self._goal_pos_write_mode = str(self.get_parameter("goal_position_write_mode").value).lower()
        self._goal_pos_regs = int(self.get_parameter("goal_position_regs").value)
        self._drl_snap_enabled = bool(self.get_parameter("drl_snap_enabled").value)
        self._plc_feedback_enabled = bool(self.get_parameter("plc_feedback_enabled").value)
        self._plc_addr_pos = int(self.get_parameter("plc_addr_pos").value)
        self._plc_addr_cur = int(self.get_parameter("plc_addr_cur").value)
        self._plc_addr_code = int(self.get_parameter("plc_addr_code").value)
        self._tcp_ack_timeout = float(self.get_parameter("tcp_ack_timeout_sec").value)
        self._tcp_state_hz = float(self.get_parameter("tcp_state_hz").value)
        self._tcp_state_stream_enabled = bool(self.get_parameter("tcp_state_stream_enabled").value)
        self._tcp_cmd_frame_wait_sec = max(0.0, float(self.get_parameter("tcp_cmd_frame_wait_sec").value))
        self._tcp_protocol = str(self.get_parameter("tcp_protocol").value).lower()
        if self._tcp_protocol not in ("json", "binary"):
            self._tcp_protocol = "json"
        self._tcp_wd_enabled = bool(self.get_parameter("tcp_watchdog_enabled").value)
        self._tcp_wd_period = float(self.get_parameter("tcp_watchdog_period_sec").value)
        self._tcp_wd_stale = float(self.get_parameter("tcp_watchdog_stale_sec").value)
        self._tcp_rx_max = int(self.get_parameter("tcp_rx_buf_max_bytes").value)
        self._tcp_rx_keep = int(self.get_parameter("tcp_rx_buf_keep_bytes").value)
        self._done_tol = int(self.get_parameter("done_pos_tolerance").value)
        self._done_min_motion = int(self.get_parameter("done_min_motion").value)
        self._done_require_reached = bool(self.get_parameter("done_require_reached").value)
        self._grip_enabled = bool(self.get_parameter("grip_detect_enabled").value)
        self._action_max_wait = float(self.get_parameter("action_max_wait_sec").value)
        self._pulse_open = int(self.get_parameter("pulse_open").value)
        self._pulse_closed = int(self.get_parameter("pulse_closed_preset").value)
        self._cur_init = int(self.get_parameter("init_current").value)
        self._cur_grip = int(self.get_parameter("grip_current").value)
        self._action_enabled = bool(self.get_parameter("gripper_command_action_enabled").value)
        self._direct_cmd_enabled = bool(self.get_parameter("direct_cmd_topic_enabled").value)
        self._direct_cmd_topic = str(self.get_parameter("direct_cmd_topic").value)
        self._direct_cmd_latest_only = bool(self.get_parameter("direct_cmd_latest_only").value)
        self._direct_cmd_streaming = bool(self.get_parameter("direct_cmd_streaming").value)
        self._direct_cmd_streaming_ack_timeout = float(self.get_parameter("direct_cmd_streaming_ack_timeout_sec").value)
        self._direct_cmd_streaming_no_ack = bool(self.get_parameter("direct_cmd_streaming_no_ack").value)
        self._mode = str(self.get_parameter("mode").value).lower()
        self._set_pos_svc_enabled = bool(self.get_parameter("set_position_service_enabled").value)
        self._set_pos_svc_name = str(self.get_parameter("set_position_service").value)

        if self._plc_feedback_enabled:
            self._plc_addr_pos = self._clamp_plc_out_int_addr(self._plc_addr_pos, "plc_addr_pos")
            self._plc_addr_cur = self._clamp_plc_out_int_addr(self._plc_addr_cur, "plc_addr_cur")
            self._plc_addr_code = self._clamp_plc_out_int_addr(self._plc_addr_code, "plc_addr_code")

        # ---- 상태 변수 ----
        self._current_hz_pos: int = 0
        self._current_hz_cur: int = 0
        self._last_gpos_lo: int | None = None
        self._last_state_rx_t: float = 0.0
        self._sock: socket.socket | None = None
        self._socket_active: bool = False
        self._tcp_rx_buf: bytes = b""
        self._ack_lock = threading.Lock()
        self._ack_waiters: dict[int, threading.Event] = {}
        self._ack_results: dict[int, dict] = {}
        self._next_cmd_id: int = 1
        self._tcp_hello_seen: bool = False
        self._tcp_pong_seen: bool = False
        self._tcp_state_seen: bool = False
        self._tcp_ack_seen: bool = False
        self._tcp_sniff_logged: bool = False
        self._recv_thread: threading.Thread | None = None
        self._last_pong_rx_t: float = 0.0
        self._tcp_reconnect_lock = threading.Lock()
        self._tcp_reconnect_inflight: bool = False
        self._executing: bool = False
        self._last_sent_direct_pulse: int | None = None
        self._last_sent_direct_cur: int | None = None
        self._direct_stream_lock = threading.Lock()

        # ---- ROS 인터페이스 ----
        self._cli_drl = self.create_client(
            DrlStart, f"{self._prefix}/drl/drl_start", callback_group=self._cb
        )
        self._state_pub = self.create_publisher(JointState, "/gripper/state", qos_profile_sensor_data)
        self.create_timer(1.0 / self._state_hz, self._publish_state, callback_group=self._cb)

        self._direct_cmd_q: queue.Queue | None = None
        self._direct_cmd_worker_thread: threading.Thread | None = None
        if self._direct_cmd_enabled:
            self._direct_cmd_q = queue.Queue(maxsize=20)
            self._direct_cmd_worker_thread = threading.Thread(
                target=self._direct_cmd_worker_loop, daemon=True
            )
            self._direct_cmd_worker_thread.start()
            self.create_subscription(
                String, self._direct_cmd_topic, self._on_direct_cmd, 10, callback_group=self._cb
            )

        if self._set_pos_svc_enabled and _SET_POS_AVAILABLE:
            self.create_service(
                GripperSetPosition,
                self._set_pos_svc_name,
                self._set_position_callback,
                callback_group=self._cb,
            )
            self.get_logger().info(f"[gripper] SetPosition 서비스: {self._set_pos_svc_name}")
        elif self._set_pos_svc_enabled and not _SET_POS_AVAILABLE:
            self.get_logger().warn("[gripper] GripperSetPosition import 불가 — 서비스 생략")

        if self._action_enabled:
            if not _ACTION_AVAILABLE:
                self.get_logger().error(
                    "gripper_command_action_enabled=true 이지만 action 인터페이스 import 불가 — action 서버 생략"
                )
            else:
                ActionServer(
                    self,
                    GripperCommand,
                    "/gripper/grasp",
                    execute_callback=self._execute_callback,
                    goal_callback=lambda _: GoalResponse.ACCEPT,
                    cancel_callback=lambda _: CancelResponse.ACCEPT,
                    callback_group=self._cb,
                )

        self.add_on_set_parameters_callback(self._on_param_set)

        if self._tcp_wd_enabled and self._cmd_transport == "tcp":
            threading.Thread(target=self._tcp_watchdog_loop, daemon=True).start()

        self._init_timer = self.create_timer(1.0, self._init_drl_server, callback_group=self._cb)
        self.get_logger().info(
            f"[gripper] transport={self._cmd_transport} proto={self._tcp_protocol} "
            f"ip={self._robot_ip}:{self._tcp_port} action={'on' if self._action_enabled else 'off'}"
        )

    # ------------------------------------------------------------------ helpers

    def _tcp_state_period_sec(self) -> float:
        if not self._tcp_state_stream_enabled:
            return 999999.0
        hz = min(50.0, max(0.0, float(self._tcp_state_hz)))
        return 1.0 / hz if hz > 0 else 999999.0

    def _build_drl_server_code(self) -> str:
        return (
            DRL_SERVER_CODE
            .replace("__SLAVE_ID__", str(self._slave_id))
            .replace("__TCP_PORT__", str(self._tcp_port))
            .replace("__TCP_PROTOCOL__", str(self._tcp_protocol))
            .replace("__STATE_PERIOD_SEC__", str(float(self._tcp_state_period_sec())))
            .replace("__TCP_STATE_STREAM_ENABLED__", "True" if self._tcp_state_stream_enabled else "False")
            .replace("__TCP_CMD_FRAME_WAIT_SEC__", str(float(self._tcp_cmd_frame_wait_sec)))
            .replace("__PRESENT_CURRENT_REG__", str(self._present_current_reg))
            .replace("__PRESENT_POSITION_REG__", str(self._present_position_reg))
            .replace("__PRESENT_POSITION_REGS__", str(self._present_position_regs))
            .replace("__GOAL_POS_REG__", str(self._goal_pos_reg))
            .replace("__GOAL_CUR_REG__", str(self._goal_cur_reg))
            .replace("__SNAP_ENABLED__", "True" if self._drl_snap_enabled else "False")
            .replace("__PLC_FEEDBACK_ENABLED__", "True" if self._plc_feedback_enabled else "False")
            .replace("__PLC_ADDR_POS__", str(int(self._plc_addr_pos)))
            .replace("__PLC_ADDR_CUR__", str(int(self._plc_addr_cur)))
            .replace("__PLC_ADDR_CODE__", str(int(self._plc_addr_code)))
        )

    def _clamp_plc_out_int_addr(self, addr: int, name: str) -> int:
        a = int(addr)
        if a < 0 or a > 23:
            clamped = 0 if a < 0 else 23
            self.get_logger().warn(f"{name}={a} out of range (0~23). clamped to {clamped}.")
            return clamped
        return a

    def _configure_tcp_socket(self, sock: socket.socket) -> None:
        try:
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        except Exception:
            pass
        for opt_name, val in (("TCP_KEEPIDLE", 10), ("TCP_KEEPINTVL", 3), ("TCP_KEEPCNT", 3)):
            try:
                opt = getattr(socket, opt_name, None)
                if opt is not None:
                    sock.setsockopt(socket.IPPROTO_TCP, opt, int(val))
            except Exception:
                pass

    # ------------------------------------------------------------------ binary protocol

    def _send_bin(self, mtype: int, seq: int, payload: bytes = b"") -> None:
        hdr = TCP_HDR.pack(TCP_MAGIC, TCP_VERSION, int(mtype) & 0xFF, int(seq) & 0xFFFF, len(payload))
        self._sock.sendall(hdr + (payload or b""))

    def _bin_try_parse_one(self) -> tuple[int, int, bytes] | None:
        buf = self._tcp_rx_buf
        if len(buf) < TCP_HDR.size:
            return None
        try:
            magic, ver, mtype, seq, plen = TCP_HDR.unpack(buf[:TCP_HDR.size])
        except Exception:
            self._tcp_rx_buf = buf[1:]
            return None
        if magic != TCP_MAGIC or ver != TCP_VERSION or plen < 0 or plen > 65535:
            self._tcp_rx_buf = buf[1:]
            return None
        if len(buf) < TCP_HDR.size + plen:
            return None
        payload = buf[TCP_HDR.size: TCP_HDR.size + plen]
        self._tcp_rx_buf = buf[TCP_HDR.size + plen:]
        return int(mtype), int(seq), payload

    # ------------------------------------------------------------------ send helpers

    def _send_frame(self, obj: dict) -> None:
        """JSON 프레임 전송. binary 모드에서 ping은 binary로 대신 전송."""
        if self._tcp_protocol == "binary":
            if str(obj.get("type", "")).lower() == "ping":
                self._send_bin(TCP_T_PING, 0, b"")
            return
        payload = json.dumps(obj).encode("utf-8")
        self._sock.sendall(struct.pack(">H", len(payload)) + payload)

    def _send_cmd_and_wait_ack(self, frames: list[bytes], timeout_sec: float | None = None) -> tuple[bool, str]:
        if not (self._tcp_hello_seen or self._tcp_state_seen or self._tcp_pong_seen):
            if not self._tcp_handshake(timeout_sec=1.5):
                return False, "no hello/state/pong from server"

        cmd_id = self._next_cmd_id
        self._next_cmd_id += 1
        ev = threading.Event()
        with self._ack_lock:
            self._ack_waiters[cmd_id] = ev
            self._ack_results.pop(cmd_id, None)

        try:
            if self._tcp_protocol == "binary":
                parts = [struct.pack(">H", len(frames))]
                for b in frames:
                    parts.append(struct.pack(">H", len(b)))
                    parts.append(b)
                self._send_bin(TCP_T_CMD, cmd_id, b"".join(parts))
            else:
                msg = {"type": "cmd", "id": cmd_id, "frames": [b.hex() for b in frames]}
                self._send_frame(msg)
        except Exception as e:
            with self._ack_lock:
                self._ack_waiters.pop(cmd_id, None)
            return False, f"send failed: {e}"

        to = float(timeout_sec) if timeout_sec is not None else float(self._tcp_ack_timeout)
        if not ev.wait(timeout=to):
            # 재주입 직후 늦은 ACK 대응: 1초 grace window
            grace_deadline = time.time() + 1.0
            while time.time() < grace_deadline:
                with self._ack_lock:
                    late = self._ack_results.get(cmd_id)
                if late is not None:
                    ev.set()
                    break
                time.sleep(0.02)
            if not ev.is_set():
                with self._ack_lock:
                    self._ack_waiters.pop(cmd_id, None)
                return False, "ack timeout"

        with self._ack_lock:
            self._ack_waiters.pop(cmd_id, None)
            ack = self._ack_results.pop(cmd_id, None) or {}
        return bool(ack.get("ok", False)), str(ack.get("err", ""))

    def _send_cmd_fire_and_forget(self, frames: list[bytes], timeout_sec: float | None = None) -> tuple[bool, str]:
        if not self._socket_active:
            return False, "tcp offline"
        try:
            cmd_id = self._next_cmd_id
            self._next_cmd_id += 1
            if self._tcp_protocol == "binary":
                parts = [struct.pack(">H", len(frames))]
                for b in frames:
                    parts.append(struct.pack(">H", len(b)))
                    parts.append(b)
                self._send_bin(TCP_T_CMD, cmd_id, b"".join(parts))
            else:
                msg = {"type": "cmd", "id": cmd_id, "frames": [b.hex() for b in frames]}
                self._send_frame(msg)
            return True, ""
        except Exception as e:
            return False, f"send failed: {e}"

    def _tcp_handshake(self, timeout_sec: float = 5.0) -> bool:
        self._tcp_pong_seen = False
        try:
            self._send_frame({"type": "ping"})
        except Exception:
            return False
        start = time.time()
        while time.time() - start < timeout_sec:
            if self._tcp_pong_seen or self._tcp_hello_seen or self._tcp_state_seen:
                return True
            time.sleep(0.05)
        return False

    # ------------------------------------------------------------------ recv loop

    def _recv_loop(self) -> None:
        while self._socket_active and rclpy.ok():
            try:
                chunk = self._sock.recv(512)
                if not chunk:
                    self.get_logger().warn("[gripper] 소켓 연결 끊김")
                    break
                self._tcp_rx_buf += chunk
                if self._tcp_rx_max > 0 and len(self._tcp_rx_buf) > self._tcp_rx_max:
                    self._tcp_rx_buf = self._tcp_rx_buf[-max(1024, self._tcp_rx_keep):]

                if not self._tcp_sniff_logged and len(self._tcp_rx_buf) > 0:
                    sniff = self._tcp_rx_buf[:32]
                    self.get_logger().info(
                        f"[gripper] TCP 초기 수신({len(sniff)}B): hex={sniff.hex()} ascii={sniff!r}"
                    )
                    self._tcp_sniff_logged = True

                last_state_bin: tuple | None = None
                last_state_msg: dict | None = None

                if self._tcp_protocol == "binary":
                    while True:
                        parsed = self._bin_try_parse_one()
                        if not parsed:
                            break
                        mtype, seq, payload = parsed
                        if mtype == TCP_T_PONG:
                            self._tcp_pong_seen = True
                            self._tcp_hello_seen = True
                            self._last_pong_rx_t = time.time()
                            self.get_logger().info("[gripper] TCP pong 수신 (binary OK)")
                        elif mtype == TCP_T_ACK:
                            ok = bool(payload[0]) if len(payload) >= 1 else False
                            err = ""
                            if (not ok) and len(payload) >= 3:
                                elen = (payload[1] << 8) | payload[2]
                                if elen > 0 and len(payload) >= 3 + elen:
                                    try:
                                        err = payload[3:3 + elen].decode("utf-8", errors="ignore")
                                    except Exception:
                                        pass
                            ack_msg = {"type": "ack", "id": int(seq), "ok": ok, "err": err}
                            with self._ack_lock:
                                self._ack_results[int(seq)] = ack_msg
                                ev = self._ack_waiters.get(int(seq))
                            if ev:
                                ev.set()
                            if not self._tcp_ack_seen:
                                self._tcp_ack_seen = True
                                self.get_logger().info("[gripper] TCP binary ACK 수신 시작")
                        elif mtype == TCP_T_STATE:
                            if len(payload) >= _STATE_SIZE:
                                try:
                                    cur, pos, gcur, gpos_lo, gpos_hi = struct.unpack(_STATE_FMT, payload[:_STATE_SIZE])
                                    last_state_bin = (int(cur), int(pos), int(gcur), int(gpos_lo), int(gpos_hi))
                                except Exception:
                                    pass
                else:
                    while len(self._tcp_rx_buf) >= 2:
                        n = (self._tcp_rx_buf[0] << 8) | self._tcp_rx_buf[1]
                        if len(self._tcp_rx_buf) < 2 + n:
                            break
                        payload = self._tcp_rx_buf[2:2 + n]
                        self._tcp_rx_buf = self._tcp_rx_buf[2 + n:]
                        try:
                            msg = json.loads(payload.decode("utf-8", errors="ignore"))
                        except Exception:
                            continue
                        mtype_s = msg.get("type", "")
                        if mtype_s == "state":
                            last_state_msg = msg
                        elif mtype_s == "hello":
                            self._tcp_hello_seen = True
                            self.get_logger().info(f"[gripper] TCP hello: {msg}")
                        elif mtype_s == "pong":
                            self._tcp_pong_seen = True
                            self._tcp_hello_seen = True
                            self._last_pong_rx_t = time.time()
                        elif mtype_s == "ack":
                            cmd_id = int(msg.get("id", 0))
                            with self._ack_lock:
                                self._ack_results[cmd_id] = msg
                                ev = self._ack_waiters.get(cmd_id)
                            if ev:
                                ev.set()
                            if not self._tcp_ack_seen:
                                self._tcp_ack_seen = True
                                self.get_logger().info("[gripper] TCP JSON ACK 수신 시작")
                        elif mtype_s == "snap":
                            if isinstance(msg.get("snap"), dict):
                                self.get_logger().info(f"[gripper] SNAP(270-290): {msg.get('snap')}")

                # 배치 중 최신 상태만 반영 (레이턴시 최소화)
                if last_state_bin is not None:
                    cur, raw_pos, gcur, gpos_lo, gpos_hi = last_state_bin
                    self._current_hz_cur = int(cur)
                    if self._pos_use_low:
                        raw_pos = raw_pos & 0xFFFF
                    elif self._pos_word_order == "lo_hi":
                        raw_pos = ((raw_pos & 0xFFFF) << 16) | ((raw_pos >> 16) & 0xFFFF)
                    if self._pos_scale and self._pos_scale != 1.0:
                        self._current_hz_pos = int(round(float(raw_pos) / self._pos_scale))
                    else:
                        self._current_hz_pos = int(raw_pos)
                    self._last_state_rx_t = time.time()
                    self._last_gpos_lo = int(gpos_lo)
                    if not self._tcp_state_seen:
                        self._tcp_state_seen = True
                        self.get_logger().info("[gripper] TCP binary STATE 수신 시작")
                    self.get_logger().debug(
                        f"[gripper] cur={cur} pos={raw_pos} gcur={gcur} gpos_lo={gpos_lo} gpos_hi={gpos_hi}"
                    )
                elif last_state_msg is not None:
                    m = last_state_msg
                    self._current_hz_cur = int(m.get("cur", 0))
                    raw_pos = int(m.get("pos", 0))
                    if self._pos_use_low:
                        raw_pos = raw_pos & 0xFFFF
                    elif self._pos_word_order == "lo_hi":
                        raw_pos = ((raw_pos & 0xFFFF) << 16) | ((raw_pos >> 16) & 0xFFFF)
                    if self._pos_scale and self._pos_scale != 1.0:
                        self._current_hz_pos = int(round(float(raw_pos) / self._pos_scale))
                    else:
                        self._current_hz_pos = int(raw_pos)
                    self._last_state_rx_t = time.time()
                    self._tcp_hello_seen = True
                    if "gpos_lo" in m:
                        try:
                            self._last_gpos_lo = int(m.get("gpos_lo"))
                        except Exception:
                            pass
                    if not self._tcp_state_seen:
                        self._tcp_state_seen = True
                        self.get_logger().info("[gripper] TCP JSON STATE 수신 시작")

            except socket.timeout:
                continue
            except OSError:
                if self._socket_active:
                    self.get_logger().error("[gripper] 소켓 수신 OSError")
                break
            except Exception as e:
                self.get_logger().error(f"[gripper] 소켓 수신 에러: {e}")
                break
        self._socket_active = False

    # ------------------------------------------------------------------ watchdog / reconnect

    def _tcp_watchdog_loop(self) -> None:
        while rclpy.ok():
            try:
                time.sleep(max(0.2, self._tcp_wd_period))
                if not self._tcp_wd_enabled or self._cmd_transport != "tcp":
                    continue
                if not self._socket_active or self._tcp_reconnect_inflight:
                    continue
                try:
                    self._send_frame({"type": "ping"})
                except Exception:
                    self._socket_active = False
                    continue
                now = time.time()
                last_rx = max(self._last_pong_rx_t or 0.0, self._last_state_rx_t or 0.0)
                if last_rx and (now - last_rx) > self._tcp_wd_stale:
                    self.get_logger().warn("[gripper] TCP watchdog stale -> reconnect")
                    threading.Thread(target=self._reconnect_tcp_only, daemon=True).start()
            except Exception:
                continue

    def _reconnect_tcp_only(self) -> None:
        if not self._tcp_reconnect_lock.acquire(blocking=False):
            return
        self._tcp_reconnect_inflight = True
        try:
            self._socket_active = False
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._sock = None
            self._last_state_rx_t = 0.0
            self._last_pong_rx_t = 0.0

            for attempt in range(10):
                try:
                    time.sleep(0.2 if attempt == 0 else 0.5)
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(3.0)
                    sock.connect((self._robot_ip, self._tcp_port))
                    self._configure_tcp_socket(sock)
                    self._sock = sock
                    self._socket_active = True
                    self._tcp_rx_buf = b""
                    self._tcp_hello_seen = False
                    self._tcp_state_seen = False
                    self._tcp_ack_seen = False
                    self._tcp_sniff_logged = False
                    self._tcp_pong_seen = False
                    self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
                    self._recv_thread.start()
                    if self._tcp_handshake(timeout_sec=2.5):
                        self._last_pong_rx_t = time.time()
                        self.get_logger().info("[gripper] TCP reconnect 성공")
                        return
                    self._socket_active = False
                    self._sock.close()
                    self._sock = None
                except Exception:
                    continue
        finally:
            self._tcp_reconnect_inflight = False
            try:
                self._tcp_reconnect_lock.release()
            except Exception:
                pass

    def _reinject_tcp_server(self) -> None:
        """파라미터 변경 시 DRL TCP 서버를 재주입하고 재접속한다."""
        if not self._tcp_reconnect_lock.acquire(blocking=False):
            return
        self._tcp_reconnect_inflight = True
        try:
            try:
                if self._sock and self._socket_active:
                    self._sock.sendall(struct.pack(">H", 4) + b"STOP")
            except Exception:
                pass
            self._socket_active = False
            if self._sock:
                try:
                    self._sock.close()
                except Exception:
                    pass
            self._sock = None
            self._last_state_rx_t = 0.0
            self._last_pong_rx_t = 0.0

            try:
                killer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                killer.settimeout(0.7)
                killer.connect((self._robot_ip, self._tcp_port))
                killer.sendall(b"STOP")
                killer.close()
                time.sleep(0.5)
            except Exception:
                pass

            if not self._cli_drl.service_is_ready():
                return
            req = DrlStart.Request()
            req.robot_system = 0
            req.code = self._build_drl_server_code()
            self._cli_drl.call_async(req).add_done_callback(
                lambda f: self.get_logger().info(
                    "[gripper] DRL 서버 재주입 완료" if (f.result() and f.result().success) else "[gripper] DRL 서버 재주입 실패"
                )
            )

            for attempt in range(10):
                try:
                    time.sleep(2.0 if attempt == 0 else 0.8)
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(3.0)
                    sock.connect((self._robot_ip, self._tcp_port))
                    self._configure_tcp_socket(sock)
                    self._sock = sock
                    self._socket_active = True
                    self._tcp_rx_buf = b""
                    self._tcp_hello_seen = False
                    self._tcp_state_seen = False
                    self._tcp_ack_seen = False
                    self._tcp_sniff_logged = False
                    self._tcp_pong_seen = False
                    self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
                    self._recv_thread.start()
                    if self._tcp_handshake(timeout_sec=5.0):
                        self._last_pong_rx_t = time.time()
                        self.get_logger().info("[gripper] TCP 재주입 후 재접속 성공")
                        return
                    self._socket_active = False
                    self._sock.close()
                    self._sock = None
                except Exception:
                    continue
        finally:
            self._tcp_reconnect_inflight = False
            try:
                self._tcp_reconnect_lock.release()
            except Exception:
                pass

    # ------------------------------------------------------------------ DRL call

    def _call_drl(self, code: str, timeout_sec: float = 5.0) -> bool:
        if not self._cli_drl.service_is_ready():
            return False
        req = DrlStart.Request()
        req.robot_system = 0
        req.code = code
        event = threading.Event()
        ok: dict[str, bool] = {"v": False}

        def _done(f: object) -> None:
            try:
                res = f.result()  # type: ignore[union-attr]
                ok["v"] = bool(res and res.success)
            except Exception:
                ok["v"] = False
            event.set()

        self._cli_drl.call_async(req).add_done_callback(_done)
        event.wait(timeout=timeout_sec)
        return ok["v"]

    # ------------------------------------------------------------------ init

    def _init_drl_server(self) -> None:
        self._init_timer.cancel()

        if self._mode == "virtual":
            self.get_logger().info("[gripper] virtual 모드 — DRL/flange 초기화 생략")
            return

        if self._cmd_transport == "tcp" and self._tcp_external:
            self.get_logger().info("[gripper] tcp_external_server=true: DRL 주입 생략")
            for attempt in range(15):
                try:
                    time.sleep(0.2 if attempt == 0 else 0.5)
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(3.0)
                    sock.connect((self._robot_ip, self._tcp_port))
                    self._configure_tcp_socket(sock)
                    self._sock = sock
                    self._socket_active = True
                    self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
                    self._recv_thread.start()
                    self.get_logger().info(f"[gripper] TCP 접속 성공 (external, port={self._tcp_port})")
                    return
                except Exception as e:
                    self.get_logger().warn(f"[gripper] TCP 대기중 ({attempt+1}/15): {e}")
            self.get_logger().error("[gripper] 외부 TCP 서버 접속 실패")
            return

        if not self._cli_drl.service_is_ready():
            self._init_attempt = getattr(self, "_init_attempt", 0) + 1
            if self._init_attempt <= 60:
                self.get_logger().warn(
                    f"[gripper] DRL 서비스 미준비 ({self._init_attempt}/60) — 2초 후 재시도"
                )
                self._init_timer = self.create_timer(2.0, self._init_drl_server, callback_group=self._cb)
                return
            self.get_logger().error(f"[gripper] DRL 서비스 연결 실패: {self._prefix}/drl/drl_start")
            return
        self._init_attempt = 0

        if self._cmd_transport == "drl":
            try:
                init_pkts = [
                    ModbusRTU.fc06(self._slave_id, 256, 1),
                    ModbusRTU.fc06(self._slave_id, 275, self._cur_init),
                ]
                if self._call_drl(build_drl_write_packets(init_pkts), timeout_sec=5.0):
                    self.get_logger().info("[gripper] DRL 초기화 완료 (토크ON/기본전류)")
                else:
                    self.get_logger().error("[gripper] DRL 초기화 실패")
            except Exception as e:
                self.get_logger().error(f"[gripper] DRL 초기화 예외: {e}")
            return

        # TCP 모드: 기존 서버 종료 후 DRL 서버 배포
        try:
            killer = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            killer.settimeout(0.7)
            killer.connect((self._robot_ip, self._tcp_port))
            killer.sendall(b"STOP")
            killer.close()
            time.sleep(0.5)
        except Exception:
            pass

        req = DrlStart.Request()
        req.robot_system = 0
        req.code = self._build_drl_server_code()
        self._cli_drl.call_async(req)
        self.get_logger().info(f"[gripper] DRL 서버({self._tcp_protocol}) 배포 완료 — 소켓 접속 대기...")

        for attempt in range(15):
            try:
                time.sleep(1.5 if attempt == 0 else 1.0)
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(3.0)
                sock.connect((self._robot_ip, self._tcp_port))
                self._configure_tcp_socket(sock)
                self._sock = sock
                self._socket_active = True
                self._tcp_rx_buf = b""
                self._tcp_hello_seen = False
                self._tcp_state_seen = False
                self._tcp_ack_seen = False
                self._tcp_sniff_logged = False
                self._tcp_pong_seen = False
                self.get_logger().info(f"[gripper] TCP 접속 성공 (port={self._tcp_port})")
                self._recv_thread = threading.Thread(target=self._recv_loop, daemon=True)
                self._recv_thread.start()
                if not self._tcp_handshake(timeout_sec=5.0):
                    self.get_logger().error("[gripper] TCP 핸드셰이크 실패 — 재시도")
                    self._socket_active = False
                    self._sock.close()
                    self._sock = None
                    continue
                return
            except Exception as e:
                self.get_logger().warn(f"[gripper] TCP 대기중 ({attempt+1}/15): {e}")
        self.get_logger().error("[gripper] TCP 소켓 확립 실패")

    # ------------------------------------------------------------------ param callback

    def _on_param_set(self, params) -> SetParametersResult:
        try:
            need_reinject = False
            for p in params:
                n = p.name
                if n == "present_current_reg":
                    self._present_current_reg = int(p.value); need_reinject = True
                elif n == "present_position_reg":
                    self._present_position_reg = int(p.value); need_reinject = True
                elif n == "present_position_regs":
                    self._present_position_regs = int(p.value); need_reinject = True
                elif n == "goal_current_reg":
                    self._goal_cur_reg = int(p.value); need_reinject = True
                elif n == "goal_position_reg":
                    self._goal_pos_reg = int(p.value); need_reinject = True
                elif n == "goal_position_write_mode":
                    self._goal_pos_write_mode = str(p.value).lower()
                elif n == "goal_position_regs":
                    self._goal_pos_regs = int(p.value)
                elif n == "goal_position_scale":
                    self._goal_pos_scale = float(p.value)
                elif n == "position_scale":
                    self._pos_scale = float(p.value)
                elif n == "grip_current_threshold":
                    self._grip_threshold = float(p.value)
                elif n == "done_pos_tolerance":
                    self._done_tol = int(p.value)
                elif n == "done_min_motion":
                    self._done_min_motion = int(p.value)
                elif n == "done_require_reached":
                    self._done_require_reached = bool(p.value)
                elif n == "grip_detect_enabled":
                    self._grip_enabled = bool(p.value)
                elif n == "action_max_wait_sec":
                    self._action_max_wait = float(p.value)
                elif n == "drl_snap_enabled":
                    self._drl_snap_enabled = bool(p.value); need_reinject = True
                elif n == "plc_feedback_enabled":
                    self._plc_feedback_enabled = bool(p.value); need_reinject = True
                elif n == "plc_addr_pos":
                    self._plc_addr_pos = int(p.value); need_reinject = True
                elif n == "plc_addr_cur":
                    self._plc_addr_cur = int(p.value); need_reinject = True
                elif n == "plc_addr_code":
                    self._plc_addr_code = int(p.value); need_reinject = True
                elif n == "tcp_ack_timeout_sec":
                    self._tcp_ack_timeout = float(p.value)
                elif n == "tcp_watchdog_enabled":
                    self._tcp_wd_enabled = bool(p.value)
                elif n == "tcp_watchdog_period_sec":
                    self._tcp_wd_period = float(p.value)
                elif n == "tcp_watchdog_stale_sec":
                    self._tcp_wd_stale = float(p.value)

            if need_reinject and self._cmd_transport == "tcp" and not self._tcp_external:
                self.get_logger().warn("[gripper] TCP 관련 파라미터 변경 → DRL 서버 재주입")
                threading.Thread(target=self._reinject_tcp_server, daemon=True).start()
            return SetParametersResult(successful=True)
        except Exception as e:
            return SetParametersResult(successful=False, reason=str(e))

    # ------------------------------------------------------------------ direct cmd

    def _resolve_preset_action(self, action: str, pulse: int = 0, current: int = 0) -> tuple[int, int]:
        a = action.strip()
        if a in ("open", "release"):
            return self._pulse_open, self._cur_init
        if a == "close":
            return self._pulse_closed, self._cur_grip
        if a == "custom":
            p = pulse if pulse >= 0 else self._pulse_closed
            c = current if current > 0 else self._cur_init
            return p, c
        return -1, -1

    def _on_direct_cmd(self, msg: String) -> None:
        raw = (msg.data or "").strip()
        if not raw:
            return
        parts = raw.split()
        action = parts[0]
        pulse = int(parts[1]) if len(parts) >= 2 else 0
        current = int(parts[2]) if len(parts) >= 3 else 0
        t_pulse, t_cur = self._resolve_preset_action(action, pulse=pulse, current=current)
        if t_pulse == -1:
            self.get_logger().warn(f"[gripper] direct cmd 알 수 없는 동작: {raw!r}")
            return

        # streaming no-ack 경로: lock 안에서 동기 전송, 중복 스킵
        if self._direct_cmd_streaming and self._direct_cmd_streaming_no_ack and self._cmd_transport == "tcp":
            self._direct_cmd_streaming_inline(action, int(t_pulse), int(t_cur))
            return

        if not self._direct_cmd_q:
            return
        item = (action, int(t_pulse), int(t_cur))
        if self._direct_cmd_latest_only:
            while True:
                try:
                    self._direct_cmd_q.get_nowait()
                except queue.Empty:
                    break
        try:
            self._direct_cmd_q.put_nowait(item)
        except queue.Full:
            try:
                self._direct_cmd_q.get_nowait()
            except Exception:
                pass
            try:
                self._direct_cmd_q.put_nowait(item)
            except Exception:
                pass

    def _direct_cmd_streaming_inline(self, action: str, t_pulse: int, t_cur: int) -> None:
        with self._direct_stream_lock:
            if self._last_sent_direct_pulse == t_pulse and self._last_sent_direct_cur == t_cur:
                return
            if not self._socket_active:
                return
            pos_frames = self._direct_cmd_pos_frames(t_pulse)
            frames: list[bytes] = []
            if self._last_sent_direct_cur != t_cur:
                frames.append(ModbusRTU.fc06(self._slave_id, self._goal_cur_reg, t_cur))
            frames.extend(pos_frames)
            ok, err = self._send_cmd_fire_and_forget(frames)
            if not ok:
                self.get_logger().warning(f"[gripper] inline send failed: {err}", throttle_duration_sec=2.0)
                return
            self._last_sent_direct_pulse = t_pulse
            self._last_sent_direct_cur = t_cur

    def _drain_direct_cmd_latest(self) -> tuple[str, int, int]:
        assert self._direct_cmd_q is not None
        action, t_pulse, t_cur = self._direct_cmd_q.get()
        while True:
            try:
                action, t_pulse, t_cur = self._direct_cmd_q.get_nowait()
            except queue.Empty:
                break
        return action, t_pulse, t_cur

    def _direct_cmd_worker_loop(self) -> None:
        assert self._direct_cmd_q is not None
        while rclpy.ok():
            try:
                if self._direct_cmd_latest_only:
                    action, t_pulse, t_cur = self._drain_direct_cmd_latest()
                else:
                    action, t_pulse, t_cur = self._direct_cmd_q.get()
                self._direct_cmd_worker(action, t_pulse, t_cur)
            except Exception:
                continue

    def _direct_cmd_pos_frames(self, t_pulse: int) -> list[bytes]:
        goal_val = int(round(float(t_pulse) * (self._goal_pos_scale if self._goal_pos_scale else 1.0)))
        pos_pkt_fc06 = ModbusRTU.fc06(self._slave_id, self._goal_pos_reg, int(goal_val) & 0xFFFF)
        pos_pkt_fc16 = ModbusRTU.fc16(self._slave_id, self._goal_pos_reg, 2, [int(goal_val) & 0xFFFF, 0])
        if self._goal_pos_regs <= 1:
            return [pos_pkt_fc06]
        if self._goal_pos_write_mode == "fc16":
            return [pos_pkt_fc16]
        return [pos_pkt_fc06]

    def _direct_cmd_worker(self, action: str, t_pulse: int, t_cur: int) -> None:
        try:
            self.get_logger().info(f"[gripper] direct cmd: action={action} pos={t_pulse} cur={t_cur}")

            if self._cmd_transport == "drl":
                if self._mode == "virtual":
                    self._current_hz_pos = t_pulse
                    self.get_logger().info(f"[gripper] virtual mock pulse={t_pulse}")
                    return
                pkts = [
                    ModbusRTU.fc06(self._slave_id, self._goal_cur_reg, t_cur),
                    ModbusRTU.fc16(self._slave_id, self._goal_pos_reg, 2, [t_pulse & 0xFFFF, 0]),
                ]
                ok = self._call_drl(build_drl_write_packets(pkts), timeout_sec=5.0)
                if ok:
                    self._current_hz_pos = t_pulse
                self.get_logger().info(f"[gripper] direct cmd(drl): ok={ok}")
                return

            if not self._socket_active:
                self.get_logger().warn("[gripper] direct cmd 실패: TCP 오프라인")
                return

            cur_pkt = ModbusRTU.fc06(self._slave_id, self._goal_cur_reg, t_cur)
            pos_frames = self._direct_cmd_pos_frames(t_pulse)
            ack_to = self._direct_cmd_streaming_ack_timeout if self._direct_cmd_streaming else None
            use_no_ack = bool(self._direct_cmd_streaming and self._direct_cmd_streaming_no_ack)
            send_fn = self._send_cmd_fire_and_forget if use_no_ack else self._send_cmd_and_wait_ack

            frames: list[bytes] = [cur_pkt] + pos_frames
            ok, err = send_fn(frames, timeout_sec=ack_to)
            if ok:
                self._last_sent_direct_pulse = t_pulse
                self._last_sent_direct_cur = t_cur
                self._current_hz_pos = t_pulse
            else:
                self.get_logger().warn(f"[gripper] direct cmd 전송 실패: {err}")
        except Exception as e:
            self.get_logger().error(f"[gripper] direct cmd 예외: {e}")

    # ------------------------------------------------------------------ set_position service

    def _set_position_callback(self, request, response):
        t_pulse = int(request.position)
        t_cur = int(request.current) if request.current > 0 else self._cur_init
        timeout = float(request.timeout_sec) if request.timeout_sec > 0 else self._tcp_ack_timeout

        if self._mode == "virtual":
            self._current_hz_pos = t_pulse
            response.success = True
            response.message = "virtual mock"
            response.final_position = t_pulse
            response.final_current = t_cur
            return response

        if self._cmd_transport != "tcp" or not self._socket_active:
            response.success = False
            response.message = "TCP 오프라인 또는 transport=drl (tcp 모드에서만 사용 가능)"
            response.final_position = self._current_hz_pos
            response.final_current = self._current_hz_cur
            return response

        # 1) Modbus 쓰기 + ACK 대기 — 실패 시 즉시 반환
        try:
            cur_pkt = ModbusRTU.fc06(self._slave_id, self._goal_cur_reg, t_cur)
            pos_frames = self._direct_cmd_pos_frames(t_pulse)
            ok, err = self._send_cmd_and_wait_ack([cur_pkt] + pos_frames, timeout_sec=timeout)
            if not ok:
                self.get_logger().error(f"[gripper] set_position ack 실패: {err}")
                response.success = False
                response.message = f"ack 실패: {err}"
                response.final_position = self._current_hz_pos
                response.final_current = self._current_hz_cur
                return response
        except Exception as e:
            self.get_logger().error(f"[gripper] set_position 실패: {e}")
            response.success = False
            response.message = str(e)
            response.final_position = self._current_hz_pos
            response.final_current = self._current_hz_cur
            return response

        # 2) 완료 확인 루프 (chamjo _execute_callback 패턴)
        #    state stream 활성 + 신선할 때: 위치 도달 또는 파지 전류 감지까지 폴링
        #    settle timeout 후에도 ACK 성공이므로 success=True 유지
        if self._tcp_state_stream_enabled and (time.time() - self._last_state_rx_t) < 2.0:
            settle_deadline = time.time() + 0.5
            while time.time() < settle_deadline:
                time.sleep(0.05)
                if self._grip_enabled and abs(self._current_hz_cur) > self._grip_threshold:
                    self.get_logger().info(f"[gripper] 파지 감지 cur={self._current_hz_cur}")
                    break
                if abs(self._current_hz_pos - t_pulse) < self._done_tol:
                    self.get_logger().info(f"[gripper] 위치 도달 pos={self._current_hz_pos}")
                    break
        else:
            # state stream 비활성/만료 — 낙관적 업데이트
            self._current_hz_pos = t_pulse

        response.success = True
        response.message = "완료"
        response.final_position = self._current_hz_pos
        response.final_current = self._current_hz_cur
        return response

    # ------------------------------------------------------------------ action

    def _execute_callback(self, goal_handle: object) -> object:
        self._executing = True
        try:
            req = goal_handle.request  # type: ignore[union-attr]
            # approach_direction 파싱: "open" / "close" / "custom <pulse> <cur>"
            approach = str(req.approach_direction).strip() if hasattr(req, "approach_direction") else "close"
            parts = approach.split()
            action_name = parts[0] if parts else "close"
            pulse_arg = int(parts[1]) if len(parts) >= 2 else 0
            cur_arg = int(parts[2]) if len(parts) >= 3 else 0
            t_pulse, t_cur_default = self._resolve_preset_action(action_name, pulse=pulse_arg, current=cur_arg)
            if t_pulse < 0:
                t_pulse, t_cur_default = self._pulse_closed, self._cur_grip
            force = float(req.grasp_force) if hasattr(req, "grasp_force") and req.grasp_force > 0 else float(t_cur_default)
            t_cur = int(min(max(force, 10.0), 1000.0))

            def _publish_fb(phase: str, progress: float) -> None:
                try:
                    if GripperCommand is None:
                        return
                    fb = GripperCommand.Feedback()  # type: ignore[union-attr]
                    fb.phase = phase
                    fb.progress = float(progress)
                    goal_handle.publish_feedback(fb)  # type: ignore[union-attr]
                except Exception:
                    pass

            _publish_fb("sending", 0.0)

            if self._cmd_transport == "drl":
                if self._mode == "virtual":
                    self._current_hz_pos = t_pulse
                    result = GripperCommand.Result()  # type: ignore[union-attr]
                    result.success = True
                    result.message = "virtual mock"
                    goal_handle.succeed()  # type: ignore[union-attr]
                    return result

                _publish_fb("drl_executing", 0.2)
                drl_code = build_drl_move_and_poll(
                    slave_id=self._slave_id,
                    target_pulse=t_pulse,
                    target_current=t_cur,
                    grip_current_threshold=int(self._grip_threshold),
                    pos_tolerance=self._done_tol,
                    max_loops=60,
                )
                ok = self._call_drl(drl_code, timeout_sec=15.0)
                if ok:
                    self._current_hz_pos = t_pulse
                _publish_fb("done", 1.0)
                result = GripperCommand.Result()  # type: ignore[union-attr]
                result.success = ok
                result.message = "완료(drl)" if ok else "DRL 실행 실패"
                if ok:
                    goal_handle.succeed()  # type: ignore[union-attr]
                else:
                    goal_handle.abort()  # type: ignore[union-attr]
                return result

            if not self._socket_active:
                result = GripperCommand.Result()  # type: ignore[union-attr]
                result.success = False
                result.message = "TCP 오프라인"
                goal_handle.abort()  # type: ignore[union-attr]
                return result

            try:
                cur_pkt = ModbusRTU.fc06(self._slave_id, self._goal_cur_reg, t_cur)
                pos_frames = self._direct_cmd_pos_frames(t_pulse)
                ok1, err1 = self._send_cmd_and_wait_ack([cur_pkt])
                if not ok1:
                    raise RuntimeError(f"cur ack failed: {err1}")
                time.sleep(0.05)
                ok2, err2 = self._send_cmd_and_wait_ack(pos_frames)
                if not ok2:
                    raise RuntimeError(f"pos ack failed: {err2}")
            except Exception as e:
                self.get_logger().error(f"[gripper] execute 예외: {e}")
                result = GripperCommand.Result()  # type: ignore[union-attr]
                result.success = False
                result.message = str(e)
                goal_handle.abort()  # type: ignore[union-attr]
                return result

            # 완료 대기 + 피드백
            start_t = time.time()
            gripped = False
            while (time.time() - start_t) < self._action_max_wait:
                elapsed = time.time() - start_t
                _publish_fb("moving", min(0.9, elapsed / max(1.0, self._action_max_wait)))
                time.sleep(0.1)
                if self._grip_enabled and abs(self._current_hz_cur) > self._grip_threshold:
                    gripped = True
                    break
                if abs(self._current_hz_pos - t_pulse) < self._done_tol:
                    break

            _publish_fb("done", 1.0)
            result = GripperCommand.Result()  # type: ignore[union-attr]
            result.success = True
            result.message = "파지 감지" if gripped else "위치 도달"
            goal_handle.succeed()  # type: ignore[union-attr]
            return result
        finally:
            self._executing = False

    # ------------------------------------------------------------------ state publish

    def _publish_state(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["gripper_joint"]
        msg.position = [float(self._current_hz_pos)]
        msg.velocity = [0.0]
        msg.effort = [float(self._current_hz_cur)]
        self._state_pub.publish(msg)

    def destroy_node(self) -> None:
        if self._sock and self._socket_active:
            try:
                self._sock.sendall(struct.pack(">H", 4) + b"STOP")
                self._sock.close()
            except Exception:
                pass
        super().destroy_node()


def main(args: list | None = None) -> None:
    rclpy.init(args=args)
    node = GripperNode()
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
