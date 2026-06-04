"""RH-P12-RN gripper HAL driver — GripperInterface 구현.

gripper_node (ROS2)와 통신하거나, TCP 직접 제어로 동작한다.
unit_actions/ 에서 이 드라이버를 사용해 그리퍼를 제어한다.
"""
from __future__ import annotations

import json
import logging
import socket
import struct
import threading
import time

from hal.gripper_interface import GripperInterface

logger = logging.getLogger(__name__)

# pulse 범위: 0=열림, 700=닫힘 (RH-P12-RN(A) 기본)
PULSE_OPEN = 0
PULSE_CLOSED = 700
PULSE_MAX = 700

# Modbus RTU 레지스터 맵 (RH-P12-RN(A) 기본)
_REG_TORQUE_ENABLE = 256
_REG_GOAL_CURRENT = 275
_REG_GOAL_POSITION = 282
_REG_PRESENT_POSITION = 290
_REG_PRESENT_CURRENT = 287


def _crc16(data: bytes) -> bytes:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return struct.pack("<H", crc)


def _fc06(slave_id: int, addr: int, value: int) -> bytes:
    body = bytes([slave_id, 0x06]) + struct.pack(">HH", addr, value)
    return body + _crc16(body)


def _fc16(slave_id: int, addr: int, values: list[int]) -> bytes:
    n = len(values)
    body = bytes([slave_id, 0x10]) + struct.pack(">HHB", addr, n, n * 2)
    for v in values:
        body += struct.pack(">H", v)
    return body + _crc16(body)


class GripperDriver(GripperInterface):
    """RH-P12-RN TCP/Modbus 직접 제어 드라이버.

    gripper_node(ROS2)의 HAL 계층 버전. unit_actions/ 에서 직접 사용.
    robot_ip: Doosan 컨트롤러 IP — 호출자가 config에서 읽어서 전달.
    tcp_port: gripper_node의 DRL TCP 서버 포트 (기본 9105).
    """

    def __init__(
        self,
        robot_ip: str,
        tcp_port: int = 9105,
        slave_id: int = 1,
        default_force_n: float = 30.0,
        timeout_sec: float = 5.0,
    ) -> None:
        self._robot_ip = robot_ip
        self._tcp_port = tcp_port
        self._slave_id = slave_id
        self._default_force_n = default_force_n
        self._timeout_sec = timeout_sec

        self._sock: socket.socket | None = None
        self._lock = threading.Lock()
        self._current_pulse: int = 0
        self._current_ma: int = 0

        self._connect()

    # ── 내부 통신 ────────────────────────────────────────────────

    def _connect(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(self._timeout_sec)
            sock.connect((self._robot_ip, self._tcp_port))
            try:
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            except Exception:
                pass
            self._sock = sock
            logger.info("[gripper_driver] TCP 접속: %s:%d", self._robot_ip, self._tcp_port)
        except Exception as e:
            self._sock = None
            logger.error("[gripper_driver] TCP 접속 실패: %s", e)

    def _send_frames(self, frames: list[bytes]) -> bool:
        """JSON 프레임으로 Modbus 패킷 전송."""
        if self._sock is None:
            logger.error("[gripper_driver] 소켓 없음 — 재접속 시도")
            self._connect()
        if self._sock is None:
            return False

        try:
            with self._lock:
                cmd_id = int(time.time() * 1000) & 0xFFFF
                msg = {"type": "cmd", "id": cmd_id, "frames": [b.hex() for b in frames]}
                payload = json.dumps(msg).encode("utf-8")
                self._sock.sendall(struct.pack(">H", len(payload)) + payload)

                deadline = time.time() + self._timeout_sec
                buf = b""
                while time.time() < deadline:
                    try:
                        chunk = self._sock.recv(512)
                    except socket.timeout:
                        continue
                    if not chunk:
                        raise ConnectionError("소켓 끊김")
                    buf += chunk
                    while len(buf) >= 2:
                        n = (buf[0] << 8) | buf[1]
                        if len(buf) < 2 + n:
                            break
                        pkt = buf[2:2 + n]
                        buf = buf[2 + n:]
                        try:
                            resp = json.loads(pkt.decode("utf-8", errors="ignore"))
                        except Exception:
                            continue
                        if resp.get("type") == "ack" and resp.get("id") == cmd_id:
                            ok = bool(resp.get("ok", False))
                            if not ok:
                                logger.error("[gripper_driver] ACK 실패: %s", resp.get("err"))
                            return ok
                return False
        except Exception as e:
            logger.error("[gripper_driver] 전송 예외: %s", e)
            self._sock = None
            return False

    def _force_n_to_ma(self, force_n: float) -> int:
        """N 단위 force를 mA로 변환 (선형 근사, RH-P12-RN 스펙 기반)."""
        # 최대 파지력 ~40N @ 900mA 근사
        ma = int(round(force_n / 40.0 * 900))
        return max(50, min(ma, 1000))

    def _pulse_to_normalized(self, pulse: int) -> float:
        return max(0.0, min(float(pulse) / float(PULSE_MAX), 1.0))

    def _normalized_to_pulse(self, norm: float) -> int:
        return int(round(max(0.0, min(norm, 1.0)) * PULSE_MAX))

    # ── GripperInterface 구현 ─────────────────────────────────────

    def set_position(self, position: float, force: float = 20.0) -> bool:
        """position: 0.0=열림, 1.0=닫힘 / force: N"""
        pulse = self._normalized_to_pulse(position)
        ma = self._force_n_to_ma(force)

        frames = [
            _fc06(self._slave_id, _REG_GOAL_CURRENT, ma),
            _fc16(self._slave_id, _REG_GOAL_POSITION, [pulse & 0xFFFF, 0]),
        ]
        try:
            ok = self._send_frames(frames)
            if ok:
                self._current_pulse = pulse
                self._current_ma = ma
                logger.info("[gripper_driver] set_position pulse=%d ma=%d", pulse, ma)
            else:
                logger.error(
                    "[gripper_driver] set_position 실패 pulse=%d event=error track=A", pulse
                )
            return ok
        except Exception as e:
            logger.error("[gripper_driver] set_position 예외: %s event=error track=A", e)
            return False

    def get_position(self) -> float:
        return self._pulse_to_normalized(self._current_pulse)

    def is_grasping(self) -> bool:
        return abs(self._current_ma) > 50

    def open(self) -> bool:
        return self.set_position(0.0, force=10.0)

    def close(self, force: float = 20.0) -> bool:
        return self.set_position(1.0, force=force)

    def emergency_stop(self) -> None:
        """그리퍼 즉시 정지 — 토크 OFF."""
        try:
            frame = _fc06(self._slave_id, _REG_TORQUE_ENABLE, 0)
            self._send_frames([frame])
            logger.warning("[gripper_driver] emergency_stop 실행 — 토크 OFF")
        except Exception as e:
            logger.error("[gripper_driver] emergency_stop 예외: %s", e)

    def __del__(self) -> None:
        if self._sock:
            try:
                self._sock.close()
            except Exception:
                pass
