"""ROS2 비의존 PLC client 추상화.

현재는 실제 Modbus 송신 대신 로그만 남기는 mock 성격의 client다. 상위 로직이
PLC 상태 갱신 API를 먼저 고정하고, 실제 LS Electric XBC-DR14E 연결은
`ros2_ws/src/plc` 래퍼에서 검증하는 구조를 유지한다.
"""

import logging
from dataclasses import dataclass

from plc_core.states import LEDColor, LEDMode, STATE_LED_MAP, SystemState

logger = logging.getLogger(__name__)


@dataclass
class PLCStatus:
    """운영자 표시용 PLC 상태 snapshot."""

    led_color: LEDColor
    led_mode: LEDMode
    system_state: SystemState


class PLCClient:
    """PLC 상태 제어를 위한 순수 Python client 인터페이스.

    Phase 0/초기 테스트에서는 hardware 없이 로그만 남긴다. 나중에 이 class를
    실제 pymodbus 구현으로 교체하더라도 상위 모듈은 `set_state()`와
    `get_status()`만 사용하면 된다.
    """

    def __init__(self, port: str = "/dev/plc", baudrate: int = 9600) -> None:
        self._port = port
        self._baudrate = baudrate
        self._connected = False
        self._current_state = SystemState.IDLE

    def connect(self) -> bool:
        """PLC 연결을 연다.

        현재 구현은 mock이라 항상 성공한다. 실제 구현에서는 serial/TCP 연결 실패를
        False로 반환하거나 예외로 올려 상위 계층이 빨간 LED/경고를 내게 해야 한다.
        """

        logger.info("[PLCClient] connect - port=%s baudrate=%d (mock)", self._port, self._baudrate)
        self._connected = True
        return True

    def disconnect(self) -> None:
        """PLC 연결 상태를 닫힘으로 표시한다."""

        self._connected = False
        logger.info("[PLCClient] disconnected")

    def set_state(self, state: SystemState) -> PLCStatus:
        """상위 시스템 상태를 LED 색/모드로 변환해 적용하고 snapshot을 반환한다."""

        color, mode = STATE_LED_MAP[state]
        self._current_state = state
        logger.info("[PLCClient] set_state - state=%s led_color=%s led_mode=%s", state, color, mode)
        self.set_led(color, mode)
        return self.get_status()

    def set_led(self, color: LEDColor, mode: LEDMode) -> bool:
        """LED 표시를 적용한다.

        mock에서는 로그만 남긴다. 실제 PLC에서는 이 지점이 coil/register 쓰기로
        바뀌며, 실패 시 silent fallback 없이 False 또는 예외로 알려야 한다.
        """

        logger.info("[PLCClient] set_led - color=%s mode=%s", color, mode)
        return True

    def set_error(self) -> PLCStatus:
        """일반 오류 상태를 빨간 점멸로 표시한다."""

        return self.set_state(SystemState.ERROR)

    def set_estop(self) -> PLCStatus:
        """E-stop 상태를 빨간 고정등으로 표시한다."""

        return self.set_state(SystemState.E_STOP)

    def get_status(self) -> PLCStatus:
        """마지막으로 적용된 시스템 상태를 PLCStatus 형태로 반환한다."""

        color, mode = STATE_LED_MAP[self._current_state]
        return PLCStatus(led_color=color, led_mode=mode, system_state=self._current_state)

    def is_connected(self) -> bool:
        """현재 client가 연결 상태로 간주되는지 반환한다."""

        return self._connected
