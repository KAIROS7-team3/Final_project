"""PLC LED와 시스템 상태의 공통 enum 정의.

`plc_core`는 ROS2를 import하지 않는 순수 Python 계층이다. Track A/B ROS2
패키지와 테스트 코드가 같은 상태 이름을 쓰게 하려고 enum을 이곳에 둔다.
"""

from enum import Enum


class LEDColor(str, Enum):
    """운영자에게 보여 줄 LED 색상."""

    GREEN = "green"
    YELLOW = "yellow"
    RED = "red"
    WHITE = "white"


class LEDMode(str, Enum):
    """LED 점등 방식."""

    SOLID = "solid"
    PULSE = "pulse"
    FLASH = "flash"


class SystemState(str, Enum):
    """로봇/음성/PLC가 공유하는 상위 상태."""

    IDLE = "idle"
    LISTENING = "listening"
    INFERRING = "inferring"
    MOVING = "moving"
    ERROR = "error"
    E_STOP = "e_stop"


# PLCStatus.msg에 맞춘 상태 -> LED 표시 매핑.
# 실제 PLC bit/register encoding은 현장 배선에 따라 바뀔 수 있지만, 상위 상태가
# 어떤 색/패턴으로 보일지는 여기서 한 번에 확인할 수 있게 둔다.
STATE_LED_MAP: dict[SystemState, tuple[LEDColor, LEDMode]] = {
    SystemState.IDLE: (LEDColor.WHITE, LEDMode.SOLID),
    SystemState.LISTENING: (LEDColor.GREEN, LEDMode.PULSE),
    SystemState.INFERRING: (LEDColor.GREEN, LEDMode.FLASH),
    SystemState.MOVING: (LEDColor.GREEN, LEDMode.SOLID),
    SystemState.ERROR: (LEDColor.RED, LEDMode.FLASH),
    SystemState.E_STOP: (LEDColor.RED, LEDMode.SOLID),
}
