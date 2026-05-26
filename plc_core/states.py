from enum import Enum


class LEDColor(str, Enum):
    GREEN = "green"
    YELLOW = "yellow"
    RED = "red"
    WHITE = "white"


class LEDMode(str, Enum):
    SOLID = "solid"
    PULSE = "pulse"
    FLASH = "flash"


class SystemState(str, Enum):
    IDLE = "idle"
    LISTENING = "listening"
    INFERRING = "inferring"
    MOVING = "moving"
    ERROR = "error"
    E_STOP = "e_stop"


# Mapping of SystemState → (LEDColor, LEDMode) per PLCStatus.msg spec
STATE_LED_MAP: dict[SystemState, tuple[LEDColor, LEDMode]] = {
    SystemState.IDLE: (LEDColor.WHITE, LEDMode.SOLID),
    SystemState.LISTENING: (LEDColor.GREEN, LEDMode.PULSE),
    SystemState.INFERRING: (LEDColor.GREEN, LEDMode.FLASH),
    SystemState.MOVING: (LEDColor.GREEN, LEDMode.SOLID),
    SystemState.ERROR: (LEDColor.RED, LEDMode.FLASH),
    SystemState.E_STOP: (LEDColor.RED, LEDMode.SOLID),
}
