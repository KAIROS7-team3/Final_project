import logging
from dataclasses import dataclass

from plc_core.states import LEDColor, LEDMode, STATE_LED_MAP, SystemState

logger = logging.getLogger(__name__)


@dataclass
class PLCStatus:
    led_color: LEDColor
    led_mode: LEDMode
    system_state: SystemState


class PLCClient:
    """Modbus RTU/TCP client for LS Electric XBC-DR10E PLC.

    In Phase 0 (no hardware), operates as a mock that logs all commands.
    Phase 1 will wire this to pymodbus.
    """

    def __init__(self, port: str = "/dev/plc", baudrate: int = 9600) -> None:
        self._port = port
        self._baudrate = baudrate
        self._connected = False
        self._current_state = SystemState.IDLE

    def connect(self) -> bool:
        logger.info("[PLCClient] connect - port=%s baudrate=%d (mock)", self._port, self._baudrate)
        self._connected = True
        return True

    def disconnect(self) -> None:
        self._connected = False
        logger.info("[PLCClient] disconnected")

    def set_state(self, state: SystemState) -> bool:
        color, mode = STATE_LED_MAP[state]
        self._current_state = state
        logger.info("[PLCClient] set_state - state=%s led_color=%s led_mode=%s", state, color, mode)
        return self.set_led(color, mode)

    def set_led(self, color: LEDColor, mode: LEDMode) -> bool:
        logger.info("[PLCClient] set_led - color=%s mode=%s", color, mode)
        return True

    def set_error(self) -> bool:
        return self.set_state(SystemState.ERROR)

    def set_estop(self) -> bool:
        return self.set_state(SystemState.E_STOP)

    def get_status(self) -> PLCStatus:
        color, mode = STATE_LED_MAP[self._current_state]
        return PLCStatus(led_color=color, led_mode=mode, system_state=self._current_state)

    def is_connected(self) -> bool:
        return self._connected
