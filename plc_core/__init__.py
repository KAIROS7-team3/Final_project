"""ROS2 비의존 PLC core 공개 API.

상위 로직은 LED 색/모드 enum과 PLCClient 인터페이스를 여기서 가져간다.
실제 Modbus client도 ROS2와 분리해 core 계층이 `rclpy`에 의존하지 않도록
유지한다.
"""

from plc_core.client import PLCClient, PLCStatus
from plc_core.config import ModbusPLCConfig, PLCConfigError
from plc_core.modbus_client import ModbusPLCClient, PLCError
from plc_core.states import LEDColor, LEDMode, STATE_LED_MAP, SystemState

__all__ = [
    "LEDColor",
    "LEDMode",
    "ModbusPLCClient",
    "ModbusPLCConfig",
    "PLCClient",
    "PLCConfigError",
    "PLCError",
    "PLCStatus",
    "STATE_LED_MAP",
    "SystemState",
]
