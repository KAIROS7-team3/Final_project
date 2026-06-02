from __future__ import annotations

import sys
import types

from plc_core.config import ModbusPLCConfig
from plc_core.states import LEDColor, LEDMode, STATE_LED_MAP, SystemState


if "rclpy" not in sys.modules:
    rclpy_module = types.ModuleType("rclpy")
    rclpy_executors_module = types.ModuleType("rclpy.executors")
    rclpy_node_module = types.ModuleType("rclpy.node")
    rclpy_qos_module = types.ModuleType("rclpy.qos")

    class _ExternalShutdownException(Exception):
        pass

    class _Node:
        pass

    class _QoSProfile:
        def __init__(self, *args, **kwargs) -> None:
            pass

    class _ReliabilityPolicy:
        BEST_EFFORT = object()

    rclpy_executors_module.ExternalShutdownException = _ExternalShutdownException
    rclpy_node_module.Node = _Node
    rclpy_qos_module.QoSProfile = _QoSProfile
    rclpy_qos_module.ReliabilityPolicy = _ReliabilityPolicy
    sys.modules["rclpy"] = rclpy_module
    sys.modules["rclpy.executors"] = rclpy_executors_module
    sys.modules["rclpy.node"] = rclpy_node_module
    sys.modules["rclpy.qos"] = rclpy_qos_module

if "interfaces.msg" not in sys.modules:
    interfaces_module = sys.modules.setdefault("interfaces", types.ModuleType("interfaces"))
    interfaces_msg_module = types.ModuleType("interfaces.msg")

    class _PLCStatus:
        led_color: str = ""
        led_mode: str = ""
        system_state: str = ""

    interfaces_msg_module.PLCStatus = _PLCStatus
    sys.modules["interfaces.msg"] = interfaces_msg_module

if "std_msgs.msg" not in sys.modules:
    std_msgs_module = sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs"))
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")

    class _Bool:
        def __init__(self) -> None:
            self.data = False

    class _Int32:
        def __init__(self) -> None:
            self.data = 0

    class _String:
        def __init__(self) -> None:
            self.data = ""

    std_msgs_msg_module.Bool = _Bool
    std_msgs_msg_module.Int32 = _Int32
    std_msgs_msg_module.String = _String
    sys.modules["std_msgs.msg"] = std_msgs_msg_module

from plc.plc_node import XgbRos2ModbusNode


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


def test_error_and_estop_led_contract() -> None:
    assert STATE_LED_MAP[SystemState.ERROR] == (LEDColor.RED, LEDMode.FLASH)
    assert STATE_LED_MAP[SystemState.E_STOP] == (LEDColor.RED, LEDMode.SOLID)


def test_safety_hook_addresses_can_be_disabled() -> None:
    config = ModbusPLCConfig(
        port="/dev/ttyUSB0",
        baudrate=115200,
        parity="N",
        stopbits=1,
        bytesize=8,
        device_id=1,
        start_coil_labels=("M0000",),
        start_coil_addresses=(0,),
        start_coil_outputs=("P0040",),
        reset_coil_label="M0010",
        reset_coil_address=16,
        read_register_label="P020",
        read_register_address=0,
        write_register_label="P000",
        write_register_address=0,
        pulse_duration_s=0.2,
    )

    assert config.watchdog_coil_address is None
    assert config.estop_input_address is None


def test_semantic_e_stop_maps_to_configured_error_output() -> None:
    outputs = ModbusPLCConfig.parse_system_state_outputs(
        ("idle", "error", "e_stop"),
        ("M0000", "M0003", "M0003"),
    )

    assert outputs[SystemState.E_STOP] == ("M0003",)


def test_publish_estop_uses_bool_topic_message() -> None:
    node = XgbRos2ModbusNode.__new__(XgbRos2ModbusNode)
    node._estop_pub = FakePublisher()

    node._publish_estop(True)
    node._publish_estop(False)

    assert [message.data for message in node._estop_pub.messages] == [True, False]
