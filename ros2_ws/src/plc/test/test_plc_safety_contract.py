from __future__ import annotations

import sqlite3
import sys
import types
import threading
from pathlib import Path

from plc_core.client import PLCStatus
from plc_core.config import ModbusPLCConfig
from plc_core.states import LEDColor, LEDMode, STATE_LED_MAP, SystemState

PLC_PACKAGE_ROOT = Path(__file__).resolve().parents[1]
if str(PLC_PACKAGE_ROOT) not in sys.path:
    sys.path.insert(0, str(PLC_PACKAGE_ROOT))


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
        RELIABLE = object()

    class _DurabilityPolicy:
        TRANSIENT_LOCAL = object()

    rclpy_executors_module.ExternalShutdownException = _ExternalShutdownException
    rclpy_node_module.Node = _Node
    rclpy_qos_module.DurabilityPolicy = _DurabilityPolicy
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


class FakeLogger:
    def __init__(self) -> None:
        self.messages: list[tuple[str, str]] = []

    def info(self, message: str) -> None:
        self.messages.append(("info", message))

    def warning(self, message: str) -> None:
        self.messages.append(("warning", message))

    def error(self, message: str) -> None:
        self.messages.append(("error", message))


class FakeTimer:
    def __init__(self, delay_s: float, callback) -> None:
        self.delay_s = delay_s
        self.callback = callback
        self.cancelled = False

    def cancel(self) -> None:
        self.cancelled = True


class FakePLC:
    def __init__(
        self,
        *,
        connected: bool = True,
        fail_write: bool = False,
        watchdog_value: bool = True,
    ) -> None:
        self.connected = connected
        self.fail_write = fail_write
        self.watchdog_value = watchdog_value
        self.calls: list[tuple[str, object]] = []
        self.current_state = SystemState.IDLE

    def connect(self) -> bool:
        self.calls.append(("connect", {}))
        self.connected = True
        return True

    def write_coil(self, address: int, value: bool) -> None:
        if self.fail_write:
            from plc_core.modbus_client import PLCError

            raise PLCError("simulated coil write failure")
        self.calls.append(("write_coil", {"address": address, "value": value}))

    def write_start_coils(self, value: bool) -> None:
        self.calls.append(("write_start_coils", {"value": value}))

    def read_watchdog(self) -> bool:
        self.calls.append(("read_watchdog", {}))
        return self.watchdog_value

    def set_system_state(
        self,
        state: SystemState,
        *,
        apply_outputs: bool,
    ) -> PLCStatus:
        self.calls.append(
            (
                "set_system_state",
                {"state": state, "apply_outputs": apply_outputs},
            )
        )
        self.current_state = state
        return self.get_status()

    def set_error(self, *, apply_outputs: bool = True) -> PLCStatus:
        return self.set_system_state(SystemState.ERROR, apply_outputs=apply_outputs)

    def get_status(self) -> PLCStatus:
        color, mode = STATE_LED_MAP[self.current_state]
        return PLCStatus(led_color=color, led_mode=mode, system_state=self.current_state)


def make_config() -> ModbusPLCConfig:
    return ModbusPLCConfig(
        port="/dev/ttyUSB0",
        baudrate=115200,
        parity="N",
        stopbits=1,
        bytesize=8,
        device_id=1,
        start_coil_labels=("M0000", "M0001", "M0002", "M0003", "M0004", "M0005"),
        start_coil_addresses=(0, 1, 2, 3, 4, 5),
        start_coil_outputs=("P0040", "P0041", "P0042", "P0043", "P0043", "P0044"),
        reset_coil_label="M0100",
        reset_coil_address=256,
        read_register_label="P020",
        read_register_address=0,
        write_register_label="P000",
        write_register_address=0,
        pulse_duration_s=0.2,
        watchdog_coil_label="M0050",
        watchdog_coil_address=80,
        system_state_outputs={
            SystemState.IDLE: ("M0000",),
            SystemState.MOVING: ("M0002",),
            SystemState.E_STOP: ("M0003",),
            SystemState.ERROR: ("M0004",),
            SystemState.WATCHDOG: ("M0005",),
        },
    )


def make_node(
    *,
    connected: bool = True,
    fail_write: bool = False,
) -> XgbRos2ModbusNode:
    node = XgbRos2ModbusNode.__new__(XgbRos2ModbusNode)
    node._config = types.SimpleNamespace(
        modbus=make_config(),
        watchdog_period_s=0.1,
        watchdog_timeout_s=0.5,
        connect_retry_count=1,
        connect_retry_backoff_s=0.1,
        db_path="robot_arm.db",
    )
    node._plc = FakePLC(connected=connected, fail_write=fail_write)
    node._estop_latched = threading.Event()
    node._watchdog_latched = threading.Event()
    node._watchdog_last_value = None
    node._watchdog_same_sample_count = 0
    node._estop_pub = FakePublisher()
    node._status_pub = FakePublisher()
    node._pulse_timers = []
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger

    def create_timer(delay_s: float, callback) -> FakeTimer:
        timer = FakeTimer(delay_s, callback)
        node._pulse_timers.append(timer)
        return timer

    node.create_timer = create_timer
    return node


def test_error_and_estop_led_contract() -> None:
    assert STATE_LED_MAP[SystemState.ERROR] == (LEDColor.RED, LEDMode.FLASH)
    assert STATE_LED_MAP[SystemState.E_STOP] == (LEDColor.RED, LEDMode.SOLID)
    assert STATE_LED_MAP[SystemState.WATCHDOG] == (LEDColor.WHITE, LEDMode.FLASH)
    assert STATE_LED_MAP[SystemState.LISTENING] == (LEDColor.YELLOW, LEDMode.PULSE)
    assert STATE_LED_MAP[SystemState.MOVING] == (LEDColor.RED, LEDMode.PULSE)


def test_pythonpath_environment_hook_points_to_workspace_root() -> None:
    environment_dir = PLC_PACKAGE_ROOT / "environment"

    sh_text = (environment_dir / "plc_core_pythonpath.sh").read_text(encoding="utf-8")

    assert not (environment_dir / "plc_core_pythonpath.dsv").exists()
    assert '${AMENT_CURRENT_PREFIX}/../..' in sh_text
    assert '${AMENT_CURRENT_PREFIX}/../../..' not in sh_text


def test_watchdog_hook_targets_dedicated_m050_coil() -> None:
    config = make_config()

    assert config.watchdog_coil_label == "M0050"
    assert config.watchdog_coil_address == 80
    assert config.estop_input_address is None


def test_semantic_states_map_to_latest_ladder_outputs() -> None:
    outputs = ModbusPLCConfig.parse_system_state_outputs(
        ("idle", "moving", "e_stop", "error", "watchdog"),
        ("M0000", "M0002", "M0003", "M0004", "M0005"),
    )

    assert outputs[SystemState.IDLE] == ("M0000",)
    assert outputs[SystemState.E_STOP] == ("M0003",)
    assert outputs[SystemState.ERROR] == ("M0004",)
    assert outputs[SystemState.WATCHDOG] == ("M0005",)


def test_publish_estop_uses_bool_topic_message() -> None:
    node = XgbRos2ModbusNode.__new__(XgbRos2ModbusNode)
    node._estop_pub = FakePublisher()

    node._publish_estop(True)
    node._publish_estop(False)

    assert [message.data for message in node._estop_pub.messages] == [True, False]


def test_estop_latch_blocks_output_commands() -> None:
    node = make_node()
    node._estop_latched.set()

    assert node._write_m_coil(0, True, "M0000") is False

    assert ("write_coil", {"address": 0, "value": True}) not in node._plc.calls
    assert node._estop_pub.messages[-1].data is True
    assert node._status_pub.messages[-1].system_state == SystemState.E_STOP.value


def test_watchdog_timer_handles_0_2_on_0_2_off_heartbeat_without_latching() -> None:
    node = make_node()

    for sample in (True, True, False, False, True, True, False, False):
        node._plc.watchdog_value = sample
        node.watchdog_timer_callback()

    assert not node._watchdog_latched.is_set()
    assert node._status_pub.messages == []
    assert ("read_watchdog", {}) in node._plc.calls


def test_watchdog_timer_latches_after_0_5s_of_no_change() -> None:
    node = make_node()

    node.watchdog_timer_callback()
    assert not node._watchdog_latched.is_set()
    node.watchdog_timer_callback()
    assert not node._watchdog_latched.is_set()
    node.watchdog_timer_callback()
    assert not node._watchdog_latched.is_set()
    node.watchdog_timer_callback()
    assert not node._watchdog_latched.is_set()
    node.watchdog_timer_callback()
    assert not node._watchdog_latched.is_set()
    node.watchdog_timer_callback()

    assert node._watchdog_latched.is_set()
    assert node._status_pub.messages[-1].system_state == SystemState.WATCHDOG.value
    assert ("read_watchdog", {}) in node._plc.calls


def test_watchdog_latch_blocks_output_commands() -> None:
    node = make_node()
    node._watchdog_latched.set()

    assert node._write_m_coil(0, True, "M0000") is False

    assert node._status_pub.messages[-1].system_state == SystemState.WATCHDOG.value


def test_estop_priority_over_watchdog_latch() -> None:
    node = make_node()
    node._watchdog_latched.set()
    node._estop_latched.set()

    node._publish_fault_state()

    assert node._status_pub.messages[-1].system_state == SystemState.E_STOP.value


def test_watchdog_timer_does_not_override_estop() -> None:
    node = make_node()
    node._estop_latched.set()

    node.watchdog_timer_callback()

    assert ("read_watchdog", {}) not in node._plc.calls
    assert node._status_pub.messages[-1].system_state == SystemState.E_STOP.value


def test_plc_error_is_recorded_to_system_events(tmp_path: Path) -> None:
    db_path = tmp_path / "robot_arm.db"
    with sqlite3.connect(db_path) as conn:
        conn.execute(
            """
            CREATE TABLE system_events (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                event_type TEXT NOT NULL,
                track TEXT,
                severity TEXT NOT NULL,
                notes TEXT
            )
            """
        )
        conn.commit()

    node = make_node(fail_write=True)
    node._config.db_path = str(db_path)

    assert node._write_m_coil(0, True, "M0000") is False

    with sqlite3.connect(db_path) as conn:
        row = conn.execute(
            "SELECT event_type, track, severity, notes FROM system_events"
        ).fetchone()

    assert row == (
        "plc_error",
        None,
        "error",
        "M0000 control failed: simulated coil write failure",
    )


def test_system_state_estop_writes_direct_output_without_reset() -> None:
    node = make_node()
    message = types.SimpleNamespace(data="e_stop")

    node.system_state_callback(message)

    assert ("write_coil", {"address": 256, "value": True}) not in node._plc.calls
    assert ("write_coil", {"address": 3, "value": True}) in node._plc.calls
    assert node._estop_latched.is_set()
    assert node._estop_pub.messages[-1].data is True
    assert node._status_pub.messages[-1].system_state == SystemState.E_STOP.value


def test_system_state_watchdog_is_ignored_when_estop_is_latched() -> None:
    node = make_node()
    node._estop_latched.set()
    message = types.SimpleNamespace(data="watchdog")

    node.system_state_callback(message)

    assert node._watchdog_latched.is_set() is False
    assert node._status_pub.messages[-1].system_state == SystemState.E_STOP.value


def test_pulse_m_coil_schedules_release_without_blocking() -> None:
    node = make_node()

    node._pulse_m_coil(0, "M0000")

    assert node._plc.calls[0] == ("write_coil", {"address": 0, "value": True})
    assert ("write_coil", {"address": 0, "value": False}) not in node._plc.calls
    assert node._pulse_timers

    node._pulse_timers[-1].callback()

    assert ("write_coil", {"address": 0, "value": False}) in node._plc.calls
    assert node._status_pub.messages[-1].system_state == SystemState.IDLE.value


def test_read_timer_does_not_reconnect_when_disconnected() -> None:
    node = make_node(connected=False)

    node.read_timer_callback()

    assert ("connect", {}) not in node._plc.calls


def test_connect_with_retry_sets_idle_outputs_on_startup() -> None:
    node = make_node()

    assert node._connect_with_retry() is True
    assert ("connect", {}) in node._plc.calls
    assert (
        "set_system_state",
        {"state": SystemState.IDLE, "apply_outputs": False},
    ) in node._plc.calls
    assert node._status_pub.messages[-1].system_state == SystemState.IDLE.value
    assert node._pulse_timers

    node._pulse_timers[0].callback()

    assert ("write_coil", {"address": 256, "value": False}) in node._plc.calls
    assert ("write_coil", {"address": 0, "value": True}) in node._plc.calls

    while node._pulse_timers:
        node._pulse_timers[0].callback()

    assert node._plc.calls[-1] == (
        "write_coil",
        {"address": 0, "value": False},
    )
