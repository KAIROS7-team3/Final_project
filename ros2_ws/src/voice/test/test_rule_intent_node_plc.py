from __future__ import annotations

import sys
import types


if "rclpy" not in sys.modules:
    rclpy_module = types.ModuleType("rclpy")
    rclpy_node_module = types.ModuleType("rclpy.node")

    class _Node:
        pass

    rclpy_node_module.Node = _Node
    sys.modules["rclpy"] = rclpy_module
    sys.modules["rclpy.node"] = rclpy_node_module

interfaces_module = sys.modules.setdefault("interfaces", types.ModuleType("interfaces"))
interfaces_msg_module = sys.modules.setdefault(
    "interfaces.msg",
    types.ModuleType("interfaces.msg"),
)
interfaces_srv_module = sys.modules.setdefault(
    "interfaces.srv",
    types.ModuleType("interfaces.srv"),
)

class _Intent:
    intent_type: str = ""
    tool_id: str = ""
    confidence: float = 0.0
    raw_utterance: str = ""
    timestamp = None


class _CheckToolFeasibility:
    class Request:
        intent: str = ""
        tool_id: str = ""


interfaces_msg_module.Intent = _Intent
interfaces_srv_module.CheckToolFeasibility = _CheckToolFeasibility

if "std_msgs.msg" not in sys.modules:
    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")

    class _String:
        def __init__(self) -> None:
            self.data = ""

    std_msgs_msg_module.String = _String
    sys.modules.setdefault("std_msgs", std_msgs_module)
    sys.modules["std_msgs.msg"] = std_msgs_msg_module

from voice.rule_intent_node import RuleIntentNode


class FakeParameterValue:
    def __init__(self, *, bool_value: bool = False, string_array_value=None) -> None:
        self.bool_value = bool_value
        self.string_array_value = string_array_value or []


class FakeParameter:
    def __init__(self, value: FakeParameterValue) -> None:
        self._value = value

    def get_parameter_value(self) -> FakeParameterValue:
        return self._value


class FakeClock:
    class _Now:
        @staticmethod
        def to_msg():
            return object()

    @staticmethod
    def now() -> _Now:
        return FakeClock._Now()


class FakeLogger:
    def __init__(self) -> None:
        self.errors = []
        self.warnings = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warning(self, message: str) -> None:
        self.warnings.append(message)

    def debug(self, message: str) -> None:
        pass


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakePlcState:
    def __init__(self) -> None:
        self.states = []

    def publish(self, state: str) -> None:
        self.states.append(state)


class FakeFeasibilityClient:
    def __init__(self, available: bool) -> None:
        self.available = available

    def wait_for_service(self, timeout_sec: float) -> bool:
        return self.available


def make_node(*, service_available: bool = True) -> RuleIntentNode:
    node = RuleIntentNode.__new__(RuleIntentNode)
    node._publisher = FakePublisher()
    node._plc_state = FakePlcState()
    node._feasibility_client = FakeFeasibilityClient(service_available)
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node.get_clock = lambda: FakeClock()

    def get_parameter(name: str) -> FakeParameter:
        values = {
            "require_wake_word": FakeParameterValue(bool_value=True),
            "wake_words": FakeParameterValue(string_array_value=["코봇"]),
        }
        return FakeParameter(values[name])

    node.get_parameter = get_parameter
    return node


def string_message(text: str):
    message = sys.modules["std_msgs.msg"].String()
    message.data = text
    return message


def test_cancel_command_publishes_inferring_then_idle() -> None:
    node = make_node()

    node._handle_raw_text(string_message("코봇 취소"))

    assert node._plc_state.states == ["inferring", "idle"]
    assert len(node._publisher.messages) == 1
    assert node._publisher.messages[0].intent_type == "cancel"


def test_db_service_unavailable_publishes_error() -> None:
    node = make_node(service_available=False)

    node._handle_raw_text(string_message("코봇 스패너 가져와"))

    assert node._plc_state.states == ["inferring", "error"]
    assert node._publisher.messages == []
