from __future__ import annotations

import sys
import types
from concurrent.futures import Future


if "rclpy" not in sys.modules:
    rclpy_module = types.ModuleType("rclpy")
    rclpy_node_module = types.ModuleType("rclpy.node")

    class _Node:
        pass

    rclpy_node_module.Node = _Node
    sys.modules["rclpy"] = rclpy_module
    sys.modules["rclpy.node"] = rclpy_node_module

if "interfaces.msg" not in sys.modules:
    interfaces_module = types.ModuleType("interfaces")
    interfaces_msg_module = types.ModuleType("interfaces.msg")
    interfaces_srv_module = types.ModuleType("interfaces.srv")

    class _Intent:
        intent_type: str = ""
        tool_id: str = ""
        confidence: float = 0.0
        raw_utterance: str = ""
        timestamp = None

    class _RobotStatus:
        is_moving: bool = False

    class _CheckToolFeasibility:
        class Request:
            intent: str = ""
            tool_id: str = ""

    interfaces_msg_module.Intent = _Intent
    interfaces_msg_module.RobotStatus = _RobotStatus
    interfaces_srv_module.CheckToolFeasibility = _CheckToolFeasibility
    sys.modules.setdefault("interfaces", interfaces_module)
    sys.modules["interfaces.msg"] = interfaces_msg_module
    sys.modules["interfaces.srv"] = interfaces_srv_module

if "std_msgs.msg" not in sys.modules:
    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")

    class _String:
        data: str = ""

    std_msgs_msg_module.String = _String
    sys.modules.setdefault("std_msgs", std_msgs_module)
    sys.modules["std_msgs.msg"] = std_msgs_msg_module

from voice.gemma_intent import GemmaIntentResult
from voice.gemma_intent_node import GemmaIntentNode


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakeLogger:
    def __init__(self) -> None:
        self.debug_messages: list[str] = []
        self.warning_messages: list[str] = []
        self.error_messages: list[str] = []

    def debug(self, message: str) -> None:
        self.debug_messages.append(message)

    def warning(self, message: str) -> None:
        self.warning_messages.append(message)

    def error(self, message: str) -> None:
        self.error_messages.append(message)


class FakeClock:
    class _Now:
        @staticmethod
        def to_msg() -> str:
            return "timestamp"

    def now(self) -> "FakeClock._Now":
        return self._Now()


class FakeClassifier:
    def __init__(self, result: GemmaIntentResult) -> None:
        self.result = result
        self.inputs: list[str] = []

    def classify(self, raw_text: str) -> GemmaIntentResult:
        self.inputs.append(raw_text)
        return self.result


class FakeParameterValue:
    def __init__(
        self,
        *,
        bool_value: bool = False,
        string_array_value: list[str] | None = None,
        string_value: str = "",
        integer_value: int = 0,
        double_value: float = 0.0,
    ) -> None:
        self.bool_value = bool_value
        self.string_array_value = string_array_value or []
        self.string_value = string_value
        self.integer_value = integer_value
        self.double_value = double_value


class FakeParameter:
    def __init__(self, value: FakeParameterValue) -> None:
        self._value = value

    def get_parameter_value(self) -> FakeParameterValue:
        return self._value


class FakeFeasibilityClient:
    def __init__(self, feasible: bool = True, reason: str = "") -> None:
        self.feasible = feasible
        self.reason = reason
        self.requests = []

    def wait_for_service(self, timeout_sec: float) -> bool:
        return True

    def call_async(self, request) -> Future:
        self.requests.append(request)
        future: Future = Future()

        response = types.SimpleNamespace(
            feasible=self.feasible,
            reason=self.reason,
        )
        future.set_result(response)
        return future


def _build_node(
    classifier: FakeClassifier,
    feasibility_client: FakeFeasibilityClient | None = None,
    *,
    require_wake_word: bool = False,
) -> GemmaIntentNode:
    node = GemmaIntentNode.__new__(GemmaIntentNode)
    node._classifier = classifier
    node._feasibility_client = feasibility_client or FakeFeasibilityClient()
    node._publisher = FakePublisher()
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node.get_clock = lambda: FakeClock()
    node.get_parameter = lambda name: {
        "require_wake_word": FakeParameter(
            FakeParameterValue(bool_value=require_wake_word)
        ),
        "wake_words": FakeParameter(
            FakeParameterValue(string_array_value=["코봇"])
        ),
    }[name]
    return node


def test_cancel_is_published_without_db_gate() -> None:
    node = _build_node(
        FakeClassifier(
            GemmaIntentResult("cancel", "", 0.12, raw_output='{"intent_type":"cancel"}')
        )
    )

    node._handle_raw_text(types.SimpleNamespace(data="작업 취소"))

    assert len(node._publisher.messages) == 1
    assert node._publisher.messages[0].intent_type == "cancel"
    assert node._publisher.messages[0].raw_utterance == "작업 취소"


def test_fetch_requires_db_gate_before_publish() -> None:
    classifier = FakeClassifier(
        GemmaIntentResult(
            "fetch",
            "spanner_16mm",
            0.95,
            raw_output='{"intent_type":"fetch"}',
        )
    )
    feasibility_client = FakeFeasibilityClient(feasible=True)
    node = _build_node(classifier, feasibility_client)

    node._handle_raw_text(types.SimpleNamespace(data="스패너 가져와"))

    assert classifier.inputs == ["스패너 가져와"]
    assert len(feasibility_client.requests) == 1
    assert feasibility_client.requests[0].intent == "fetch"
    assert feasibility_client.requests[0].tool_id == "spanner_16mm"
    assert len(node._publisher.messages) == 1
    assert node._publisher.messages[0].intent_type == "fetch"
    assert node._publisher.messages[0].tool_id == "spanner_16mm"
    assert node._publisher.messages[0].raw_utterance == "스패너 가져와"


def test_fetch_rejected_by_db_gate_is_not_published() -> None:
    classifier = FakeClassifier(
        GemmaIntentResult(
            "fetch",
            "spanner_16mm",
            0.95,
            raw_output='{"intent_type":"fetch"}',
        )
    )
    feasibility_client = FakeFeasibilityClient(feasible=False, reason="out")
    node = _build_node(classifier, feasibility_client)

    node._handle_raw_text(types.SimpleNamespace(data="스패너 가져와"))

    assert len(node._publisher.messages) == 0
    assert feasibility_client.requests[0].intent == "fetch"
    assert node._logger.warning_messages[-1] == "DB gate rejected voice intent: out"


def test_missing_wake_word_blocks_input_when_enabled() -> None:
    classifier = FakeClassifier(
        GemmaIntentResult(
            "fetch",
            "spanner_16mm",
            0.95,
            raw_output='{"intent_type":"fetch"}',
        )
    )
    node = _build_node(classifier, require_wake_word=True)

    node._handle_raw_text(types.SimpleNamespace(data="스패너 가져와"))

    assert classifier.inputs == []
    assert len(node._publisher.messages) == 0
