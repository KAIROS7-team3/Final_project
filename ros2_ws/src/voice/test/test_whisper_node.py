from __future__ import annotations

import sys
import threading
import types

import pytest


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

    class _RobotStatus:
        is_moving: bool = False

    interfaces_msg_module.RobotStatus = _RobotStatus
    sys.modules.setdefault("interfaces", interfaces_module)
    sys.modules["interfaces.msg"] = interfaces_msg_module

if "std_msgs.msg" not in sys.modules:
    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")

    class _String:
        data: str = ""

    std_msgs_msg_module.String = _String
    sys.modules.setdefault("std_msgs", std_msgs_module)
    sys.modules["std_msgs.msg"] = std_msgs_msg_module

from voice.whisper_node import WhisperNode


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakeLogger:
    def __init__(self) -> None:
        self.warnings = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


class FakeParameterValue:
    def __init__(self, bool_value: bool = False) -> None:
        self.bool_value = bool_value


class FakeParameter:
    def __init__(self, value: FakeParameterValue) -> None:
        self._value = value

    def get_parameter_value(self) -> FakeParameterValue:
        return self._value


@pytest.fixture
def whisper_node():
    node = WhisperNode.__new__(WhisperNode)
    node._state_lock = threading.Lock()
    node._is_moving = False
    node._publisher = FakePublisher()
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    node.get_parameter = lambda name: {
        "reject_hallucinated_transcripts": FakeParameter(
            FakeParameterValue(bool_value=True)
        )
    }[name]
    return node


def test_publish_transcript_blocks_when_robot_is_moving(whisper_node) -> None:
    with whisper_node._state_lock:
        whisper_node._is_moving = True

    published = whisper_node.publish_transcript("스패너 가져와")

    assert published is False
    assert whisper_node._publisher.messages == []
    assert whisper_node._logger.warnings == [
        "voice input blocked because robot is moving"
    ]


def test_publish_transcript_ignores_empty_text(whisper_node) -> None:
    published = whisper_node.publish_transcript("   ")

    assert published is False
    assert whisper_node._publisher.messages == []


def test_publish_transcript_publishes_trimmed_text_when_idle(whisper_node) -> None:
    published = whisper_node.publish_transcript("  스패너 가져와  ")

    assert published is True
    assert len(whisper_node._publisher.messages) == 1
    assert whisper_node._publisher.messages[0].data == "스패너 가져와"


def test_publish_transcript_rejects_repeated_hallucination(whisper_node) -> None:
    published = whisper_node.publish_transcript("렌치 렌치 렌치 렌치")

    assert published is False
    assert whisper_node._publisher.messages == []
    assert whisper_node._logger.warnings[-1].startswith(
        "voice input rejected as hallucinated transcript:"
    )
