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

interfaces_module = sys.modules.setdefault("interfaces", types.ModuleType("interfaces"))
interfaces_msg_module = sys.modules.setdefault(
    "interfaces.msg",
    types.ModuleType("interfaces.msg"),
)

class _RobotStatus:
    is_moving: bool = False


interfaces_msg_module.RobotStatus = _RobotStatus

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


class FakePlcState:
    def __init__(self) -> None:
        self.states = []

    def publish(self, state: str) -> None:
        self.states.append(state)


@pytest.fixture
def whisper_node():
    node = WhisperNode.__new__(WhisperNode)
    node._state_lock = threading.Lock()
    node._is_moving = False
    node._publisher = FakePublisher()
    node._plc_state = FakePlcState()
    node._logger = FakeLogger()
    node.get_logger = lambda: node._logger
    return node


def test_publish_transcript_blocks_when_robot_is_moving(whisper_node) -> None:
    with whisper_node._state_lock:
        whisper_node._is_moving = True

    published = whisper_node.publish_transcript("스패너 가져와")

    assert published is False
    assert whisper_node._publisher.messages == []
    assert whisper_node._plc_state.states == []
    assert whisper_node._logger.warnings == [
        "voice input blocked because robot is moving"
    ]


def test_publish_transcript_ignores_empty_text(whisper_node) -> None:
    published = whisper_node.publish_transcript("   ")

    assert published is False
    assert whisper_node._publisher.messages == []
    assert whisper_node._plc_state.states == []


def test_publish_transcript_publishes_trimmed_text_when_idle(whisper_node) -> None:
    published = whisper_node.publish_transcript("  스패너 가져와  ")

    assert published is True
    assert len(whisper_node._publisher.messages) == 1
    assert whisper_node._publisher.messages[0].data == "스패너 가져와"
    assert whisper_node._plc_state.states == ["listening"]
