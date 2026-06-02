from __future__ import annotations

import sys
import types


if "std_msgs.msg" not in sys.modules:
    std_msgs_module = types.ModuleType("std_msgs")
    std_msgs_msg_module = types.ModuleType("std_msgs.msg")

    class _String:
        def __init__(self) -> None:
            self.data = ""

    std_msgs_msg_module.String = _String
    sys.modules.setdefault("std_msgs", std_msgs_module)
    sys.modules["std_msgs.msg"] = std_msgs_msg_module

from voice.plc_state_publisher import (
    PLC_STATE_INFERRING,
    PLC_STATE_LISTENING,
    PLC_STATE_TOPIC,
    VoicePlcStatePublisher,
)


class FakePublisher:
    def __init__(self) -> None:
        self.messages = []

    def publish(self, message) -> None:
        self.messages.append(message)


class FakeNode:
    def __init__(self) -> None:
        self.topic = ""
        self.depth = 0
        self.publisher = FakePublisher()

    def create_publisher(self, message_type, topic: str, depth: int) -> FakePublisher:
        self.topic = topic
        self.depth = depth
        return self.publisher


def test_voice_plc_state_publisher_uses_system_state_topic() -> None:
    node = FakeNode()

    publisher = VoicePlcStatePublisher(node)

    assert node.topic == PLC_STATE_TOPIC
    assert node.depth == 1
    publisher.publish(PLC_STATE_LISTENING)
    publisher.publish(PLC_STATE_INFERRING)
    assert [message.data for message in node.publisher.messages] == [
        "listening",
        "inferring",
    ]
