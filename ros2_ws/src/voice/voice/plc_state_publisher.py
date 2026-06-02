"""PLC semantic state publisher helpers for voice-side ROS2 nodes."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from rclpy.node import Node

PLC_STATE_TOPIC = "/plc/system_state"
PLC_STATE_IDLE = "idle"
PLC_STATE_LISTENING = "listening"
PLC_STATE_INFERRING = "inferring"
PLC_STATE_ERROR = "error"


class VoicePlcStatePublisher:
    """Publish voice-stage PLC states without depending on the plc package."""

    def __init__(self, node: Node, topic: str = PLC_STATE_TOPIC) -> None:
        # std_msgs/String만 사용해 interfaces 변경 없이 PLC semantic 상태를 공유한다.
        # import를 지연시켜 ROS2 없는 단위 테스트에서도 모듈 import가 가능하게 한다.
        from std_msgs.msg import String

        self._message_type: type[Any] = String
        self._publisher = node.create_publisher(String, topic, 1)

    def publish(self, state: str) -> None:
        """Publish one semantic PLC state string."""

        message = self._message_type()
        message.data = state
        # voice 단계는 listening/inferring/error/idle까지만 책임지고,
        # moving 이후는 DB simulator나 실제 motion/orchestrator가 이어 받는다.
        self._publisher.publish(message)
