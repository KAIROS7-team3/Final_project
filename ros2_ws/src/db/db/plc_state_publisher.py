"""PLC semantic state topic publisher helpers for DB-side ROS2 nodes."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from rclpy.node import Node

PLC_STATE_TOPIC = "/plc/system_state"
PLC_STATE_IDLE = "idle"
PLC_STATE_MOVING = "moving"
PLC_STATE_ERROR = "error"


class PlcStatePublisher:
    """Publish semantic PLC states without exposing PLC memory addresses."""

    def __init__(self, node: Node, topic: str = PLC_STATE_TOPIC) -> None:
        # DB 패키지도 PLC memory address를 알지 않는다. 의미 상태 문자열만 발행한다.
        # import를 지연시켜 ROS2 없는 helper 테스트에서 std_msgs stub을 사용할 수 있게 한다.
        from std_msgs.msg import String

        self._message_type: type[Any] = String
        self._publisher = node.create_publisher(String, topic, 1)

    def publish(self, state: str) -> None:
        """Publish one semantic PLC state string."""

        message = self._message_type()
        message.data = state
        self._publisher.publish(message)


def plc_state_for_fod_transition(new_status: str) -> str | None:
    """Map DB FOD timeout transitions to a PLC semantic state."""

    # FOD timeout은 운영자가 즉시 알아야 하는 오류 상태다.
    # 현재 래더에서는 error/e_stop 모두 M0003 -> P0043 경고 출력으로 매핑한다.
    if new_status in {"missing", "fod_alert"}:
        return PLC_STATE_ERROR
    return None
