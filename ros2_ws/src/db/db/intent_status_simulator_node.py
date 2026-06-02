"""음성 intent를 받아 motion 완료를 흉내 내는 수동 테스트 노드.

운영 제어 경로가 아니다. 하드웨어를 움직이지 않고 다음 흐름만 검증하기 위해 둔다.
`/voice/raw_text -> /voice/intent -> DB update`

중요: 테스트 노드라도 DB Gate를 다시 확인한다. voice node에서 이미 확인했더라도
이 노드가 직접 DB 상태를 바꾸므로, gate 우회 경로를 만들지 않기 위해 한 번 더
`/db/CheckToolFeasibility`를 호출한다.
"""

from __future__ import annotations

import rclpy
from interfaces.msg import Intent
from interfaces.srv import CheckToolFeasibility, UpdateToolStatus
from rclpy.node import Node

from db.intent_status_mapping import simulated_status_for_intent
from db.plc_state_publisher import (
    PLC_STATE_ERROR,
    PLC_STATE_IDLE,
    PLC_STATE_MOVING,
    PlcStatePublisher,
)


class IntentStatusSimulatorNode(Node):
    """`/voice/intent`를 simulated motion 완료 DB update로 변환하는 노드."""

    def __init__(self) -> None:
        super().__init__("intent_status_simulator_node")
        self.declare_parameter("track", "A")
        self._track = self.get_parameter("track").get_parameter_value().string_value
        # 테스트 노드지만 DB Gate를 다시 통과시켜 S-2 우회를 만들지 않는다.
        self._feasibility_client = self.create_client(
            CheckToolFeasibility,
            "/db/CheckToolFeasibility",
        )
        self._update_client = self.create_client(
            UpdateToolStatus,
            "/db/UpdateToolStatus",
        )
        self._plc_state = PlcStatePublisher(self)
        self.create_subscription(Intent, "/voice/intent", self._handle_intent, 1)
        self.get_logger().warning(
            "manual simulator active: voice intents will update DB after DB gate"
        )

    def _handle_intent(self, message: Intent) -> None:
        """fetch/return intent를 테스트용 상태 전이 요청으로 변환한다."""

        update = simulated_status_for_intent(message.intent_type)
        tool_id = message.tool_id.strip()
        if update is None or not tool_id:
            self.get_logger().info(
                f"simulator ignored intent={message.intent_type} tool_id={message.tool_id}"
            )
            return

        if not self._feasibility_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/db/CheckToolFeasibility service is not available")
            # DB Gate를 확인할 수 없으면 S-2상 DB 상태 변경을 진행하지 않는다.
            return

        request = CheckToolFeasibility.Request()
        request.intent = message.intent_type
        request.tool_id = tool_id
        future = self._feasibility_client.call_async(request)
        future.add_done_callback(
            lambda done: self._handle_feasibility(done, message, update.new_status)
        )

    def _handle_feasibility(
        self,
        future: rclpy.task.Future,
        message: Intent,
        new_status: str,
    ) -> None:
        """DB Gate 승인 후에만 `/db/UpdateToolStatus`를 호출한다."""

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"DB feasibility check failed: {exc}")
            # Gate 확인 실패는 운영자가 볼 수 있게 PLC error로 연결한다.
            self._plc_state.publish(PLC_STATE_ERROR)
            return

        if not response.feasible:
            self.get_logger().warning(
                f"simulated DB update rejected by DB gate: {response.reason}"
            )
            self._plc_state.publish(PLC_STATE_ERROR)
            return

        if not self._update_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("/db/UpdateToolStatus service is not available")
            self._plc_state.publish(PLC_STATE_ERROR)
            return

        request = UpdateToolStatus.Request()
        request.tool_id = message.tool_id.strip()
        request.new_status = new_status
        request.event_type = message.intent_type
        request.track = self._track
        # raw_utterance를 notes에 남겨 수동 테스트 중 어떤 문장이 DB 상태 변경으로
        # 이어졌는지 나중에 추적할 수 있게 한다.
        request.notes = (
            "manual voice intent simulation"
            if not message.raw_utterance
            else f"manual voice intent simulation: {message.raw_utterance}"
        )
        update_future = self._update_client.call_async(request)
        # 이 테스트 노드는 실제 arm motion 대신 "움직이는 중" 상태만 흉내 낸다.
        self._plc_state.publish(PLC_STATE_MOVING)
        update_future.add_done_callback(self._handle_update_result)

    def _handle_update_result(self, future: rclpy.task.Future) -> None:
        """시뮬레이션 DB 쓰기의 최종 결과를 로그로 남긴다."""

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"DB status update failed: {exc}")
            self._plc_state.publish(PLC_STATE_ERROR)
            return

        if response.success:
            self.get_logger().info(f"simulated DB update complete: {response.message}")
            # simulated motion 완료 후에는 PLC 표시를 대기 상태로 되돌린다.
            self._plc_state.publish(PLC_STATE_IDLE)
            return
        self.get_logger().error(f"simulated DB update rejected: {response.message}")
        self._plc_state.publish(PLC_STATE_ERROR)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = IntentStatusSimulatorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
