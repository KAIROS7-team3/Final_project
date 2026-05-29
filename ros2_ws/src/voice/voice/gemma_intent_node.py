"""Voice intent node with DB feasibility gate.

현재는 Gemma 4 대신 deterministic parser를 사용한다. 역할은
`/voice/raw_text`를 받아 intent로 바꾸고, fetch/return은 DB Gate를 통과한
경우에만 `/voice/intent`로 publish하는 것이다.

전체 흐름:
    std_msgs/String `/voice/raw_text`
    -> wake-word gate
    -> command_parser.parse_command()
    -> check_tool_feasibility service
    -> interfaces/Intent `/voice/intent`

주의: 이 노드는 DB 상태를 바꾸지 않는다. DB 상태 변경은 실제 motion 완료 후
orchestrator/motion 쪽에서 `/update_tool_status`를 호출해야 한다.
"""

from __future__ import annotations

import rclpy
from interfaces.msg import Intent
from interfaces.srv import CheckToolFeasibility
from rclpy.node import Node
from std_msgs.msg import String

from voice.command_parser import parse_command
from voice.wake_word import apply_wake_word_gate


class GemmaIntentNode(Node):
    """STT 원문을 intent 메시지로 변환하고 DB Gate로 불가능한 명령을 차단한다."""

    def __init__(self) -> None:
        super().__init__("gemma_intent_node")
        # True면 "로봇 스패너 가져와"처럼 wake word로 시작하는 문장만 처리한다.
        self.declare_parameter("require_wake_word", False)

        # 현장 운영자가 부를 호출어 목록. 기본은 "로봇" 하나다.
        self.declare_parameter("wake_words", ["로봇"])

        # downstream orchestrator는 이미 DB Gate를 통과한 intent만 받는다.
        self._publisher = self.create_publisher(Intent, "/voice/intent", 1)

        # fetch/return 명령은 DB 상태에 따라 허용 여부가 달라진다.
        self._feasibility_client = self.create_client(
            CheckToolFeasibility,
            "check_tool_feasibility",
        )
        # whisper_node가 publish한 원문 STT 결과를 받는다.
        self.create_subscription(String, "/voice/raw_text", self._handle_raw_text, 10)

    def _handle_raw_text(self, message: String) -> None:
        """Whisper STT 원문 한 건을 처리한다."""

        # wake word가 필요한 모드라면 호출어를 확인하고, 통과 시 호출어를 제거한
        # 실제 명령 본문만 parser에 넘긴다.
        gate = apply_wake_word_gate(
            message.data,
            list(self.get_parameter("wake_words").get_parameter_value().string_array_value),
            self.get_parameter("require_wake_word").get_parameter_value().bool_value,
        )
        if not gate.accepted:
            self.get_logger().debug("voice input ignored because wake word is missing")
            return

        # 현재는 Gemma 4 대신 deterministic keyword parser를 사용한다.
        parsed = parse_command(gate.command_text)
        if parsed.intent_type not in {"fetch", "return"}:
            # cancel/unknown은 DB Gate 대상이 아니므로 그대로 publish한다.
            self._publish_intent(
                parsed.intent_type,
                parsed.tool_id,
                parsed.confidence,
                gate.command_text,
            )
            return

        request = CheckToolFeasibility.Request()
        request.intent = parsed.intent_type
        request.tool_id = parsed.tool_id
        # DB Gate는 비동기로 호출해 raw_text 콜백을 오래 막지 않는다.
        future = self._feasibility_client.call_async(request)
        future.add_done_callback(
            lambda done: self._handle_feasibility(done, parsed, gate.command_text)
        )

    def _handle_feasibility(self, future, parsed, raw_utterance: str) -> None:
        """DB Gate 응답을 확인하고 허용된 명령만 `/voice/intent`로 내보낸다."""

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"DB feasibility check failed: {exc}")
            return
        if not response.feasible:
            # 예: 이미 out인 공구를 다시 fetch하려는 경우 여기서 차단된다.
            self.get_logger().warning(f"DB gate rejected voice intent: {response.reason}")
            return
        self._publish_intent(
            parsed.intent_type,
            parsed.tool_id,
            parsed.confidence,
            raw_utterance,
        )

    def _publish_intent(
        self,
        intent_type: str,
        tool_id: str,
        confidence: float,
        raw_utterance: str,
    ) -> None:
        """프로젝트 표준 Intent 메시지를 구성해 publish한다."""

        message = Intent()
        message.intent_type = intent_type
        message.tool_id = tool_id
        message.confidence = float(confidence)
        message.raw_utterance = raw_utterance
        message.timestamp = self.get_clock().now().to_msg()
        self._publisher.publish(message)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = GemmaIntentNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
