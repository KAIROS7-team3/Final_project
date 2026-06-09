"""Gemma 4 기반 Voice intent node.

이 노드는 `/voice/raw_text`를 받아 Gemma로 의도를 분류하고,
`fetch`/`return`만 DB feasibility gate를 거친 뒤 `/voice/intent`로 publish한다.

`cancel`과 `unknown`은 즉시 publish할 수 있지만, malformed output, 낮은
confidence, DB 오류, 불명확한 공구 ID는 모두 fail-closed로 처리한다.
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from interfaces.msg import Intent
from interfaces.srv import CheckToolFeasibility
from rclpy.node import Node
from std_msgs.msg import String

from voice.gemma_intent import GemmaConfig, GemmaIntentClassifier, default_toolbox_path
from voice.wake_word import DEFAULT_WAKE_WORDS, apply_wake_word_gate


class GemmaIntentNode(Node):
    """Gemma 결과를 프로젝트 표준 Intent 메시지로 발행하는 노드."""

    def __init__(
        self,
        classifier: GemmaIntentClassifier | None = None,
        feasibility_client=None,
    ) -> None:
        super().__init__("gemma_intent_node")

        self.declare_parameter("require_wake_word", True)
        self.declare_parameter("wake_words", list(DEFAULT_WAKE_WORDS))
        self.declare_parameter(
            "gemma_model_id",
            "~/models/gemma/gemma-3-1b-it",
        )
        self.declare_parameter("gemma_prompt_template_path", "")
        self.declare_parameter("toolbox_path", "")
        self.declare_parameter("gemma_device", "auto")
        self.declare_parameter("gemma_confidence_threshold", 0.75)
        self.declare_parameter("gemma_max_new_tokens", 128)
        self.declare_parameter("gemma_temperature", 0.0)
        self.declare_parameter("gemma_warmup", True)

        self._publisher = self.create_publisher(Intent, "/voice/intent", 1)
        self._feasibility_client = feasibility_client or self.create_client(
            CheckToolFeasibility,
            "/db/CheckToolFeasibility",
        )
        self.create_subscription(
            String,
            "/voice/raw_text",
            self._handle_raw_text,
            10,
        )

        if classifier is not None:
            self._classifier = classifier
            return

        prompt_template_path = self.get_parameter("gemma_prompt_template_path")
        prompt_template_path = (
            prompt_template_path.get_parameter_value().string_value
            or str(Path(__file__).with_name("gemma_prompt.txt"))
        )
        toolbox_path = self.get_parameter("toolbox_path")
        toolbox_path = (
            toolbox_path.get_parameter_value().string_value
            or str(default_toolbox_path())
        )

        config = GemmaConfig(
            model_id=self.get_parameter("gemma_model_id")
            .get_parameter_value()
            .string_value,
            prompt_template_path=prompt_template_path,
            toolbox_path=toolbox_path,
            device=self.get_parameter("gemma_device")
            .get_parameter_value()
            .string_value,
            confidence_threshold=self.get_parameter("gemma_confidence_threshold")
            .get_parameter_value()
            .double_value,
            max_new_tokens=self.get_parameter("gemma_max_new_tokens")
            .get_parameter_value()
            .integer_value,
            temperature=self.get_parameter("gemma_temperature")
            .get_parameter_value()
            .double_value,
            warmup=self.get_parameter("gemma_warmup")
            .get_parameter_value()
            .bool_value,
        )
        self._classifier = GemmaIntentClassifier(config)

    def _handle_raw_text(self, message: String) -> None:
        """Whisper 원문 한 건을 Gemma intent로 변환한다."""

        gate = apply_wake_word_gate(
            message.data,
            list(
                self.get_parameter("wake_words")
                .get_parameter_value()
                .string_array_value
            ),
            self.get_parameter("require_wake_word")
            .get_parameter_value()
            .bool_value,
        )
        if not gate.accepted:
            self.get_logger().debug("voice input ignored because wake word is missing")
            return

        parsed = self._classifier.classify(gate.command_text)
        if parsed.intent_type in {"cancel", "unknown"}:
            self._publish_intent(
                parsed.intent_type,
                parsed.tool_id,
                parsed.confidence,
                gate.command_text,
            )
            return

        if parsed.intent_type not in {"fetch", "return"}:
            self.get_logger().warning(
                f"unsupported Gemma intent rejected: {parsed.intent_type}"
            )
            return

        if not self._feasibility_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                "/db/CheckToolFeasibility service is not available; voice intent rejected"
            )
            return

        request = CheckToolFeasibility.Request()
        request.intent = parsed.intent_type
        request.tool_id = parsed.tool_id
        future = self._feasibility_client.call_async(request)
        future.add_done_callback(
            lambda done: self._handle_feasibility(
                done,
                parsed.intent_type,
                parsed.tool_id,
                parsed.confidence,
                gate.command_text,
            )
        )

    def _handle_feasibility(
        self,
        future,
        intent_type: str,
        tool_id: str,
        confidence: float,
        raw_utterance: str,
    ) -> None:
        """DB Gate 응답을 확인하고 통과한 명령만 publish한다."""

        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(
                f"DB feasibility check failed; voice intent rejected: {exc}"
            )
            return

        if not response.feasible:
            self.get_logger().warning(f"DB gate rejected voice intent: {response.reason}")
            return

        self._publish_intent(intent_type, tool_id, confidence, raw_utterance)

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
