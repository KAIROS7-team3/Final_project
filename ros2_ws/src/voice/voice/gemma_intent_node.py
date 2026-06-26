"""Gemma 4 기반 Voice intent node.

이 노드는 `/voice/raw_text`를 받아 Gemma로 의도를 분류하고,
`fetch`/`return`만 DB feasibility gate를 거친 뒤 `/voice/intent`로 publish한다.

external_followup_control 모드 (Option C):
  whisper_node의 keyword 기반 follow-up을 끄고, 이 노드가 Gemma 결과에 따라
  후속 발화 수집을 직접 제어한다. Gemma가 unknown을 반환하면 텍스트를 누적하고
  다음 청크를 기다린다. followup_max_retries 초과 또는 followup_context_timeout
  초과 시 컨텍스트를 초기화한다.

  "성현아" → 웨이크워드 감지, 컨텍스트 누적 시작
  "드라이버" → 누적, Gemma("드라이버") → unknown → 대기
  "반납"     → 누적, Gemma("드라이버 반납") → return → publish

`cancel`과 `unknown`은 즉시 publish할 수 있지만, malformed output, 낮은
confidence, DB 오류, 불명확한 공구 ID는 모두 fail-closed로 처리한다.
"""

from __future__ import annotations

import threading
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

        # Option C: Gemma가 follow-up 필요 여부를 직접 판단
        self.declare_parameter("followup_max_retries", 2)
        self.declare_parameter("followup_context_timeout", 8.0)

        # 텍스트 누적 컨텍스트 (Option C 모드에서 사용)
        self._pending_context: str = ""
        self._followup_count: int = 0
        self._context_lock = threading.Lock()
        self._context_timer: threading.Timer | None = None

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

    # ------------------------------------------------------------------
    # 컨텍스트 누적 헬퍼
    # ------------------------------------------------------------------

    def _clear_context(self) -> None:
        self._cancel_context_timer()
        with self._context_lock:
            self._pending_context = ""
            self._followup_count = 0

    def _reset_context_timer(self) -> None:
        self._cancel_context_timer()
        timeout = (
            self.get_parameter("followup_context_timeout")
            .get_parameter_value()
            .double_value
        )
        t = threading.Timer(timeout, self._on_context_timeout)
        t.daemon = True
        t.start()
        self._context_timer = t

    def _cancel_context_timer(self) -> None:
        if self._context_timer is not None:
            self._context_timer.cancel()
            self._context_timer = None

    def _on_context_timeout(self) -> None:
        with self._context_lock:
            ctx = self._pending_context
        if ctx:
            self.get_logger().warning(
                f"명령 수집 타임아웃 — 컨텍스트 초기화 (ctx={ctx!r})"
            )
        self._clear_context()

    # ------------------------------------------------------------------
    # 메인 콜백
    # ------------------------------------------------------------------

    def _handle_raw_text(self, message: String) -> None:
        """Whisper 원문 한 건을 Gemma intent로 변환한다."""

        wake_words = list(
            self.get_parameter("wake_words").get_parameter_value().string_array_value
        )
        require_ww = (
            self.get_parameter("require_wake_word").get_parameter_value().bool_value
        )

        with self._context_lock:
            in_accumulation = bool(self._pending_context)

        if in_accumulation:
            # 누적 중 — 웨이크워드 있으면 제거, 없어도 수락
            gate = apply_wake_word_gate(message.data, wake_words, require_wake_word=True)
            command_text = (
                gate.command_text.strip() if gate.accepted else message.data.strip()
            )
            if not command_text:
                return
            with self._context_lock:
                prev = self._pending_context.replace("\x00", "").strip()
                self._pending_context = (
                    (prev + " " + command_text).strip() if prev else command_text
                )
                context = self._pending_context
        else:
            # 첫 발화 — 웨이크워드 게이트 적용
            gate = apply_wake_word_gate(message.data, wake_words, require_ww)
            if not gate.accepted:
                self.get_logger().debug(
                    "voice input ignored because wake word is missing"
                )
                return
            command_text = gate.command_text.strip()
            if not command_text:
                # 웨이크워드만 — 누적 모드 시작 (_pending_context에 sentinel 세팅)
                with self._context_lock:
                    self._pending_context = "\x00"  # 비어있지 않음, Gemma엔 전달 안 됨
                    self._followup_count = 0
                self._reset_context_timer()
                self.get_logger().info("웨이크워드 감지 — 명령을 말씀해주세요...")
                return
            with self._context_lock:
                self._pending_context = command_text
                self._followup_count = 0
            context = command_text

        self._reset_context_timer()
        parsed = self._classifier.classify(context)

        self.get_logger().info(
            f"Gemma: intent={parsed.intent_type} tool={parsed.tool_id} "
            f"conf={parsed.confidence:.2f} ctx={context!r}"
        )

        if parsed.intent_type == "cancel":
            self._clear_context()
            self._publish_intent("cancel", "", parsed.confidence, context)
            return

        if parsed.intent_type in {"fetch", "return"}:
            self._clear_context()
            self._do_db_gate(parsed.intent_type, parsed.tool_id, parsed.confidence, context)
            return

        # unknown — 재시도 판단
        max_retries = (
            self.get_parameter("followup_max_retries")
            .get_parameter_value()
            .integer_value
        )
        with self._context_lock:
            count = self._followup_count
            if count < max_retries:
                self._followup_count += 1

        if count < max_retries:
            self.get_logger().info(
                f"의도 불명확(conf={parsed.confidence:.2f}) — "
                f"추가 설명 대기 중 ({count + 1}/{max_retries})..."
            )
        else:
            self.get_logger().warning(
                f"최대 재시도({max_retries}) 초과 — 명령 폐기 (ctx={context!r})"
            )
            self._clear_context()

    # ------------------------------------------------------------------
    # DB Gate
    # ------------------------------------------------------------------

    def _do_db_gate(
        self,
        intent_type: str,
        tool_id: str,
        confidence: float,
        raw_utterance: str,
    ) -> None:
        if not self._feasibility_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                "/db/CheckToolFeasibility service is not available; voice intent rejected"
            )
            return

        request = CheckToolFeasibility.Request()
        request.intent = intent_type
        request.tool_id = tool_id
        future = self._feasibility_client.call_async(request)
        future.add_done_callback(
            lambda done: self._handle_feasibility(
                done, intent_type, tool_id, confidence, raw_utterance
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

    # ------------------------------------------------------------------
    # Publish
    # ------------------------------------------------------------------

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

    def destroy_node(self) -> bool:
        self._cancel_context_timer()
        return super().destroy_node()


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
