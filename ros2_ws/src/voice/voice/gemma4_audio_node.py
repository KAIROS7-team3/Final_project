"""Gemma 4 E2B 오디오 직접 의도 분류 ROS2 노드.

Whisper STT 없이 마이크 오디오를 Gemma 4 E2B에 직접 입력해
fetch/return/cancel/unknown 의도를 `/voice/intent`로 발행한다.

기존 whisper_node + gemma_intent_node 파이프라인과 병행 구성 가능하다.
같은 /voice/intent 토픽을 공유하므로 하류 노드 변경은 불필요하다.

웨이크워드:
  config 파라미터 wake_words 가 비어 있으면 모든 발화를 처리한다.
  리스트에 값이 있으면 해당 단어로 시작하는 발화만 분류로 넘기고,
  나머지는 무시한다. 웨이크워드 판별도 Gemma 4 가 오디오에서 직접 수행한다.
"""

from __future__ import annotations

import threading
import time

import rclpy
from interfaces.msg import Intent
from interfaces.srv import CheckToolFeasibility
from rclpy.node import Node

from voice.audio_input import (
    SAMPLE_RATE_HZ,
    AudioInputError,
    MicRecorder,
    MicRecorderConfig,
)
from voice.noise_filter import DeepFilterEnhancer, DeepFilterLoadError
from voice.gemma4_audio_intent import (
    Gemma4AudioClassifier,
    Gemma4AudioConfig,
    Gemma4AudioLoadError,
)
from voice.wake_word import DEFAULT_WAKE_WORDS, apply_wake_word_gate


class Gemma4AudioNode(Node):
    """마이크 오디오 → Gemma 4 오디오 추론 → /voice/intent 발행 노드."""

    def __init__(self) -> None:
        super().__init__("gemma4_audio_node")

        self.declare_parameter("enable_microphone", True)
        self.declare_parameter("gemma4_model_id", "~/models/gemma/gemma-4-e2b-it")
        self.declare_parameter("gemma4_device", "auto")
        self.declare_parameter("gemma4_confidence_threshold", 0.85)
        self.declare_parameter("gemma4_max_new_tokens", 96)
        self.declare_parameter("gemma4_warmup", True)
        self.declare_parameter("toolbox_path", "")

        # 웨이크워드 — 기본값 "성현" 시리즈
        self.declare_parameter("wake_words", list(DEFAULT_WAKE_WORDS))
        self.declare_parameter("require_wake_word", True)

        self.declare_parameter("max_utterance_seconds", 5.0)
        self.declare_parameter("silence_threshold", 0.02)

        # Silero VAD — RMS 대신 신경망 기반 음성 감지
        self.declare_parameter("use_silero_vad", False)
        self.declare_parameter("silero_vad_threshold", 0.5)
        # trailing_silence_seconds: 단어 간 짧은 끊김에 잘리지 않도록 1.5s 기본
        self.declare_parameter("trailing_silence_seconds", 1.5)

        # DeepFilterNet 3 — Gemma 4 추론 전 노이즈 제거
        self.declare_parameter("use_deepfilter", False)
        self.declare_parameter("deepfilter_device", "cuda")

        self._publisher = self.create_publisher(Intent, "/voice/intent", 1)
        self._feasibility_client = self.create_client(
            CheckToolFeasibility, "/db/CheckToolFeasibility"
        )

        self._is_moving = False
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._worker: threading.Thread | None = None
        self._deepfilter: DeepFilterEnhancer | None = None

        if self.get_parameter("use_deepfilter").get_parameter_value().bool_value:
            self._init_deepfilter()

        if self.get_parameter("enable_microphone").get_parameter_value().bool_value:
            self._start_worker()

    def _init_deepfilter(self) -> None:
        device = (
            self.get_parameter("deepfilter_device").get_parameter_value().string_value
        )
        try:
            self._deepfilter = DeepFilterEnhancer(device=device)
            self.get_logger().info(f"DeepFilterNet 3 활성화 (device={device})")
        except DeepFilterLoadError as exc:
            self.get_logger().error(f"DeepFilterNet 로드 실패 — 비활성화: {exc}")

    def _start_worker(self) -> None:
        cfg = Gemma4AudioConfig(
            model_id=self.get_parameter("gemma4_model_id")
            .get_parameter_value().string_value,
            device=self.get_parameter("gemma4_device")
            .get_parameter_value().string_value,
            confidence_threshold=self.get_parameter("gemma4_confidence_threshold")
            .get_parameter_value().double_value,
            max_new_tokens=self.get_parameter("gemma4_max_new_tokens")
            .get_parameter_value().integer_value,
            warmup=self.get_parameter("gemma4_warmup")
            .get_parameter_value().bool_value,
            toolbox_path=self.get_parameter("toolbox_path")
            .get_parameter_value().string_value,
        )

        trailing_s = (
            self.get_parameter("trailing_silence_seconds")
            .get_parameter_value().double_value
        )
        trailing_chunks = max(
            1, int(trailing_s * SAMPLE_RATE_HZ / 1024)
        )

        try:
            recorder = MicRecorder(
                MicRecorderConfig(
                    sample_rate_hz=SAMPLE_RATE_HZ,
                    max_duration_s=self.get_parameter("max_utterance_seconds")
                    .get_parameter_value().double_value,
                    silence_threshold=self.get_parameter("silence_threshold")
                    .get_parameter_value().double_value,
                    trailing_silence_chunks=trailing_chunks,
                    use_silero_vad=self.get_parameter("use_silero_vad")
                    .get_parameter_value().bool_value,
                    silero_vad_threshold=self.get_parameter("silero_vad_threshold")
                    .get_parameter_value().double_value,
                )
            )
            classifier = Gemma4AudioClassifier(cfg)
        except (AudioInputError, Gemma4AudioLoadError) as exc:
            self.get_logger().error(f"Gemma4AudioNode 초기화 실패: {exc}")
            return

        self._worker = threading.Thread(
            target=self._listen_loop,
            args=(recorder, classifier),
            daemon=True,
        )
        self._worker.start()
        self.get_logger().info("Gemma4AudioNode 시작 — 마이크 대기 중")

    def _listen_loop(
        self,
        recorder: MicRecorder,
        classifier: Gemma4AudioClassifier,
    ) -> None:
        while not self._stop_event.is_set():
            if self._is_moving_safe():
                time.sleep(0.1)
                continue

            audio = recorder.record_utterance()
            if audio.size == 0 or self._is_moving_safe():
                continue

            # 웨이크워드가 설정된 경우: 먼저 텍스트 없이 게이트 통과 확인 불가.
            # 대신 2단계 방식 — 짧은 발화로 웨이크워드를 감지하면 후속 녹음.
            # require_wake_word=False면 모든 발화를 Gemma에 직접 넘긴다.
            require_ww = (
                self.get_parameter("require_wake_word")
                .get_parameter_value().bool_value
            )

            if require_ww:
                audio = self._handle_wake_word(audio, recorder)
                if audio is None:
                    continue

            # DeepFilterNet 3 노이즈 제거 (활성화된 경우)
            if self._deepfilter is not None:
                try:
                    audio = self._deepfilter.enhance(audio)
                except Exception as exc:
                    self.get_logger().warning(f"DeepFilter 실패, 원본 사용: {exc}")

            try:
                result = classifier.classify(audio)
            except Exception as exc:
                self.get_logger().error(f"Gemma4 분류 실패: {exc}")
                time.sleep(0.5)
                continue

            self.get_logger().info(
                f"intent={result.intent_type} tool={result.tool_id} "
                f"conf={result.confidence:.2f}"
            )

            if result.intent_type == "unknown":
                continue

            if result.intent_type == "cancel":
                self._publish(result.intent_type, result.tool_id, result.confidence, "")
                continue

            if result.intent_type in {"fetch", "return"}:
                self._check_feasibility_and_publish(result)

    def _handle_wake_word(
        self,
        audio,
        recorder: MicRecorder,
    ):
        """웨이크워드만 감지하면 후속 녹음을 받아 합친다.

        오디오를 faster-whisper 없이 처리해야 하므로,
        짧은 발화(≤ 0.5초 실질 음성)를 웨이크워드 전용으로 간주한다.
        대신 Gemma에 "웨이크워드만 있고 명령이 없으면 unknown" 을 알리는
        방식 대신, 기존 텍스트 wake word gate를 STT 없이 흉내낸다.

        구현: 발화가 짧고(<= 1.5초) 내용 전체가 웨이크워드 형태이면
        후속 발화를 받아 연결한다.
        """
        import numpy as np

        duration_s = audio.size / SAMPLE_RATE_HZ
        wake_words = list(
            self.get_parameter("wake_words").get_parameter_value().string_array_value
        )

        # 발화가 충분히 길면(>1.5초) 웨이크워드+명령 합체로 간주, 그대로 분류
        if duration_s > 1.5:
            return audio

        # 짧은 발화 → 웨이크워드 단독일 가능성 높음 → 후속 녹음
        self.get_logger().info(
            f"짧은 발화({duration_s:.1f}s) — 후속 명령 대기 중..."
        )
        follow_audio = recorder.record_utterance()

        if follow_audio.size == 0:
            self.get_logger().debug("후속 발화 없음 — 무시")
            return None

        # 웨이크워드 발화 + 후속 발화 연결
        combined = np.concatenate([audio, follow_audio])
        self.get_logger().info(
            f"2단계 오디오 합침 ({audio.size/SAMPLE_RATE_HZ:.1f}s + "
            f"{follow_audio.size/SAMPLE_RATE_HZ:.1f}s)"
        )
        return combined

    def _check_feasibility_and_publish(self, result) -> None:
        if not self._feasibility_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("DB feasibility 서비스 없음 — intent 폐기")
            return

        req = CheckToolFeasibility.Request()
        req.intent = result.intent_type
        req.tool_id = result.tool_id
        future = self._feasibility_client.call_async(req)
        future.add_done_callback(
            lambda f: self._on_feasibility(
                f, result.intent_type, result.tool_id, result.confidence
            )
        )

    def _on_feasibility(self, future, intent_type, tool_id, confidence) -> None:
        try:
            resp = future.result()
        except Exception as exc:
            self.get_logger().error(f"DB feasibility 오류: {exc}")
            return
        if not resp.feasible:
            self.get_logger().warning(f"DB gate 거부: {resp.reason}")
            return
        self._publish(intent_type, tool_id, confidence, "")

    def _publish(
        self, intent_type: str, tool_id: str, confidence: float, raw_utterance: str
    ) -> None:
        msg = Intent()
        msg.intent_type = intent_type
        msg.tool_id = tool_id
        msg.confidence = float(confidence)
        msg.raw_utterance = raw_utterance
        msg.timestamp = self.get_clock().now().to_msg()
        self._publisher.publish(msg)

    def _is_moving_safe(self) -> bool:
        with self._state_lock:
            return self._is_moving

    def destroy_node(self) -> bool:
        self._stop_event.set()
        if self._worker is not None:
            self._worker.join(timeout=3.0)
        return super().destroy_node()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = Gemma4AudioNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
