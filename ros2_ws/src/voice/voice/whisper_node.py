"""Whisper STT ROS2 node.

마이크 입력을 녹음하고 Whisper로 변환한 뒤 `/voice/raw_text`에 publish한다.
S-7 규칙에 따라 로봇 이동 중에는 음성 입력을 막는다.

전체 흐름:
    PyAudio microphone
    -> MicRecorder.record_utterance()
    -> WhisperTranscriber.transcribe()
    -> std_msgs/String `/voice/raw_text`

이 노드는 intent를 직접 판단하지 않는다. 명령 해석과 DB Gate는
`gemma_intent_node`가 담당하고, `rule_intent_node`는 fallback 용도로만 남긴다.
"""

from __future__ import annotations

import threading
import time

import rclpy
from interfaces.msg import RobotStatus
from rclpy.node import Node
from std_msgs.msg import String

from voice.audio_input import (
    SAMPLE_RATE_HZ,
    AudioInputError,
    MicRecorder,
    MicRecorderConfig,
)
from voice.transcript_filter import is_likely_hallucinated_transcript
from voice.transcriber import (
    WhisperConfig,
    WhisperLoadError,
    WhisperTranscriber,
)
from voice.wake_word import DEFAULT_WAKE_WORDS, apply_wake_word_gate


class WhisperNode(Node):
    """마이크 녹음 worker와 Whisper STT를 ROS2 topic으로 연결하는 노드."""

    def __init__(self) -> None:
        super().__init__("whisper_node")
        # enable_microphone=False로 실행하면 테스트에서 직접 publish_transcript()만
        # 검증할 수 있다.
        self.declare_parameter("enable_microphone", True)

        # Whisper 모델 크기. 한국어 인식률과 GPU/CPU 처리 시간을 같이 고려한다.
        self.declare_parameter("whisper_model_size", "small")

        # `auto`는 CUDA 사용 가능 시 GPU, 아니면 CPU를 뜻한다.
        self.declare_parameter("whisper_device", "auto")

        # STT 백엔드 선택: "faster" (faster-whisper, VAD 내장) 또는 "openai".
        self.declare_parameter("whisper_backend", "faster")

        # 한국어 단발 명령 정확도를 높이기 위한 Whisper decoding 옵션이다.
        self.declare_parameter("whisper_beam_size", 10)
        self.declare_parameter("whisper_best_of", 5)
        self.declare_parameter("whisper_no_speech_threshold", 0.6)
        self.declare_parameter("whisper_logprob_threshold", -1.0)
        self.declare_parameter("whisper_compression_ratio_threshold", 2.4)
        self.declare_parameter(
            "whisper_initial_prompt",
            (
                "코봇, 코버, 코보, 코부, 고봇, 고버, 고보, 고부, 꼬부, "
                "공구함, 두산 로봇, 스테이징, 십자 드라이버, 커터칼, 라쳇 렌치, "
                "멕가이버, 스패너 16mm, 복스 소켓 19mm, 가져와, 꺼내줘, 반납, "
                "돌려놔, 취소"
            ),
        )

        # 한 번의 발화를 너무 길게 잡지 않도록 최대 녹음 시간을 제한한다.
        self.declare_parameter("max_utterance_seconds", 4.0)

        # RMS 기반 발화 시작 기준. 마이크 입력이 작으면 이 값을 낮춰야 한다.
        self.declare_parameter("silence_threshold", 0.02)

        # Whisper가 반복 토큰 환각을 냈을 때 publish를 막기 위한 경량 필터다.
        self.declare_parameter("reject_hallucinated_transcripts", True)

        # 웨이크워드만 발화되었을 때 즉시 후속 녹음을 받아 합친다.
        # 예: "코봇" → 대기 → "스크류드라이버 가져와" → "코봇 스크류드라이버 가져와"
        self.declare_parameter("follow_up_on_wake_word_only", True)
        self.declare_parameter("wake_words", list(DEFAULT_WAKE_WORDS))

        # /robot/status 콜백과 마이크 worker thread가 공유하는 상태다.
        self._is_moving = False
        self._state_lock = threading.Lock()
        self._stop_event = threading.Event()
        self._recorder: MicRecorder | None = None
        self._worker: threading.Thread | None = None

        # STT 결과는 아직 intent가 아닌 원문 텍스트이므로 raw_text topic에 낸다.
        self._publisher = self.create_publisher(String, "/voice/raw_text", 10)

        # 로봇이 움직이는 동안에는 모터음/충돌음이 명령으로 오인식될 수 있으므로
        # RobotStatus.is_moving을 구독해 S-7 gate로 사용한다.
        self.create_subscription(
            RobotStatus,
            "/robot/status",
            self._handle_robot_status,
            1,
        )

        enable_microphone = self.get_parameter(
            "enable_microphone"
        ).get_parameter_value().bool_value
        if enable_microphone:
            self._start_microphone_worker()

    def _handle_robot_status(self, message: RobotStatus) -> None:
        """RobotStatus 메시지에서 is_moving 값을 받아 S-7 gate를 갱신한다."""

        with self._state_lock:
            self._is_moving = bool(message.is_moving)

    def publish_transcript(self, transcript: str) -> bool:
        """STT 결과를 `/voice/raw_text`로 publish한다.

        Returns:
            publish했으면 True, 로봇 이동 중이거나 빈 문자열이면 False.
        """

        if self._robot_is_moving():
            self.get_logger().warning(
                "voice input blocked because robot is moving"
            )
            return False
        text = transcript.strip()
        if not text:
            # Whisper가 빈 문자열을 반환한 경우 downstream으로 보내지 않는다.
            return False
        if self._should_reject_transcript(text):
            self.get_logger().warning(
                f"voice input rejected as hallucinated transcript: {text}"
            )
            return False
        message = String()
        message.data = text
        self._publisher.publish(message)
        return True

    def destroy_node(self) -> bool:
        """노드 종료 시 background thread와 마이크 장치를 정리한다."""

        self._stop_event.set()
        if self._worker is not None:
            self._worker.join(timeout=2.0)
        if self._recorder is not None:
            self._recorder.close()
        return super().destroy_node()

    def _start_microphone_worker(self) -> None:
        """마이크 recorder와 Whisper transcriber를 만들고 worker thread를 시작한다."""

        try:
            # recorder는 PyAudio 장치 접근을 담당한다. 장치가 없거나 PyAudio가
            # 설치되지 않았으면 AudioInputError가 발생한다.
            self._recorder = MicRecorder(
                MicRecorderConfig(
                    sample_rate_hz=SAMPLE_RATE_HZ,
                    max_duration_s=self.get_parameter("max_utterance_seconds")
                    .get_parameter_value()
                    .double_value,
                    silence_threshold=self.get_parameter("silence_threshold")
                    .get_parameter_value()
                    .double_value,
                )
            )
            # transcriber는 실제 Whisper 모델을 lazy-load한다.
            transcriber = WhisperTranscriber(
                WhisperConfig(
                    backend=self.get_parameter("whisper_backend")
                    .get_parameter_value()
                    .string_value,
                    model_size=self.get_parameter("whisper_model_size")
                    .get_parameter_value()
                    .string_value,
                    device=self.get_parameter("whisper_device")
                    .get_parameter_value()
                    .string_value,
                    beam_size=self.get_parameter("whisper_beam_size")
                    .get_parameter_value()
                    .integer_value,
                    best_of=self.get_parameter("whisper_best_of")
                    .get_parameter_value()
                    .integer_value,
                    initial_prompt=self.get_parameter("whisper_initial_prompt")
                    .get_parameter_value()
                    .string_value,
                    no_speech_threshold=self.get_parameter(
                        "whisper_no_speech_threshold"
                    )
                    .get_parameter_value()
                    .double_value,
                    logprob_threshold=self.get_parameter(
                        "whisper_logprob_threshold"
                    )
                    .get_parameter_value()
                    .double_value,
                    compression_ratio_threshold=self.get_parameter(
                        "whisper_compression_ratio_threshold"
                    )
                    .get_parameter_value()
                    .double_value,
                )
            )
        except (AudioInputError, WhisperLoadError) as exc:
            # 마이크/STT가 실패해도 ROS2 노드 자체는 살아 있게 두어 로그와
            # parameter 상태를 확인할 수 있게 한다.
            self.get_logger().error(f"microphone STT disabled: {exc}")
            return

        # rclpy.spin()과 별도로 마이크 입력은 blocking read를 사용하므로
        # background thread에서 계속 듣는다.
        self._worker = threading.Thread(
            target=self._listen_loop,
            args=(self._recorder, transcriber),
            daemon=True,
        )
        self._worker.start()

    def _listen_loop(
        self,
        recorder: MicRecorder,
        transcriber: WhisperTranscriber,
    ) -> None:
        """마이크에서 발화를 반복 녹음하고 STT 결과를 publish하는 루프."""

        while not self._stop_event.is_set():
            if self._robot_is_moving():
                # 이동 중에는 녹음 자체를 쉬어 불필요한 STT 연산도 막는다.
                time.sleep(0.1)
                continue

            audio = recorder.record_utterance()
            if audio.size == 0 or self._robot_is_moving():
                # 녹음 직후 로봇이 움직이기 시작했을 수도 있으므로 한 번 더 확인한다.
                continue

            try:
                # Whisper 예외가 노드 전체를 죽이지 않도록 worker 안에서 잡는다.
                transcript = transcriber.transcribe(audio, SAMPLE_RATE_HZ)
            except Exception as exc:
                self.get_logger().error(f"Whisper transcription failed: {exc}")
                time.sleep(0.5)
                continue

            # 웨이크워드만 말했고 명령이 없으면 즉시 후속 녹음
            transcript = self._maybe_append_followup(
                transcript, recorder, transcriber
            )

            self.publish_transcript(transcript)

    def _maybe_append_followup(
        self,
        transcript: str,
        recorder: MicRecorder,
        transcriber: WhisperTranscriber,
    ) -> str:
        """웨이크워드만 발화된 경우 후속 명령을 한 번 더 녹음해 합쳐서 반환한다.

        "코봇" 만 말하고 멈췄을 때 → "명령을 말씀해주세요" 로그 출력 →
        다음 발화를 녹음 → "코봇 스크류드라이버 가져와" 형태로 합침.
        """

        if not (
            self.get_parameter("follow_up_on_wake_word_only")
            .get_parameter_value()
            .bool_value
        ):
            return transcript

        wake_words = list(
            self.get_parameter("wake_words").get_parameter_value().string_array_value
        )
        gate = apply_wake_word_gate(transcript, wake_words, require_wake_word=True)

        if not (gate.accepted and not gate.command_text.strip()):
            return transcript

        self.get_logger().info(
            f"웨이크워드 감지 ({transcript!r}) — 명령을 말씀해주세요..."
        )

        try:
            follow_audio = recorder.record_utterance()
        except Exception as exc:
            self.get_logger().warning(f"후속 녹음 실패: {exc}")
            return transcript

        if follow_audio.size == 0 or self._robot_is_moving():
            return transcript

        try:
            follow_text = transcriber.transcribe(follow_audio, SAMPLE_RATE_HZ)
        except Exception as exc:
            self.get_logger().error(f"후속 STT 실패: {exc}")
            return transcript

        if not follow_text.strip():
            self.get_logger().debug("후속 발화 없음 — 웨이크워드 단독 무시")
            return transcript

        combined = transcript.strip() + " " + follow_text.strip()
        self.get_logger().info(f"2단계 발화 합침: {combined!r}")
        return combined

    def _should_reject_transcript(self, transcript: str) -> bool:
        """환각성 반복 토큰이 많으면 downstream으로 넘기지 않는다."""

        reject_hallucinated = self.get_parameter(
            "reject_hallucinated_transcripts"
        ).get_parameter_value().bool_value
        return reject_hallucinated and is_likely_hallucinated_transcript(transcript)

    def _robot_is_moving(self) -> bool:
        """Thread-safe하게 현재 motion gate 상태를 읽는다."""

        with self._state_lock:
            return self._is_moving


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = WhisperNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
