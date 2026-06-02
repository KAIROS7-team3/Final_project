"""Whisper 모델 로드와 STT 호출을 감싸는 wrapper.

`whisper_node`가 녹음한 numpy 오디오를 받아 실제 STT를 수행한다.
ROS2 의존성은 없으므로 단위 테스트에서 fake model을 넣어 설정값을 검증할 수
있다.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from typing import Protocol

import numpy as np

from voice.audio_input import SAMPLE_RATE_HZ


class WhisperLoadError(RuntimeError):
    """openai-whisper 로드 또는 초기화 실패 시 발생한다."""


class WhisperModel(Protocol):
    """테스트에서 fake Whisper 모델을 주입하기 위한 최소 protocol.

    openai-whisper 모델 전체 타입에 의존하지 않고, 여기서 필요한
    `transcribe()` 메서드만 요구한다.
    """

    def transcribe(self, audio: np.ndarray, **kwargs) -> dict[str, object]:
        ...


ModelLoader = Callable[[str, str], WhisperModel]


@dataclass(frozen=True)
class WhisperConfig:
    """Whisper 추론 설정.

    한국어 단발 명령 인식을 안정화하기 위해 language와 temperature를 고정한다.

    Attributes:
        model_size: tiny/base/small/medium/large 계열 모델명. 기본은 small.
        device: `auto`면 CUDA 사용 가능 여부를 보고 cuda/cpu를 선택한다.
        language: 한국어 명령만 쓰므로 자동 감지 대신 `ko`로 고정한다.
        task: 번역이 아니라 원문 transcription을 수행한다.
        beam_size: 한국어 명령 후보를 더 넓게 탐색한다.
        best_of: temperature fallback 사용 시 후보 수를 제한한다.
        initial_prompt: 프로젝트 공구명과 명령 표현을 Whisper에 힌트로 제공한다.
        temperature: 0.0으로 두어 같은 입력에서 흔들림을 줄인다.
        condition_on_previous_text: 단발 명령에서는 이전 문맥을 끄는 것이 안전하다.
    """

    model_size: str = "small"
    device: str = "auto"
    language: str = "ko"          # 언어 고정 (자동감지 끄기)
    task: str = "transcribe"
    beam_size: int = 10           # 기본 5 → 10
    best_of: int = 5
    temperature: float = 0.0      # 결정론적 출력
    initial_prompt: str = (
        "코봇, 공구함, 두산 로봇, 스테이징, 십자 드라이버, 커터칼, 라쳇 렌치, "
        "멕가이버, 스패너 16mm, 복스 소켓 19mm, 가져와, 꺼내줘, 반납, "
        "돌려놔, 취소"
    )
    condition_on_previous_text: bool = False  # 오류 누적 방지


class WhisperTranscriber:
    """Whisper 모델을 lazy-load하고 numpy 오디오를 텍스트로 변환한다.

    모델 파일 로드는 비용이 크기 때문에 객체 생성 시점이 아니라 첫 transcribe
    호출 시점에 수행한다. 이렇게 하면 노드 초기화 실패 원인을 분리하기 쉽다.
    """

    def __init__(
        self,
        config: WhisperConfig | None = None,
        model_loader: ModelLoader | None = None,
    ) -> None:
        self.config = config or WhisperConfig()
        # 테스트에서는 실제 Whisper를 로드하지 않도록 fake loader를 주입할 수 있다.
        self._model_loader = model_loader
        self._model: WhisperModel | None = None
        self._device = self._resolve_device(self.config.device)

    def transcribe(
        self,
        audio: np.ndarray,
        sample_rate_hz: int = SAMPLE_RATE_HZ,
    ) -> str:
        """16 kHz mono float32 audio를 Whisper 텍스트로 변환한다."""

        if sample_rate_hz != SAMPLE_RATE_HZ:
            raise ValueError(
                f"WhisperTranscriber expects {SAMPLE_RATE_HZ} Hz audio, "
                f"got {sample_rate_hz} Hz"
            )

        audio_np = np.asarray(audio, dtype=np.float32)
        if audio_np.ndim != 1:
            raise ValueError("WhisperTranscriber expects mono 1-D audio")

        model = self._get_model()
        # Whisper에는 이미 mono float32 16 kHz 배열만 넘긴다. 여기서 contiguous 배열로
        # 바꿔 PyTorch 변환 과정에서 불필요한 copy/stride 문제를 줄인다.
        result = model.transcribe(
            np.ascontiguousarray(audio_np),
            language=self.config.language,
            task=self.config.task,
            beam_size=self.config.beam_size,
            best_of=self.config.best_of,
            temperature=self.config.temperature,
            initial_prompt=self.config.initial_prompt or None,
            condition_on_previous_text=self.config.condition_on_previous_text,
            fp16=self._device == "cuda",
        )
        return str(result.get("text", "")).strip()

    def _get_model(self) -> WhisperModel:
        """첫 STT 요청 시 Whisper 모델을 로드하고 이후에는 재사용한다."""

        if self._model is not None:
            return self._model
        if self._model_loader is not None:
            self._model = self._model_loader(
                self.config.model_size,
                self._device,
            )
            return self._model

        try:
            import whisper
        except ImportError as exc:
            raise WhisperLoadError(
                "openai-whisper is required for STT"
            ) from exc

        # 최초 실행 시 모델 파일이 ~/.cache/whisper 아래에 없으면 다운로드가 발생한다.
        self._model = whisper.load_model(
            self.config.model_size,
            device=self._device,
        )
        return self._model

    @staticmethod
    def _resolve_device(device: str) -> str:
        """`auto`를 실제 실행 장치명으로 변환한다."""

        if device != "auto":
            # 사용자가 cuda/cpu를 명시한 경우에는 그대로 따른다.
            return device
        try:
            import torch
        except ImportError:
            # torch가 없으면 CUDA 여부를 확인할 방법이 없으므로 CPU로 고정한다.
            return "cpu"
        return "cuda" if torch.cuda.is_available() else "cpu"
