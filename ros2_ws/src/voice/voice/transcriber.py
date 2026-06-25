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
    """Whisper 백엔드 로드 또는 초기화 실패 시 발생한다."""


class WhisperModel(Protocol):
    """테스트에서 fake Whisper 모델을 주입하기 위한 최소 protocol.

    openai-whisper / faster-whisper 전체 타입에 의존하지 않고,
    `transcribe()` 메서드만 요구한다.
    """

    def transcribe(self, audio: np.ndarray, **kwargs) -> dict[str, object]:
        ...


ModelLoader = Callable[[str, str], WhisperModel]


@dataclass(frozen=True)
class WhisperConfig:
    """Whisper 추론 설정.

    Attributes:
        backend: ``"faster"`` (faster-whisper, VAD 내장) 또는 ``"openai"``.
        model_size: tiny/base/small/medium/large 계열 모델명.
        device: ``auto``면 CUDA 가능 시 cuda, 아니면 cpu.
        language: 한국어 명령만 쓰므로 ``ko`` 고정.
        beam_size: 한국어 명령 후보 탐색 폭.
        best_of: temperature fallback 후보 수.
        initial_prompt: Whisper vocab 힌트 (비우면 hallucination 감소).
        temperature: 0.0으로 결정론적 출력.
        condition_on_previous_text: 단발 명령이므로 False 권장.
        no_speech_threshold: 이 확률 이상이면 무음으로 판정해 빈 문자열 반환.
        logprob_threshold: 낮은 log-prob 구간 필터 (음수 클수록 엄격).
        compression_ratio_threshold: 반복 환각 압축비 필터.
        vad_filter: faster-whisper 전용 — 무음 구간 hallucination 제거.
        vad_min_silence_ms: VAD 무음 판정 최소 시간(ms).
    """

    backend: str = "faster"
    model_size: str = "small"
    device: str = "auto"
    language: str = "ko"
    task: str = "transcribe"
    beam_size: int = 10
    best_of: int = 5
    temperature: float = 0.0
    no_speech_threshold: float = 0.6
    logprob_threshold: float = -1.0
    compression_ratio_threshold: float = 2.4
    initial_prompt: str = ""
    condition_on_previous_text: bool = False
    vad_filter: bool = True
    vad_min_silence_ms: int = 500


class _FasterWhisperWrapper:
    """faster-whisper를 WhisperModel protocol 형태로 감싸는 어댑터.

    - `vad_filter=True` 로 무음 구간 hallucination을 제거한다.
    - `logprob_threshold` → `log_prob_threshold` 이름 변환을 처리한다.
    - `fp16` 같은 openai-whisper 전용 kwarg를 조용히 무시한다.
    - 반환값은 openai-whisper 호환 dict({"text": str})이다.
    """

    # openai-whisper kwargs 중 faster-whisper에서 이름이 다른 것
    _RENAMED = {"logprob_threshold": "log_prob_threshold"}

    # openai-whisper 전용 kwargs (faster-whisper에 전달하면 TypeError)
    _IGNORED = frozenset({"fp16"})

    def __init__(self, model_size: str, device: str, config: WhisperConfig) -> None:
        try:
            from faster_whisper import WhisperModel  # type: ignore[import]
        except ImportError as exc:
            raise WhisperLoadError(
                "faster-whisper가 필요합니다: pip install faster-whisper"
            ) from exc
        compute_type = "float16" if device == "cuda" else "int8"
        self._fw_model = WhisperModel(
            model_size, device=device, compute_type=compute_type
        )
        self._config = config

    def transcribe(self, audio: np.ndarray, **kwargs) -> dict[str, object]:
        fw_kwargs: dict[str, object] = {}
        for key, value in kwargs.items():
            if key in self._IGNORED:
                continue
            fw_key = self._RENAMED.get(key, key)
            fw_kwargs[fw_key] = value

        # VAD 필터는 openai-whisper에 없는 faster-whisper 전용 기능이므로
        # config에서 직접 읽어 추가한다.
        fw_kwargs["vad_filter"] = self._config.vad_filter
        if self._config.vad_filter:
            fw_kwargs["vad_parameters"] = {
                "min_silence_duration_ms": self._config.vad_min_silence_ms
            }

        segments, _info = self._fw_model.transcribe(audio, **fw_kwargs)
        text = " ".join(seg.text.strip() for seg in segments)
        return {"text": text}


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
        kwargs: dict[str, object] = dict(
            language=self.config.language,
            task=self.config.task,
            beam_size=self.config.beam_size,
            best_of=self.config.best_of,
            temperature=self.config.temperature,
            initial_prompt=self.config.initial_prompt or None,
            condition_on_previous_text=self.config.condition_on_previous_text,
            no_speech_threshold=self.config.no_speech_threshold,
            logprob_threshold=self.config.logprob_threshold,
            compression_ratio_threshold=self.config.compression_ratio_threshold,
        )
        if self.config.backend == "openai":
            kwargs["fp16"] = self._device == "cuda"

        result = model.transcribe(np.ascontiguousarray(audio_np), **kwargs)
        return str(result.get("text", "")).strip()

    def _get_model(self) -> WhisperModel:
        """첫 STT 요청 시 Whisper 모델을 로드하고 이후에는 재사용한다."""

        if self._model is not None:
            return self._model

        if self._model_loader is not None:
            self._model = self._model_loader(self.config.model_size, self._device)
            return self._model

        if self.config.backend == "faster":
            self._model = _FasterWhisperWrapper(
                self.config.model_size, self._device, self.config
            )
        else:
            try:
                import whisper  # type: ignore[import]
            except ImportError as exc:
                raise WhisperLoadError(
                    "openai-whisper is required: pip install openai-whisper"
                ) from exc
            self._model = whisper.load_model(
                self.config.model_size, device=self._device
            )

        return self._model

    @staticmethod
    def _resolve_device(device: str) -> str:
        """`auto`를 실제 실행 장치명으로 변환한다."""

        if device != "auto":
            return device
        try:
            import torch  # type: ignore[import]
        except ImportError:
            return "cpu"
        return "cuda" if torch.cuda.is_available() else "cpu"
