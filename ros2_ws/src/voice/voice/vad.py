"""마이크 입력용 Voice Activity Detection helper.

Silero VAD는 선택 의존성이다. 단위 테스트와 마이크 없는 개발 환경이 깨지지 않게
기본 import 시점에는 로드하지 않고, ROS parameter로 VAD가 켜진 런타임에서만
detector 객체를 만든다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Protocol

import numpy as np


class VadLoadError(RuntimeError):
    """설정된 VAD backend를 로드할 수 없을 때 발생한다."""


class SpeechDetector(Protocol):
    """MicRecorder가 비음성 오디오를 버릴 때 사용하는 최소 인터페이스."""

    def has_speech(self, audio: np.ndarray, sample_rate_hz: int) -> bool:
        ...


@dataclass(frozen=True)
class SileroVadConfig:
    """Silero speech timestamp detector에 전달할 설정값."""

    threshold: float = 0.5
    min_speech_duration_ms: int = 250
    min_silence_duration_ms: int = 100
    speech_pad_ms: int = 100


class SileroVadDetector:
    """Silero VAD를 lazy-load하는 wrapper.

    테스트에서는 `model`과 `get_speech_timestamps`를 fake로 주입할 수 있고,
    실제 실행에서는 `silero_vad` 패키지를 여기서 처음 import한다.
    """

    def __init__(
        self,
        config: SileroVadConfig | None = None,
        model: object | None = None,
        get_speech_timestamps=None,
    ) -> None:
        self.config = config or SileroVadConfig()
        if model is not None and get_speech_timestamps is not None:
            self._model = model
            self._get_speech_timestamps = get_speech_timestamps
            return

        try:
            from silero_vad import get_speech_timestamps, load_silero_vad
        except ImportError as exc:
            raise VadLoadError(
                "silero-vad is required when enable_vad=true"
            ) from exc

        self._model = load_silero_vad()
        self._get_speech_timestamps = get_speech_timestamps

    def has_speech(self, audio: np.ndarray, sample_rate_hz: int) -> bool:
        """Silero가 하나 이상의 speech segment를 찾으면 True를 반환한다."""

        if sample_rate_hz not in {8_000, 16_000}:
            raise ValueError("Silero VAD supports 8000 Hz or 16000 Hz audio")

        audio_np = np.asarray(audio, dtype=np.float32)
        if audio_np.ndim != 1:
            raise ValueError("Silero VAD expects mono 1-D audio")
        if audio_np.size == 0:
            return False

        # Silero는 speech timestamp list를 반환한다. 길이가 0이면 RMS threshold를
        # 통과한 소리라도 사람 음성으로 보지 않고 downstream Whisper 호출을 막는다.
        timestamps = self._get_speech_timestamps(
            audio_np,
            self._model,
            sampling_rate=sample_rate_hz,
            threshold=self.config.threshold,
            min_speech_duration_ms=self.config.min_speech_duration_ms,
            min_silence_duration_ms=self.config.min_silence_duration_ms,
            speech_pad_ms=self.config.speech_pad_ms,
        )
        return bool(timestamps)
