"""마이크 발화 구간을 녹음하는 helper.

이 파일은 ROS2를 직접 알지 않는다. 역할은 PyAudio 기본 입력 장치에서
16 kHz, mono, float32 오디오를 읽고, 사람이 말한 것으로 보이는 구간만
하나의 numpy 배열로 반환하는 것이다.

VAD 모드:
  use_silero_vad=False (기본): 단순 RMS 임계값
  use_silero_vad=True:         Silero VAD 신경망 — 사람 목소리만 감지, 소음 무시
                                PyTorch가 설치된 경우 사용 가능하며 CPU에서 실행한다.
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)

# Whisper는 내부적으로 16 kHz mono 오디오를 기준으로 동작한다.
SAMPLE_RATE_HZ = 16_000

# PyAudio에서 한 번에 읽는 프레임 수. 16 kHz 기준 약 64 ms.
# Silero VAD는 16kHz에서 512 또는 1024 샘플을 허용하므로 그대로 호환된다.
CHUNK_SIZE = 1024


class AudioInputError(RuntimeError):
    """마이크 장치 또는 PyAudio 초기화 실패 시 발생한다."""


@dataclass(frozen=True)
class MicRecorderConfig:
    """마이크 입력과 발화 감지 설정.

    Attributes:
        sample_rate_hz: 녹음 샘플레이트. 16 kHz 고정.
        chunk_size: 한 번에 읽을 오디오 프레임 수.
        max_duration_s: 한 발화로 녹음할 최대 시간.
        silence_threshold: RMS VAD 모드에서 사용하는 에너지 임계값.
        trailing_silence_chunks: 발화 후 이 chunk 수만큼 조용하면 종료.
        pre_roll_chunks: 발화 시작 직전 chunk를 함께 포함해 첫 음절 잘림을 줄인다.
        use_silero_vad: True면 RMS 대신 Silero VAD 신경망을 사용한다.
        silero_vad_threshold: Silero 음성 확률 임계값 (0.0~1.0).
    """

    sample_rate_hz: int = SAMPLE_RATE_HZ
    chunk_size: int = CHUNK_SIZE
    max_duration_s: float = 4.0
    silence_threshold: float = 0.02
    trailing_silence_chunks: int = 15
    pre_roll_chunks: int = 5
    use_silero_vad: bool = False
    silero_vad_threshold: float = 0.5


def _load_silero_vad():
    """Silero VAD 모델을 로드한다. 실패하면 None을 반환한다."""
    try:
        from silero_vad import load_silero_vad
        model = load_silero_vad()
        model.eval()
        logger.info("[audio_input] Silero VAD 로드 완료 (CPU)")
        return model
    except Exception as exc:
        logger.warning(f"[audio_input] Silero VAD 로드 실패, RMS VAD로 fallback: {exc}")
        return None


class MicRecorder:
    """PyAudio 기본 입력 장치에서 발화 한 번을 녹음한다.

    `record_utterance()`는 음성이 감지될 때까지 chunk를 읽고, 발화가 끝났다고
    판단되면 지금까지 모은 chunk를 하나의 numpy 배열로 이어 붙여 반환한다.
    """

    def __init__(self, config: MicRecorderConfig | None = None) -> None:
        self.config = config or MicRecorderConfig()
        try:
            import pyaudio
        except ImportError as exc:
            raise AudioInputError(
                "pyaudio is required for microphone input"
            ) from exc

        self._pyaudio = pyaudio
        self._pa = pyaudio.PyAudio()
        self._stream = self._pa.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=self.config.sample_rate_hz,
            input=True,
            frames_per_buffer=self.config.chunk_size,
        )

        self._silero: object | None = None
        if self.config.use_silero_vad:
            self._silero = _load_silero_vad()
            if self._silero is None:
                logger.warning(
                    "[audio_input] Silero 로드 실패 — RMS VAD로 동작합니다"
                )

    def _is_speech(self, chunk: np.ndarray) -> bool:
        """청크가 음성인지 판단한다. Silero 미사용 시 RMS로 fallback.

        Silero VAD는 16kHz에서 정확히 512 샘플을 요구한다.
        CHUNK_SIZE(1024)를 512 단위로 나눠 각 sub-chunk의 최대 확률로 판단한다.
        """
        if self._silero is not None:
            import torch
            _SILERO_STEP = 512
            probs: list[float] = []
            for i in range(0, len(chunk) - _SILERO_STEP + 1, _SILERO_STEP):
                sub = chunk[i : i + _SILERO_STEP].copy()  # read-only 방지
                with torch.no_grad():
                    p = self._silero(
                        torch.from_numpy(sub).unsqueeze(0),
                        self.config.sample_rate_hz,
                    ).item()
                probs.append(p)
            return max(probs, default=0.0) > self.config.silero_vad_threshold
        rms = float(np.sqrt(np.mean(chunk ** 2))) if chunk.size else 0.0
        return rms > self.config.silence_threshold

    def record_utterance(self) -> np.ndarray:
        """음성이 끝나거나 max_duration_s에 도달할 때까지 한 발화를 녹음한다.

        Returns:
            말한 구간의 float32 오디오 배열. 음성을 감지하지 못하면 빈 배열.
        """
        # Silero는 내부 LSTM 상태가 있으므로 발화마다 초기화한다.
        if self._silero is not None:
            self._silero.reset_states()

        chunks: list[np.ndarray] = []
        pre_roll: deque[np.ndarray] = deque(maxlen=self.config.pre_roll_chunks)
        silence_count = 0
        speaking = False
        max_chunks = int(
            self.config.sample_rate_hz
            / self.config.chunk_size
            * self.config.max_duration_s
        )

        for _ in range(max_chunks):
            try:
                raw = self._stream.read(
                    self.config.chunk_size,
                    exception_on_overflow=False,
                )
            except OSError:
                # 추론 중 마이크 버퍼 overflow → 스트림 재오픈 후 빈 배열 반환
                logger.warning("[audio_input] 스트림 overflow — 재오픈")
                self._reopen_stream()
                return np.zeros(0, dtype=np.float32)
            chunk = np.frombuffer(raw, dtype=np.float32)

            if self._is_speech(chunk):
                if not speaking:
                    chunks.extend(pre_roll)
                    pre_roll.clear()
                speaking = True
                silence_count = 0
                chunks.append(chunk)
            elif speaking:
                chunks.append(chunk)
                silence_count += 1
                if silence_count > self.config.trailing_silence_chunks:
                    break
            else:
                pre_roll.append(chunk)

        if not chunks:
            return np.zeros(0, dtype=np.float32)
        return np.concatenate(chunks)

    def _reopen_stream(self) -> None:
        """PyAudio 스트림을 닫고 다시 연다. overflow 복구용."""
        try:
            self._stream.stop_stream()
            self._stream.close()
        except Exception:
            pass
        self._stream = self._pa.open(
            format=self._pyaudio.paFloat32,
            channels=1,
            rate=self.config.sample_rate_hz,
            input=True,
            frames_per_buffer=self.config.chunk_size,
        )
        logger.info("[audio_input] 스트림 재오픈 완료")

    def close(self) -> None:
        """PyAudio stream과 handle을 정리한다."""
        self._stream.stop_stream()
        self._stream.close()
        self._pa.terminate()
