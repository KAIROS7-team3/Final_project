"""Whisper STT에 넘길 마이크 발화 구간을 녹음하는 helper.

이 파일은 ROS2를 직접 알지 않는다. 역할은 PyAudio 기본 입력 장치에서
16 kHz, mono, float32 오디오를 읽고, 사람이 말한 것으로 보이는 구간만
하나의 numpy 배열로 반환하는 것이다.

현재 발화 감지는 단순 RMS 임계값과 pre-roll 방식이다. 주변 소음이 크거나
마이크 입력이 작으면 오인식이 생길 수 있으므로, 추후 VAD로 교체할 수 있다.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

# Whisper는 내부적으로 16 kHz mono 오디오를 기준으로 동작한다.
SAMPLE_RATE_HZ = 16_000

# PyAudio에서 한 번에 읽는 프레임 수다. 16 kHz 기준 약 64 ms 단위다.
CHUNK_SIZE = 1024


class AudioInputError(RuntimeError):
    """마이크 장치 또는 PyAudio 초기화 실패 시 발생한다."""


@dataclass(frozen=True)
class MicRecorderConfig:
    """마이크 입력과 RMS 기반 발화 감지 설정.

    Attributes:
        sample_rate_hz: 녹음 샘플레이트. Whisper 입력과 맞추기 위해 16 kHz 사용.
        chunk_size: 한 번에 읽을 오디오 프레임 수.
        max_duration_s: 한 발화로 녹음할 최대 시간. 무한 대기를 막는다.
        silence_threshold: RMS가 이 값보다 크면 말하는 중으로 판단한다.
        trailing_silence_chunks: 말한 뒤 이 chunk 수만큼 조용하면 발화를 종료한다.
        pre_roll_chunks: 발화 시작 직전 chunk를 함께 넣어 첫 음절 잘림을 줄인다.
    """

    sample_rate_hz: int = SAMPLE_RATE_HZ
    chunk_size: int = CHUNK_SIZE
    max_duration_s: float = 4.0
    silence_threshold: float = 0.02
    trailing_silence_chunks: int = 15
    pre_roll_chunks: int = 5


class MicRecorder:
    """PyAudio 기본 입력 장치에서 발화 한 번을 녹음한다.

    `record_utterance()`는 음성이 감지될 때까지 chunk를 읽고, 발화가 끝났다고
    판단되면 지금까지 모은 chunk를 하나의 numpy 배열로 이어 붙여 반환한다.
    """

    def __init__(self, config: MicRecorderConfig | None = None) -> None:
        self.config = config or MicRecorderConfig()
        try:
            # PyAudio는 시스템 오디오 장치 접근이 필요하므로 import 실패를
            # 명확한 AudioInputError로 바꿔 상위 노드가 STT를 비활성화하게 한다.
            import pyaudio
        except ImportError as exc:
            raise AudioInputError(
                "pyaudio is required for microphone input"
            ) from exc

        self._pyaudio = pyaudio
        self._pa = pyaudio.PyAudio()
        # paFloat32를 사용해 WhisperTranscriber가 바로 float32 numpy 배열을
        # 넘겨받을 수 있게 한다. channels=1은 mono 입력을 의미한다.
        self._stream = self._pa.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=self.config.sample_rate_hz,
            input=True,
            frames_per_buffer=self.config.chunk_size,
        )

    def record_utterance(self) -> np.ndarray:
        """음성이 끝나거나 max_duration_s에 도달할 때까지 한 발화를 녹음한다.

        Returns:
            말한 구간의 float32 오디오 배열. 음성을 감지하지 못하면 빈 배열을
            반환한다.
        """

        chunks: list[np.ndarray] = []
        pre_roll: deque[np.ndarray] = deque(maxlen=self.config.pre_roll_chunks)
        silence_count = 0
        speaking = False
        # 최대 반복 횟수를 chunk 기준으로 계산한다.
        # 예: 16000 / 1024 * 4초 ~= 62회.
        max_chunks = int(
            self.config.sample_rate_hz
            / self.config.chunk_size
            * self.config.max_duration_s
        )

        for _ in range(max_chunks):
            raw = self._stream.read(
                self.config.chunk_size,
                exception_on_overflow=False,
            )
            chunk = np.frombuffer(raw, dtype=np.float32)
            rms = float(np.sqrt(np.mean(chunk**2))) if chunk.size else 0.0

            # 현재는 단순 RMS gate다. 추후 소음 환경에서는 VAD로 교체할 수 있다.
            if rms > self.config.silence_threshold:
                # 처음 threshold를 넘는 순간부터 발화가 시작됐다고 본다.
                if not speaking:
                    chunks.extend(pre_roll)
                    pre_roll.clear()
                speaking = True
                silence_count = 0
                chunks.append(chunk)
            elif speaking:
                # 이미 말하기가 시작된 뒤의 무음은 trailing silence로 포함한다.
                # Whisper가 문장 끝을 더 안정적으로 판단하게 하기 위함이다.
                chunks.append(chunk)
                silence_count += 1
                if silence_count > self.config.trailing_silence_chunks:
                    break
            else:
                pre_roll.append(chunk)

        if not chunks:
            return np.zeros(0, dtype=np.float32)
        return np.concatenate(chunks)

    def close(self) -> None:
        """PyAudio stream과 handle을 정리한다."""

        self._stream.stop_stream()
        self._stream.close()
        self._pa.terminate()
