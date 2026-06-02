"""Whisper STT에 넘길 마이크 발화 구간을 녹음하는 helper.

이 파일은 ROS2를 직접 알지 않는다. 역할은 PyAudio 기본 입력 장치에서
16 kHz, mono, float32 오디오를 읽고, 사람이 말한 것으로 보이는 구간만
하나의 numpy 배열로 반환하는 것이다.

발화 시작/종료는 RMS 임계값과 pre-roll 방식으로 빠르게 잡고, 선택적으로
VAD 라이브러리를 한 번 더 통과시켜 사람 음성이 아닌 잡음을 버린다.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

import numpy as np

from voice.vad import SpeechDetector

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
        min_speech_chunks: RMS 기준 발화 chunk가 이 값보다 적으면 잡음으로 버린다.
        enable_vad: True이면 녹음된 발화를 VAD로 한 번 더 검증한다.
        input_device_index: PyAudio 입력 장치 index. -1이면 기본 장치를 사용한다.
        input_device_name: 입력 장치 이름 일부. index보다 우선순위가 낮다.
    """

    sample_rate_hz: int = SAMPLE_RATE_HZ
    chunk_size: int = CHUNK_SIZE
    max_duration_s: float = 4.0
    silence_threshold: float = 0.02
    trailing_silence_chunks: int = 15
    pre_roll_chunks: int = 5
    min_speech_chunks: int = 4
    enable_vad: bool = False
    input_device_index: int = -1
    input_device_name: str = ""


class MicRecorder:
    """PyAudio 기본 입력 장치에서 발화 한 번을 녹음한다.

    `record_utterance()`는 음성이 감지될 때까지 chunk를 읽고, 발화가 끝났다고
    판단되면 지금까지 모은 chunk를 하나의 numpy 배열로 이어 붙여 반환한다.
    """

    def __init__(
        self,
        config: MicRecorderConfig | None = None,
        speech_detector: SpeechDetector | None = None,
    ) -> None:
        self.config = config or MicRecorderConfig()
        self._speech_detector = speech_detector
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
        input_device_index = self._resolve_input_device_index(
            self._pa,
            self.config,
        )
        self._stream = self._pa.open(
            format=pyaudio.paFloat32,
            channels=1,
            rate=self.config.sample_rate_hz,
            input=True,
            input_device_index=input_device_index,
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
        speech_chunk_count = 0
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

            # RMS gate로 후보 발화 구간을 만들고, VAD는 아래에서 한 번 더 확인한다.
            if rms > self.config.silence_threshold:
                # 처음 threshold를 넘는 순간부터 발화가 시작됐다고 본다.
                if not speaking:
                    chunks.extend(pre_roll)
                    pre_roll.clear()
                speaking = True
                silence_count = 0
                speech_chunk_count += 1
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
        if speech_chunk_count < self.config.min_speech_chunks:
            return np.zeros(0, dtype=np.float32)

        audio = np.concatenate(chunks)
        if self.config.enable_vad:
            if self._speech_detector is None:
                return np.zeros(0, dtype=np.float32)
            if not self._speech_detector.has_speech(
                audio,
                self.config.sample_rate_hz,
            ):
                return np.zeros(0, dtype=np.float32)
        return audio

    def close(self) -> None:
        """PyAudio stream과 handle을 정리한다."""

        self._stream.stop_stream()
        self._stream.close()
        self._pa.terminate()

    @staticmethod
    def _resolve_input_device_index(pa, config: MicRecorderConfig) -> int | None:
        """설정된 PyAudio 입력 장치를 index로 해석한다."""

        if config.input_device_index >= 0:
            # 숫자 index가 명시되면 이름 검색보다 우선한다. `list_audio_devices`로
            # 확인한 정확한 장치를 지정할 때 사용한다.
            return config.input_device_index

        requested_name = config.input_device_name.strip().casefold()
        if not requested_name:
            # None을 넘기면 PyAudio가 시스템 기본 입력 장치를 사용한다.
            return None

        available_inputs: list[str] = []
        for index in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(index)
            if int(info.get("maxInputChannels", 0)) <= 0:
                continue
            name = str(info.get("name", ""))
            available_inputs.append(f"{index}:{name}")
            if requested_name in name.casefold():
                # 이름 전체가 아니라 일부만 맞아도 선택한다. 예: "USB", "pulse".
                return index

        available = ", ".join(available_inputs) or "none"
        raise AudioInputError(
            f"input_device_name={config.input_device_name!r} was not found; "
            f"available input devices: {available}"
        )


def list_audio_devices() -> list[str]:
    """PyAudio 입력 장치 목록을 사람이 읽기 쉬운 문자열로 반환한다."""

    try:
        import pyaudio
    except ImportError as exc:
        raise AudioInputError(
            "pyaudio is required to list microphone devices"
        ) from exc

    pa = pyaudio.PyAudio()
    try:
        devices: list[str] = []
        for index in range(pa.get_device_count()):
            info = pa.get_device_info_by_index(index)
            if int(info.get("maxInputChannels", 0)) <= 0:
                continue
            devices.append(
                f"{index}: {info.get('name')} "
                f"(inputs={info.get('maxInputChannels')}, "
                f"rate={info.get('defaultSampleRate')})"
            )
        return devices
    finally:
        pa.terminate()


def list_audio_devices_main() -> None:
    """Console entry point for checking PyAudio input device names."""

    for device in list_audio_devices():
        print(device)
