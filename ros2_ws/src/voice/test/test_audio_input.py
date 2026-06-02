from __future__ import annotations

import numpy as np

from voice.audio_input import MicRecorder, MicRecorderConfig


class FakeStream:
    def __init__(self, chunks: list[np.ndarray]) -> None:
        self._chunks = list(chunks)

    def read(self, _chunk_size: int, exception_on_overflow: bool = False) -> bytes:
        del exception_on_overflow
        return self._chunks.pop(0).astype(np.float32).tobytes()


class FakeSpeechDetector:
    def __init__(self, result: bool) -> None:
        self.result = result
        self.calls = 0

    def has_speech(self, _audio: np.ndarray, _sample_rate_hz: int) -> bool:
        self.calls += 1
        return self.result


def build_recorder(detector: FakeSpeechDetector) -> MicRecorder:
    recorder = MicRecorder.__new__(MicRecorder)
    recorder.config = MicRecorderConfig(
        sample_rate_hz=16,
        chunk_size=4,
        max_duration_s=1.0,
        silence_threshold=0.1,
        trailing_silence_chunks=1,
        pre_roll_chunks=1,
        min_speech_chunks=2,
        enable_vad=True,
    )
    recorder._speech_detector = detector
    recorder._stream = FakeStream(
        [
            np.zeros(4, dtype=np.float32),
            np.full(4, 0.2, dtype=np.float32),
            np.full(4, 0.2, dtype=np.float32),
            np.zeros(4, dtype=np.float32),
        ]
    )
    return recorder


def test_record_utterance_returns_empty_when_vad_rejects_audio() -> None:
    detector = FakeSpeechDetector(False)
    recorder = build_recorder(detector)

    audio = recorder.record_utterance()

    assert audio.size == 0
    assert detector.calls == 1


def test_record_utterance_returns_audio_when_vad_accepts_audio() -> None:
    detector = FakeSpeechDetector(True)
    recorder = build_recorder(detector)

    audio = recorder.record_utterance()

    assert audio.size > 0
    assert detector.calls == 1


class FakePyAudio:
    def get_device_count(self) -> int:
        return 3

    def get_device_info_by_index(self, index: int) -> dict[str, object]:
        devices = [
            {"name": "HDMI Output", "maxInputChannels": 0},
            {"name": "pulse", "maxInputChannels": 32},
            {"name": "USB Microphone", "maxInputChannels": 1},
        ]
        return devices[index]


def test_resolve_input_device_by_name() -> None:
    config = MicRecorderConfig(input_device_name="usb")

    index = MicRecorder._resolve_input_device_index(FakePyAudio(), config)

    assert index == 2
