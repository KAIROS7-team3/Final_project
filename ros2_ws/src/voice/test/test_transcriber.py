from __future__ import annotations

import numpy as np
import pytest

from voice.audio_input import SAMPLE_RATE_HZ
from voice.transcriber import WhisperConfig, WhisperTranscriber


class FakeWhisperModel:
    def __init__(self) -> None:
        self.kwargs = None

    def transcribe(self, audio: np.ndarray, **kwargs) -> dict[str, object]:
        self.kwargs = kwargs
        return {"text": " 스패너 가져다 줘 "}


def test_transcriber_uses_korean_deterministic_settings() -> None:
    fake_model = FakeWhisperModel()
    transcriber = WhisperTranscriber(
        WhisperConfig(model_size="small", device="cpu"),
        model_loader=lambda _model_size, _device: fake_model,
    )

    text = transcriber.transcribe(np.ones(SAMPLE_RATE_HZ, dtype=np.float32))

    assert text == "스패너 가져다 줘"
    assert fake_model.kwargs["language"] == "ko"
    assert fake_model.kwargs["task"] == "transcribe"
    assert fake_model.kwargs["beam_size"] == 10
    assert fake_model.kwargs["best_of"] == 5
    assert "스패너 16mm" in fake_model.kwargs["initial_prompt"]
    assert fake_model.kwargs["temperature"] == 0.0
    assert fake_model.kwargs["condition_on_previous_text"] is False
    assert fake_model.kwargs["no_speech_threshold"] == 0.6
    assert fake_model.kwargs["logprob_threshold"] == -1.0
    assert fake_model.kwargs["compression_ratio_threshold"] == 2.4
    assert fake_model.kwargs["fp16"] is False


def test_transcriber_rejects_non_whisper_sample_rate() -> None:
    transcriber = WhisperTranscriber(
        WhisperConfig(device="cpu"),
        model_loader=lambda _model_size, _device: FakeWhisperModel(),
    )

    with pytest.raises(ValueError):
        transcriber.transcribe(
            np.ones(100, dtype=np.float32),
            sample_rate_hz=8000,
        )


def test_transcriber_rejects_multi_channel_audio() -> None:
    transcriber = WhisperTranscriber(
        WhisperConfig(device="cpu"),
        model_loader=lambda _model_size, _device: FakeWhisperModel(),
    )

    with pytest.raises(ValueError):
        transcriber.transcribe(np.ones((100, 2), dtype=np.float32))
