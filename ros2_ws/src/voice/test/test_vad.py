from __future__ import annotations

import numpy as np
import pytest

from voice.audio_input import SAMPLE_RATE_HZ
from voice.vad import SileroVadConfig, SileroVadDetector


class FakeSileroModel:
    pass


def test_silero_vad_returns_true_when_timestamps_exist() -> None:
    calls = []

    def fake_get_speech_timestamps(audio, model, **kwargs):
        calls.append((audio, model, kwargs))
        return [{"start": 0, "end": 1600}]

    detector = SileroVadDetector(
        SileroVadConfig(threshold=0.6),
        model=FakeSileroModel(),
        get_speech_timestamps=fake_get_speech_timestamps,
    )

    assert detector.has_speech(np.ones(1600, dtype=np.float32), SAMPLE_RATE_HZ)
    assert calls[0][2]["threshold"] == 0.6
    assert calls[0][2]["sampling_rate"] == SAMPLE_RATE_HZ


def test_silero_vad_returns_false_for_empty_audio() -> None:
    detector = SileroVadDetector(
        model=FakeSileroModel(),
        get_speech_timestamps=lambda *_args, **_kwargs: [{"start": 0, "end": 1}],
    )

    assert detector.has_speech(np.array([], dtype=np.float32), SAMPLE_RATE_HZ) is False


def test_silero_vad_rejects_unsupported_sample_rate() -> None:
    detector = SileroVadDetector(
        model=FakeSileroModel(),
        get_speech_timestamps=lambda *_args, **_kwargs: [],
    )

    with pytest.raises(ValueError):
        detector.has_speech(np.ones(1600, dtype=np.float32), 44_100)
