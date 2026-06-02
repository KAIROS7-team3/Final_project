from __future__ import annotations

import os
from pathlib import Path

import numpy as np
import pytest

from voice.command_parser import parse_command
from voice.transcriber import WhisperConfig, WhisperLoadError, WhisperTranscriber


def test_whisper_sample_files_match_expected_intents() -> None:
    sample_dir = Path(os.environ.get("VOICE_SAMPLE_DIR", "test/audio_samples"))
    manifest_path = sample_dir / "manifest.tsv"
    if not manifest_path.exists():
        pytest.skip("voice sample manifest not available")

    transcriber = WhisperTranscriber(WhisperConfig(model_size="small"))
    failures: list[str] = []
    for line in manifest_path.read_text(encoding="utf-8").splitlines():
        if not line.strip() or line.startswith("#"):
            continue
        wav_path, expected_intent, expected_tool_id = line.split("\t")
        try:
            text = transcriber.transcribe(_load_wav_float32(sample_dir / wav_path))
        except WhisperLoadError as exc:
            pytest.skip(str(exc))
        parsed = parse_command(text)
        if (
            parsed.intent_type != expected_intent
            or parsed.tool_id != expected_tool_id
        ):
            failures.append(
                f"{wav_path}: expected {expected_intent}/{expected_tool_id}, "
                f"got {parsed.intent_type}/{parsed.tool_id} from {text!r}"
            )

    assert not failures


def _load_wav_float32(path: Path) -> np.ndarray:
    import wave

    with wave.open(str(path), "rb") as wav_file:
        if wav_file.getframerate() != 16_000:
            raise ValueError(f"{path} must be 16 kHz")
        if wav_file.getnchannels() != 1:
            raise ValueError(f"{path} must be mono")
        frames = wav_file.readframes(wav_file.getnframes())
        sample_width = wav_file.getsampwidth()

    if sample_width == 2:
        return np.frombuffer(frames, dtype=np.int16).astype(np.float32) / 32768.0
    if sample_width == 4:
        return np.frombuffer(frames, dtype=np.float32)
    raise ValueError(f"{path} must use 16-bit PCM or float32 samples")
