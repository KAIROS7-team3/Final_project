from __future__ import annotations

from pathlib import Path
import wave

import numpy as np

from voice.sample_recorder import append_manifest, write_wav


def test_write_wav_creates_16khz_mono_pcm_file(tmp_path: Path) -> None:
    wav_path = tmp_path / "spanner_fetch_01.wav"

    write_wav(
        wav_path,
        np.array([-2.0, -0.5, 0.0, 0.5, 2.0], dtype=np.float32),
        16_000,
    )

    with wave.open(str(wav_path), "rb") as wav_file:
        assert wav_file.getnchannels() == 1
        assert wav_file.getsampwidth() == 2
        assert wav_file.getframerate() == 16_000
        assert wav_file.getnframes() == 5


def test_append_manifest_writes_expected_sample_row(tmp_path: Path) -> None:
    manifest_path = tmp_path / "manifest.tsv"

    append_manifest(
        manifest_path,
        tmp_path / "socket_return_01.wav",
        "return",
        "socket_19mm",
    )

    assert manifest_path.read_text(encoding="utf-8") == (
        "socket_return_01.wav\treturn\tsocket_19mm\n"
    )
