"""로컬 Whisper 회귀 테스트용 음성 샘플 녹음 CLI.

현장 마이크와 사용자의 실제 발화를 기준으로 STT 품질을 확인하려면 작은 wav 샘플을
축적해야 한다. 이 helper는 한 발화를 16 kHz mono wav로 저장하고, 선택적으로
`manifest.tsv`에 기대 intent/tool_id를 append한다.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import wave

import numpy as np

from voice.audio_input import (
    SAMPLE_RATE_HZ,
    AudioInputError,
    MicRecorder,
    MicRecorderConfig,
)


def write_wav(path: Path, audio: np.ndarray, sample_rate_hz: int) -> None:
    """mono float32 오디오를 표준 16-bit PCM WAV로 저장한다."""

    path.parent.mkdir(parents=True, exist_ok=True)
    # MicRecorder는 float32 [-1, 1] 배열을 반환한다. WAV fixture는 일반 도구에서
    # 재생하기 쉽도록 16-bit PCM으로 변환한다.
    clipped = np.clip(np.asarray(audio, dtype=np.float32), -1.0, 1.0)
    pcm = (clipped * 32767.0).astype(np.int16)

    with wave.open(str(path), "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(sample_rate_hz)
        wav_file.writeframes(pcm.tobytes())


def append_manifest(
    manifest_path: Path,
    wav_path: Path,
    expected_intent: str,
    expected_tool_id: str,
) -> None:
    """로컬 `manifest.tsv`에 샘플 한 줄을 추가한다."""

    manifest_path.parent.mkdir(parents=True, exist_ok=True)
    line = f"{wav_path.name}\t{expected_intent}\t{expected_tool_id}\n"
    with manifest_path.open("a", encoding="utf-8") as manifest_file:
        manifest_file.write(line)


def build_parser() -> argparse.ArgumentParser:
    """CLI 인자 parser를 만든다."""

    parser = argparse.ArgumentParser(
        description="Record one 16 kHz mono voice sample for Whisper tests."
    )
    parser.add_argument("output", type=Path, help="Output wav path.")
    parser.add_argument("--intent", choices=("fetch", "return", "cancel", "unknown"))
    parser.add_argument("--tool-id", default="", help="Expected project tool_id.")
    parser.add_argument(
        "--manifest",
        type=Path,
        help="Optional manifest.tsv path to append after recording.",
    )
    parser.add_argument("--max-duration", type=float, default=4.0)
    parser.add_argument("--silence-threshold", type=float, default=0.035)
    parser.add_argument("--min-speech-chunks", type=int, default=4)
    return parser


def main(argv: list[str] | None = None) -> None:
    """마이크에서 한 발화를 녹음해 wav와 optional manifest를 생성한다."""

    args = build_parser().parse_args(argv)

    if args.manifest is not None and args.intent is None:
        # manifest에는 정답 intent가 반드시 필요하다. 정답 없는 샘플은 회귀 테스트에서
        # pass/fail을 계산할 수 없다.
        raise SystemExit("--intent is required when --manifest is used")

    recorder: MicRecorder | None = None
    try:
        recorder = MicRecorder(
            MicRecorderConfig(
                max_duration_s=args.max_duration,
                silence_threshold=args.silence_threshold,
                min_speech_chunks=args.min_speech_chunks,
            )
        )
        audio = recorder.record_utterance()
    except AudioInputError as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if recorder is not None:
            recorder.close()

    if audio.size == 0:
        # 잡음만 들어온 파일을 fixture로 남기면 이후 STT 정확도 판단이 흐려진다.
        raise SystemExit("no speech detected; wav file was not written")

    write_wav(args.output, audio, SAMPLE_RATE_HZ)

    if args.manifest is not None:
        append_manifest(
            args.manifest,
            args.output,
            str(args.intent),
            args.tool_id,
        )
