"""Gemma 4 E2B 오디오 직접 의도 분류기.

Whisper STT 없이 마이크 오디오를 Gemma 4 E2B에 직접 넣어
fetch/return/cancel/unknown 의도와 tool_id를 한 번에 출력한다.

파이프라인:
    numpy float32 audio (16kHz)
    -> Gemma4AudioFeatureExtractor (mel spectrogram)
    -> Gemma4ForConditionalGeneration (audio + text prompt)
    -> JSON 파싱 -> GemmaIntentResult

ROS2 의존성 없음 — 단위 테스트에서 fake backend 주입 가능.
"""

from __future__ import annotations

import json
import re
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from voice.gemma_intent import (
    GemmaIntentResult,
    GemmaParseError,
    ToolSpec,
    _build_tool_alias_map,
    _normalize_tool_key,
    _tool_catalog_lines,
    load_tool_catalog,
)
from voice.audio_input import SAMPLE_RATE_HZ

_PROMPT_PATH = Path(__file__).with_name("gemma4_audio_prompt.txt")

_FAIL_CLOSED = GemmaIntentResult(
    intent_type="unknown",
    tool_id="",
    confidence=0.0,
    needs_confirm=False,
    raw_output="",
)


class Gemma4AudioLoadError(RuntimeError):
    """Gemma 4 모델 로드 실패 시 발생한다."""


@dataclass(frozen=True)
class Gemma4AudioConfig:
    """Gemma 4 오디오 분류기 설정.

    Attributes:
        model_id: 로컬 모델 경로 또는 HuggingFace repo ID.
        device: ``auto`` / ``cuda`` / ``cpu``.
        confidence_threshold: 이 값 미만이면 결과를 unknown으로 처리.
        max_new_tokens: 생성 최대 토큰 수 (JSON 응답이므로 96면 충분).
        warmup: 로드 직후 묵음으로 워밍업 추론 실행 여부.
        toolbox_path: 공구 카탈로그 YAML 경로 (기본값: 패키지 내 toolbox.yaml).
    """

    model_id: str = "~/models/gemma/gemma-4-e2b-it"
    device: str = "auto"
    confidence_threshold: float = 0.85
    max_new_tokens: int = 96
    warmup: bool = True
    toolbox_path: str = ""


def load_audio_prompt_template() -> str:
    """gemma4_audio_prompt.txt를 읽는다."""
    return _PROMPT_PATH.read_text(encoding="utf-8")


def _build_audio_prompt(catalog: tuple[ToolSpec, ...]) -> str:
    """공구 카탈로그를 채운 텍스트 프롬프트를 반환한다."""
    tmpl = load_audio_prompt_template()
    return tmpl.replace("__TOOL_CATALOG__", _tool_catalog_lines(catalog))


def _parse_output(raw: str, tool_alias_to_id: dict[str, str]) -> GemmaIntentResult:
    """Gemma 출력에서 JSON을 추출해 GemmaIntentResult로 변환한다."""
    cleaned = raw.strip()
    m = re.search(r"```(?:json)?\s*(\{.*?\})\s*```", cleaned, re.DOTALL)
    if m:
        cleaned = m.group(1)
    else:
        m = re.search(r"\{.*\}", cleaned, re.DOTALL)
        if m:
            cleaned = m.group(0)

    try:
        data = json.loads(cleaned)
    except json.JSONDecodeError as exc:
        raise GemmaParseError(f"JSON 파싱 실패: {exc} / raw={raw!r}") from exc

    intent_type = str(data.get("intent_type", "unknown")).lower()
    if intent_type not in {"fetch", "return", "cancel", "unknown"}:
        raise GemmaParseError(f"알 수 없는 intent_type: {intent_type!r}")

    raw_tool = str(data.get("tool_id", "")).strip()
    normalized = _normalize_tool_key(raw_tool)
    tool_id = tool_alias_to_id.get(normalized, "")
    if not tool_id:
        # canonical ID 직접 입력도 허용
        tool_id = raw_tool if raw_tool in tool_alias_to_id.values() else ""

    if intent_type in {"fetch", "return"} and not tool_id:
        raise GemmaParseError(f"fetch/return에 tool_id 없음 (raw={raw_tool!r})")

    confidence = float(data.get("confidence", 0.0))
    needs_confirm = bool(data.get("needs_confirm", False))

    return GemmaIntentResult(
        intent_type=intent_type,
        tool_id=tool_id,
        confidence=confidence,
        needs_confirm=needs_confirm,
        raw_output=cleaned,
    )


class _Gemma4AudioBackend:
    """transformers Gemma4 오디오 추론 백엔드."""

    def __init__(self, config: Gemma4AudioConfig) -> None:
        self._config = config
        self._proc = None
        self._model = None

    def _load(self) -> None:
        try:
            import torch
            from transformers import AutoProcessor, Gemma4ForConditionalGeneration
        except ImportError as exc:
            raise Gemma4AudioLoadError("transformers / torch가 필요합니다.") from exc

        model_id = str(Path(self._config.model_id).expanduser())
        self._proc = AutoProcessor.from_pretrained(model_id, local_files_only=True)
        self._model = Gemma4ForConditionalGeneration.from_pretrained(
            model_id,
            local_files_only=True,
            dtype=torch.bfloat16,
            device_map=self._config.device,
        )

    def generate(self, audio: np.ndarray, text_prompt: str) -> str:
        """오디오 + 텍스트 프롬프트를 넣어 모델 출력 문자열을 반환한다."""
        import torch

        if self._model is None:
            self._load()

        messages = [
            {
                "role": "user",
                "content": [
                    {"type": "audio", "audio": audio.astype(np.float32)},
                    {"type": "text", "text": text_prompt},
                ],
            }
        ]

        inputs = self._proc.apply_chat_template(
            messages,
            add_generation_prompt=True,
            tokenize=True,
            return_dict=True,
            return_tensors="pt",
        ).to(self._model.device, dtype=torch.bfloat16)

        with torch.inference_mode():
            output_ids = self._model.generate(
                **inputs,
                max_new_tokens=self._config.max_new_tokens,
                do_sample=False,
            )

        new_tokens = output_ids[0][inputs["input_ids"].shape[-1]:]
        return self._proc.decode(new_tokens, skip_special_tokens=True).strip()


class Gemma4AudioClassifier:
    """마이크 오디오를 받아 공구 fetch/return 의도를 분류한다.

    STT 없이 Gemma 4 E2B 오디오 인코더를 직접 활용한다.
    """

    def __init__(
        self,
        config: Gemma4AudioConfig | None = None,
        backend: _Gemma4AudioBackend | None = None,
    ) -> None:
        self._config = config or Gemma4AudioConfig()
        self._backend = backend or _Gemma4AudioBackend(self._config)

        self._catalog = load_tool_catalog(
            Path(self._config.toolbox_path) if self._config.toolbox_path else None
        )
        self._tool_alias_to_id = _build_tool_alias_map(self._catalog)
        self._text_prompt = _build_audio_prompt(self._catalog)

        if self._config.warmup:
            self._warmup()

    def _warmup(self) -> None:
        silent = np.zeros(SAMPLE_RATE_HZ, dtype=np.float32)
        try:
            self._backend.generate(silent, self._text_prompt)
        except Exception:
            pass

    def classify(self, audio: np.ndarray) -> GemmaIntentResult:
        """16kHz float32 오디오를 받아 의도 분류 결과를 반환한다."""
        import logging
        _log = logging.getLogger(__name__)
        try:
            raw = self._backend.generate(audio, self._text_prompt)
            _log.debug("[gemma4_audio] raw output: %s", raw)
            parsed = _parse_output(raw, self._tool_alias_to_id)
        except GemmaParseError as exc:
            _log.warning("[gemma4_audio] parse error: %s", exc)
            return _FAIL_CLOSED
        except Exception as exc:
            _log.error("[gemma4_audio] generate error: %s", exc)
            return _FAIL_CLOSED

        if parsed.confidence < self._config.confidence_threshold:
            return GemmaIntentResult(
                intent_type="unknown",
                tool_id="",
                confidence=parsed.confidence,
                needs_confirm=True,
                raw_output=parsed.raw_output,
            )

        return parsed
