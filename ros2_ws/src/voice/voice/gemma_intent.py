"""Gemma 4 기반 음성 의도 분류 유틸리티.

이 모듈은 ROS2에 의존하지 않는다.

역할:
- raw text를 Gemma 프롬프트로 변환
- Gemma 출력의 strict JSON 파싱
- intent/tool_id/confidence 정규화
- low confidence 또는 malformed output을 fail-closed로 처리

실제 모델 로딩은 `GemmaIntentClassifier`가 담당하고, 테스트에서는 fake
backend를 주입할 수 있다.
"""

from __future__ import annotations

import json
import re
from pathlib import Path
from dataclasses import dataclass
from typing import Protocol

VALID_INTENTS = {"fetch", "return", "cancel", "unknown"}
TOOL_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
CODE_FENCE_PATTERN = re.compile(r"^```(?:json)?\s*(.*?)\s*```$", re.DOTALL)


@dataclass(frozen=True)
class ToolSpec:
    """Gemma 프롬프트에 넣을 공구 카탈로그 한 항목."""

    tool_id: str
    label: str
    aliases: tuple[str, ...]


TOOL_CATALOG: tuple[ToolSpec, ...] = (
    ToolSpec(
        tool_id="screwdriver",
        label="십자 드라이버",
        aliases=("십자 드라이버", "드라이버", "screwdriver"),
    ),
    ToolSpec(
        tool_id="utility_knife",
        label="커터칼",
        aliases=("커터", "커터칼", "utility knife", "knife"),
    ),
    ToolSpec(
        tool_id="ratchet_wrench",
        label="라쳇 렌치",
        aliases=("라쳇", "라쳇 렌치", "ratchet", "ratchet wrench"),
    ),
    ToolSpec(
        tool_id="multi_tool",
        label="멕가이버",
        aliases=("멕가이버", "맥가이버", "multi tool", "multitool"),
    ),
    ToolSpec(
        tool_id="spanner_16mm",
        label="스패너 16mm",
        aliases=("스패너", "스패너 16mm", "16mm", "spanner", "spanner 16mm"),
    ),
    ToolSpec(
        tool_id="socket_19mm",
        label="복스 소켓 19mm",
        aliases=("복스", "소켓", "복스 소켓", "19mm", "socket", "socket 19mm"),
    ),
)

ALIAS_TO_TOOL_ID: dict[str, str] = {
    alias.strip().lower(): spec.tool_id
    for spec in TOOL_CATALOG
    for alias in spec.aliases
}
TOOL_IDS = {spec.tool_id for spec in TOOL_CATALOG}


class GemmaLoadError(RuntimeError):
    """Gemma backend 로딩 또는 warmup 실패 시 발생한다."""


class GemmaParseError(RuntimeError):
    """Gemma 출력이 strict JSON 규약을 따르지 않을 때 발생한다."""


class GemmaBackend(Protocol):
    """테스트에서 fake backend를 주입하기 위한 최소 protocol."""

    def generate(self, prompt: str) -> str:
        """프롬프트 한 건에 대한 원문 응답을 반환한다."""


@dataclass(frozen=True)
class GemmaConfig:
    """Gemma 추론 설정."""

    model_id: str = "~/models/gemma/gemma-3-1b-it"
    device: str = "auto"
    confidence_threshold: float = 0.75
    max_new_tokens: int = 128
    temperature: float = 0.0
    warmup: bool = True


@dataclass(frozen=True)
class GemmaIntentResult:
    """Gemma가 해석한 최종 intent 결과."""

    intent_type: str
    tool_id: str
    confidence: float
    needs_confirm: bool = False
    raw_output: str = ""


def build_prompt(raw_text: str) -> str:
    """Gemma에 넣을 한국어 intent 분류 프롬프트를 만든다."""

    tool_catalog_lines = "\n".join(
        f"- {spec.tool_id} ({spec.label}): {', '.join(spec.aliases)}"
        for spec in TOOL_CATALOG
    )

    return (
        "당신은 공구 전달 로봇의 음성 의도 분류기다.\n"
        "입력된 한국어 발화를 보고 반드시 JSON 객체 하나만 출력한다.\n"
        "설명, 마크다운, 코드펜스 바깥의 문장은 출력하지 않는다.\n\n"
        "출력 스키마:\n"
        '{"intent_type":"fetch|return|cancel|unknown",'
        '"tool_id":"canonical_tool_id_or_empty",'
        '"confidence":0.0,'
        '"needs_confirm":false}\n\n'
        "규칙:\n"
        "- intent_type은 fetch, return, cancel, unknown 중 하나다.\n"
        "- fetch/return은 tool_id가 필수다.\n"
        "- cancel/unknown은 tool_id를 빈 문자열로 둔다.\n"
        "- confidence는 0.0부터 1.0 사이의 실수다.\n"
        "- 애매한 발화, 공구가 빠진 발화, 상태가 불명확한 발화는 unknown이다.\n"
        "- 모델이 자신 없으면 needs_confirm를 true로 두되, fetch/return은 "
        "낮은 confidence 또는 확인 필요 상태에서 unknown으로 떨어질 수 있다.\n"
        "- tool_id는 아래 공구 카탈로그의 canonical ID만 쓴다.\n\n"
        f"공구 카탈로그:\n{tool_catalog_lines}\n\n"
        f"입력 발화: {raw_text.strip()}\n"
    )


def resolve_model_id_path(model_id: str) -> str:
    """Gemma 모델 경로의 `~`를 실제 홈 디렉터리로 확장한다."""

    return str(Path(model_id).expanduser())


def parse_gemma_output(output: str) -> GemmaIntentResult:
    """Gemma 원문 응답을 strict JSON으로 파싱한다."""

    cleaned = _strip_code_fence(output.strip())
    try:
        data = json.loads(cleaned)
    except json.JSONDecodeError as exc:
        raise GemmaParseError("Gemma output must be valid JSON") from exc
    if not isinstance(data, dict):
        raise GemmaParseError("Gemma output must be a JSON object")

    intent_type = str(data.get("intent_type", "")).strip().lower()
    if intent_type not in VALID_INTENTS:
        raise GemmaParseError(f"unsupported intent_type: {intent_type!r}")

    raw_confidence = data.get("confidence", None)
    if raw_confidence is None:
        raise GemmaParseError("confidence is required")
    try:
        confidence = float(raw_confidence)
    except (TypeError, ValueError) as exc:
        raise GemmaParseError("confidence must be numeric") from exc
    if not 0.0 <= confidence <= 1.0:
        raise GemmaParseError("confidence must be between 0.0 and 1.0")

    needs_confirm = bool(data.get("needs_confirm", False))
    tool_id = _normalize_tool_id(str(data.get("tool_id", "")).strip())

    if intent_type in {"fetch", "return"} and not tool_id:
        raise GemmaParseError("fetch/return requires a canonical tool_id")
    if intent_type in {"cancel", "unknown"}:
        tool_id = ""

    return GemmaIntentResult(
        intent_type=intent_type,
        tool_id=tool_id,
        confidence=confidence,
        needs_confirm=needs_confirm,
        raw_output=cleaned,
    )


class GemmaIntentClassifier:
    """Gemma backend를 통해 raw text를 intent 결과로 변환한다."""

    def __init__(
        self,
        config: GemmaConfig | None = None,
        backend: GemmaBackend | None = None,
    ) -> None:
        self.config = config or GemmaConfig()
        self._backend = backend or self._load_backend()
        if self.config.warmup:
            self.warmup()

    def warmup(self) -> None:
        """짧은 더미 추론으로 모델을 예열한다."""

        try:
            self._backend.generate(build_prompt("취소"))
        except Exception as exc:  # pragma: no cover - backend runtime path
            raise GemmaLoadError("Gemma warmup failed") from exc

    def classify(self, raw_text: str) -> GemmaIntentResult:
        """raw text를 Gemma로 분류하고 fail-closed 결과를 반환한다."""

        prompt = build_prompt(raw_text)
        try:
            raw_output = self._backend.generate(prompt)
        except Exception as exc:  # pragma: no cover - backend runtime path
            raise GemmaLoadError("Gemma inference failed") from exc

        try:
            parsed = parse_gemma_output(raw_output)
        except GemmaParseError:
            return GemmaIntentResult(
                intent_type="unknown",
                tool_id="",
                confidence=0.0,
                needs_confirm=True,
                raw_output=raw_output,
            )

        if parsed.intent_type in {"fetch", "return"}:
            if parsed.needs_confirm or parsed.confidence < self.config.confidence_threshold:
                return GemmaIntentResult(
                    intent_type="unknown",
                    tool_id="",
                    confidence=0.0,
                    needs_confirm=True,
                    raw_output=parsed.raw_output,
                )

        return parsed

    def _load_backend(self) -> GemmaBackend:
        """기본 HuggingFace backend를 로드한다."""

        try:
            from transformers import AutoModelForCausalLM, AutoTokenizer
        except ImportError as exc:  # pragma: no cover - optional dependency path
            raise GemmaLoadError("transformers is required for Gemma runtime") from exc

        try:
            import torch
        except ImportError as exc:  # pragma: no cover - optional dependency path
            raise GemmaLoadError("torch is required for Gemma runtime") from exc

        resolved_device = _resolve_device(self.config.device, torch)
        model_id = resolve_model_id_path(self.config.model_id)
        return _TransformersGemmaBackend(
            model_id=model_id,
            device=resolved_device,
            max_new_tokens=self.config.max_new_tokens,
            temperature=self.config.temperature,
            tokenizer_cls=AutoTokenizer,
            model_cls=AutoModelForCausalLM,
            torch_module=torch,
        )


class _TransformersGemmaBackend:
    """transformers 기반 기본 backend."""

    def __init__(
        self,
        model_id: str,
        device: str,
        max_new_tokens: int,
        temperature: float,
        tokenizer_cls,
        model_cls,
        torch_module,
    ) -> None:
        self._device = device
        self._max_new_tokens = max_new_tokens
        self._temperature = temperature
        self._torch = torch_module
        self._tokenizer = tokenizer_cls.from_pretrained(model_id)
        self._model = model_cls.from_pretrained(model_id)
        self._model.to(device)
        self._model.eval()

    def generate(self, prompt: str) -> str:
        rendered_prompt = self._render_prompt(prompt)
        inputs = self._tokenizer(rendered_prompt, return_tensors="pt")
        inputs = {key: value.to(self._device) for key, value in inputs.items()}

        generate_kwargs = {
            "max_new_tokens": self._max_new_tokens,
            "pad_token_id": self._tokenizer.eos_token_id,
        }
        if self._temperature > 0.0:
            generate_kwargs["do_sample"] = True
            generate_kwargs["temperature"] = self._temperature
        else:
            generate_kwargs["do_sample"] = False

        with self._torch.inference_mode():
            output_tokens = self._model.generate(**inputs, **generate_kwargs)

        input_length = inputs["input_ids"].shape[-1]
        generated_tokens = output_tokens[0][input_length:]
        return self._tokenizer.decode(generated_tokens, skip_special_tokens=True).strip()

    def _render_prompt(self, prompt: str) -> str:
        if hasattr(self._tokenizer, "apply_chat_template"):
            messages = [
                {
                    "role": "system",
                    "content": (
                        "당신은 공구 전달 로봇의 음성 의도 분류기다. "
                        "반드시 JSON 객체 하나만 출력한다."
                    ),
                },
                {"role": "user", "content": prompt},
            ]
            return self._tokenizer.apply_chat_template(
                messages,
                tokenize=False,
                add_generation_prompt=True,
            )
        return prompt


def _strip_code_fence(text: str) -> str:
    match = CODE_FENCE_PATTERN.match(text)
    if match:
        return match.group(1).strip()
    return text


def _normalize_tool_id(tool_id: str) -> str:
    candidate = tool_id.strip().lower().replace("-", "_").replace(" ", "_")
    if not candidate:
        return ""
    if candidate in TOOL_IDS:
        return candidate
    return ALIAS_TO_TOOL_ID.get(candidate, "")


def _resolve_device(device: str, torch_module) -> str:
    if device != "auto":
        return device
    return "cuda" if torch_module.cuda.is_available() else "cpu"
