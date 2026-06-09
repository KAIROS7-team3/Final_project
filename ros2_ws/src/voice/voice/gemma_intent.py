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
from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol

try:
    import yaml
except ImportError:  # pragma: no cover - optional dependency guard
    yaml = None

VALID_INTENTS = {"fetch", "return", "cancel", "unknown"}
TOOL_ID_PATTERN = re.compile(r"^[a-z][a-z0-9_]*$")
CODE_FENCE_PATTERN = re.compile(r"^```(?:json)?\s*(.*?)\s*```$", re.DOTALL)
DEFAULT_PROMPT_TEMPLATE_PATH = Path(__file__).with_name("gemma_prompt.txt")
FALLBACK_TOOLBOX_PATH = Path(__file__).resolve().parents[4] / "config" / "toolbox.yaml"


@dataclass(frozen=True)
class ToolSpec:
    """Gemma 프롬프트에 넣을 공구 카탈로그 한 항목."""

    tool_id: str
    label: str
    aliases: tuple[str, ...]


def _normalize_tool_key(value: str) -> str:
    return value.strip().lower().replace("-", "_").replace(" ", "_")


def default_toolbox_path() -> Path:
    """toolbox.yaml의 기본 위치를 해석한다.

    설치본이 있으면 `share/voice/config/toolbox.yaml`을 우선하고, 없으면
    소스 트리 루트의 `config/toolbox.yaml`로 fallback한다.
    """

    try:
        from ament_index_python.packages import get_package_share_directory

        return Path(get_package_share_directory("voice")) / "config" / "toolbox.yaml"
    except Exception:  # pragma: no cover - package lookup/runtime fallback path
        return FALLBACK_TOOLBOX_PATH


def _dedupe_preserve_order(values: tuple[str, ...]) -> tuple[str, ...]:
    """중복 alias를 제거하되 원래 순서를 유지한다."""

    seen: set[str] = set()
    unique_values: list[str] = []
    for value in values:
        if value in seen:
            continue
        seen.add(value)
        unique_values.append(value)
    return tuple(unique_values)


def load_tool_catalog(toolbox_path: str | Path | None = None) -> tuple[ToolSpec, ...]:
    """toolbox.yaml에서 canonical tool catalog를 읽는다."""

    if yaml is None:  # pragma: no cover - dependency guard
        raise GemmaLoadError("PyYAML is required to load toolbox.yaml")

    path = Path(toolbox_path).expanduser() if toolbox_path else default_toolbox_path()
    try:
        raw_catalog = yaml.safe_load(path.read_text(encoding="utf-8"))
    except OSError as exc:
        raise GemmaLoadError(f"failed to load Gemma toolbox catalog: {path}") from exc
    except Exception as exc:  # pragma: no cover - parser/runtime path
        raise GemmaLoadError(f"failed to parse Gemma toolbox catalog: {path}") from exc

    if not isinstance(raw_catalog, dict):
        raise GemmaLoadError("Gemma toolbox catalog must be a mapping")

    raw_tools = raw_catalog.get("tools", [])
    if not isinstance(raw_tools, list) or not raw_tools:
        raise GemmaLoadError(
            "Gemma toolbox catalog must contain a non-empty tools list"
        )

    tool_catalog: list[ToolSpec] = []
    for raw_spec in raw_tools:
        if not isinstance(raw_spec, dict):
            raise GemmaLoadError("each toolbox entry must be a mapping")

        tool_id = str(raw_spec.get("tool_id", "")).strip()
        label = str(raw_spec.get("display_name", "")).strip()
        if not tool_id or not label:
            raise GemmaLoadError("toolbox entries require tool_id and display_name")
        if not TOOL_ID_PATTERN.match(tool_id):
            raise GemmaLoadError(f"unsupported tool_id in toolbox.yaml: {tool_id!r}")

        raw_aliases = raw_spec.get("aliases", ())
        if raw_aliases is None:
            alias_values: tuple[str, ...] = ()
        elif isinstance(raw_aliases, (list, tuple)):
            alias_values = tuple(
                str(alias).strip()
                for alias in raw_aliases
                if str(alias).strip()
            )
        else:
            raise GemmaLoadError("toolbox aliases must be a sequence of strings")

        aliases = _dedupe_preserve_order((label, *alias_values))
        tool_catalog.append(ToolSpec(tool_id=tool_id, label=label, aliases=aliases))

    return tuple(tool_catalog)


def _build_tool_alias_map(tool_catalog: tuple[ToolSpec, ...]) -> dict[str, str]:
    """alias -> canonical tool_id 매핑을 만든다."""

    return {
        _normalize_tool_key(alias): spec.tool_id
        for spec in tool_catalog
        for alias in spec.aliases
    }


def _tool_catalog_lines(tool_catalog: tuple[ToolSpec, ...]) -> str:
    return "\n".join(
        f"- {spec.tool_id} ({spec.label}): {', '.join(spec.aliases)}"
        for spec in tool_catalog
    )


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
    prompt_template_path: str = str(DEFAULT_PROMPT_TEMPLATE_PATH)
    toolbox_path: str = field(default_factory=lambda: str(default_toolbox_path()))
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


def load_prompt_template(prompt_template_path: str | Path | None = None) -> str:
    """외부 텍스트 파일에서 Gemma 프롬프트 템플릿을 읽는다."""

    path = (
        Path(prompt_template_path).expanduser()
        if prompt_template_path
        else DEFAULT_PROMPT_TEMPLATE_PATH
    )
    try:
        template = path.read_text(encoding="utf-8")
    except OSError as exc:
        raise GemmaLoadError(f"failed to load Gemma prompt template: {path}") from exc

    if "__TOOL_CATALOG__" not in template or "__RAW_TEXT__" not in template:
        raise GemmaLoadError(
            "Gemma prompt template must contain __TOOL_CATALOG__ and __RAW_TEXT__"
        )
    return template


def build_prompt(
    raw_text: str,
    prompt_template: str | None = None,
    tool_catalog: tuple[ToolSpec, ...] | None = None,
) -> str:
    """Gemma에 넣을 한국어 intent 분류 프롬프트를 만든다."""

    template = prompt_template or load_prompt_template()
    catalog = tool_catalog or load_tool_catalog()
    tool_catalog_lines = _tool_catalog_lines(catalog)

    return template.replace("__TOOL_CATALOG__", tool_catalog_lines).replace(
        "__RAW_TEXT__", raw_text.strip()
    )


def resolve_model_id_path(model_id: str) -> str:
    """Gemma 모델 경로의 `~`를 실제 홈 디렉터리로 확장한다."""

    return str(Path(model_id).expanduser())


def parse_gemma_output(
    output: str,
    *,
    tool_alias_to_id: dict[str, str] | None = None,
    valid_tool_ids: set[str] | None = None,
) -> GemmaIntentResult:
    """Gemma 원문 응답을 strict JSON으로 파싱한다."""

    if tool_alias_to_id is None and valid_tool_ids is None:
        catalog = load_tool_catalog()
        tool_alias_to_id = _build_tool_alias_map(catalog)
        valid_tool_ids = {spec.tool_id for spec in catalog}
    elif tool_alias_to_id is None:
        catalog = load_tool_catalog()
        tool_alias_to_id = _build_tool_alias_map(catalog)
        if valid_tool_ids is None:
            valid_tool_ids = {spec.tool_id for spec in catalog}
    elif valid_tool_ids is None:
        valid_tool_ids = set(tool_alias_to_id.values())

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
    tool_id = _normalize_tool_id(
        str(data.get("tool_id", "")).strip(),
        tool_alias_to_id=tool_alias_to_id,
        valid_tool_ids=valid_tool_ids,
    )

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
        self._tool_catalog = load_tool_catalog(self.config.toolbox_path)
        self._tool_alias_to_id = _build_tool_alias_map(self._tool_catalog)
        self._tool_ids = {spec.tool_id for spec in self._tool_catalog}
        self._prompt_template = load_prompt_template(
            self.config.prompt_template_path
        )
        self._backend = backend or self._load_backend()
        if self.config.warmup:
            self.warmup()

    def warmup(self) -> None:
        """짧은 더미 추론으로 모델을 예열한다."""

        try:
            self._backend.generate(
                build_prompt("취소", self._prompt_template, self._tool_catalog)
            )
        except Exception as exc:  # pragma: no cover - backend runtime path
            raise GemmaLoadError("Gemma warmup failed") from exc

    def classify(self, raw_text: str) -> GemmaIntentResult:
        """raw text를 Gemma로 분류하고 fail-closed 결과를 반환한다."""

        prompt = build_prompt(raw_text, self._prompt_template, self._tool_catalog)
        try:
            raw_output = self._backend.generate(prompt)
        except Exception as exc:  # pragma: no cover - backend runtime path
            raise GemmaLoadError("Gemma inference failed") from exc

        try:
            parsed = parse_gemma_output(
                raw_output,
                tool_alias_to_id=self._tool_alias_to_id,
                valid_tool_ids=self._tool_ids,
            )
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


def _normalize_tool_id(
    tool_id: str,
    *,
    tool_alias_to_id: dict[str, str],
    valid_tool_ids: set[str],
) -> str:
    candidate = _normalize_tool_key(tool_id)
    if not candidate:
        return ""
    if candidate in valid_tool_ids:
        return candidate
    return tool_alias_to_id.get(candidate, "")


def _resolve_device(device: str, torch_module) -> str:
    if device != "auto":
        return device
    return "cuda" if torch_module.cuda.is_available() else "cpu"
