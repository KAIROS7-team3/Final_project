from __future__ import annotations

import pytest

from voice.gemma_intent import (
    DEFAULT_PROMPT_TEMPLATE_PATH,
    GemmaConfig,
    GemmaIntentClassifier,
    GemmaIntentResult,
    ToolSpec,
    build_prompt,
    load_prompt_template,
    load_tool_catalog,
    parse_gemma_output,
    resolve_model_id_path,
)


class FakeBackend:
    def __init__(self, outputs: list[str]) -> None:
        self.outputs = outputs
        self.prompts: list[str] = []

    def generate(self, prompt: str) -> str:
        self.prompts.append(prompt)
        return self.outputs.pop(0)


def test_build_prompt_contains_raw_text_and_tool_catalog() -> None:
    prompt = build_prompt("스패너 가져와")

    assert "스패너 가져와" in prompt
    assert "spanner_16mm" in prompt
    assert "JSON 객체 하나만 출력" in prompt


def test_build_prompt_uses_custom_tool_catalog() -> None:
    prompt = build_prompt(
        "니들 가져와",
        "규칙:\n__TOOL_CATALOG__\n입력: __RAW_TEXT__\n",
        (
            ToolSpec(
                tool_id="needle_nose",
                label="니들 노즈 플라이어",
                aliases=("니들 노즈 플라이어", "needle nose"),
            ),
        ),
    )

    assert "- needle_nose (니들 노즈 플라이어): 니들 노즈 플라이어, needle nose" in prompt
    assert "니들 가져와" in prompt


def test_load_tool_catalog_reads_root_toolbox_yaml() -> None:
    catalog = load_tool_catalog()

    assert any(
        spec.tool_id == "socket_19mm" and "복스 소켓 19mm" in spec.aliases
        for spec in catalog
    )


def test_gemma_config_default_toolbox_path_loads_catalog() -> None:
    config = GemmaConfig()
    catalog = load_tool_catalog(config.toolbox_path)

    assert config.toolbox_path.endswith("toolbox.yaml")
    assert len(catalog) == 6


def test_load_prompt_template_reads_external_file() -> None:
    template = load_prompt_template(DEFAULT_PROMPT_TEMPLATE_PATH)

    assert "__TOOL_CATALOG__" in template
    assert "__RAW_TEXT__" in template
    assert "JSON 객체 하나만 출력" in template


def test_load_prompt_template_uses_custom_file(tmp_path) -> None:
    custom_path = tmp_path / "custom_prompt.txt"
    custom_path.write_text(
        "규칙:\n__TOOL_CATALOG__\n입력: __RAW_TEXT__\n",
        encoding="utf-8",
    )

    template = load_prompt_template(str(custom_path))

    assert "규칙:" in template
    assert "__TOOL_CATALOG__" in template
    assert "__RAW_TEXT__" in template


def test_resolve_model_id_path_expands_tilde() -> None:
    resolved = resolve_model_id_path("~/models/gemma/gemma-3-1b-it")

    assert resolved.endswith("/models/gemma/gemma-3-1b-it")
    assert resolved.startswith("/")


def test_parse_gemma_output_accepts_fenced_json_and_aliases() -> None:
    result = parse_gemma_output(
        """```json
        {"intent_type":"fetch","tool_id":"스패너","confidence":0.91,"needs_confirm":false}
        ```"""
    )

    assert result == GemmaIntentResult(
        intent_type="fetch",
        tool_id="spanner_16mm",
        confidence=0.91,
        needs_confirm=False,
        raw_output='{"intent_type":"fetch","tool_id":"스패너","confidence":0.91,"needs_confirm":false}',
    )


@pytest.mark.parametrize(
    ("alias", "expected_tool_id"),
    [
        ("스패너 16mm", "spanner_16mm"),
        ("복스 소켓 19mm", "socket_19mm"),
    ],
)
def test_parse_gemma_output_normalizes_multiword_aliases(
    alias: str,
    expected_tool_id: str,
) -> None:
    result = parse_gemma_output(
        (
            f'{{"intent_type":"fetch","tool_id":"{alias}",'
            '"confidence":0.91,"needs_confirm":false}'
        )
    )

    assert result.tool_id == expected_tool_id


def test_classifier_downgrades_low_confidence_fetch_to_unknown() -> None:
    backend = FakeBackend(
        [
            (
                '{"intent_type":"fetch","tool_id":"spanner_16mm",'
                '"confidence":0.42,"needs_confirm":false}'
            ),
        ]
    )
    classifier = GemmaIntentClassifier(
        GemmaConfig(confidence_threshold=0.75, warmup=False),
        backend=backend,
    )

    result = classifier.classify("스패너 가져와")

    assert result.intent_type == "unknown"
    assert result.tool_id == ""
    assert result.confidence == 0.0
    assert backend.prompts


def test_classifier_keeps_cancel_even_when_confidence_is_low() -> None:
    backend = FakeBackend(
        [
            '{"intent_type":"cancel","tool_id":"","confidence":0.12,"needs_confirm":true}',
        ]
    )
    classifier = GemmaIntentClassifier(
        GemmaConfig(confidence_threshold=0.75, warmup=False),
        backend=backend,
    )

    result = classifier.classify("작업 취소")

    assert result.intent_type == "cancel"
    assert result.tool_id == ""
    assert result.confidence == 0.12


def test_classifier_returns_unknown_for_malformed_output() -> None:
    backend = FakeBackend(["아무 말이나 붙인 응답"])
    classifier = GemmaIntentClassifier(
        GemmaConfig(confidence_threshold=0.75, warmup=False),
        backend=backend,
    )

    result = classifier.classify("스패너 가져와")

    assert result.intent_type == "unknown"
    assert result.tool_id == ""
    assert result.confidence == 0.0
