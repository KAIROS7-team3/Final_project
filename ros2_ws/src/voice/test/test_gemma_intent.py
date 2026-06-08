from __future__ import annotations

from voice.gemma_intent import (
    GemmaConfig,
    GemmaIntentClassifier,
    GemmaIntentResult,
    build_prompt,
    parse_gemma_output,
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


def test_classifier_downgrades_low_confidence_fetch_to_unknown() -> None:
    backend = FakeBackend(
        [
            '{"intent_type":"fetch","tool_id":"spanner_16mm","confidence":0.42,"needs_confirm":false}',
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
