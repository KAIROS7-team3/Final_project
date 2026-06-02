"""Intent classifier 평가용 helper.

Whisper/Gemma 조합의 regression test에서 intent 정확도와 tool_id까지 맞는
exact match 정확도를 별도로 계산한다.

intent_accuracy:
    fetch/return/cancel/unknown 같은 의도만 맞았는지 본다.

exact_match_accuracy:
    의도와 tool_id가 모두 맞았는지 본다. 실제 로봇 동작에서는 tool_id 오류가
    바로 잘못된 공구 선택으로 이어지므로 이 값을 별도로 추적한다.
"""

from __future__ import annotations

from dataclasses import dataclass

# Phase 기준: 의도 분류는 97% 이상을 통과 기준으로 둔다.
GEMMA_INTENT_ACCURACY_TARGET = 0.97


@dataclass(frozen=True)
class IntentExample:
    """평가 데이터의 정답 한 건.

    raw_utterance는 사람이 말한 문장이고, expected_* 값은 기대 intent 결과다.
    """

    raw_utterance: str
    expected_intent_type: str
    expected_tool_id: str


@dataclass(frozen=True)
class IntentPrediction:
    """모델 또는 파서가 예측한 intent 결과."""

    intent_type: str
    tool_id: str
    confidence: float


@dataclass(frozen=True)
class IntentFailure:
    """exact match 실패를 사람이 확인하기 쉽게 남기는 레코드.

    어떤 발화에서 어떤 tool_id 또는 intent가 틀렸는지 regression report에
    표시하기 위한 구조체다.
    """

    raw_utterance: str
    expected_intent_type: str
    expected_tool_id: str
    actual_intent_type: str
    actual_tool_id: str


@dataclass(frozen=True)
class IntentEvaluationResult:
    """평가 전체 요약과 실패 목록."""

    total: int
    intent_accuracy: float
    exact_match_accuracy: float
    passes_target: bool
    failures: tuple[IntentFailure, ...]


def evaluate_predictions(
    examples: list[IntentExample],
    predictions: list[IntentPrediction],
    target_accuracy: float = GEMMA_INTENT_ACCURACY_TARGET,
) -> IntentEvaluationResult:
    """정답 목록과 예측 목록을 비교해 정확도와 실패 사례를 계산한다."""

    if len(examples) != len(predictions):
        raise ValueError("examples and predictions must have the same length")
    if not examples:
        # 빈 평가셋은 성공으로 보지 않는다. 데이터가 없으면 품질을 판단할 수 없다.
        return IntentEvaluationResult(0, 0.0, 0.0, False, ())

    intent_matches = 0
    exact_matches = 0
    failures: list[IntentFailure] = []

    for example, prediction in zip(examples, predictions):
        # intent만 맞은 경우와 tool_id까지 맞은 경우를 분리해 원인 분석을 쉽게 한다.
        intent_match = prediction.intent_type == example.expected_intent_type
        exact_match = intent_match and prediction.tool_id == example.expected_tool_id
        intent_matches += int(intent_match)
        exact_matches += int(exact_match)
        if not exact_match:
            failures.append(
                IntentFailure(
                    raw_utterance=example.raw_utterance,
                    expected_intent_type=example.expected_intent_type,
                    expected_tool_id=example.expected_tool_id,
                    actual_intent_type=prediction.intent_type,
                    actual_tool_id=prediction.tool_id,
                )
            )

    intent_accuracy = intent_matches / len(examples)
    exact_match_accuracy = exact_matches / len(examples)
    return IntentEvaluationResult(
        total=len(examples),
        intent_accuracy=intent_accuracy,
        exact_match_accuracy=exact_match_accuracy,
        passes_target=intent_accuracy >= target_accuracy,
        failures=tuple(failures),
    )
