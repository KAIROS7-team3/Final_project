import pytest

from voice.intent_evaluator import (
    GEMMA_INTENT_ACCURACY_TARGET,
    IntentExample,
    IntentPrediction,
    evaluate_predictions,
)


def test_evaluate_predictions_reports_accuracy_and_exact_match() -> None:
    examples = [
        IntentExample("스패너 가져와", "fetch", "spanner_16mm"),
        IntentExample("복스 반납", "return", "socket_19mm"),
    ]
    predictions = [
        IntentPrediction("fetch", "spanner_16mm", 0.99),
        IntentPrediction("return", "socket_19mm", 0.98),
    ]

    result = evaluate_predictions(examples, predictions)

    assert result.total == 2
    assert result.intent_accuracy == 1.0
    assert result.exact_match_accuracy == 1.0
    assert result.passes_target is True
    assert result.failures == ()
    assert GEMMA_INTENT_ACCURACY_TARGET == 0.97


def test_evaluate_predictions_records_failures() -> None:
    examples = [IntentExample("스패너 가져와", "fetch", "spanner_16mm")]
    predictions = [IntentPrediction("fetch", "socket_19mm", 0.7)]

    result = evaluate_predictions(examples, predictions)

    assert result.intent_accuracy == 1.0
    assert result.exact_match_accuracy == 0.0
    assert result.failures[0].actual_tool_id == "socket_19mm"


def test_evaluate_predictions_requires_matching_lengths() -> None:
    with pytest.raises(ValueError):
        evaluate_predictions(
            [IntentExample("스패너 가져와", "fetch", "spanner_16mm")],
            [],
        )
