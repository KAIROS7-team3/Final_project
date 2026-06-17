from voice.transcript_filter import is_likely_hallucinated_transcript


def test_rejects_repeated_tool_name_hallucination() -> None:
    assert is_likely_hallucinated_transcript("렌치, 렌치, 렌치, 렌치") is True


def test_allows_short_command_text() -> None:
    assert is_likely_hallucinated_transcript("스패너 반납") is False


def test_allows_mixed_command_text() -> None:
    assert is_likely_hallucinated_transcript("코버 스패너 반납") is False
