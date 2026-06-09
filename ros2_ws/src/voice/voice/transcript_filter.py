"""Whisper STT 결과의 환각/반복 패턴을 가볍게 걸러내는 helper."""

from __future__ import annotations

from collections import Counter
import re

_TOKEN_RE = re.compile(r"[0-9A-Za-z가-힣]+")


def tokenize_transcript(text: str) -> list[str]:
    """문장부호를 제거하고 비교용 토큰만 뽑는다."""

    return [token for token in _TOKEN_RE.findall(text.lower()) if token]


def is_likely_hallucinated_transcript(text: str) -> bool:
    """반복 토큰이 대부분인 Whisper 환각 문장을 보수적으로 차단한다."""

    tokens = tokenize_transcript(text)
    if len(tokens) < 3:
        return False

    counts = Counter(tokens)
    most_common = counts.most_common(1)[0][1]
    unique_ratio = len(counts) / len(tokens)

    if most_common >= 3 and unique_ratio <= 0.5:
        return True

    run_length = 1
    for previous, current in zip(tokens, tokens[1:]):
        if current == previous:
            run_length += 1
            if run_length >= 3:
                return True
        else:
            run_length = 1

    return False
