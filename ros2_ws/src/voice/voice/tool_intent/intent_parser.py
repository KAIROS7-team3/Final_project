"""잡음 섞인 STT 문장을 지원 공구 ID로 매핑하는 점수 기반 parser.

Whisper는 "스패너"를 "스페인어", "복스알"을 "박스알"처럼 잘못 적을 수 있다.
이 모듈은 그런 오인식 후보를 alias table로 점수화하고, 애매한 경우에는
`need_confirm=True`로 실행을 막는다.
"""

from __future__ import annotations

import re
import unicodedata
from dataclasses import dataclass
from typing import TypedDict

from voice.tool_intent.tool_aliases import (
    ALIASES,
    DISPLAY_NAMES,
    GENERIC_TOOL_ALIASES,
    GENERIC_WRENCH_TERMS,
    NUMBER_HINTS,
    RATCHET_TERMS,
    SOCKET_TERMS,
    SPANNER_TERMS,
    TOOL_IDS,
    UNCERTAIN_ALIASES,
    UNKNOWN_TOOL_ID,
)

# 이 값보다 낮은 confidence는 사람 확인이 필요하다고 본다.
CONFIRM_THRESHOLD = 0.63

# 1등과 2등 점수가 이 margin보다 가까우면 공구 후보가 애매하다고 본다.
AMBIGUOUS_MARGIN = 1.2

# 공구 자체를 특정하지 못한 일반 요청에 사용할 안내 문구.
GENERIC_CONFIRM_TEXT = (
    "어떤 공구를 가져올까요? 드라이버, 커터 칼, 라쳇 렌치, "
    "멀티툴, 스패너 16mm, 복스 소켓 19mm 중에서 말씀해 주세요."
)
SIZE_CONFLICT_CONFIRM_TEXT = (
    "공구명과 치수가 충돌합니다. "
    "스패너 16mm와 복스 소켓 19mm 중 무엇인가요?"
)
WRENCH_CONFIRM_TEXT = (
    "렌치 종류가 여러 개입니다. "
    "라쳇 렌치, 스패너 16mm, 복스 소켓 19mm 중 무엇인가요?"
)


class ToolIntentResult(TypedDict):
    """공구명 parser의 JSON 직렬화 가능한 출력 schema."""

    tool_id: str
    confidence: float
    need_confirm: bool
    confirm_text: str


@dataclass(frozen=True)
class MatchEvidence:
    """tool score에 기여한 alias match 한 건.

    `uncertain=True`인 alias는 실제 공구명이 아니라 오인식 가능성이 큰 단어다.
    예를 들어 "스캐너"는 스패너로 들렸을 가능성이 있지만 바로 실행하면 위험하다.
    """

    alias: str
    score: float
    uncertain: bool


def normalize_text(text: str) -> str:
    """한국어/영어 alias matching을 위해 STT 문자열을 정규화한다."""

    # NFKC는 전각/호환 문자와 특수 단위 표기를 일반 문자에 가깝게 맞춘다.
    normalized = unicodedata.normalize("NFKC", text or "").casefold()
    normalized = re.sub(r"[\t\r\n]+", " ", normalized)
    normalized = normalized.replace("㎜", "mm")
    normalized = normalized.replace("미리미터", "미리")
    normalized = normalized.replace("밀리미터", "밀리")
    normalized = re.sub(r"[^0-9a-z가-힣]+", " ", normalized)
    return re.sub(r"\s+", " ", normalized).strip()


def parse_tool_intent(text: str) -> ToolIntentResult:
    """Whisper STT 문장을 프로젝트 표준 tool intent schema로 변환한다."""

    normalized = normalize_text(text)
    if not normalized:
        return _result(UNKNOWN_TOOL_ID, 0.0, False, "")

    generic_only = _is_generic_tool_request(normalized)
    if generic_only:
        # "공구 가져와"처럼 후보를 좁힐 수 없는 요청은 실행하지 않고 선택지를 묻는다.
        return _result(
            UNKNOWN_TOOL_ID,
            0.22,
            True,
            GENERIC_CONFIRM_TEXT,
        )

    scores, evidence = _score_tools(normalized)
    number_conflict = _has_number_conflict(normalized)
    generic_wrench = _is_generic_wrench_request(normalized)

    if not scores:
        return _result(UNKNOWN_TOOL_ID, 0.0, False, "")

    ranked = sorted(scores.items(), key=lambda item: item[1], reverse=True)
    best_tool_id, best_score = ranked[0]
    second_score = ranked[1][1] if len(ranked) > 1 else 0.0
    confidence = _confidence(best_score, second_score)

    need_confirm = False
    confirm_text = ""
    if number_conflict:
        # "소켓 16미리"처럼 이름과 치수가 서로 다른 공구를 가리키면 강제로 확인한다.
        need_confirm = True
        confirm_text = SIZE_CONFLICT_CONFIRM_TEXT
        confidence = min(confidence, 0.58)
    elif generic_wrench:
        # "렌치"는 라쳇/스패너/소켓 모두로 해석될 수 있어 확인이 필요하다.
        need_confirm = True
        confirm_text = WRENCH_CONFIRM_TEXT
        confidence = min(confidence, 0.48)
    elif _has_uncertain_evidence(best_tool_id, evidence):
        # 오인식 alias로만 잡힌 경우 confidence를 낮추고 확인을 요구한다.
        need_confirm = True
        confirm_text = _tool_confirm_text(best_tool_id)
        confidence = min(confidence, 0.62)
    elif _needs_score_confirmation(confidence, best_score, second_score):
        need_confirm = True
        confirm_text = _tool_confirm_text(best_tool_id)

    return _result(best_tool_id, confidence, need_confirm, confirm_text)


def _score_tools(
    normalized: str,
) -> tuple[dict[str, float], dict[str, list[MatchEvidence]]]:
    """모든 공구 alias를 훑어 공구별 score와 match 근거를 만든다."""

    scores: dict[str, float] = {}
    evidence: dict[str, list[MatchEvidence]] = {}
    compact_text = _compact(normalized)

    for tool_id in TOOL_IDS:
        for alias in ALIASES[tool_id]:
            alias_normalized = normalize_text(alias)
            if not alias_normalized:
                continue
            matched = _alias_matches(
                normalized,
                compact_text,
                alias_normalized,
            )
            if matched <= 0.0:
                continue
            uncertain = _is_uncertain_alias(tool_id, alias_normalized)
            # uncertain alias는 후보로는 인정하지만 점수를 낮춰 바로 실행될 가능성을 줄인다.
            score = matched * (0.55 if uncertain else 1.0)
            scores[tool_id] = scores.get(tool_id, 0.0) + score
            evidence.setdefault(tool_id, []).append(
                MatchEvidence(alias_normalized, score, uncertain)
            )

    for tool_id, hints in NUMBER_HINTS.items():
        if _contains_any(normalized, hints):
            # 치수 힌트는 짧지만 공구 구분에 중요하므로 별도 점수를 준다.
            scores[tool_id] = scores.get(tool_id, 0.0) + 1.0
            evidence.setdefault(tool_id, []).append(
                MatchEvidence("number_hint", 1.0, False)
            )

    return scores, evidence


def _alias_matches(normalized: str, compact_text: str, alias: str) -> float:
    """alias가 문장에 얼마나 강하게 매칭되는지 점수로 반환한다."""

    compact_alias = _compact(alias)
    if normalized == alias or compact_text == compact_alias:
        return 4.2 + _length_bonus(alias)
    if alias in normalized:
        return 2.2 + _length_bonus(alias)
    if compact_alias and compact_alias in compact_text:
        return 1.9 + _length_bonus(alias)
    return 0.0


def _length_bonus(alias: str) -> float:
    """긴 alias일수록 우연히 맞을 가능성이 낮으므로 작은 bonus를 준다."""

    return min(len(_compact(alias)) / 12.0, 1.4)


def _confidence(best_score: float, second_score: float) -> float:
    """1등 점수와 2등과의 간격을 confidence로 변환한다."""

    separation = max(best_score - second_score, 0.0)
    raw = 0.28 + min(best_score / 8.0, 0.52) + min(separation / 8.0, 0.18)
    return round(min(raw, 0.98), 2)


def _is_generic_tool_request(normalized: str) -> bool:
    """문장이 특정 공구가 아니라 "공구/툴"만 요청하는지 판단한다."""

    generic_aliases = {normalize_text(alias) for alias in GENERIC_TOOL_ALIASES}
    if normalized not in generic_aliases:
        return False

    compact_text = _compact(normalized)
    for tool_id, aliases in ALIASES.items():
        if tool_id == "multi_tool":
            continue
        for alias in aliases:
            alias_normalized = normalize_text(alias)
            if alias_normalized in generic_aliases:
                continue
            compact_alias = _compact(alias_normalized)
            if alias_normalized in normalized or compact_alias in compact_text:
                return False
    return True


def _is_generic_wrench_request(normalized: str) -> bool:
    """라쳇/스패너/소켓 중 하나로 좁혀지지 않은 일반 렌치 요청인지 판단한다."""

    if not _contains_any(normalized, GENERIC_WRENCH_TERMS):
        return False
    if _contains_any(normalized, RATCHET_TERMS):
        return False
    if _contains_any(normalized, SPANNER_TERMS):
        return False
    if _contains_any(normalized, SOCKET_TERMS):
        return False
    return True


def _has_number_conflict(normalized: str) -> bool:
    """공구명과 치수 힌트가 서로 다른 tool_id를 가리키는지 확인한다."""

    has_16 = _contains_any(normalized, NUMBER_HINTS["spanner_16mm"])
    has_19 = _contains_any(normalized, NUMBER_HINTS["socket_19mm"])
    has_spanner = _contains_any(normalized, SPANNER_TERMS)
    has_socket = _contains_any(normalized, SOCKET_TERMS)
    return (has_socket and has_16) or (has_spanner and has_19)


def _has_uncertain_evidence(
    tool_id: str,
    evidence: dict[str, list[MatchEvidence]],
) -> bool:
    """최고 점수 공구가 uncertain alias 근거를 포함하는지 확인한다."""

    matches = evidence.get(tool_id, [])
    return bool(matches) and any(match.uncertain for match in matches)


def _is_uncertain_alias(tool_id: str, alias: str) -> bool:
    """해당 alias가 실행 전 확인이 필요한 오인식 후보인지 확인한다."""

    uncertain_aliases = {
        normalize_text(item) for item in UNCERTAIN_ALIASES.get(tool_id, set())
    }
    return alias in uncertain_aliases


def _contains_any(text: str, keywords: tuple[str, ...]) -> bool:
    """공백 포함/미포함 표현을 모두 고려해 keyword 포함 여부를 확인한다."""

    compact_text = _compact(text)
    return any(
        normalize_text(keyword) in text
        or _compact(normalize_text(keyword)) in compact_text
        for keyword in keywords
    )


def _compact(text: str) -> str:
    """공백을 모두 제거한 비교용 문자열을 만든다."""

    return re.sub(r"\s+", "", text)


def _needs_score_confirmation(
    confidence: float,
    best_score: float,
    second_score: float,
) -> bool:
    """confidence가 낮거나 2등 후보가 가까우면 확인이 필요한지 판단한다."""

    close_second = (best_score - second_score) < AMBIGUOUS_MARGIN
    return (
        confidence < CONFIRM_THRESHOLD
        or (close_second and second_score > 0.0)
    )


def _tool_confirm_text(tool_id: str) -> str:
    """특정 tool_id 후보에 대한 한국어 확인 문구를 만든다."""

    return f"{DISPLAY_NAMES[tool_id]}로 이해했습니다. 맞으면 확인해 주세요."


def _result(
    tool_id: str,
    confidence: float,
    need_confirm: bool,
    confirm_text: str,
) -> ToolIntentResult:
    """반환 dict의 타입과 confidence rounding을 한 곳에서 맞춘다."""

    return {
        "tool_id": tool_id,
        "confidence": round(float(confidence), 2),
        "need_confirm": bool(need_confirm),
        "confirm_text": confirm_text,
    }
