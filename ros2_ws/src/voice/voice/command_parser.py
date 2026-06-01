"""Gemma 4 연동 전까지 사용하는 임시 키워드 기반 명령 파서.

이 파일은 STT 결과 문자열을 프로젝트 표준 intent/tool_id 형태로 바꾼다.
나중에 Gemma intent classifier가 붙어도 테스트 기준선으로 남겨둘 수 있다.

입력 예:
    "스패너 가져와"

출력 예:
    ParsedCommand(intent_type="fetch", tool_id="spanner_16mm", confidence=0.65)

의도적으로 단순한 포함 검색을 사용한다. 현장 데모에서 명령어 종류가 제한되어
있고, Gemma 4가 붙기 전까지 deterministic한 동작이 더 중요하기 때문이다.
"""

from __future__ import annotations

from dataclasses import dataclass

# 한국어 명령은 조사가 붙어도 매칭되도록 단어 전체보다 핵심 어간을 둔다.
FETCH_KEYWORDS = ("가져", "꺼내", "fetch", "bring")
RETURN_KEYWORDS = ("반납", "돌려", "넣어", "return")
CANCEL_KEYWORDS = ("취소", "중지", "cancel", "stop")
SPANNER_SIZE_KEYWORDS = (
    "16mm",
    "16 mm",
    "16미리",
    "16 밀리",
    "십육미리",
    "십육 밀리",
)
SOCKET_SIZE_KEYWORDS = (
    "19mm",
    "19 mm",
    "19미리",
    "19 밀리",
    "십구미리",
    "십구 밀리",
)
SPANNER_NAME_KEYWORDS = ("스패너", "spanner")
SOCKET_NAME_KEYWORDS = ("복스", "소켓", "socket")

# DB의 tool_id와 사용자가 실제로 말할 가능성이 높은 별칭을 연결한다.
TOOL_ALIASES = {
    "screwdriver": ("드라이버", "십자", "screwdriver"),
    "utility_knife": ("커터", "커터칼", "knife"),
    "ratchet_wrench": ("라쳇", "ratchet"),
    "multi_tool": ("멕가이버", "맥가이버", "multi tool"),
    "spanner_16mm": ("스패너", "16mm", "spanner"),
    "socket_19mm": ("복스", "소켓", "19mm", "socket"),
}


@dataclass(frozen=True)
class ParsedCommand:
    """STT 문장을 해석한 최소 intent 결과."""

    intent_type: str
    tool_id: str
    confidence: float


def parse_command(text: str) -> ParsedCommand:
    """STT 문자열 하나를 intent/tool_id/confidence로 변환한다."""

    normalized = text.strip().lower()
    if not normalized:
        return ParsedCommand("unknown", "", 0.0)

    # cancel은 특정 공구가 없어도 의미가 있으므로 가장 먼저 확인한다.
    if _contains_any(normalized, CANCEL_KEYWORDS):
        return ParsedCommand("cancel", "", 0.9)

    intent_type = "unknown"
    # "반납해줘", "돌려놔", "넣어줘" 같은 표현을 return으로 묶는다.
    if _contains_any(normalized, RETURN_KEYWORDS):
        intent_type = "return"
    # "가져와", "꺼내줘" 같은 표현을 fetch로 묶는다.
    elif _contains_any(normalized, FETCH_KEYWORDS):
        intent_type = "fetch"

    if _has_size_conflict(normalized):
        return ParsedCommand("unknown", "", 0.0)

    tool_id = _match_tool(normalized)
    if intent_type in {"fetch", "return"} and tool_id:
        return ParsedCommand(intent_type, tool_id, 0.65)
    if not tool_id:
        return ParsedCommand("unknown", "", 0.0)
    # intent 또는 tool_id 중 하나만 잡힌 경우다. downstream에서 낮은 confidence로
    # 취급할 수 있게 0.3을 반환한다.
    return ParsedCommand(intent_type, tool_id, 0.3)


def _contains_any(text: str, keywords: tuple[str, ...]) -> bool:
    """text 안에 keyword 중 하나라도 포함되어 있는지 확인한다."""

    return any(keyword in text for keyword in keywords)


def _match_tool(text: str) -> str:
    """공구 별칭 중 하나가 STT 문자열에 포함되면 DB tool_id를 반환한다."""

    for tool_id, aliases in TOOL_ALIASES.items():
        if any(alias in text for alias in aliases):
            return tool_id
    return ""


def _has_size_conflict(text: str) -> bool:
    """공구명과 치수 힌트가 서로 다른 공구를 가리키면 확정하지 않는다."""

    spanner_name = _contains_any(text, SPANNER_NAME_KEYWORDS)
    socket_name = _contains_any(text, SOCKET_NAME_KEYWORDS)
    spanner_size = _contains_any(text, SPANNER_SIZE_KEYWORDS)
    socket_size = _contains_any(text, SOCKET_SIZE_KEYWORDS)
    return (socket_name and spanner_size) or (spanner_name and socket_size)
