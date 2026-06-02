"""보정된 tool parser 결과를 fetch/return 명령으로 연결하는 bridge.

`voice.command_parser`는 명령 의도(fetch/return/cancel)를 빠르게 잡고,
`tool_intent.intent_parser`는 Whisper 오인식이 섞인 공구명을 보정한다.
이 모듈은 두 결과를 합쳐 DB Gate로 보낼 수 있는 command를 만든다.
"""

from __future__ import annotations

from dataclasses import dataclass

from voice.command_parser import parse_command
from voice.tool_intent.intent_parser import parse_tool_intent
from voice.tool_intent.tool_aliases import UNKNOWN_TOOL_ID

FETCH_REQUEST_TERMS = (
    # parse_command가 놓친 짧은 요청 표현을 보완하기 위한 후보 단어들이다.
    "가져",
    "갖고",
    "꺼내",
    "줘",
    "주세요",
    "찾아",
    "fetch",
    "bring",
    "give",
    "please",
)

RETURN_REQUEST_TERMS = (
    # 반납 의도는 공구명 보정과 별도로 명확한 동사가 있어야만 return으로 본다.
    "반납",
    "돌려",
    "넣어",
    "return",
    "restore",
)


@dataclass(frozen=True)
class ResolvedToolCommand:
    """공구명 보정을 적용한 최종 voice command 결과."""

    intent_type: str
    tool_id: str
    confidence: float
    need_confirm: bool
    confirm_text: str


def resolve_tool_command(
    text: str,
    allow_uncertain_tool_intent: bool = False,
) -> ResolvedToolCommand:
    """STT 문장을 보정된 tool parser 기반 command로 변환한다.

    `allow_uncertain_tool_intent`는 bench test용이다. 운영 기본값에서는
    `need_confirm=True`인 명령을 `unknown`으로 바꿔 DB Gate와 motion stack에
    도달하지 못하게 한다.
    """

    command = parse_command(text)
    tool_intent = parse_tool_intent(text)
    tool_id = str(tool_intent["tool_id"])
    confidence = float(tool_intent["confidence"])
    need_confirm = bool(tool_intent["need_confirm"])
    confirm_text = str(tool_intent["confirm_text"])

    if command.intent_type == "cancel":
        # cancel은 공구 ID가 필요 없고 DB Gate 대상도 아니므로 그대로 통과시킨다.
        return ResolvedToolCommand("cancel", "", command.confidence, False, "")

    intent_type = command.intent_type
    # 기본 parser가 intent를 못 잡았더라도, 공구명과 요청 동사가 같이 보이면
    # fetch/return으로 복구한다. 단, 아래에서 uncertain 결과는 다시 차단한다.
    if (
        intent_type == "unknown"
        and tool_id != UNKNOWN_TOOL_ID
        and _looks_like_fetch_request(text)
    ):
        intent_type = "fetch"
    elif (
        intent_type == "unknown"
        and tool_id != UNKNOWN_TOOL_ID
        and _looks_like_return_request(text)
    ):
        intent_type = "return"

    if intent_type not in {"fetch", "return"}:
        # fetch/return이 아니면 DB Gate 대상이 아니다. tool 후보가 있어도 동작 명령은
        # 아니므로 downstream motion으로 이어지지 않는다.
        return ResolvedToolCommand(
            intent_type,
            tool_id if tool_id != UNKNOWN_TOOL_ID else "",
            confidence if tool_id != UNKNOWN_TOOL_ID else command.confidence,
            need_confirm,
            confirm_text,
        )

    if tool_id == UNKNOWN_TOOL_ID:
        # 공구를 특정하지 못한 fetch/return은 실행할 수 없다.
        return ResolvedToolCommand(
            "unknown",
            "",
            confidence,
            need_confirm,
            confirm_text,
        )

    if need_confirm and not allow_uncertain_tool_intent:
        # 애매한 STT 보정은 사람 확인 없이 실행하지 않는다. tool_id는 로그/안내에
        # 활용할 수 있게 남기지만 intent_type은 unknown으로 내려 보낸다.
        return ResolvedToolCommand(
            "unknown",
            tool_id,
            confidence,
            True,
            confirm_text,
        )

    return ResolvedToolCommand(
        intent_type,
        tool_id,
        confidence,
        need_confirm,
        confirm_text,
    )


def _looks_like_fetch_request(text: str) -> bool:
    """문장에 fetch 요청처럼 보이는 표현이 있는지 확인한다."""

    normalized = text.casefold()
    return any(term in normalized for term in FETCH_REQUEST_TERMS)


def _looks_like_return_request(text: str) -> bool:
    """문장에 return 요청처럼 보이는 표현이 있는지 확인한다."""

    normalized = text.casefold()
    return any(term in normalized for term in RETURN_REQUEST_TERMS)
