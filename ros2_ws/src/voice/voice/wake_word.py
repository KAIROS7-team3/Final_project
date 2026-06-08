"""선택형 wake-word gate.

운영 중 주변 대화가 명령으로 들어오는 것을 줄이기 위해 "로봇 ..."처럼
정해진 호출어로 시작하는 문장만 통과시킬 수 있다.

예:
    require_wake_word=False: "스패너 가져와" -> 통과
    require_wake_word=True:  "스패너 가져와" -> 차단
    require_wake_word=True:  "로봇 스패너 가져와" -> "스패너 가져와"로 통과
"""

from __future__ import annotations

from dataclasses import dataclass
import re

DEFAULT_WAKE_WORDS: tuple[str, ...] = (
    "코봇",
    "코 봇",
    "코보",
    "코 보",
    "코부",
    "코 부",
    "코브",
    "코 브",
    "코버",
    "코 버",
    "고봇",
    "고 봇",
    "고보",
    "고 보",
    "고부",
    "고 부",
    "고브",
    "고 브",
    "고버",
    "고 버",
    "코벗",
    "코 벗",
    "고벗",
    "고 벗",
    "꼬보",
    "꼬 보",
    "꼬부",
    "꼬 부",
    "코보트",
    "코 보트",
    "고보트",
    "고 보트",
)


@dataclass(frozen=True)
class WakeWordResult:
    """Wake-word 검사 결과와 실제 명령 본문.

    accepted=False이면 command_text는 빈 문자열이다.
    accepted=True이면 command_text에는 wake word가 제거된 명령만 들어간다.
    """

    accepted: bool
    command_text: str


def apply_wake_word_gate(
    text: str,
    wake_words: list[str],
    require_wake_word: bool,
) -> WakeWordResult:
    """Wake word 설정을 적용하고, 통과한 경우 호출어를 제거한다."""

    normalized = text.strip()
    if not require_wake_word:
        # 테스트/개발 중에는 호출어 없이 바로 명령을 넣는 것이 편하다.
        return WakeWordResult(True, normalized)

    lowered = normalized.lower()
    for wake_word in wake_words:
        stripped_wake_word = wake_word.strip()
        candidate = stripped_wake_word.lower()
        if not candidate:
            # 빈 wake word가 설정에 들어와도 모든 문장을 통과시키지 않도록 무시한다.
            continue
        # startswith를 써서 "로봇 스패너 가져와" 형태만 허용한다.
        if lowered.startswith(candidate):
            return WakeWordResult(True, _strip_wake_word_prefix(normalized, stripped_wake_word))

    return WakeWordResult(False, "")


def _strip_wake_word_prefix(text: str, wake_word: str) -> str:
    """wake word가 잡힌 경우 command 본문만 남긴다.

    단일 토큰 wake word의 경우 STT가 `코부츠`처럼 끝을 덧붙여도 첫 토큰 전체를
    제거해 잔여 음절이 남지 않게 한다.
    """

    stripped_text = text.lstrip()
    stripped_wake_word = wake_word.strip()
    if not stripped_wake_word:
        return stripped_text

    if " " in stripped_wake_word:
        remainder = stripped_text[len(stripped_wake_word) :]
        return remainder.lstrip(" \t\n\r,，.。!?！？-")

    index = 0
    while index < len(stripped_text) and _is_token_char(stripped_text[index]):
        index += 1
    while index < len(stripped_text) and not _is_token_char(stripped_text[index]):
        index += 1
    return stripped_text[index:].strip()


def _is_token_char(character: str) -> bool:
    """wake word 토큰의 일부로 볼 문자만 남긴다."""

    return bool(re.match(r"[0-9A-Za-z가-힣]", character))
