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
            # 원본 normalized 문자열에서 wake word 길이만큼 잘라 공백을 제거한다.
            return WakeWordResult(True, normalized[len(stripped_wake_word) :].strip())

    return WakeWordResult(False, "")
