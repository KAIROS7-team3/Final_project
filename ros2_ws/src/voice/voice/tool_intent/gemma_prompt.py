"""향후 Gemma 4 공구명 후보정에 사용할 prompt builder.

현재 기본 voice 경로는 deterministic parser를 사용한다. Gemma 4를 다시 붙일 때도
모델이 자유 문장을 생성하지 않고 정해진 JSON schema만 반환하도록 prompt를
강하게 제한해야 한다.
"""

from __future__ import annotations

# 모델이 선택할 수 있는 tool_id를 문자열로 고정한다. 이 목록 밖의 값을 만들면
# downstream DB Gate와 tool table이 해석할 수 없다.
TOOL_ID_SCHEMA = (
    "screwdriver | utility_knife | ratchet_wrench | multi_tool | "
    "spanner_16mm | socket_19mm | unknown"
)


def build_gemma_prompt(stt_text: str) -> str:
    """Gemma 4에 전달할 JSON-only prompt를 만든다."""

    # prompt 안에 실제 오인식 사례를 명시해, 모델이 "스페인어" 같은 일반 단어를
    # 그대로 번역하지 않고 프로젝트 공구 후보로 해석하도록 유도한다.
    return f"""너는 공구 운반 로봇의 음성 명령 해석기다.
사용자는 6개 공구 중 하나만 요청할 수 있다.
가능한 tool_id는 {TOOL_ID_SCHEMA} 뿐이다.
Whisper STT 결과는 틀릴 수 있다.
"스페인어"는 "스패너"일 수 있다.
"박스"는 "복스"일 수 있다.
"런치"는 "렌치"일 수 있다.
모르면 unknown으로 답한다.
애매하면 need_confirm=true로 답한다.
confidence는 0.0부터 1.0 사이 숫자로 답한다.
confirm_text는 확인이 필요할 때만 짧은 한국어 질문으로 채운다.
반드시 JSON만 출력한다.
설명 문장은 출력하지 않는다.

출력 형식:
{{
  "tool_id": "{TOOL_ID_SCHEMA}",
  "confidence": 0.0,
  "need_confirm": true,
  "confirm_text": ""
}}

Whisper STT 결과:
{stt_text!r}
"""
