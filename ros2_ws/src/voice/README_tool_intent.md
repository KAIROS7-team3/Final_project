# Tool Intent Parser

## 목적

`tool_intent` 모듈은 Whisper STT 결과가 발음이 비슷한 단어, 외국어, 일반 단어로 잘못 인식돼도 공구 운반 로봇이 6개 공구 중 하나를 안정적으로 고르도록 돕는 순수 Python 후처리 모듈이다.

지원하는 `tool_id`는 `screwdriver`, `utility_knife`, `ratchet_wrench`, `multi_tool`, `spanner_16mm`, `socket_19mm`, `unknown`이다.

## 파일 구조

```text
voice/
  tool_intent/
    __init__.py
    tool_aliases.py
    intent_parser.py
    gemma_prompt.py
test_tool_intent.py
README_tool_intent.md
```

- `tool_aliases.py`: 공구별 별칭, 오인식 표현, 숫자 힌트 데이터
- `intent_parser.py`: `parse_tool_intent(text)`와 `normalize_text(text)`
- `gemma_prompt.py`: 이후 Gemma 4 후보정 트랙에 넘길 JSON-only 프롬프트 생성
- `test_tool_intent.py`: 단독 실행 가능한 회귀 테스트

## 실행 방법

```bash
cd ~/Final_Project/ros2_ws/src/voice
python3 - <<'PY'
from voice.tool_intent import parse_tool_intent

print(parse_tool_intent("스페인어 가져와"))
PY
```

출력 예:

```python
{
    "tool_id": "spanner_16mm",
    "confidence": 0.62,
    "need_confirm": True,
    "confirm_text": "스패너 16mm로 이해했습니다. 맞으면 확인해 주세요.",
}
```

## 테스트 방법

```bash
cd ~/Final_Project/ros2_ws/src/voice
python3 test_tool_intent.py
```

pytest로도 실행할 수 있다.

```bash
cd ~/Final_Project/ros2_ws/src/voice
python3 -m pytest test_tool_intent.py
```

## ROS2 voice node import 예시

```python
from voice.tool_intent import build_gemma_prompt, parse_tool_intent


def handle_stt_text(stt_text: str) -> dict:
    parsed = parse_tool_intent(stt_text)
    if parsed["tool_id"] == "unknown" or parsed["need_confirm"]:
        prompt = build_gemma_prompt(stt_text)
        # Gemma 4 후보정 호출부에 prompt를 넘긴 뒤 JSON 결과만 사용한다.
        return {**parsed, "gemma_prompt": prompt}
    return parsed
```
