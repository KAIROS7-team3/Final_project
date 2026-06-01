"""Track A/B 음성 처리 패키지.

이 패키지는 크게 두 단계로 나뉜다.

1. `whisper_node`
   마이크 입력을 Whisper STT로 변환해서 `/voice/raw_text`에 publish한다.
2. `rule_intent_node`
   raw text를 fetch/return/cancel intent로 해석하고 DB Gate를 통과한 명령만
   `/voice/intent`에 publish한다.
"""
