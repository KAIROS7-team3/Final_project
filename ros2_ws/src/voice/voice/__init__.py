"""Track A/B 음성 처리 패키지.

이 패키지는 크게 두 단계로 나뉜다.

1. `whisper_node`
   마이크 입력을 Whisper STT로 변환해서 `/voice/raw_text`에 publish한다.
2. `gemma_intent_node`
   raw text를 Gemma 4 로컬 의도 분류로 해석하고 DB Gate를 통과한 명령만
   `/voice/intent`에 publish한다.

`rule_intent_node`는 baseline/rollback 용도의 후순위 경로로 남긴다.
"""
