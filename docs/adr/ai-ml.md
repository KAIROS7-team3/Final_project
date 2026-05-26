# ADR — AI / ML

> 참조: [인덱스](index.md)

---

## ADR-002: 공구 인식 — YOLOv8 + 깊이 포즈

- **결정**: YOLOv8 + 깊이 기반 6D 포즈 추정
- **이유**: Vector 16 HX GPU 활용 가능, YAML 기반 공구 클래스 확장 용이
- **Track C 예외**: VLA 모델이 raw RGB-D를 직접 입력받아 end-to-end 처리

---

## ADR-003: 행동 프레임워크 (Track A/B) — py_trees

- **결정**: Behavior Tree (py_trees)
- **이유**: 모듈형 서브트리 독립 테스트 가능, 반응형 폴백 패턴 구현 용이

---

## ADR-004: Track C VLA 전략 — 파인튜닝

- **결정**: 사전학습 VLA 모델을 Doosan e0509 demonstration 데이터로 파인튜닝
- **후보**: OpenVLA, π0, Octo 등 (미결 #5에서 최종 선정)
- **출력**: joint trajectory + gripper command (연속값 0~1)
- **실행**: Doosan Python SDK 직접 제어 (ROS2 우회)
- **데이터**: 9종 × fetch+return × 50 demo = 약 900 demonstration (순수 녹화 ~7.5h, 실패 포함 20h+)
- **후속 조치**: Phase 6 전 모델 선정 + fine-tuning 인프라 확보 필수

---

## ADR-007: Track A/B 의도 LLM — Gemma 4 로컬

- **결정**: Gemma 4 로컬 추론
- **이유**: 프라이버시 보호, 모호한 명령 처리, DB 가용성 확인 연동
- **트레이드오프**: 규칙 기반 파서 대비 +~800ms 지연
- **참고**: 특정 버전 고정이 아님 — 후속 모델로 교체 가능 (아키텍처는 동일)

---

## 미결 사항

| # | 항목 | 결정 시점 |
|---|------|----------|
| 4 | Gemma 4 파인튜닝 필요 여부 | Phase 5 전 |
| 5 | Track C VLA 모델 선정 (OpenVLA / π0 / 기타) | Phase 6 전 |
| 6 | Track B RL 학습 전략 (시뮬 전용 vs Demo+RL) | Phase 5 전 |
| 22 | VLA 입력 형식 (단일 프레임 vs 시퀀스) | 모델 선정 후 (#5) |
| 23 | Track B RL 시뮬레이션 환경 (Isaac Sim vs MuJoCo) | Phase 5 전 |
| 24 | RL 학습 Sim-to-real 전략 | Phase 5 전 |
