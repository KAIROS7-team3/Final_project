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

---

## ADR-013: Track B RL 시뮬레이션 환경 — Isaac Sim (Isaac Lab)

- **결정**: Isaac Sim (NVIDIA Isaac Lab) 채택 (#23 결정, 2026-05-27)
- **이유**: GPU 병렬 환경으로 RL sample 수집 throughput 우위. Vector 16 HX GPU 활용 가능. Doosan e0509 URDF 임포트 지원. Omniverse Replicator 연동 가능(#35).
- **운영 분리**: Gazebo Classic 11은 BT 골든 파일 회귀 테스트 전용 유지. Isaac Sim은 Track B RL 학습 전용 환경.
- **주의**: 설치 용량 ~50GB. Omniverse 라이센스 필요 (무료 연구용).

---

## ADR-014: RL Sim-to-real 전략 — DR+SI 하이브리드

- **결정**: System Identification + Domain Randomization 하이브리드 (#24 결정, 2026-05-27)
- **절차**:
  1. **SI**: 실측 e0509 joint 마찰·관성 파라미터 측정 → `config/sim_params.yaml`에 반영
  2. **DR**: 측정값 기준 ±20% 범위 랜덤화 + 조명·텍스처·페이로드 랜덤화
  3. 정책 배포 후 실제 하드웨어 성능 gap 측정 → fine-tuning 여부 판단
- **이유**: 순수 DR은 실제 파라미터 편차가 클 경우 성능 저하 위험. SI 기반 DR은 가장 안정적인 sim-to-real 전이 전략.

---

## ADR-015: Track B RL 학습 전략 — 시뮬 전용 우선

- **결정**: 시뮬 전용 (Pure RL) 우선 적용 (#6 결정, 2026-05-27)
- **이유**: 먼저 시뮬 전용으로 시도하고 성능 결과를 보고 Demo+RL 도입 여부를 판단한다. fetch/return 작업은 보상 설계가 명확하므로 순수 RL도 수렴 가능성 있음.
- **전환 조건**: 시뮬 전용 RL이 G5b 수락 기준(9종 × 3 사이클 100% 성공) 달성 실패 시 Demo+RL (BC 사전학습 + RL fine-tuning)로 전환. 이 경우 Track C demo 900개를 BC 학습에 재활용.

---

## ADR-016: Omniverse Replicator 도입 범위 — YOLOv8 한정

- **결정**: Omniverse Replicator를 YOLOv8 합성 데이터 증강에만 한정 도입 (#35 결정, 2026-05-27)
- **적용 범위**:
  - ✅ YOLOv8 학습용 합성 데이터 생성 (조명·배경·공구 자세 랜덤화)
  - ❌ RL 학습 도메인 랜덤화 → Isaac Lab 내장 DR 기능 사용
  - ❌ VLA fine-tuning → 실제 demonstration 데이터 사용
- **이유**: v1.0 범위 내 파이프라인 복잡성 통제. RL과 VLA에 Replicator를 추가 적용할 경우 의존성 및 유지 비용이 과도하게 증가.

---

## 미결 사항

| # | 항목 | 결정 시점 |
|---|------|----------|
| 4 | Gemma 4 파인튜닝 필요 여부 | Phase 5a 전 |
| 5 | Track C VLA 모델 선정 (OpenVLA / π0 / 기타) | Phase 6 전 |
| 22 | VLA 입력 형식 (단일 프레임 vs 시퀀스) | 모델 선정 후 (#5) |
