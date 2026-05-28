# ADR — 시스템 아키텍처

> 참조: [인덱스](index.md)

---

## ADR-001: 모션 제어 백엔드 — 3-트랙 비교

- **결정**: Track A(DSR), Track B(RL), Track C(VLA) 병행 구현 후 Phase 7에서 비교
- **공유 기반**: 물리 하드웨어, `db_core/`, `plc_core/`, DB 스키마, PLC 프로토콜
- **Track A/B 전용**: `hal/`, `unit_actions/`
- **후속 조치**: Phase 7 비교 평가 후 주 백엔드 결정

---

## ADR-006: Unit Action Library — 순수 Python, Track A/B 전용

- **결정**: `unit_actions/` 순수 Python 모듈, Track A/B용 ROS2 어댑터 분리
- **이유**: Track A/B 비즈니스 로직 공유, ROS2 독립 단위 테스트 가능
- **제약**: `unit_actions/`에 `rclpy` import 금지 (E-2), Track C는 unit_actions 미사용

---

## ADR-011: GPU VRAM 예산 — 트랙별 전환

| 컴포넌트 | 추정 VRAM |
|----------|-----------|
| Whisper small | ~1 GB |
| YOLOv11ss | ~0.5 GB |
| Gemma 4 7B (Q4) | ~4–6 GB |
| **Track A 합계** | **~5.5–7.5 GB** |
| + RL policy | ~1 GB |
| **Track B 합계** | **~6.5–8.5 GB** |
| OpenVLA-7B (Q4) | ~4–5 GB |
| **Track C 합계 (Q4)** | **~5–6 GB** |

- VRAM 제약으로 트랙 동시 실행 불가 → 트랙 하나씩 전환
- Phase 0 초기에 `nvidia-smi`로 실제 VRAM 확인 권장
