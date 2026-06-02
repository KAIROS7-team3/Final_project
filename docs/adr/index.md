# ADR 인덱스

> ADR 전체 목록. 상세 내용은 카테고리 파일 참조.
> 새 ADR 추가 시 이 파일과 해당 카테고리 파일을 함께 갱신.

---

## ADR 카탈로그

| 번호 | 제목 | 카테고리 | 상태 |
|------|------|---------|------|
| ADR-001 | 모션 제어 백엔드 — 3-트랙 비교 | [아키텍처](architecture.md) | 확정 |
| ADR-002 | 공구 인식 — YOLOv11s + 깊이 포즈 | [AI/ML](ai-ml.md) | 확정 |
| ADR-003 | 행동 프레임워크 (Track A/B) — py_trees | [AI/ML](ai-ml.md) | 확정 |
| ADR-004 | Track C VLA 전략 — 파인튜닝 | [AI/ML](ai-ml.md) | 확정 |
| ADR-005 | VLA 안전 경계 — SafetyValidator 필수 | [안전](safety.md) | 확정 |
| ADR-006 | Unit Action Library — 순수 Python | [아키텍처](architecture.md) | 확정 |
| ADR-007 | Track A/B 의도 LLM — Gemma 4 로컬 | [AI/ML](ai-ml.md) | 확정 |
| ADR-008 | DB 엔진 — SQLite WAL | [데이터](data.md) | 확정 |
| ADR-009 | PLC 프로토콜 — Modbus RTU (RS-485) | [하드웨어](hardware.md) | 확정 |
| ADR-010 | Doosan 제어 인터페이스 — 트랙별 분리 | [하드웨어](hardware.md) | 확정 |
| ADR-011 | GPU VRAM 예산 — 트랙별 전환 | [아키텍처](architecture.md) | 확정 |
| ADR-012 | 인터페이스 명명/구조 통일 | [인터페이스](interfaces.md) | 확정 |
| ADR-013 | Track B RL 시뮬레이션 환경 — Isaac Sim (Isaac Lab) | [AI/ML](ai-ml.md) | 확정 |
| ADR-014 | RL Sim-to-real 전략 — DR+SI 하이브리드 | [AI/ML](ai-ml.md) | 확정 |
| ADR-015 | Track B RL 학습 전략 — 시뮬 전용 우선 | [AI/ML](ai-ml.md) | 확정 |
| ADR-016 | Omniverse Replicator 도입 범위 — YOLOv11s 한정 | [AI/ML](ai-ml.md) | 확정 |

---

## 미결 사항

| # | 항목 | 결정 시점 | 카테고리 |
|---|------|----------|---------|
| 4 | Gemma 4 파인튜닝 필요 여부 | Phase 5a 전 | [AI/ML](ai-ml.md) |
| 5 | Track C VLA 모델 선정 (OpenVLA / π0 / 기타) | Phase 6 전 | [AI/ML](ai-ml.md) |
| 7 | 안전 E-Stop v2.0 적용 시점 | Phase 7 이후 | [안전](safety.md) |
| 22 | VLA 입력 형식 (단일 프레임 vs 시퀀스) | 모델 선정 후 | [AI/ML](ai-ml.md) |
| 33 | Wake word 감지 방식 (항상-on STT / 키워드 모델 / 물리 버튼) | Phase 4 전 | [안전](safety.md) |
| 34 | 로그 파일 보존 기간 및 외부 집계 도구 (ELK / Loki / 없음) | Phase 7 전 | [데이터](data.md) |

---

## 확정된 결정 (히스토리)

> 상세 ADR 없이 확정된 소규모 결정들.

| # | 항목 | 결정 내용 |
|---|------|-----------|
| 1 | F/T 센서 | v1.0 미사용. 그리퍼 파지력·YOLOv11s으로 대체. v2.0+ 검토 가능 |
| 2 | PLC 모델 + 프로토콜 | LS Electric XBC-DR14E, Modbus RTU via RS-485 (→ ADR-009) |
| 3 | DB 엔진 | SQLite WAL 모드 (→ ADR-008) |
| 8 | FOD 대출 임계 시간 | 기본값 10분, `config/fod.yaml`로 조정 가능 |
| 9 | Track C 핸드오버 비전 감지 | v2.0+로 이동 |
| 10 | 트랙 간 하드웨어 접근 | 트랙 하나씩 전환, Track Selector가 ROS2 종료 책임 |
| 11 | Track C grasp 포즈 획득 | VLA 모델 출력에 포함 |
| 12 | DB 장애 시 폴백 | 캐시 5분 TTL, 진행 중 작업 완료 후 새 명령 차단 |
| 13 | Staging Area 거치 후 로봇 동작 | 홈 복귀 + LED 초록 |
| 14 | Track A vs B 병행 여부 | 둘 다 개발 |
| 15 | GPU VRAM 전략 | 트랙 하나씩 전환 (→ ADR-011) |
| 16 | 팀 구성 | 3–5명, `robot-arm-project.md` "팀 구성 권장안" 참조 |
| 17 | Track C 그리퍼 제어 | VLA 출력에 gripper command 포함 |
| 18 | Staging Area 복수 슬롯 | 공구별 지정 슬롯 (`config/staging_area.yaml`) |
| 19 | 동작 중 음성 명령 | 이동 중 무시, 홈 복귀 후 수신 재개 |
| 20 | Operator ID | v1.0 고정값 `'operator_01'` |
| 21 | Whisper 모델 크기 | small, 결과 불량 시 medium 업그레이드 |
| 25 | hal/ vs Doosan SDK | Track C는 hal/ 우회, Doosan Python SDK 직접 |
| 26 | Staging Area 무단 회수 감지 | 주기적 YOLOv11s vision 확인 (idle 시) |
| 27 | grasp_planner 통합 위치 | 별도 PlanGrasp BT 노드 |
| 28 | 운영자 피드백 채널 | PLC LED만 사용 (v1.0) |
| 29 | 오디오 게이팅 방식 | 소프트웨어 게이팅 (VAD + is_moving 플래그) |
| 30 | Track C DB 가용성 체크 | VLA 추론 전 Python 코드가 `db_core/` 직접 쿼리 |
| 31 | Return 명령 feasibility | `staged`만 허용, 나머지 차단 |
| 32 | Track C parse_intent() | 한국어 키워드 테이블 파서. 추후 Gemma 4 전환 검토 가능 |
