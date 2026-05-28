# Voice-Commanded Tool Retrieval — 개발 일정 계획

> Updated: 2026-05-23
> **이 파일은 개발 Phase 계획 전용이다. 시스템 설계는 `docs/`를 참조.**

---

## 빠른 참조

| 문서 | 내용 |
|------|------|
| [`docs/architecture.md`](docs/architecture.md) | 시스템 설계, 트랙 구조, 패키지, DB 스키마, 안전 아키텍처 |
| [`docs/adr/index.md`](docs/adr/index.md) | ADR 인덱스, 미결 사항, 확정된 설계 결정 |
| [`docs/conventions.md`](docs/conventions.md) | 수락 기준, 네이밍, 설정 파일 위치, 검증 파이프라인 |
| [`docs/interfaces.md`](docs/interfaces.md) | 인터페이스 계약 (topic/service/action QoS·frame_id·timestamp) |
| [`docs/frames.md`](docs/frames.md) | TF tree, 좌표계 규약 (REP-103), hand-eye 변환 |
| [`docs/hardware.md`](docs/hardware.md) | 하드웨어 인벤토리, 드라이버 버전, udev 규칙, 캘리브레이션 |
| [`docs/simulation.md`](docs/simulation.md) | Gazebo 시뮬레이션 설정, BT 골든 파일 회귀, pass/fail 기준 |
| [`.claude/rules/safety.md`](.claude/rules/safety.md) | 안전 규칙 (최우선) |
| [`.claude/rules/engineering.md`](.claude/rules/engineering.md) | 엔지니어링 규칙 |
| [`.claude/rules/process.md`](.claude/rules/process.md) | 프로세스 규칙 |

---

## 프로젝트 개요

음성 명령으로 공구함에서 공구를 꺼내 **Staging Area**에 거치하고, 반납 명령 시 슬롯에 되돌려놓는 시스템. 세 가지 제어 트랙(A: Gemma 4+BT+DSR, B: Gemma 4+BT+RL, C: VLA end-to-end)을 병행 구현해 Phase 7에서 비교 평가한다.

- **하드웨어**: Doosan e0509 · RH-P12-RN · RealSense D455f · PLC
- **개발 머신**: Vector 16 HX AI A2XWIG (주) · HP ProBook 450 G10 (보조)
- **실행**: `./run.sh --track [A|B|C]` — 트랙은 하나씩 전환 (VRAM 제약)

---

## 팀 구성 권장안 (3–5명)

| 역할 | 담당 패키지 / 모듈 | 선행 조건 |
|------|-------------------|-----------|
| **인프라** | `interfaces`, `hal`, `unit_actions/`, DB 스키마, CI | — |
| **퍼셉션** | `vision` (YOLOv11s, Pose, Tracker), 데이터셋 수집 | interfaces 동결 |
| **음성/LLM** | `voice` (Whisper, Gemma 4), Track C VLA 전체 | interfaces 동결, HAL 동결 |
| **제어 A** | `motion/dsr_controller`, `orchestrator` (BT), `plc` | interfaces, unit_actions 동결 |
| **제어 B** | `motion/rl_policy`, RL 시뮬레이션, `db` | interfaces, unit_actions 동결 |

> 3명일 경우: 인프라+퍼셉션 / 음성LLM+TrackC / 제어A+B 통합.

### 확정된 5명 분배 (v1.0)

| 담당자 | 패키지 / 모듈 | 비고 |
|--------|---------------|------|
| **A** | `db`, `plc`, `voice` (Whisper + Gemma 4) | 음성과 DB/PLC를 단일 담당 |
| **B** | `motion` (DSR + RL), Isaac Sim/Lab | Track A/B 모션 + RL 시뮬 환경 |
| **C** | `vision` (YOLOv11s, Pose, Tracker) | 데이터셋 수집·학습 포함 |
| **D** | `orchestrator` (BT), `interfaces`, **공통 인프라** | CI, ROS_DOMAIN_ID, systemd, HAL 스켈레톤, unit_actions/ 시그니처 동결 책임 |
| **E** | Track C VLA 전체 (`track_c_vla.py`, demo 수집, fine-tuning) | Phase 4 안정화 후 본격 수집 |

> **공통 인프라 책임**: D가 단일 책임자(설계 결정). Phase 0 ① interfaces / ③ unit_actions 동결 + CI / ROS_DOMAIN_ID / systemd / 통합 launch 구성을 모두 D가 주관. HAL 인터페이스 서명은 D가 정의, 구현은 B가 모션 드라이버 작업과 함께 채움.
>
> **Docker는 v1.0 계획에서 제외** (네이티브 Ubuntu 22.04 + ROS2 Humble로 통합 운영). 추후 배포 단순화 필요 시 재검토.

---

## 개발 단계

### Phase 0: 환경 구성 + 공유 기반 (1–2주)

> **병렬 개발 시작 조건:** ①②③ 완료 후 각 팀 독립 작업 가능.

**① interfaces 정의 및 동결 (최우선)**
- [x] `interfaces` 패키지: 전체 msg/srv/action 정의 (4 msg · 2 srv · 6 action)
- [x] interfaces 동결 선언 — 이후 변경은 팀 합의 + `interfaces/CHANGELOG.md` 갱신 필수 (v0.1.0 — 2026-05-27)

**② HAL 인터페이스 정의**
- [x] HAL 스텁: `SimulatedArm`, `SimulatedGripper`, `SimulatedCamera` (F/T 센서는 v1.0 미사용 — ADR #1)
- [x] HAL 인터페이스 서명 동결 (`hal/arm_interface.py`, `hal/gripper_interface.py`, `hal/camera_interface.py`)

**③ unit_actions/ 인터페이스 정의**
- [x] `unit_actions/` 순수 Python 모듈 스켈레톤 (7개 모듈: grasp, move_to_pose, place_at_staging, pick_from_staging, release, return_to_slot, scan_workspace)
- [x] 함수 시그니처 동결 + 단위 테스트 (`unit_actions/tests/test_unit_actions.py`)

**① ② ③ 완료 후 병행 작업:**
- [x] ROS2 Humble 워크스페이스 구성 (`ros2_ws/src/` 7패키지 디렉토리 + interfaces 패키지 빌드 가능)
  - [x] `orchestrator` 패키지 스켈레톤: ament_python 빌드 설정 + Blackboard 스키마 + orchestrator_node / unit_action_server / bt_nodes 스텁 (BT 서브트리 구현은 Phase 5a) — **담당: D** (2026-05-27)
- [x] Doosan e0509 URDF/XACRO + Gazebo 씬 (doosan-robot2 서브모듈, dsr_moveit_config_e0509)
- [x] DB 스키마 생성 + SQLite WAL 설정 (`db_core/schema.py`)
- [x] ROS2 미들웨어 격리: `ROS_DOMAIN_ID` + `RMW_IMPLEMENTATION` + `ROS_LOCALHOST_ONLY` (`.env.example` + `run.sh`)
  - Doosan 컨트롤러 TCP 통신과 무관, HP ProBook rqt 모니터링 호환 (`ROS_LOCALHOST_ONLY=0`)
  - 같은 LAN의 다른 ROS2 프로젝트와 토픽 충돌 방지
- [ ] 공구 이미지 데이터셋 수집 시작 (YOLOv11s 학습까지 시간 필요) — **담당: C**
- [x] Track C: VLA demonstration 수집 환경 구성 (`track_c_vla.py` 골격 + `run.sh --track C`)

**병렬 개발 인프라:**
- [x] CI 설정: 빌드 + 단위 테스트 자동화 (`.github/workflows/ci.yml`)
- [x] `mocks/SPEC.md`: SimulatedArm/Gripper/Camera 동작 명세
- [ ] 공용 테스트 픽스처: 샘플 음성 파일 9종 — **담당: A** / DB seed 데이터 — **담당: D**
- [x] `interfaces/CHANGELOG.md` 생성
- [x] 통합 빌드 주기 결정 — **주 1회 (확정, 2026-05-27)**
- [x] `config/` 공유 디렉토리 구성:
  - `staging_area.yaml`, `toolbox.yaml`, `hand_eye.yaml`
  - `robot_poses.yaml`, `fod.yaml` (기본값: `checkout_timeout_minutes: 10`), `runtime.yaml`

> **Phase 0 미완료 항목**: ① 공구 이미지 데이터셋 수집 (담당: C) ② 음성/DB seed 픽스처 (담당: A, D). 모두 담당자에게 위임 — 작업 진행 상황은 주간 통합 회의에서 보고.

```bash
./run.sh --track A   # ROS2 full stack
./run.sh --track B   # ROS2 full stack + RL policy
./run.sh --track C   # ROS2 완전 종료 확인 후 track_c_vla.py 시작
```

### Phase 0.5: Track B 시뮬 환경 PoC (2주, Phase 0과 일부 병행)

> **목적**: Phase 5b 진입 전에 RL 학습 미결 사항을 조기 해소. 담당: B.
> **종료 조건**: 아래 4개 미결 사항 결정 + ADR 작성 + Phase 5b 진입 가능 판정.

- [x] **#23 결정**: Isaac Sim (Isaac Lab) 채택 — GPU 병렬 환경, Doosan e0509 URDF 지원 (ADR-013, 2026-05-27)
- [x] **#24 결정**: DR+SI 하이브리드 — SI 실측 후 ±20% DR 적용 (ADR-014, 2026-05-27)
- [x] **#6 결정**: 시뮬 전용 (Pure RL) 우선, G5b 미달 시 Demo+RL 전환 (ADR-015, 2026-05-27)
- [x] **#35 결정**: Omniverse Replicator — YOLOv11s 합성 데이터 증강 한정 도입 (ADR-016, 2026-05-27)
- [x] PoC 결과 → 4개 ADR 작성 (`docs/adr/ai-ml.md` 갱신) + `docs/simulation.md` Isaac Sim 트랙 항목 확정

> **위험 완화**: 4개 미결 사항 중 하나라도 Phase 5b 진입 시점까지 미결정이면 Phase 5b 작업 중단 — Phase 5a (Track A)만 진행.

### Phase 1: 하드웨어 드라이버 (2–3주)

- [ ] doosan-robot2 드라이버 bring-up + 관절 상태 검증
- [x] realsense-ros D455f 노드: RGB + depth 스트림 검증
  - 완료: camera_node(ApproxTimeSyncer, fps/depth stats), verify_camera.py 실측(serial 342622300205, mean depth 0.955m)
  - vision 파이프라인 노드 구현 완료: yolo_node / pose_node / tracker_node / context_builder
  - 단위 테스트 29개 통과 (test/test_hand_eye_loader, test_context_builder, test_tracker)
- [ ] RH-P12-RN 그리퍼 드라이버 노드
- [ ] PLC 드라이버 노드 (XBC-DR10E, Modbus RTU): LED 쓰기 검증
- [ ] udev 규칙: `/dev/doosan`, `/dev/gripper`, `/dev/plc`
- [ ] Hand-eye 캘리브레이션 (`config/hand_eye.yaml`)
  - eye-to-hand, easy_handeye2 준비 완료 (scripts/calibrate_hand_eye.sh, launch/handeye_calibration.launch.py)
  - 실물 로봇 + CharUco 보드 준비 시 진행 (config/hand_eye.yaml.example 참조)

### Phase 2: 공유 퍼셉션 + 음성 (3–6주)

- [ ] Whisper STT 노드 (`voice` 패키지)
- [ ] 9종 공구 이미지 데이터셋 수집
- [ ] YOLOv11s 파인튜닝
- [ ] 6D 포즈 + 오브젝트 트래커 노드
- [ ] 반고정 슬롯 오정렬 보정 로직
- [ ] `context_builder.py`: Track A/B용 scene JSON (Track C 미사용)

### Phase 3: DB + PLC 연동 (4–5주, 병행)

- [ ] `db` 패키지: CheckToolFeasibility + UpdateToolStatus 서비스
- [ ] FOD 모니터: 대출 시간 초과 → 'missing' 알림
- [ ] 이벤트 로거: fetch/return/missing/fod_alert 기록
- [ ] `plc` 패키지: LED 상태 매퍼 (정적 상태 점등 검증)

### Phase 4: Staging Area 동작 (5–7주)

- [ ] Staging Area 거치 동작 검증 (`place_at_staging`, `pick_from_staging`)
- [ ] Staging Area 좌표 캘리브레이션 (`config/staging_area.yaml`)
- [ ] 예외 케이스: 파지 실패, Staging Area 장애물, 공구 낙하
- [ ] Staging Area 주기 vision 확인 (idle 시 YOLOv11s → DB 갱신)

### Phase 5a: Track A — Gemma 4 + BT + DSR (6–8주)

> Phase 4 안정화 후 시작. 담당: A(Gemma 4) + D(BT) + B(DSR). Track B와 무관하게 독립 진행.

**Gemma 4 의도 노드 (담당: A):**
- [ ] 로컬 Gemma 4 추론 설정
- [ ] 시스템 프롬프트: 의도 분류 + 공구 ID 해석
- [ ] DB 가용성 확인 연동
- [ ] 불가 명령: 운영자 안내 + DB 로그 + PLC 노랑 점멸

**Behavior Tree (담당: D):**
- [ ] FetchTool / ReturnTool 서브트리
- [ ] 에러 복구 서브트리
- [x] Blackboard 스키마: `{active_tool_id, tool_pose, staging_state, intent}` — `orchestrator/blackboard.py`에 정의 (Phase 0 스켈레톤에서 선행, 2026-05-27)

**모션 (담당: B):**
- [ ] Track A: DSRArmDriver → unit_action_server

### Phase 5b: Track B — RL 정책 + 정책 배포 (6–10주, Phase 0.5 통과 시에만)

> **진입 조건**: Phase 0.5에서 #6/#23/#24/#35 모두 결정 완료 + Track A 5a 진행 중.
> Phase 0.5 미통과 시 본 단계 생략 — Track A 단독 진행.

- [ ] Track B: RL 학습 (Phase 0.5에서 결정된 시뮬 환경에서 진행)
- [ ] Isaac Sim 채택 시 **Omniverse Replicator** 기반 합성 데이터 파이프라인 구축 (#35 채택 시)
- [ ] 정책 배포 노드 (RLPolicyDriver → unit_action_server)
- [ ] Sim-to-real 적용 (#24 결정 전략 따라)

### Phase 6: Track C — VLA (약 10–18주, Phase 4 안정화 후 본격 시작)

> **시작 시점**: Phase 0③ 완료 시점부터 환경/모델 선정 작업은 가능하나, **demonstration 본격 수집은 Phase 4 (Staging Area 동작) 안정화 후**.
> 이유: VLA demo는 fetch+return 전 사이클 녹화 → Staging Area 거치 동작이 검증되어야 의미 있는 데이터 확보.
> VRAM 제약으로 fine-tuning은 클라우드(Lambda Labs, Vast.ai 등) 필요. Phase 0 초기에 환경 확정.

**Phase 6a: Demonstration 수집 + Fine-tuning (4–8주)**
- [ ] VLA 모델 선정 (미결 #5 결정 후)
- [ ] Fine-tuning 인프라 확보 (클라우드 또는 별도 GPU)
- [ ] 9종 × fetch+return × 50 demo = 약 900 demonstration 녹화
- [ ] VLA 모델 fine-tuning 실행
- [ ] 추론 성능 초기 검증

**Phase 6b: 통합 구현 (6–10주)**
- [ ] `track_c_vla.py`: Doosan Python SDK + VLA 모델 직접 연동
- [ ] `parse_intent()` 키워드 파서 (9종 공구 한국어 키워드 테이블)
- [ ] `check_feasibility()` DB gate
- [ ] `is_moving` 플래그 관리
- [ ] VLA 입력 파이프라인: pyrealsense2 + Whisper 직접 호출
- [ ] Safety Validator: joint limit / 속도 한계 검증
- [ ] DB 클라이언트 (직접 SQL) + PLC 클라이언트 (직접 Modbus)

### Phase 7: 비교 평가 (12–14주)

| 시나리오 | Track A | Track B | Track C |
|----------|---------|---------|---------|
| Fetch — 명확한 공구 명 | 기준 | vs A | vs A |
| Fetch — 모호한 공구 명 | Gemma 4 확인 | Gemma 4 확인 | 키워드 파서 실패 → 거부 |
| Fetch — 대출 중인 공구 | Gemma 4 차단 | Gemma 4 차단 | Python gate 차단 |
| Return — 정확한 슬롯 반납 | 기준 | vs A | vs A |
| FOD 알림 (분실 초과) | DB/PLC 알림 | DB/PLC 알림 | DB/PLC 알림 |

### Phase 8: 테스트 (2–14주, 병행)

- [ ] 단위: 기구학, YAML 로더, Gemma 4 의도 정확도, DB CRUD, PLC 상태 매핑
- [ ] 단위 (Track C): VLA 추론 정확도, SafetyValidator
- [ ] 통합 (Track A/B): `launch_testing` — 음성 → 퍼셉션 → 모션
- [ ] 통합 (Track C): subprocess 기반 E2E 테스트
- [ ] 시뮬레이션: BT 골든 파일 회귀 + Track C trajectory 회귀
- [ ] HIL: 9종 × 3 fetch+return 사이클 (트랙별)
- [ ] 비교: Phase 7 시나리오 × 3 트랙 + 지표 기록

### Phase 9: 배포 (14–15주)

- [ ] systemd 서비스: 부팅 시 ROS2 스택 + Track C 자동 시작
- [ ] 의존성 lockfile + 배포 스크립트 (`requirements.txt`, `package.xml`, colcon install)
- [ ] ROS2 bag + DB 감사 로그 전 세션 보관
- [ ] HP ProBook: rqt 모니터링 대시보드 + DB 뷰어 + PLC 상태 패널

---

## 마일스톤 게이트 (Phase 진입 조건)

> 각 Phase는 이전 Phase의 **게이트 조건을 모두 만족**해야 진입한다. 게이트 미통과 시 후속 Phase 작업 금지 — 게이트 해소 작업이 우선.
> 게이트 판정은 주간 통합 회의에서 합의. 미통과 시 issue 생성 + 다음 회의까지 해소 책임자 명시.

| 게이트 | 진입 대상 | 조건 | 판정 책임 |
|--------|-----------|------|-----------|
| **G0 → 1** | Phase 1 | `interfaces` 동결 완료 + `unit_actions/` 시그니처 동결 + `interfaces/CHANGELOG.md` 생성 + CI 단위 테스트 통과 | D |
| **G0.5 → 5b** | Phase 5b | 미결 #6/#23/#24/#35 모두 ADR 작성 완료 + `docs/simulation.md` Isaac Sim 트랙 항목 확정 | B + 팀 합의 |
| **G1 → 2** | Phase 2 | 4종 드라이버(arm/gripper/camera/plc) bring-up 검증 + hand-eye 캘리브레이션 RMSE ≤ 5mm | B, C, A |
| **G2 → 3** | Phase 3 | Whisper STT 9종 키워드 인식률 ≥ 95% + YOLOv11s 9종 mAP ≥ 0.85 + 슬롯 보정 ±5mm | A, C |
| **G3 → 4** | Phase 4 | `CheckToolFeasibility` 9종 정상 응답 + FOD 시뮬 타임아웃 동작 + PLC 4색 점등 검증 | A |
| **G4 → 5a** | Phase 5a | Staging Area 거치 9종 × 3 사이클 전 통과 + 거치 오차 ±5mm + 예외 케이스 3종 복구 | D + B, C |
| **G4 → 6** | Phase 6 본격 수집 | G4 동일 + 9종 fetch+return 단일 사이클 시뮬레이션 검증 | E |
| **G5a → 7** | Phase 7 Track A 평가 | Track A 9종 × 3 사이클 fetch+return 100% 성공 (HIL) | D + B |
| **G5b → 7** | Phase 7 Track B 평가 | Track B 9종 × 3 사이클 fetch+return 100% 성공 (HIL) + sim-to-real gap 측정 | B |
| **G6 → 7** | Phase 7 Track C 평가 | Track C 9종 × 3 사이클 100% 성공 + SafetyValidator 통과 100% | E |
| **G7 → 9** | Phase 9 배포 | 3 트랙 비교표 작성 완료 + 회귀 테스트 자동화 통과 | 팀 합의 |

### 게이트 운영 규칙

- **G0.5 미통과**: Phase 5b(Track B) 작업 금지. Track A만 진행. Phase 7도 Track A/C 2-way 비교로 축소.
- **G2 미통과**: Phase 3 시작 보류. 인식률 미달 시 데이터셋 보강 → 재학습 → 재판정.
- **G4 미통과**: Phase 5a/5b/6 모두 보류. Staging Area는 모든 트랙의 공통 의존성.
- **게이트 통과는 영구적이지 않다**: 후속 Phase에서 회귀 발생 시 해당 게이트 재검증 필요.

---

## 테스트 전략

### 테스팅 피라미드

```
              ╱╲
             ╱  ╲         비교 E2E (3 트랙 × 9 공구 × 실 하드웨어)
            ╱────╲
           ╱      ╲        HIL (실 하드웨어, 통제된 씬)
          ╱────────╲
         ╱          ╲      시뮬레이션 (Gazebo BT 회귀 + Track C trajectory 회귀)
        ╱────────────╲
       ╱              ╲    통합 (launch_testing, 멀티 노드, 전 트랙)
      ╱────────────────╲
     ╱                  ╲  단위 (기구학, 감지, joint limit, LLM 의도)
    ╱____________________╲
```

### 핵심 테스트 기준

| 테스트 | 기준 |
|--------|------|
| Gemma 4 의도 정확도 | ≥ 97% |
| DB 가용성 차단 정확도 | 100% |
| FOD 알림 지연 | ≤ 30초 |
| 공구 분류 정확도 (YOLOv11s) | ≥ 95% |
| 포즈 추정 오차 | ≤ 5mm, ≤ 3° |
| Fetch 사이클 타임 (Track A/B) | ≤ 10초 |
| Fetch 사이클 타임 (Track C) | ≤ 13초 |
| E-Stop 응답 시간 | ≤ 500ms |
| Track C SafetyValidator 차단율 | 100% |
