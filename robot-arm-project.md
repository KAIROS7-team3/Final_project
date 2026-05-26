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

음성 명령으로 공구함에서 공구를 꺼내 **Staging Area**에 거치하고, 반납 명령 시 슬롯에 되돌려놓는 시스템. 세 가지 제어 트랙(A: Gemma4+BT+DSR, B: Gemma4+BT+RL, C: VLA end-to-end)을 병행 구현해 Phase 7에서 비교 평가한다.

- **하드웨어**: Doosan e0509 · RH-P12-RN · RealSense D455f · PLC
- **개발 머신**: Vector 16 HX AI A2XWIG (주) · HP ProBook 450 G10 (보조)
- **실행**: `./run.sh --track [A|B|C]` — 트랙은 하나씩 전환 (VRAM 제약)

---

## 팀 구성 권장안 (3–5명)

| 역할 | 담당 패키지 / 모듈 | 선행 조건 |
|------|-------------------|-----------|
| **인프라** | `interfaces`, `hal`, `unit_actions/`, DB 스키마, Docker | — |
| **퍼셉션** | `vision` (YOLOv8, Pose, Tracker), 데이터셋 수집 | interfaces 동결 |
| **음성/LLM** | `voice` (Whisper, Gemma 4), Track C VLA 전체 | interfaces 동결, HAL 동결 |
| **제어 A** | `motion/dsr_controller`, `orchestrator` (BT), `plc` | interfaces, unit_actions 동결 |
| **제어 B** | `motion/rl_policy`, RL 시뮬레이션, `db` | interfaces, unit_actions 동결 |

> 3명일 경우: 인프라+퍼셉션 / 음성LLM+TrackC / 제어A+B 통합.

---

## 개발 단계

### Phase 0: 환경 구성 + 공유 기반 (1–2주)

> **병렬 개발 시작 조건:** ①②③ 완료 후 각 팀 독립 작업 가능.

**① interfaces 정의 및 동결 (최우선)**
- [ ] `interfaces` 패키지: 전체 msg/srv/action 정의
- [ ] interfaces 동결 선언 — 이후 변경은 팀 합의 + `interfaces/CHANGELOG.md` 갱신 필수

**② HAL 인터페이스 정의**
- [ ] HAL 스텁: `SimulatedArm`, `SimulatedGripper`, `SimulatedCamera` (F/T 센서는 v1.0 미사용 — ADR #1)
- [ ] HAL 인터페이스 서명 동결

**③ unit_actions/ 인터페이스 정의**
- [ ] `unit_actions/` 순수 Python 모듈 스켈레톤 (mock 구현)
- [ ] 함수 시그니처 동결

**① ② ③ 완료 후 병행 작업:**
- [ ] ROS2 Humble 워크스페이스 구성
- [ ] Docker 개발 컨테이너 (GPU 패스스루)
- [ ] Doosan e0509 URDF/XACRO + Gazebo 씬
- [ ] DB 스키마 생성 + SQLite WAL 설정 (`db_core/schema.sql`)
- [ ] CycloneDDS 단일 머신 설정
- [ ] 공구 이미지 데이터셋 수집 시작 (YOLOv8 학습까지 시간 필요)
- [ ] Track C: VLA demonstration 수집 환경 구성

**병렬 개발 인프라:**
- [ ] CI 설정: 빌드 + 단위 테스트 자동화
- [ ] `mocks/SPEC.md`: SimulatedArm/Gripper 동작 명세
- [ ] 공용 테스트 픽스처: 샘플 음성 파일 9종, DB seed 데이터
- [ ] `interfaces/CHANGELOG.md` 생성
- [ ] 통합 빌드 주기 결정 (권장: 주 1회)
- [ ] `config/` 공유 디렉토리 구성:
  - `staging_area.yaml`, `toolbox.yaml`, `hand_eye.yaml`
  - `robot_poses.yaml`, `fod.yaml` (기본값: `checkout_timeout_minutes: 10`)

```bash
./run.sh --track A   # ROS2 full stack
./run.sh --track B   # ROS2 full stack + RL policy
./run.sh --track C   # ROS2 완전 종료 확인 후 track_c_vla.py 시작
```

### Phase 1: 하드웨어 드라이버 (2–3주)

- [ ] doosan-robot2 드라이버 bring-up + 관절 상태 검증
- [ ] realsense-ros D455f 노드: RGB + depth 스트림 검증
- [ ] RH-P12-RN 그리퍼 드라이버 노드
- [ ] PLC 드라이버 노드 (XBC-DR10E, Modbus RTU): LED 쓰기 검증
- [ ] udev 규칙: `/dev/doosan`, `/dev/gripper`, `/dev/plc`
- [ ] Hand-eye 캘리브레이션 (`config/hand_eye.yaml`)

### Phase 2: 공유 퍼셉션 + 음성 (3–6주)

- [ ] Whisper STT 노드 (`voice` 패키지)
- [ ] 9종 공구 이미지 데이터셋 수집
- [ ] YOLOv8 파인튜닝
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
- [ ] Staging Area 주기 vision 확인 (idle 시 YOLOv8 → DB 갱신)

### Phase 5: Track A/B — Gemma 4 + Behavior Tree (6–10주)

**Gemma 4 의도 노드:**
- [ ] 로컬 Gemma 4 추론 설정
- [ ] 시스템 프롬프트: 의도 분류 + 공구 ID 해석
- [ ] DB 가용성 확인 연동
- [ ] 불가 명령: 운영자 안내 + DB 로그 + PLC 주황 점멸

**Behavior Tree:**
- [ ] FetchTool / ReturnTool 서브트리
- [ ] 에러 복구 서브트리
- [ ] Blackboard 스키마: `{active_tool_id, tool_pose, staging_state, intent}`

**모션:**
- [ ] Track A: DSRArmDriver → unit_action_server
- [ ] Track B: RL 학습 (Isaac Sim / MuJoCo) + 정책 배포 노드

### Phase 6: Track C — VLA (약 10–18주, Phase 0③ 완료 후 즉시 시작)

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
| Fetch — 모호한 공구 명 | Gemma4 확인 | Gemma4 확인 | 키워드 파서 실패 → 거부 |
| Fetch — 대출 중인 공구 | Gemma4 차단 | Gemma4 차단 | Python gate 차단 |
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
- [ ] Docker Compose: STT / 퍼셉션 / Gemma 4 / VLA / 제어 / DB / PLC
- [ ] ROS2 bag + DB 감사 로그 전 세션 보관
- [ ] HP ProBook: rqt 모니터링 대시보드 + DB 뷰어 + PLC 상태 패널

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
| 공구 분류 정확도 (YOLOv8) | ≥ 95% |
| 포즈 추정 오차 | ≤ 5mm, ≤ 3° |
| Fetch 사이클 타임 (Track A/B) | ≤ 10초 |
| Fetch 사이클 타임 (Track C) | ≤ 13초 |
| E-Stop 응답 시간 | ≤ 500ms |
| Track C SafetyValidator 차단율 | 100% |
