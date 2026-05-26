# 용어 사전

> 이 프로젝트 문서·코드에 자주 등장하는 약어와 도메인 용어를 정리.
> 모르는 용어를 만나면 먼저 여기에서 검색. 누락된 용어는 PR로 추가.

---

## 로보틱스 / 시스템

| 용어 | 풀이름 / 의미 | 본 프로젝트에서 |
|------|---------------|-----------------|
| **BT** | Behavior Tree | Track A/B의 행동 결정 프레임워크 (py_trees) |
| **DOF** | Degrees of Freedom | e0509은 6-DOF |
| **DSR** | Doosan Robot SDK / Standard Robot interface | Doosan 좌표 제어 API (Track A) |
| **EE / TCP** | End Effector / Tool Center Point | 그리퍼 끝점 — 모든 픽업/거치 좌표 기준 |
| **E-stop** | Emergency Stop | 비상 정지. 응답 시간 ≤ 500ms (S-3) |
| **F/T** | Force/Torque (sensor) | 손목에 부착해 접촉력 측정. v1.0 미사용 (결정 #1) |
| **FK / IK** | Forward / Inverse Kinematics | joint ↔ Cartesian 변환 |
| **FOD** | Foreign Object Detection | 공구 분실/방치 감지. 임계 시간 초과 시 알림 (S-8) |
| **HAL** | Hardware Abstraction Layer | `hal/` 모듈 — Track A/B 전용 하드웨어 추상화 |
| **HRC** | Human-Robot Collaboration | 협동 로봇 환경 (cobot). Cartesian 속도 ≤ 250mm/s |
| **PLC** | Programmable Logic Controller | LED 상태 표시 + I/O. LS Electric XBC-DR10E, Modbus RTU/RS-485 (ADR-009) |
| **Reconciliation** | DB ↔ 실제 슬롯 상태 동기화 | 부팅 시 YOLOv8로 전체 스캔 (S-9) |
| **Staging Area** | 거치 영역 | 로봇이 공구를 놓는 중간 위치 — 운영자가 가져감. v1.0 핸드오버 대체 (S-6) |
| **TF** | Transform (ROS) | 좌표계 간 변환 트리. `tf2_ros`로 관리 |
| **udev** | Linux 디바이스 매니저 | USB 카메라/시리얼 포트 영구 이름 규칙 (`hardware.md`) |
| **URDF** | Unified Robot Description Format | 로봇 기하/관성 XML 모델 |

---

## AI / ML

| 용어 | 풀이름 / 의미 | 본 프로젝트에서 |
|------|---------------|-----------------|
| **demonstration / demo** | 사람이 시연한 (관측, 행동) 시퀀스 | Track C VLA 학습 데이터. 약 900개 필요 (ADR-004) |
| **end-to-end** | 입력→출력 단일 모델로 학습 | Track C의 핵심 (입력: RGB-D+텍스트, 출력: joint trajectory) |
| **fine-tuning** | 사전학습 모델을 도메인 데이터로 추가 학습 | Track C VLA 전략 (ADR-004) |
| **LLM** | Large Language Model | Gemma 4 — Track A/B 의도 분류 |
| **RL** | Reinforcement Learning | Track B 모션 정책 |
| **Sim-to-real** | 시뮬레이션에서 학습 → 실기 전이 | Track B RL 학습 전략 (미결 #24) |
| **STT** | Speech-to-Text | Whisper small (결정 #21) |
| **VAD** | Voice Activity Detection | 음성 구간 감지. 오디오 게이팅에 사용 (결정 #29) |
| **VLA** | Vision-Language-Action model | Track C — OpenVLA/π0 등 (미결 #5) |
| **YOLOv8** | Object detection model | 공구 분류 + 슬롯 위치 검출 |

---

## ROS2 / 미들웨어

| 용어 | 풀이름 / 의미 | 본 프로젝트에서 |
|------|---------------|-----------------|
| **action** | 비동기 장기 동작 (goal/feedback/result) | 6종 unit action (MoveToPose, Grasp …) |
| **colcon** | ROS2 빌드 도구 | `colcon build`, `colcon test` |
| **DDS** | Data Distribution Service | ROS2 통신 미들웨어 |
| **launch** | 다중 노드 시작 스크립트 | `ros2 launch <pkg> <file>` |
| **msg / srv / action** | 메시지 / 서비스 / 액션 정의 | `interfaces/` 패키지 |
| **node** | ROS2 실행 단위 | 패키지마다 여러 개 |
| **QoS** | Quality of Service | Reliable/BestEffort, depth — `interfaces.md` §4 |
| **rclpy** | ROS Client Library (Python) | Track A/B만 사용. `unit_actions/`·`track_c_vla.py`는 import 금지 (E-2) |
| **rosbag** | 토픽 녹화 도구 | 디버깅 + Track C demo 수집 |
| **topic** | pub/sub 통신 채널 | 예: `/voice/raw_text` |

---

## 캘리브레이션 / 좌표

| 용어 | 의미 |
|------|------|
| **base_link** | 로봇 베이스 좌표계 (REP-103, x: forward, y: left, z: up) |
| **camera_optical_frame** | 카메라 내부 좌표계 (REP-103 광학 규약, z: 광축) |
| **eye-in-hand** | 카메라가 엔드이펙터에 부착된 구성 (본 프로젝트) |
| **eye-to-hand** | 카메라가 외부 고정된 구성 (사용 안 함) |
| **hand-eye calibration** | 카메라 ↔ EE 변환 행렬 추정 (`config/hand_eye.yaml`) |
| **pre-grasp** | grasp 직전의 안전 진입 포즈 (보통 목표 위 100mm) |
| **REP-103** | ROS Enhancement Proposal — 좌표/단위 표준 |
| **TCP** | Tool Center Point (≠ TCP/IP). 엔드이펙터 작업 기준점 |

---

## DB / 데이터

| 용어 | 의미 |
|------|------|
| **append-only** | UPDATE/DELETE 금지, INSERT만 허용. `tool_events` 테이블 |
| **DB Gate** | 명령 실행 전 DB 가용성 확인 (S-2) |
| **enum** | 허용 값이 한정된 문자열 컬럼 (CHECK 제약) |
| **idempotent** | 여러 번 실행해도 결과 동일. 마이그레이션의 필수 속성 |
| **migration** | 스키마 변경 SQL 파일 (`db_core/migrations/`) |
| **WAL** | Write-Ahead Logging — SQLite의 동시성 모드. 본 프로젝트 DB 엔진으로 확정 (ADR-008) |

---

## 안전 / 운영

| 용어 | 의미 |
|------|------|
| **DB Gate** | DB 확인 후 명령 실행 (S-2) |
| **SafetyValidator** | VLA 출력 검증기 (S-1, ADR-005) |
| **SafetyWatchdog** | 하트비트 감시. 500ms 타임아웃 시 자동 정지 (S-4) |
| **soft limit** | 소프트웨어로 제한하는 joint 범위 — 하드웨어 리밋보다 좁음 (S-5) |
| **track switching** | VRAM 제약으로 트랙을 하나씩만 실행 (결정 #10, ADR-011) |
| **wake word** | 음성 명령 시작을 알리는 키워드 (false positive 방지) |

---

## 프로젝트 / 프로세스

| 용어 | 의미 |
|------|------|
| **ADR** | Architecture Decision Record — `docs/adr/` (인덱스: `adr/index.md`) |
| **CHANGELOG** | 변경 이력 (Keep a Changelog 형식). P-2 |
| **Conventional Commits** | 커밋 메시지 표준. P-4 |
| **Keep a Changelog** | CHANGELOG 작성 표준 ([keepachangelog.com](https://keepachangelog.com/)) |
| **Phase 0–9** | 개발 단계. `robot-arm-project.md` |
| **PR** | Pull Request |
| **Track A/B/C** | 3가지 제어 방식. README §시스템 구성 |
| **v1.0 / v2.0+** | 릴리스 버전. v2.0+ 기능은 v1.0 금지 (S-6) |

---

## AI 에이전트 / 도구

| 용어 | 의미 |
|------|------|
| **agent** | 특정 역할 전용 AI (`.claude/agents/`) — robot-arm-planner, safety-reviewer, interface-guardian |
| **CLAUDE.md** | Claude Code가 자동 로드하는 프로젝트 설명서 |
| **Claude Code** | Anthropic의 터미널 코딩 어시스턴트 (권장 도구) |
| **Codex CLI** | OpenAI의 터미널 코딩 에이전트 (AGENTS.md 기반, 대안 가능) |
| **Cursor** | 에디터 통합 AI (rules 정도만 호환) |
| **hook** | 도구 실행 전후 자동 명령 (`.claude/settings.json`) |
| **OMC** | oh-my-claudecode 플러그인 (선택, 부가 기능) |
| **rules** | AI 행동 규칙 (`.claude/rules/`) |
| **skill** | 재사용 가능한 전문 지식 모음 (`.claude/skills/`) |

---

## 추가 / 수정

용어 추가 시:
- 알파벳/한글 순서 유지
- 가능한 한 본 프로젝트에서의 의미를 함께 표기
- 외부 표준이면 출처 링크 (REP-103, Keep a Changelog 등)
