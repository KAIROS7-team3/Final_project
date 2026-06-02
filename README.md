# Voice-Commanded Tool Delivery Robot Arm

> 음성 명령으로 공구함에서 공구를 꺼내 전달하고 반납 시 제자리에 돌려놓는 협동 로봇팔 시스템.
> **현재 단계:** Phase 1 진행 중 — D455f bring-up 완료, vision 파이프라인 뼈대 구현, Hand-eye 캘리브레이션 대기.

---

## 한 줄 요약

Doosan e0509 협동 로봇팔이 운영자의 음성 명령을 듣고 공구함의 9종 공구를 Staging Area에 거치/회수하는 시스템. 세 가지 제어 방식(BT+DSR, BT+RL, end-to-end VLA)을 동일 하드웨어에서 비교 평가한다.

## 누구를 위한 저장소인가

- **연구**: 구조화된 BT+unit_actions vs end-to-end 학습 기반 제어 비교
- **운영**: 실험실/소규모 정비소에서 공구 관리 자동화
- **교육**: 협동 로봇 + 비전 + LLM + 안전 시스템 통합 사례

---

## Quick Start

### 처음 클론한 팀원

| 순서 | 작업 | 상세 |
|------|------|------|
| 1 | AI 도구 설정 | [`docs/ai-setup.md`](docs/ai-setup.md) — Claude Code/Codex/Cursor 비교 + 설정 |
| 2 | 프로젝트 개요 파악 | [`CLAUDE.md`](CLAUDE.md) (Claude Code) 또는 [`AGENTS.md`](AGENTS.md) (Codex/Gemini) — 아키텍처 한눈에 보기 |
| 3 | Phase 일정 확인 | [`robot-arm-project.md`](robot-arm-project.md) — Phase 0–9 로드맵 |
| 4 | 환경 변수 설정 | `cp .env.example .env` → 값 채우기 (팀장에게 문의) |
| 5 | 룰 숙지 | [`.claude/rules/`](.claude/rules/) — safety/engineering/process |

### 실행 (코드 구현 후)

```bash
./run.sh --track A          # Gemma 4 + BT + DSR
./run.sh --track B          # Gemma 4 + BT + RL
./run.sh --track C          # 키워드 파서 + VLA (no ROS2)
./run.sh --track A --sim    # 시뮬레이션
./run.sh --help             # 전체 옵션
```

---

## 시스템 구성

### 하드웨어

| 구성요소 | 모델 |
|----------|------|
| 로봇팔 | Doosan e0509 (6-DOF 협동) |
| 그리퍼 | ROBOTIS RH-P12-RN |
| 카메라 | Intel RealSense D455f (eye-to-hand, 탑뷰 고정) |
| PLC | LS Electric XBC-DR14E |
| 메인 PC | Vector 16 HX AI A2XWIG |
| 보조 PC | HP ProBook 450 G10 (모니터링) |

상세 → [`docs/hardware.md`](docs/hardware.md)

### 소프트웨어 스택

| 레이어 | Track A/B | Track C |
|--------|-----------|---------|
| OS | Ubuntu 22.04 | 동일 |
| 미들웨어 | ROS2 Humble | (없음 — 단일 Python) |
| STT | Whisper small | 동일 |
| 의도 분류 | Gemma 4 (로컬) | Python 키워드 파서 |
| 모션 | DSR 좌표 제어 (A) / RL 정책 (B) | VLA 모델 end-to-end |
| 비전 | YOLOv8 + 6D pose | VLA 입력 (raw RGB-D) |
| 하드웨어 인터페이스 | `doosan-robot2` ROS2 드라이버 | Doosan Python SDK 직접 |

---

## 저장소 구조

```
.
├── README.md                  ← 이 파일
├── CLAUDE.md                  ← Claude Code 사용자용 프로젝트 개요
├── AGENTS.md                  ← Codex/Gemini CLI 사용자용 미러 (동일 룰 요약)
├── robot-arm-project.md       ← Phase 0–9 개발 계획
├── run.sh                     ← Track Selector
├── architecture.html          ← 인터랙티브 시각화
├── docs/                      ← 모든 설계 문서
│   ├── architecture.md            아키텍처 + 패키지 구조
│   ├── adr/                       ADR (카테고리별) + 미결 사항 — index.md로 시작
│   ├── conventions.md             네이밍 + 수락 기준
│   ├── interfaces.md              msg/srv/action 계약
│   ├── frames.md                  좌표계 + TF tree
│   ├── hardware.md                하드웨어 인벤토리
│   ├── simulation.md              Gazebo 골든 파일 회귀 + Isaac Sim (Track B RL)
│   ├── db-schema.md               DB 논리 스키마
│   ├── ai-setup.md                AI 도구 설정 안내
│   ├── logging.md                 로깅 인프라 (파일 위치, 순환, DB 이벤트)
│   ├── demo-collection-workflow.md  Track C demonstration 수집 워크플로우
│   └── glossary.md                용어 사전
├── config/                    ← 비-시크릿 운영 파라미터
│   ├── runtime.yaml               robot_model, whisper_size 등
│   ├── staging_area.yaml          공구별 거치 좌표
│   ├── toolbox.yaml               슬롯 + 공구 카탈로그
│   ├── hand_eye.yaml              카메라-EE 변환
│   ├── robot_poses.yaml           home/scan 포즈
│   └── fod.yaml                   FOD 임계
├── interfaces/                ← ROS2 msg/srv/action (코드 구현 시)
│   └── CHANGELOG.md
├── .env.example               ← 시크릿 변수 템플릿 (값 없음)
├── .claude/                   ← AI 에이전트 설정 (Claude Code 기준)
│   ├── rules/                     safety/engineering/process 룰
│   ├── agents/                    robot-arm-planner/safety-reviewer/interface-guardian
│   ├── skills/                    프로젝트 전문 지식 25종 (디렉토리 포맷)
│   │   ├── README.md                  스킬 카탈로그·출처·슬림화 정책
│   │   └── <name>/SKILL.md            각 스킬 (자동 키워드 트리거)
│   └── settings.json              권한 + hooks (PreToolUse 자동 리마인더)
└── .omc/                      ← OMC 플러그인 (skills/specs만 추적, state/sessions 제외)
    ├── skills/                    팀 공유 스킬
    └── specs/                     원본 설계 문서
```

---

## 개발 단계

| Phase | 내용 | 상태 |
|-------|------|------|
| 0 | 환경 구성 + interfaces/HAL/unit_actions 동결 | ✅ 완료 |
| 0.5 | Track B 시뮬 환경 PoC — Isaac Sim/RL/sim-to-real ADR 확정 | ✅ 완료 |
| 1 | 하드웨어 드라이버 (Doosan/RealSense/Gripper/PLC) | 🔄 진행 중 |
| 2 | 공유 퍼셉션 + 음성 (YOLOv8 + Whisper) | 🔄 뼈대 구현 (모델·캘리브 대기) |
| 3 | DB + PLC 연동 (FOD 모니터, LED 매퍼) | 대기 |
| 4 | Staging Area 동작 (거치/회수) | 대기 |
| 5 | Track A/B — Gemma 4 + Behavior Tree | 대기 |
| 6 | Track C — VLA demonstration + fine-tuning + 통합 | 대기 |
| 7 | 트랙 비교 평가 | 대기 |
| 8 | 테스트 (단위/통합/HIL) | 대기 |
| 9 | 배포 (systemd + 의존성 lockfile + 모니터링) | 대기 |

상세 → [`robot-arm-project.md`](robot-arm-project.md)

---

## 기여 가이드

상세 → [`CONTRIBUTING.md`](CONTRIBUTING.md)

1. **룰 숙지 필수** — [`.claude/rules/`](.claude/rules/) (safety > engineering > process)
2. **PR 전 검토** — 안전 코드는 `safety-reviewer`, 공유 인터페이스 변경은 `interface-guardian` 에이전트
3. **커밋 형식** — Conventional Commits 변형, [`.claude/rules/process.md`](.claude/rules/process.md) P-4
4. **테스트** — 안전 critical 경로는 테스트 없으면 머지 금지 (P-1)

---

## 최근 작업 이력

### 2026-05-27 — Phase 1 D455f Bring-up + Vision Pipeline 뼈대 (역할 C)

> 브랜치: `feat/vision` | 커밋 13개

#### 완료 항목

| 항목 | 파일 | 비고 |
|------|------|------|
| D455f ROS2 bring-up 패키지 초기 구성 | `ros2_ws/src/vision/` | package.xml, setup.py, `__init__.py` |
| udev 규칙 (RealSense) | `scripts/udev/99-realsense-d455.rules` | VID 8086 / PID 0b5c, `MODE="0666"` |
| 카메라 실측 검증 | `scripts/verify_camera.py` | Serial 342622300205, mean depth 0.955 m |
| 카메라 Intrinsics 기록 | `config/camera_info.yaml` | fx=645.11 fy=644.34 cx=650.51 cy=369.42 (1280×720) |
| Hand-eye 캘리브레이션 준비 | `scripts/calibrate_hand_eye.sh`, `launch/handeye_calibration.launch.py` | easy_handeye2, CharUco 8×6, eye-to-hand |
| Hand-eye 설정 예시 | `config/hand_eye.yaml.example` | `calibration_date: null` → HandEyeNotCalibratedError 발생 |
| YOLOv8 검출 노드 뼈대 | `vision/yolo_node.py` | `model_path: null` → 추론 비활성, 파인튜닝 후 경로 기입 |
| 6D 포즈 추정 노드 뼈대 | `vision/pose_node.py` | aligned depth + bbox → 3D, hand-eye 미캘리브 시 camera frame |
| 멀티 오브젝트 트래커 | `vision/tracker_node.py` | EMA(α=0.3), min_hits=3, max_misses=5 |
| Scene Context Builder | `vision/context_builder.py` | tracked_poses → `/vision/scene_context` (JSON) |
| Hand-eye 변환 로더 | `vision/hand_eye_loader.py` | 순수 Python, rclpy import 없음 |
| 카메라 스트림 검증 노드 | `vision/camera_node.py` | fps/depth stats, zero_ratio 경고 |
| 통합 런치 파일 | `launch/vision_pipeline.launch.py` | D455f + 4 노드 단일 명령 기동 |
| 단위 테스트 29개 | `test/test_hand_eye_loader.py` 외 2 | rclpy 모킹, 전원 통과 |

#### 확인된 인터페이스 계약 불일치 (팀 D 협의 필요)

아래 세 항목은 `interfaces.md` 동결 계약과 현재 구현이 다르다.
`interface-guardian` 검토 후 `interfaces/CHANGELOG.md`와 `interfaces.md`를 동시 갱신해야 머지 가능.

| 번호 | 항목 | 계약 (interfaces.md) | 구현 현황 | 요청 |
|------|------|----------------------|-----------|------|
| ① | `/vision/tool_poses` 타입 | `geometry_msgs/PoseArray` | `vision_msgs/Detection3DArray` | 타입 변경 승인 요청 — `PoseArray`는 `tool_id` 필드 없음 |
| ② | `tracker_node` 구독 토픽 | `/vision/detections` (2D) | `/vision/tool_poses` (3D) | 아키텍처 결정 요청 — 3D 기반 EMA 트래킹이 설계 의도 |
| ③ | 신규 토픽 미등록 | 없음 | `/vision/tracked_poses`, `/vision/scene_context` | interfaces.md §4 등록 + QoS 확정 요청 |

---

## 다음 연계 작업 (팀원별)

### 역할 C (본인) — 우선순위 순

| 순번 | 작업 | 선행 조건 |
|------|------|-----------|
| 1 | **Hand-eye 캘리브레이션 실행** | 로봇(역할 B) bring-up + CharUco 보드 출력 |
| 2 | **`config/hand_eye.yaml` 갱신** | 캘리브 완료, reprojection error < 1.0 px |
| 3 | **`config/camera_info.yaml` — `height_from_base_m` 기입** | 캘리브 완료 후 실측 |
| 4 | **YOLOv8 파인튜닝** (`config/vision.yaml model_path` 기입) | Phase 0 9종 공구 확정 + 데이터셋 수집 |
| 5 | **슬롯 오정렬 보정 로직** (`pose_node` 또는 별도 모듈) | Hand-eye 완료 후 실측 오차 확인 |
| 6 | **`/vision/detections` → `tracker_node` 연결 결정** | 팀 D 협의 결과 반영 |

### 역할 D (interfaces / orchestrator) — 요청 사항

| 순번 | 작업 | 참조 |
|------|------|------|
| 1 | `interface-guardian` 실행 — `/vision/tool_poses` 타입 변경 승인 | 위 ① |
| 2 | `interfaces.md §4` 토픽 테이블 갱신 — 신규 토픽 2개 등록, tracker_node 구독 토픽 수정 | 위 ②③ |
| 3 | `interfaces/CHANGELOG.md` 갱신 | P-2 |
| 4 | Orchestrator가 소비할 토픽 확정 — `/vision/tracked_poses` vs `/vision/scene_context` 중 어느 것을 BT Blackboard에 넣을지 | BT 설계 결정 |

### 역할 B (motion / robot driver) — 연계 필요

| 순번 | 작업 | 우리 의존성 |
|------|------|-------------|
| 1 | `doosan-robot2` bring-up + `base_link` TF 발행 확인 | `pose_node`의 base_link 좌표 정확도가 여기에 의존 |
| 2 | Hand-eye 캘리브레이션 공동 진행 | 로봇이 다양한 자세로 이동해야 샘플 수집 가능 |

### 역할 A (voice / Gemma 4) — 연계 대기

| 순번 | 작업 | 우리 제공 항목 |
|------|------|---------------|
| 1 | `gemma_intent_node`에서 `/vision/scene_context` JSON 소비 | `context_builder.py`가 발행 중 (hand-eye 캘리브 전까지 camera frame 기준) |
| 2 | scene JSON 스키마 확인 | `vision/context_builder.py` docstring 상단 |

---

## 라이선스

TBD — 프로젝트 이름·원격 저장소 확정 후 결정.
