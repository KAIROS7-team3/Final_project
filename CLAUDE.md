# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

---

## 이 저장소에 대해

이 저장소는 **음성 명령 공구 전달 로봇팔 시스템**의 기획 및 설계 문서를 관리하는 공간이다. 현재는 구현 코드가 없으며, 아키텍처 설계와 프로젝트 계획 문서로 구성된다.

---

## 주요 문서

### 에이전트 작업 기준 문서 (자주 참조)

| 파일 | 설명 |
|------|------|
| `docs/architecture.md` | 시스템 설계, 트랙 구조, 패키지, DB 스키마, 안전 아키텍처 |
| `docs/adr/index.md` | ADR 전체 인덱스, 미결 사항, 확정된 결정 히스토리 |
| `docs/adr/<category>.md` | ADR 상세 (architecture / ai-ml / safety / hardware / data / interfaces) |
| `docs/conventions.md` | 수락 기준, 네이밍, 설정 파일 위치, 검증 파이프라인 |
| `docs/interfaces.md` | 인터페이스 계약 — topic/service/action QoS·frame_id·timestamp |
| `docs/frames.md` | TF tree, 좌표계 규약 (REP-103), hand-eye 변환 |
| `docs/hardware.md` | 하드웨어 인벤토리, 드라이버 버전, udev 규칙 |
| `docs/simulation.md` | Gazebo 시뮬레이션 설정, BT 골든 파일 회귀, pass/fail 기준 |

### 기타 문서

| 파일 | 설명 |
|------|------|
| `robot-arm-project.md` | 개발 Phase 일정 계획 (Phase 0–9) |
| `architecture.html` | 시스템 아키텍처 인터랙티브 시각화 |
| `.claude/agents/robot-arm-planner.md` | 로봇팔 프로젝트 전용 계획 에이전트 정의 |

---

## 프로젝트 개요

**Doosan e0509 협동 로봇팔**이 음성 명령으로 공구함에서 공구를 꺼내 전달하고, 반납 명령 시 제자리에 돌려놓는 시스템.

- **하드웨어**: Doosan e0509 · ROBOTIS RH-P12-RN (그리퍼) · Intel RealSense D455f · PLC (LS Electric XBC-DR10E)
- **소프트웨어**: ROS2 Humble (Track A/B) · Whisper STT · Gemma 4 (로컬 LLM) · YOLOv8 · DB (FOD 관리)
- **개발 머신**: Vector 16 HX AI A2XWIG (주) · HP ProBook 450 G10 (보조)

### 세 가지 제어 트랙 (비교 목적)

| 트랙 | 의도 이해 | 결정 | 모션 | 미들웨어 |
|------|-----------|------|------|----------|
| **A** | Gemma 4 + DB check | Behavior Tree | DSR 좌표 제어 | ROS2 (7 패키지) |
| **B** | Gemma 4 + DB check | Behavior Tree | RL 정책 | ROS2 (7 패키지) |
| **C** | Python 키워드 파서 + DB gate | VLA 모델 end-to-end | Doosan Python SDK 직접 제어 | 없음 (단일 .py) |

Track A/B와 Track C는 **물리 하드웨어**와 **순수 Python 코어 라이브러리** (`db_core/`, `plc_core/`, DB 스키마, PLC 프로토콜)만 공유한다. `hal/`과 `unit_actions/`는 Track A/B 전용이다. Transport/미들웨어 레이어는 완전히 독립이다.

- **Track A/B**: ROS2 Humble 기반 전체 스택. `realsense-ros`, `doosan-robot2` ROS2 노드로 하드웨어 접근. Whisper small STT + Gemma 4 의도 분류 + Behavior Tree + unit_actions
- **Track C**: 독립 Python 프로세스. ROS2 런타임 불필요. 사전학습 VLA 모델(OpenVLA/π0 등)을 Doosan e0509 demonstration 데이터로 파인튜닝. `pyrealsense2` + Whisper small 직접 호출 → VLA 추론 → joint trajectory + gripper command → Doosan Python SDK 직접 실행. unit_actions 미사용. "구조화된 BT+unit_actions vs end-to-end 학습 기반"을 동일 하드웨어에서 비교하는 것이 목적

**Track Selector**: `./run.sh --track [A|B|C]`로 실행. Track C 시작 전 ROS2 stack 완전 종료 후 VRAM 해제 확인 필수.

---

## 아키텍처 핵심 원칙

### Unit Action Library 분리 설계

`unit_actions/`는 ROS2에 의존하지 않는 순수 Python 모듈이다. **Track A/B 전용**.
- **Track A/B**: `unit_action_server.py`가 이를 ROS2 action server로 래핑
- **Track C**: `unit_actions` 미사용 — VLA 모델이 joint commands를 직접 출력하고 Doosan Python SDK로 실행

### VLA Safety Boundary

Track C(VLA)에서 VLA 출력(joint trajectory + gripper command)은 반드시 `SafetyValidator`를 통과해야 하드웨어에 접근할 수 있다. VLA가 Doosan Python SDK를 직접 호출하는 경로는 SafetyValidator를 우회하지 않는다.

### DB 기반 명령 차단

- **Track A/B**: Gemma 4가 명령 실행 전 DB에서 공구 상태를 확인한다.
- **Track C**: Python 코드(`check_feasibility()`)가 VLA 호출 전 `db_core/`를 직접 쿼리해 가용성을 검증한다.
- 공통: `out`, `missing`, `fod_alert` 상태의 공구 fetch 및 `staged` 이외 상태의 return은 실행 전에 차단된다.

### ROS2 패키지 구조 (Track A/B)

```
interfaces/    커스텀 msg/srv/action 정의 (다른 패키지가 모두 의존)
voice/         Whisper STT + Gemma 4 의도 분류
vision/        YOLOv8 + 6D Pose + Tracker
orchestrator/  Behavior Tree + unit_action_server (ROS2 래퍼)
db/            DB 인터페이스 + FOD 모니터
motion/        DSR/RL 제어 + Handover Detector
plc/           PLC 통신 + LED 상태 매핑
```

---

## 에이전트 및 스킬

### robot-arm-planner 에이전트

로봇팔 프로젝트 기획을 위한 전용 플래닝 에이전트.

| 환경 | 호출 방법 |
|------|-----------|
| Claude Code 네이티브 (FleetView 없음) | `.claude/agents/robot-arm-planner.md` 자동 트리거 |
| FleetView / OMC 환경 (현재) | `robot arm`, `로봇팔`, `manipulator` 등 키워드 → 스킬 자동 트리거 |

**FleetView 환경에서의 호환성:** FleetView가 `.claude/agents/`를 인식하는지는 버전에 따라 다르다. 현재 세션 기준 `Agent(subagent_type="robot-arm-planner")`는 동작할 수 있으나, 동작하지 않는 환경에서는 `.omc/skills/robot-arm-planner/SKILL.md`(FleetView 호환 버전)를 키워드 트리거로 사용한다. 신규 팀원은 자신의 환경에서 한 번 시도해 보고 결정한다.

---

## 문서 수정 규칙

- `robot-arm-project.md`: 정식 계획서. 구조 변경 시 이 파일을 기준으로 업데이트한다.
- `architecture.html`: 명시적 요청이 있을 때만 수정한다. (`settings.json` deny 규칙으로 보호)
- `.omc/specs/robot-arm-plan-voice-tool-retrieval.md`: 상세 설계 원본. 새로운 설계 결정은 여기에 먼저 반영한다.
- 세 문서 간 내용이 충돌하면 `robot-arm-project.md`를 최신 기준으로 삼는다.

---

## 엔지니어링 룰 (요약)

세부 룰은 [`.claude/rules/`](.claude/rules/) 폴더 참조. **모든 코드 작성·리뷰 작업은 이 룰을 따른다.**

| 파일 | 핵심 내용 | 우선순위 |
|------|-----------|----------|
| [`.claude/rules/safety.md`](.claude/rules/safety.md) | SafetyValidator·DB Gate·E-stop·Watchdog 무결성 | 🔴 최우선 |
| [`.claude/rules/engineering.md`](.claude/rules/engineering.md) | 단위(rad/m/Time), 의존성 그래프, 설정·에러·로깅·명명 | 🟠 높음 |
| [`.claude/rules/process.md`](.claude/rules/process.md) | 테스트·CHANGELOG·시크릿·커밋·브랜치·리뷰 | 🟡 보통 |

**핵심 invariant (반드시 기억):**
- Track C VLA 출력은 **반드시** SafetyValidator 통과 후 SDK 호출
- `db_core/`, `plc_core/`, `unit_actions/`, `track_c_vla.py`는 `rclpy` import 금지
- 좌표·임계값·시간은 `config/*.yaml`. 코드 하드코딩 금지
- 시크릿은 `.env` (gitignored). commit 금지
- 룰 충돌 시 `safety > engineering > process` 순으로 우선

---

## 팀 협업 규칙 (AI 에이전트 준수사항)

### 에이전트 선택 기준

| 작업 유형 | 사용 에이전트 |
|-----------|--------------|
| 프로젝트 기획 / 설계 변경 / 스펙 인터뷰 | `robot-arm-planner` |
| 안전 관련 코드 (모션, VLA 출력, E-stop) 검토 | `safety-reviewer` |
| `interfaces/`, `db_core/`, `plc_core/`, `unit_actions/` 변경 전 | `interface-guardian` |
| 일반 구현 / 디버깅 / 리팩토링 | 기본 (Claude Code) |

에이전트 정의 파일: `.claude/agents/*.md`

### 파일 수정 권한

| 파일 / 경로 | 수정 조건 |
|-------------|-----------|
| `robot-arm-project.md` | 설계 결정 확정 또는 명백한 오류 수정 시만 |
| `architecture.html` | 명시적 요청 시만 (`settings.json`에서 deny) |
| `.claude/settings.json` | 팀 합의 후만 (deny 규칙으로 보호) |
| `.claude/agents/*.md` | 팀 합의 후만 |
| `.claude/rules/*.md` | 팀 합의 후만 (PR 리뷰 + 루트 CHANGELOG 갱신) |
| `interfaces/` (코드베이스) | `interface-guardian` 검토 후만 |
| `unit_actions/` 시그니처 | `interface-guardian` 검토 후만 |
| `db_core/`, `plc_core/` API | `interface-guardian` 검토 후만 |

### 스킬 사용 규칙

- 프로젝트 스킬은 `.claude/skills/` (git 공유). 팀원은 별도 설치 불필요.
- 스킬에 예시 파일 추가 시 `skill-name/SKILL.md` 디렉토리 형식으로 마이그레이션 후 `examples/` 서브디렉토리 사용.
- 에이전트가 특정 스킬에 의존할 경우 에이전트 `.md`의 `skills:` frontmatter에 명시.

### 검증 파이프라인 (코드베이스 생성 후 적용)

상세 표 → [`docs/conventions.md` §5](docs/conventions.md)

### 브랜치 / 커밋 규칙

- `git push`: 에이전트가 실행 시 사용자 확인 프롬프트 필수 — 어떤 브랜치·커밋을 push하는지 확인 후 승인. `git push --force`는 settings.json deny로 절대 금지.
- 커밋 메시지 형식 → [`.claude/rules/process.md` P-4](.claude/rules/process.md) (Conventional Commits 변형)
- `interfaces/` 변경 커밋에는 반드시 `interfaces/CHANGELOG.md` 갱신 포함.

---

## 팀원 초기 설정 가이드

```bash
# 1. 저장소 클론
git clone <repo-url> && cd robot-arm-project

# 2. OMC 플러그인 설치 (개인 레벨, 1회만)
claude plugin install oh-my-claudecode@omc

# 3. 프로젝트 열기 — 스킬/에이전트 자동 로드
claude

# 4. 개인 설정 파일 생성 (gitignored)
touch .claude/settings.local.json
```

**자동으로 사용 가능해지는 것 (별도 설치 불필요):**
- `.claude/skills/` — robotics 스킬 10종 (ros2, robotics-testing, robotics-design-patterns 등)
- `.claude/agents/` — robot-arm-planner, safety-reviewer, interface-guardian
- OMC 플러그인 — 세션 메모리, 스킬 자동 트리거, 서브에이전트 오케스트레이션
