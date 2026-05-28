# AGENTS.md

> Codex CLI · Gemini CLI · 기타 비-Claude Code 도구용 프로젝트 지침서.
> Claude Code 사용자는 [`CLAUDE.md`](CLAUDE.md)를 우선 참조한다.

---

## 이 문서의 위치

| 도구 | 자동 로드 파일 |
|------|---------------|
| **Claude Code** | `CLAUDE.md` + `.claude/skills/` + `.claude/agents/` + `.claude/rules/` |
| **Codex CLI** | `AGENTS.md` (이 파일) |
| **Gemini CLI** | `AGENTS.md` 또는 `GEMINI.md` |

`.claude/` 디렉토리는 Claude Code 전용이다. 다른 도구는 이 파일을 통해 동일한 컨텍스트를 얻는다.
**스킬·에이전트·훅의 자동 트리거는 Claude Code 사용 시에만 작동한다.** 그 외 도구에서는 이 문서의 내용을 수동으로 따라야 한다.

---

## 1. 프로젝트 개요

**음성 명령 공구 전달 로봇팔 시스템** — Doosan e0509 협동 로봇팔이 음성 명령으로 공구함에서 공구를 꺼내 Staging Area에 전달하고, 반납 명령 시 제자리에 돌려놓는다.

- **하드웨어**: Doosan e0509 · ROBOTIS RH-P12-RN (그리퍼) · Intel RealSense D455f · PLC (LS Electric XBC-DR10E)
- **소프트웨어**: ROS2 Humble · Whisper STT · Gemma 4 (로컬 LLM) · YOLOv11s · SQLite/PostgreSQL (FOD 관리)
- **개발 머신**: Vector 16 HX AI A2XWIG (주) · HP ProBook 450 G10 (보조)

### 세 가지 제어 트랙

| 트랙 | 의도 이해 | 결정 | 모션 | 미들웨어 |
|------|-----------|------|------|----------|
| **A** | Gemma 4 + DB check | Behavior Tree | DSR 좌표 제어 | ROS2 (7 패키지) |
| **B** | Gemma 4 + DB check | Behavior Tree | RL 정책 | ROS2 (7 패키지) |
| **C** | Python 키워드 파서 + DB gate | VLA 모델 end-to-end | Doosan Python SDK 직접 제어 | 없음 (단일 .py) |

Track A/B와 Track C는 **물리 하드웨어**와 **순수 Python 코어 라이브러리**(`db_core/`, `plc_core/`, DB 스키마, PLC 프로토콜)만 공유한다. `hal/`과 `unit_actions/`는 Track A/B 전용이다.

---

## 2. 주요 문서 (수동 참조 필요)

### 항상 참조

| 파일 | 설명 |
|------|------|
| [`docs/architecture.md`](docs/architecture.md) | 시스템 설계, 트랙 구조, 패키지, DB 스키마, 안전 아키텍처 |
| [`docs/adr/index.md`](docs/adr/index.md) | ADR 인덱스, 미결 사항, 확정 결정 |
| [`docs/conventions.md`](docs/conventions.md) | 수락 기준, 네이밍, 설정 파일, 검증 파이프라인 |
| [`docs/interfaces.md`](docs/interfaces.md) | topic/service/action QoS·frame_id·timestamp |
| [`docs/frames.md`](docs/frames.md) | TF tree, 좌표계 규약 (REP-103), hand-eye 변환 |
| [`docs/hardware.md`](docs/hardware.md) | 하드웨어 인벤토리, 드라이버 버전, udev 규칙 |
| [`docs/simulation.md`](docs/simulation.md) | Gazebo 시뮬레이션, BT 골든 파일 회귀 |
| [`robot-arm-project.md`](robot-arm-project.md) | 개발 Phase 일정 (Phase 0–9) |

### 룰 파일 (우선순위 순)

| 파일 | 우선순위 | 내용 |
|------|----------|------|
| [`.claude/rules/safety.md`](.claude/rules/safety.md) | 🔴 최우선 | SafetyValidator·DB Gate·E-stop·Watchdog |
| [`.claude/rules/engineering.md`](.claude/rules/engineering.md) | 🟠 높음 | 단위·좌표·의존성·에러·로깅·명명 |
| [`.claude/rules/process.md`](.claude/rules/process.md) | 🟡 보통 | 테스트·CHANGELOG·시크릿·커밋·리뷰 |

룰 충돌 시 `safety > engineering > process` 순.

---

## 3. 핵심 안전 invariant (절대 위반 금지)

전체 규칙은 [`.claude/rules/safety.md`](.claude/rules/safety.md) 참조. 요약:

1. **Track C VLA 출력은 반드시 `SafetyValidator.check()` 통과 후 SDK 호출**
2. **DB Gate 우회 금지** — `fetch`/`return`은 `check_feasibility()` 통과 후만
3. **E-stop은 BT/VLA 상태와 무관하게 항상 호출 가능** (응답 ≤ 500ms)
4. **SafetyWatchdog 비활성화 금지** (하트비트 500ms 초과 시 자동 정지)
5. **Joint/속도 한계는 e0509 operational range 내 + 협동 모드 250mm/s 이하**
6. **v1.0 직접 핸드오버 금지** — 모든 전달은 Staging Area 거치
7. **동작 중(`is_moving=True`) 음성 수신 차단**
8. **FOD 상태 전이 무결성** — `out`/`staged` 임계 시간 초과 시 `missing` 자동
9. **부팅 reconciliation 완료 전 모든 명령 거부**

---

## 4. 엔지니어링 룰 요약

전체는 [`.claude/rules/engineering.md`](.claude/rules/engineering.md).

### 단위 및 좌표 (E-1)
- 각도/joint: **rad** (degree 금지)
- 길이: **m** (mm 금지)
- 시간: **ROS2 Time** (절대 시각)
- 좌표계: **robot_base_link** 기본
- 회전: **쿼터니언 (x, y, z, w)**

### 의존성 그래프 (E-2)
| 모듈 | 금지 import |
|------|-------------|
| `db_core/` | `rclpy`, `interfaces`, ROS2 |
| `plc_core/` | `rclpy`, `interfaces`, ROS2 |
| `unit_actions/` | `rclpy`, `interfaces`, ROS2 |
| `track_c_vla.py` | `rclpy`, `interfaces`, `unit_actions`, `hal` |

CI 검사:
```bash
grep -r 'import rclpy' db_core/ plc_core/ unit_actions/  # 결과 없어야 함
```

### 설정 관리 (E-4)
- 좌표·임계값·시간은 `config/*.yaml`. 코드 하드코딩 금지
- 시크릿은 `.env` (gitignored). git commit 금지
- `.env.example`은 git 포함 — 변수 이름만

### 에러 처리 (E-5)
모든 actuator 호출은 try/except 필수. 실패 시 3가지 수행:
1. DB 로그 (`event_type='error'` 또는 'rejected')
2. PLC 상태 갱신 (빨간 점멸 또는 경고)
3. 운영자 안내

silent fallback 금지. retry는 명시적으로 횟수 + backoff 지정.

### 코딩 스타일 (E-7)
- PEP 8 + `ruff format`
- 타입 힌트 필수 (public 함수 인자 + 반환값)
- f-string 사용. `.format()`/`%` 자제
- `dataclass(frozen=True)` for value objects

### 명명 규칙 (E-8)
- Python: `snake_case` 함수/변수, `PascalCase` 클래스, `UPPER_SNAKE_CASE` 상수
- ROS2 노드: `snake_case`
- ROS2 메시지: `PascalCase`
- 공구 ID: `<type>_<spec>` (예: `screwdriver_phillips_small`)

---

## 5. 프로세스 룰 요약

전체는 [`.claude/rules/process.md`](.claude/rules/process.md).

### 테스트 (P-1)
- 모든 새 `unit_actions/`, `db_core/`, `plc_core/` 함수: happy + failure path 최소 1개씩
- 안전 critical 경로: 단위 테스트 없으면 머지 금지
- 커버리지: 80% (안전 모듈 100%)

### 커밋 메시지 (P-4)
Conventional Commits 변형:
```
<type>(<scope>): <subject>
```
type: `feat`, `fix`, `refactor`, `test`, `docs`, `chore`, `safety`

### 브랜치 (P-5)
- `main` — 직접 push 금지
- `feat/<설명>`, `fix/<설명>`, `safety/<설명>`
- 머지: PR + 리뷰 1명 이상 + CI 통과

### 보호된 동작
- `git push --force`: 절대 금지 (`.claude/settings.json` deny 규칙)
- `git push`: 사용자 확인 프롬프트 필수
- main 직접 commit 금지

---

## 6. 파일 수정 권한

| 파일/경로 | 수정 조건 |
|----------|-----------|
| `robot-arm-project.md` | 설계 결정 확정 또는 명백한 오류 수정 시만 |
| `architecture.html` | 명시적 요청 시만 (Claude Code에서는 deny 규칙) |
| `.claude/settings.json` | 팀 합의 후만 |
| `.claude/agents/*.md` | 팀 합의 후만 |
| `.claude/rules/*.md` | 팀 합의 후만 (PR + CHANGELOG) |
| `interfaces/` 코드 | interface-guardian 패턴 검토 후 |
| `unit_actions/` 시그니처 | interface-guardian 패턴 검토 후 |
| `db_core/`, `plc_core/` API | interface-guardian 패턴 검토 후 |

Claude Code의 `interface-guardian` / `safety-reviewer` 에이전트와 동등한 검토를 codex/gemini 사용자는 [`.claude/agents/interface-guardian.md`](.claude/agents/interface-guardian.md), [`.claude/agents/safety-reviewer.md`](.claude/agents/safety-reviewer.md)의 체크리스트를 수동으로 적용한다.

---

## 7. 도메인 지식 참조 (Claude Code의 스킬 = 수동 참조 문서)

Claude Code 사용자는 `.claude/skills/`가 키워드 트리거로 자동 로드되지만, codex/gemini 사용자는 필요 시 직접 읽어야 한다.

### 프로젝트 전용 (가장 자주 참조)
| 주제 | 파일 |
|------|------|
| Doosan e0509 제어 | [`.claude/skills/doosan-e0509/SKILL.md`](.claude/skills/doosan-e0509/SKILL.md) |
| PLC Modbus 통신 | [`.claude/skills/modbus-plc/SKILL.md`](.claude/skills/modbus-plc/SKILL.md) |
| Whisper STT | [`.claude/skills/whisper-stt/SKILL.md`](.claude/skills/whisper-stt/SKILL.md) |
| VLA 파인튜닝 | [`.claude/skills/vla-finetuning/SKILL.md`](.claude/skills/vla-finetuning/SKILL.md) |
| Demonstration 수집 | [`.claude/skills/demo-collection/SKILL.md`](.claude/skills/demo-collection/SKILL.md) |
| Behavior Tree (py_trees) | [`.claude/skills/bt-py-trees/SKILL.md`](.claude/skills/bt-py-trees/SKILL.md) |
| RealSense D455f | [`.claude/skills/realsense-d455f/SKILL.md`](.claude/skills/realsense-d455f/SKILL.md) |
| Hand-eye 캘리브레이션 | [`.claude/skills/hand-eye-calibration/SKILL.md`](.claude/skills/hand-eye-calibration/SKILL.md) |

### 공통 패턴
| 주제 | 파일 |
|------|------|
| Python 패턴 | [`.claude/skills/python-patterns/SKILL.md`](.claude/skills/python-patterns/SKILL.md) |
| 에러 처리 | [`.claude/skills/error-handling-patterns/SKILL.md`](.claude/skills/error-handling-patterns/SKILL.md) |
| 설정 관리 | [`.claude/skills/config-management/SKILL.md`](.claude/skills/config-management/SKILL.md) |
| pytest | [`.claude/skills/pytest-patterns/SKILL.md`](.claude/skills/pytest-patterns/SKILL.md) |
| 성능 프로파일링 | [`.claude/skills/performance-profiling/SKILL.md`](.claude/skills/performance-profiling/SKILL.md) |
| 코드 리뷰 체크리스트 | [`.claude/skills/code-review-checklist/SKILL.md`](.claude/skills/code-review-checklist/SKILL.md) |
| Git 컨벤션 | [`.claude/skills/git-conventions/SKILL.md`](.claude/skills/git-conventions/SKILL.md) |

### 일반 robotics 참조
| 주제 | 파일 |
|------|------|
| ROS2 일반 | [`.claude/skills/ros2/SKILL.md`](.claude/skills/ros2/SKILL.md) |
| 로봇 부팅·systemd | [`.claude/skills/robot-bringup/SKILL.md`](.claude/skills/robot-bringup/SKILL.md) |
| 로봇 perception | [`.claude/skills/robot-perception/SKILL.md`](.claude/skills/robot-perception/SKILL.md) |
| robotics 디자인 패턴 | [`.claude/skills/robotics-design-patterns/SKILL.md`](.claude/skills/robotics-design-patterns/SKILL.md) |
| robotics 테스트 | [`.claude/skills/robotics-testing/SKILL.md`](.claude/skills/robotics-testing/SKILL.md) |
| robotics 보안 | [`.claude/skills/robotics-security/SKILL.md`](.claude/skills/robotics-security/SKILL.md) |

전체 목록과 원본 출처는 [`.claude/skills/README.md`](.claude/skills/README.md) 참조.
일반 robotics 10종의 원본: <https://github.com/arpitg1304/robotics-agent-skills>

---

## 8. ROS2 패키지 구조 (Track A/B)

```
interfaces/    커스텀 msg/srv/action 정의 (다른 패키지가 모두 의존)
voice/         Whisper STT + Gemma 4 의도 분류
vision/        YOLOv11s + 6D Pose + Tracker
orchestrator/  Behavior Tree + unit_action_server (ROS2 래퍼)
db/            DB 인터페이스 + FOD 모니터
motion/        DSR/RL 제어 + Handover Detector
plc/           PLC 통신 + LED 상태 매핑
```

`unit_actions/`는 ROS2 비의존 순수 Python — Track A/B의 `unit_action_server.py`가 ROS2 action server로 래핑.

---

## 9. Track Selector

```bash
./run.sh --track [A|B|C]
```

Track C 시작 전: ROS2 stack 완전 종료 후 VRAM 해제 확인 필수.

---

## 10. 팀원 초기 설정

```bash
# 1. 저장소 클론
git clone <repo-url> && cd robot-arm-project

# 2. (Claude Code 사용자만) OMC 플러그인 설치
claude plugin install oh-my-claudecode@omc

# 3. 개인 설정 파일 생성 (gitignored)
touch .claude/settings.local.json

# 4. 환경 변수
cp .env.example .env  # 실제 값 채우기
```

도구별 진입:
- Claude Code: `claude`
- Codex CLI: `codex` (이 AGENTS.md 자동 로드)
- Gemini CLI: `gemini` (AGENTS.md 자동 로드)

---

## 11. 도구 간 동등성 매트릭스

| 기능 | Claude Code | Codex / Gemini |
|------|-------------|---------------|
| 프로젝트 룰 로드 | `CLAUDE.md` 자동 | `AGENTS.md` 자동 |
| 스킬 키워드 트리거 | 25개 자동 | 수동 — `.claude/skills/<name>/SKILL.md` 직접 읽기 |
| 에이전트 자동 호출 | 3개 자동 | 수동 — `.claude/agents/<name>.md` 체크리스트 적용 |
| PreToolUse 훅 | 자동 리마인더 | 미작동 — 변경 시 본인이 룰 확인 |
| 권한 deny 규칙 | 자동 차단 | 미작동 — `.gitignore` + 코드 리뷰로 방지 |
| 메모리 시스템 | OMC 사용 시 | 도구별 자체 시스템 |

**원칙**: 도구 차이로 인한 안전 사고는 발생 안 해야 한다.
모든 codex/gemini 사용자는 `interfaces/`, `motion/`, `safety/`, `unit_actions/`, `track_c_vla.py` 변경 시 본인이 [`.claude/agents/interface-guardian.md`](.claude/agents/interface-guardian.md)와 [`.claude/agents/safety-reviewer.md`](.claude/agents/safety-reviewer.md)의 체크리스트를 수동 적용한다.

---

## 12. 미결 사항

[`docs/adr/index.md`](docs/adr/index.md) 섹션 17 미결 항목 참조. 결정 시 동일 PR에 문서 갱신 포함.
