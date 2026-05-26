# AI 에이전트 환경 설정 안내

> 이 문서는 AI 도구나 코딩에 익숙하지 않은 팀원을 위한 안내서다.
> 이 프로젝트가 AI를 어떻게 활용하고 있는지, 어떤 도구가 필요한지 설명한다.

---

## 1. 이 프로젝트와 AI

이 프로젝트는 단순히 AI에게 질문하고 답을 받는 수준을 넘어,
**AI가 프로젝트의 규칙·역할·안전 기준을 이해한 상태에서 코드 작성을 돕도록** 설정되어 있다.

예를 들어:
- "로봇이 안전 검사를 우회하는 코드를 써줘" → 안전 규칙(`safety.md`)에 따라 AI가 거부하거나 경고함
- `.env` 같은 비밀 파일을 수정하려 하면 → 권한 설정(`settings.json` deny)에 의해 차단됨
- 공구 인터페이스 파일을 수정하려 하면 → AI가 자동으로 "팀 리뷰가 필요하다"고 경고함
- 특정 작업(기획 인터뷰, 안전 코드 리뷰)은 → 그 작업 전용 AI 역할(에이전트)로 전환됨

이 모든 것이 `.claude/` 폴더와 `CLAUDE.md` 파일로 설정된다.

---

## 2. 권장 도구: Claude Code

이 프로젝트의 AI 설정은 **Claude Code** 를 기준으로 구성되어 있다.

Claude Code는 터미널(명령창)에서 실행하는 AI 코딩 어시스턴트다.
채팅창에서 AI와 대화하면서 파일을 읽고, 코드를 수정하고, 명령어를 실행할 수 있다.

### 설치 방법

```bash
# Node.js 18+ 가 설치되어 있어야 한다
npm install -g @anthropic-ai/claude-code

# 설치 후 프로젝트 폴더에서 실행
claude
```

설치 후 처음 실행하면 Anthropic 계정 로그인을 요청한다.
계정이 없으면 [claude.ai](https://claude.ai) 에서 가입한다.

---

## 3. 프로젝트 AI 설정 구조

```
프로젝트 루트/
├── CLAUDE.md                  ← AI가 가장 먼저 읽는 프로젝트 설명서
├── .claude/
│   ├── settings.json          ← AI 권한 제어 (팀 공용, git 추적)
│   ├── settings.local.json    ← 개인 설정 (gitignored, 각자 자유롭게)
│   ├── rules/                 ← AI가 반드시 따라야 할 규칙
│   │   ├── safety.md          ←   안전 규칙 (최우선)
│   │   ├── engineering.md     ←   코딩 규칙
│   │   └── process.md         ←   협업 규칙 (테스트, 커밋 등)
│   ├── agents/                ← 특수 역할 AI 정의
│   │   ├── robot-arm-planner.md
│   │   ├── safety-reviewer.md
│   │   └── interface-guardian.md
│   └── skills/                ← AI가 참조하는 전문 지식 모음 (25종, 디렉토리 포맷)
│       ├── doosan-e0509/SKILL.md
│       ├── realsense-d455f/SKILL.md
│       └── ... (25종)
```

---

## 4. 각 구성 요소 설명

### CLAUDE.md — AI용 프로젝트 설명서

Claude Code를 실행하면 AI는 `CLAUDE.md`를 가장 먼저 읽는다.
여기에는 이 프로젝트가 무엇인지, 어떤 규칙을 따라야 하는지, 어떤 파일이 어디 있는지가 적혀 있다.

사람으로 치면 **신입 팀원에게 주는 온보딩 문서**와 같다.
AI는 이 문서를 읽고 나서야 이 프로젝트가 로봇팔 시스템이라는 것,
Track A/B/C가 있다는 것, 어떤 파일을 함부로 수정하면 안 된다는 것을 이해한다.

---

### Rules — AI 행동 규칙

`.claude/rules/` 폴더 안의 규칙 파일들이다. AI는 이 규칙을 항상 따른다.

| 파일 | 내용 | 중요도 |
|------|------|--------|
| `safety.md` | 안전 관련 코드는 어떻게 작성해야 하는지 | 🔴 절대 위반 불가 |
| `engineering.md` | 코딩 스타일, 단위, 의존성 규칙 | 🟠 높음 |
| `process.md` | 테스트, 커밋 메시지, 비밀 관리 규칙 | 🟡 보통 |

예시: `safety.md`에는 "VLA 모델의 출력은 반드시 SafetyValidator를 통과해야 한다"는 규칙이 있다.
AI가 이 규칙에 위반되는 코드를 작성하려 하면 스스로 거부하거나 경고를 출력한다.

---

### Agent — 특수 역할 AI

**Agent(에이전트)**는 특정 작업에 최적화된 별도의 AI 역할이다.

기본 AI가 "무엇이든 도와주는 일반 어시스턴트"라면,
에이전트는 **특정 분야 전문가**처럼 동작한다.

이 프로젝트에는 3개의 에이전트가 있다:

| 에이전트 | 역할 | 언제 사용하나 |
|----------|------|--------------|
| `robot-arm-planner` | 프로젝트 기획 전문가. 요구사항을 인터뷰하고 설계 계획을 작성한다 | 새 기능 설계, 단계 계획 수립 시 |
| `safety-reviewer` | 안전 코드 심사 전문가. 로봇 모션·E-stop·VLA 출력 관련 코드를 검토한다 | 안전 관련 코드 머지 전 |
| `interface-guardian` | 공유 인터페이스 수호자. 팀 전체가 공유하는 API/데이터 형식 변경을 심사한다 | 공유 인터페이스 변경 전 |

**에이전트 호출 방법 (Claude Code):**
- 자연어로 요청: `"safety-reviewer 에이전트로 이 코드 검토해줘"`
- 사용 가능한 에이전트 목록 보기: `/agents`
- 일부 작업(예: 안전 코드 수정)은 AI가 자동으로 해당 에이전트를 호출하기도 한다

에이전트는 기본 AI와 **별도의 대화창에서** 실행된다. 서로 다른 전문 지식과 지시사항을 가지고 있어,
같은 질문이라도 일반 AI와 에이전트의 답이 다를 수 있다.

---

### Skill — 전문 지식 모음

**Skill(스킬)**은 AI가 특정 작업을 할 때 참조하는 전문 지식 문서다.

예를 들어 `doosan-e0509.md` 스킬에는 Doosan 로봇팔 제어 API 사용법,
`realsense-d455f.md`에는 RealSense 카메라 설정 방법이 정리되어 있다.

AI에게 "RealSense 카메라 코드 작성해줘"라고 하면,
AI는 이 스킬을 참조해서 이 프로젝트에 맞는 방식으로 코드를 작성한다.

현재 프로젝트에 있는 스킬 (25종, 디렉토리 포맷). 전체 목록은 [`.claude/skills/README.md`](../.claude/skills/README.md) 참조.

| 스킬 | 내용 |
|------|------|
| `doosan-e0509/SKILL.md` | Doosan 로봇팔 제어 |
| `realsense-d455f/SKILL.md` | RealSense 카메라 |
| `modbus-plc/SKILL.md` | PLC 통신 |
| `robotics-testing/SKILL.md` | 로보틱스 테스트 방법 |
| `git-conventions/SKILL.md` | 이 프로젝트 git 사용 방법 |
| ... | 등 |

스킬은 별도로 호출하지 않아도 된다. AI가 필요할 때 자동으로 참조한다.

---

### Settings — AI 권한 제어

`.claude/settings.json`은 AI가 할 수 있는 것과 없는 것을 정의한다.

**차단된 동작 (AI가 절대 실행 불가):**
- `git push --force` — 원격 저장소 강제 덮어쓰기
- `git reset --hard` — 작업 내용 강제 삭제
- `.env` 파일 수정 — 비밀 키 파일
- `settings.json`, `rules/`, `agents/` 수정 — 팀 합의 없이 규칙 변경 금지

**자동 경고 (AI가 경고를 출력하지만 차단하지는 않음):**
- `interfaces/`, `db_core/`, `plc_core/` 등 공유 코드 수정 시 → "interface-guardian 리뷰 필요" 경고
- `motion/`, `safety/` 등 안전 코드 수정 시 → "safety-reviewer 리뷰 필요" 경고

---

## 5. 다른 AI 도구(Cursor, Codex 등) 사용 시

### 무엇이 달라지나

| 기능 | Claude Code | Codex CLI | Cursor |
|------|:-----------:|:---------:|:------:|
| 프로젝트 설명서 자동 읽기 | ✅ `CLAUDE.md` | ✅ `AGENTS.md` (변환 필요) | ⚠️ `.cursor/rules/` |
| Rules 자동 적용 | ✅ | ✅ AGENTS.md에 포함 | ⚠️ 별도 설정 시 가능 |
| Agent 전환 | ✅ | ✅ Subagent 지원 (변환 필요) | ❌ (수동 지시만) |
| Skill 자동 참조 | ✅ | ✅ Skill 지원 (변환 필요) | ❌ |
| 파일 수정 권한 차단 | ✅ settings.json deny | ✅ sandbox + hooks | ❌ |
| 자동 경고 알림 | ✅ PreToolUse hook | ✅ PreToolUse hook | ❌ |
| MCP 서버 연동 | ✅ | ✅ | ⚠️ 제한적 |

**요약:**
- **Codex CLI**는 Claude Code와 거의 동일한 기능을 제공한다. 다만 설정 파일 형식이 다르므로 **변환 작업**이 필요하다.
- **Cursor**는 rules 정도만 적용할 수 있다. Agent·Skill·권한 차단·자동 경고는 없다.

---

### Codex CLI를 사용한다면

Codex CLI는 2025년 OpenAI가 출시한 터미널 코딩 에이전트로, Claude Code와 매우 유사한 구조를 가진다.
다음과 같이 설정 파일을 **변환**해서 사용할 수 있다.

| Claude Code | Codex CLI 대응 | 변환 방법 |
|-------------|---------------|-----------|
| `CLAUDE.md` | `AGENTS.md` | 파일명 변경 또는 `config.toml`의 `project_doc_fallback_filenames = ["CLAUDE.md"]` 설정 |
| `.claude/rules/*.md` | `AGENTS.md`에 포함 또는 fallback에 추가 | `AGENTS.md`에서 참조 링크 형태로 포함 |
| `.claude/agents/*.md` | Codex Subagent (`config.toml`) | 각 에이전트 정의를 Codex 형식으로 변환 |
| `.claude/skills/*` | Codex Skill (`SKILL.md` 형식) | 각 스킬을 `SKILL.md` 디렉토리 형식으로 변환 |
| `.claude/settings.json` deny | `config.toml` sandbox / managed denials | 경로별 차단을 sandbox 규칙으로 변환 |
| `.claude/settings.json` PreToolUse hook | `config.toml` `[hooks]` 또는 `hooks.json` | hook 명령어를 그대로 옮길 수 있음 (PreToolUse deny 지원) |

**Codex CLI 사용자 최소 설정:**
1. `~/.codex/config.toml` 또는 프로젝트 루트에 `AGENTS.md` 생성 (팀장에게 변환본 요청)
2. Subagent / Skill 정의를 Codex 형식으로 변환 (팀장 또는 AI에게 변환 의뢰)
3. Hook 명령어 이전 (PreToolUse 동일 구조 사용)

> **참고:** 현재 이 프로젝트의 `.claude/` 설정은 Claude Code 기준이다. Codex CLI용 변환본은 별도로 관리되지 않으므로, Codex 사용자가 늘어나면 팀 차원에서 `AGENTS.md` 등 변환본을 동기화하는 정책이 필요하다.

---

### Cursor를 사용한다면

Cursor는 `.cursor/rules/` 폴더에 마크다운 파일을 넣으면 AI가 해당 내용을 읽고 지시사항으로 따른다.
이 프로젝트의 `.claude/rules/*.md` 내용을 그대로 복사해 넣으면 **코딩 규칙·안전 규칙은 실질적으로 적용된다.**

단, 아래 기능은 Cursor에서 동작하지 않는다:

| 기능 | 상태 | 대응 방법 |
|------|------|-----------|
| 규칙 적용 (rules) | ⚠️ 별도 설정 필요 | `.cursor/rules/`에 규칙 파일 복사 |
| Agent 전환 | ❌ 불가 | 채팅에서 역할을 직접 지시 |
| Skill 자동 참조 | ❌ 불가 | 필요한 스킬 파일 내용을 채팅에 직접 붙여넣기 |
| 파일 수정 권한 차단 | ❌ 불가 | 팀원이 직접 주의 |
| 자동 경고 알림 | ❌ 불가 | 팀원이 직접 주의 |

**Cursor 사용자 최소 설정:**
1. `.cursor/rules/` 폴더에 `safety.md`, `engineering.md`, `process.md` 복사 (팀장에게 문의)
2. 공유 인터페이스(`interfaces/`, `db_core/` 등) 수정 전 반드시 팀장 확인
3. 안전 코드(`motion/`, `safety/`) 수정 전 반드시 팀장 확인

> **주의:** rules를 설정해도 AI가 실수할 수 있다. Cursor에는 차단 시스템이 없기 때문에
> 중요한 파일 수정은 반드시 팀장 검토를 받아야 한다.

---

## 6. Claude Code 팀원 초기 설정

```bash
# 1. Claude Code 설치
npm install -g @anthropic-ai/claude-code

# 2. 저장소 클론
git clone <repo-url>
cd robot-arm-project

# 3. 프로젝트 열기 (CLAUDE.md·rules·agents 자동 로드됨)
claude

# 4. (선택) OMC 플러그인 설치 — 세션 메모리, 추가 에이전트, 슬래시 명령 등 부가 기능
claude plugin install oh-my-claudecode@omc
# 이 프로젝트의 기본 에이전트(robot-arm-planner, safety-reviewer, interface-guardian)는
# OMC 없이도 동작한다. OMC는 추가 편의 기능을 제공한다.

# 5. (선택) 개인 설정 파일 생성 — 저장소에 올라가지 않는 개인 환경 설정
touch .claude/settings.local.json

# 6. 환경변수 설정
cp .env.example .env
# .env 파일을 열어 실제 값 입력 (팀장에게 값 문의)
```

설정 완료 후 Claude Code 채팅창에서 바로 질문하면 된다:
```
"이 프로젝트가 뭐야?" → CLAUDE.md 기반으로 설명
"Track C VLA 코드 작성해줘" → safety.md 규칙 적용된 코드 생성
"robot-arm-planner 에이전트로 Phase 1 계획 잡아줘" → 전용 에이전트 전환
```

---

## 7. 문의

AI 설정 관련 문의는 팀장(프로젝트 설정 담당)에게 한다.
규칙(`rules/`)이나 에이전트(`agents/`) 변경은 팀 합의 후 진행한다.
