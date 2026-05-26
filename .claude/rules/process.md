# 프로세스 룰

> 🟡 테스트, CHANGELOG, 시크릿, 커밋, 코드 리뷰 규칙.

## P-1. 테스트 요구사항

### 단위 테스트 (필수)
- 모든 새 `unit_actions/`, `db_core/`, `plc_core/` 함수: 최소 1개 happy path + 1개 failure path 테스트
- 안전 critical 경로(`SafetyValidator.check`, joint limit, E-stop 트리거): **단위 테스트 없으면 머지 금지**
- 커버리지 목표: 80% 이상 (안전 모듈은 100%)

### 통합 테스트
- `interfaces/` 변경 시 `launch_testing` 기반 통합 테스트 추가
- BT 노드 변경 시 골든 파일 회귀 테스트 (Gazebo 시뮬레이션)
- Track C 변경 시 subprocess 기반 E2E 테스트

### 실행 명령
```bash
# Python 단위 테스트
python -m pytest <module>/tests/

# ROS2 통합 테스트
colcon test --packages-select <package>
colcon test-result --verbose

# 안전 회귀
python -m pytest tests/safety/ -v
```

## P-2. CHANGELOG 관리

다음 변경 시 `<package>/CHANGELOG.md` 갱신 필수:

| 대상 | 트리거 |
|------|--------|
| `interfaces/CHANGELOG.md` | msg/srv/action 추가·수정·삭제 |
| `db_core/CHANGELOG.md` | DBClient API 변경, 스키마 마이그레이션 |
| `plc_core/CHANGELOG.md` | PLCClient API 변경 |
| `unit_actions/CHANGELOG.md` | 함수 시그니처 변경 |
| 루트 `CHANGELOG.md` | 룰 변경, 주요 아키텍처 결정 |

### 형식 (Keep a Changelog 기반)
```markdown
## [Unreleased]

### Added
- ToolStatus.msg에 `confidence` 필드 추가 (#42)

### Changed
- `DBClient.get_tool_status()`가 dict 대신 dataclass 반환

### Deprecated
### Removed
### Fixed
### Security
```

## P-3. 시크릿 관리

- API 키, 클라우드 토큰, DB 비밀번호는 `.env`에만 저장
- `.env`는 `.gitignore`로 차단됨 (이미 적용)
- `.env.example`은 git 포함 — 변수 이름만 (값 없음)
- 코드에서는 `os.environ['VAR_NAME']` 또는 `pydantic-settings` 사용
- 시크릿이 commit history에 포함되면 **즉시 키 revoke + git history 정리**

```bash
# .env.example (git 포함)
HUGGINGFACE_TOKEN=
LAMBDA_LABS_API_KEY=
VLA_MODEL_URL=
```

## P-4. 커밋 메시지 형식

### Conventional Commits 변형
```
<type>(<scope>): <subject>

<body (선택)>

<footer (선택)>
```

| type | 의미 |
|------|------|
| `feat` | 새 기능 |
| `fix` | 버그 수정 |
| `refactor` | 동작 변경 없는 리팩토링 |
| `test` | 테스트 추가/수정 |
| `docs` | 문서만 변경 |
| `chore` | 빌드/설정 변경 |
| `safety` | 안전 관련 변경 (특별 표시) |

### scope 예시
`interfaces`, `voice`, `vision`, `orchestrator`, `motion`, `db`, `plc`, `track-c`, `rules`, `config`

### 예시
```
feat(interfaces): add RobotStatus.msg for audio gating
fix(track-c): SafetyValidator 우회 경로 차단
safety(motion): joint limit 검증 추가 (Phase 5 회귀 방지)
docs(rules): 의존성 그래프 명시
```

## P-5. 브랜치 전략

- `main` — 항상 머지 가능 상태. 직접 push 금지
- `feat/<설명>` — 새 기능 (예: `feat/track-c-vla`)
- `fix/<설명>` — 버그 수정
- `safety/<설명>` — 안전 관련 hotfix (우선 머지)
- 머지는 PR + 리뷰 1명 이상 + CI 통과 후

### 보호된 동작 (에이전트 차단)
- `git push --force` (settings.json deny — 절대 금지)
- `git push`: 에이전트가 실행 시 사용자 확인 프롬프트 필수. 어떤 브랜치·커밋을 push하는지 확인 후 승인.
- `git rebase main` on shared branches
- main 직접 commit

## P-6. 코드 리뷰

### 리뷰어 체크리스트
- [ ] `.claude/rules/safety.md` 위반 없음
- [ ] `.claude/rules/engineering.md` E-1~E-9 준수
- [ ] 테스트 추가/갱신 (P-1)
- [ ] CHANGELOG 갱신 (P-2)
- [ ] 시크릿/토큰 노출 없음 (P-3)
- [ ] 커밋 메시지 형식 (P-4)
- [ ] interfaces 변경 시 `interface-guardian` 검토 완료
- [ ] AI가 생성한 코드·문서는 PR 설명에 명시 ("AI-assisted" 또는 "AI-generated")

### 자동 검사
- 안전 코드 변경 → `safety-reviewer` 에이전트 자동 트리거
- interfaces 변경 → `interface-guardian` 에이전트 자동 트리거
- 일반 리뷰 → 기본 Claude Code

## P-7. 미결 사항 처리

- `robot-arm-project.md` 섹션 17 미결 사항 결정 시 동일 PR에 문서 갱신 포함
- 결정 형식: `~~미결~~ **결정: <결정 내용>**`
- 결정 시점이 도래했는데 미정이면 issue 생성하여 추적
