# 기여 가이드

> 이 저장소에 기여하기 전에 반드시 읽어야 하는 규칙과 절차.
> 세부 룰은 [`.claude/rules/`](.claude/rules/) 폴더가 최종 기준이다.

---

## 초기 설정

```bash
# 1. 저장소 클론 (서브모듈 포함)
git clone https://github.com/KAIROS7-team3/Final_project.git
cd Final_project
git submodule update --init --recursive   # doosan-robot2 (humble) 포함

# 2. OMC 플러그인 설치 (개인 레벨, 1회만)
claude plugin install oh-my-claudecode@omc

# 3. 개인 설정 파일 생성 (gitignored)
touch .claude/settings.local.json

# 4. ROS2 빌드 (Track A/B)
cd ros2_ws
colcon build
source install/setup.bash
```

---

## 시작 전 체크리스트

- [ ] [CLAUDE.md](CLAUDE.md) — 아키텍처 개요 숙지
- [ ] [`.claude/rules/safety.md`](.claude/rules/safety.md) — 안전 룰 (최우선)
- [ ] [`.claude/rules/engineering.md`](.claude/rules/engineering.md) — 코딩 표준
- [ ] [`.claude/rules/process.md`](.claude/rules/process.md) — 커밋·테스트·CHANGELOG

---

## 브랜치 전략

| 브랜치 이름 | 용도 |
|------------|------|
| `main` | 항상 머지 가능 상태. **직접 push 금지** |
| `feat/<설명>` | 새 기능 (예: `feat/track-c-vla`) |
| `fix/<설명>` | 버그 수정 |
| `safety/<설명>` | 안전 관련 hotfix (우선 머지) |
| `docs/<설명>` | 문서만 변경 |

---

## PR 제출 절차

1. `main`에서 브랜치 생성 → 작업
2. 커밋 메시지 형식 확인 ([P-4](.claude/rules/process.md))
3. 변경 범위에 따라 아래 항목 완료 확인:

| 변경 대상 | 추가 작업 |
|-----------|----------|
| `interfaces/` msg/srv/action | `interfaces/CHANGELOG.md` 갱신 + `interface-guardian` 검토 |
| `db_core/`, `plc_core/` API | 해당 `CHANGELOG.md` 갱신 + `interface-guardian` 검토 |
| `unit_actions/` 시그니처 | `unit_actions/CHANGELOG.md` 갱신 + `interface-guardian` 검토 |
| 안전 관련 코드 (motion/, safety/) | `safety-reviewer` 에이전트 검토 |
| `.claude/rules/*.md` | 팀 합의 필수 + 루트 `CHANGELOG.md` 갱신 |

4. PR 설명에 다음 포함:
   - **변경 이유** (WHY)
   - AI 도구 활용 여부 ("AI-assisted" 또는 "AI-generated" 명시, P-6)
   - 관련 테스트 실행 결과

---

## 커밋 메시지 형식

```
<type>(<scope>): <subject>
```

| type | 의미 |
|------|------|
| `feat` | 새 기능 |
| `fix` | 버그 수정 |
| `refactor` | 동작 변경 없는 리팩토링 |
| `test` | 테스트 추가/수정 |
| `docs` | 문서만 변경 |
| `chore` | 빌드/설정 변경 |
| `safety` | 안전 관련 변경 (🔴 특별 주의) |

예시:
```
feat(interfaces): add RobotStatus.msg for audio gating
fix(track-c): SafetyValidator 우회 경로 차단
safety(motion): joint limit 검증 추가
docs(rules): 의존성 그래프 명시
```

---

## 테스트 요구사항

```bash
# Python 단위 테스트
python -m pytest <module>/tests/ -v

# ROS2 통합 테스트 (Track A/B)
colcon test --packages-select <package>
colcon test-result --verbose

# 안전 회귀
python -m pytest tests/safety/ -v
```

- 안전 critical 경로는 **테스트 없으면 머지 금지** (P-1)
- 커버리지 목표: 80% 이상 (안전 모듈 100%)

---

## AI 에이전트 사용 가이드

| 작업 유형 | 사용 에이전트 |
|-----------|--------------|
| 프로젝트 기획 / 설계 변경 | `robot-arm-planner` |
| 안전 코드 검토 | `safety-reviewer` |
| 공유 인터페이스 변경 전 | `interface-guardian` |
| 일반 구현 / 디버깅 | 기본 Claude Code |

AI 도구 상세 설정 → [`docs/ai-setup.md`](docs/ai-setup.md)

---

## 절대 금지 사항

- `git push --force` (CI deny 규칙으로 차단됨)
- `.env` 파일 커밋 (시크릿 노출)
- `rclpy` import를 `db_core/`, `plc_core/`, `unit_actions/`, `track_c_vla.py`에 추가
- SafetyValidator 우회 코드 (S-1)
- 좌표·임계값 코드 하드코딩 (E-4)
- `print()` 사용 (테스트 fixture 제외, E-6)

---

## 문의

팀 내 채널 또는 GitHub Issues 활용. 긴급 안전 이슈는 팀장에게 직접 연락.
