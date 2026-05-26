---
name: git-conventions
description: >
  팀 git 워크플로 표준 — Conventional Commits, 브랜치 전략, rebase·머지 패턴,
  PR 작성, 안전한 force-push, history 정리.
  첫 PR 작성, 커밋 메시지 작성, 충돌 해결, history rewriting 시 활성화.
when_to_use: >
  git 커밋 메시지 작성, PR 생성, 브랜치 머지/rebase, 충돌 해결,
  실수 commit 정리, 안전한 history 수정 시.
---

# Git 워크플로 컨벤션

> 프로젝트 룰: [`.claude/rules/process.md`](../rules/process.md) P-4, P-5.

## 1. Conventional Commits

```
<type>(<scope>): <subject>

<body — 선택>

<footer — 선택>
```

### type

| type | 의미 | CI 영향 |
|------|------|---------|
| `feat` | 새 기능 | minor 버전 ↑ |
| `fix` | 버그 수정 | patch 버전 ↑ |
| `refactor` | 동작 변경 없는 리팩토링 | — |
| `test` | 테스트 추가/수정 | — |
| `docs` | 문서만 변경 | — |
| `chore` | 빌드/설정 변경 | — |
| `safety` | 안전 관련 변경 | 우선 머지 |
| `perf` | 성능 개선 | — |
| `style` | 포매팅, 세미콜론 등 | — |
| `ci` | CI 설정 변경 | — |

### scope (이 프로젝트)
`interfaces`, `voice`, `vision`, `orchestrator`, `motion`, `db`, `plc`, `track-c`, `rules`, `config`, `skills`, `agents`

### subject 규칙
- 명령형 현재시제 ("add" not "added" or "adds")
- 첫 글자 소문자
- 마침표 없음
- 50자 이내

### 예시 ✅
```
feat(interfaces): RobotStatus.msg 추가
fix(track-c): SafetyValidator 우회 경로 차단
safety(motion): joint limit 검증 누락 수정
refactor(db): DBClient 캐시 로직 분리
docs(rules): 의존성 그래프 명시
test(unit_actions): grasp 실패 케이스 추가
```

### 예시 ❌
```
Update code             ← type 없음, 모호
fix: 버그 고침            ← scope 없음, subject 모호
feat(stuff): Added X.   ← scope 부정확, "Added" (과거형), 마침표
```

### body / footer
```
feat(interfaces): RobotStatus.msg 추가

is_moving 필드로 오디오 게이팅 신호 전달.
Track A/B와 Track C 모두 동일 형식 사용.

Closes #42
Breaking-Change: 기존 BoolStatus.msg 제거
```

- 본문은 **무엇** + **왜**. **어떻게**는 코드에 있음.
- `Breaking-Change:`는 major 버전 ↑ 트리거
- `Closes #N`, `Fixes #N`은 자동 이슈 종료

## 2. 브랜치 전략

```
main                    ← 항상 머지 가능 상태. 직접 push 금지
├── feat/track-c-vla    ← 새 기능
├── fix/joint-limit-bug ← 버그 수정
├── safety/estop-path   ← 안전 hotfix (우선 머지)
├── refactor/db-cache   ← 리팩토링
└── docs/rules-update   ← 문서
```

### 브랜치 명명
- `<type>/<짧은-설명-kebab-case>` 형식
- type은 commit type과 동일
- 30자 이내

### 워크플로
```bash
# 1. 최신 main에서 시작
git checkout main
git pull origin main

# 2. 작업 브랜치 생성
git checkout -b feat/staging-area-yaml

# 3. 작업 + 커밋 (atomic하게 — 작은 단위로)
git add config/staging_area.yaml
git commit -m "feat(config): staging_area.yaml 초기 스키마"

git add unit_actions/staging_actions.py
git commit -m "feat(unit_actions): place_at_staging / pick_from_staging 구현"

git add unit_actions/tests/test_staging.py
git commit -m "test(unit_actions): staging actions 단위 테스트"

# 4. push + PR
git push -u origin feat/staging-area-yaml
gh pr create --title "feat(config,unit_actions): Staging Area 구현"
```

> **`git push`는 에이전트 차단됨** (`settings.json` deny). 팀원이 직접 실행.

## 3. Rebase vs Merge

### main 동기화 (작업 중)
```bash
# 작업 브랜치에 main의 최신 변경 반영
git checkout feat/my-feature
git fetch origin
git rebase origin/main      # 또는 git merge origin/main
```

| 방식 | 장점 | 단점 |
|------|------|------|
| `rebase` | 선형 history, 깔끔 | force-push 필요 (revision 시) |
| `merge` | force-push 불필요, 안전 | merge commit 다수 |

**권장:** 개인 브랜치는 rebase, 공유 브랜치는 merge.

### main 머지 (PR 머지)
- **Squash merge** — feature 브랜치를 1개 commit으로 main에 추가 (recommended)
- **Rebase merge** — 모든 commit을 main에 선형 추가
- **Merge commit** — merge commit 생성

권장: `squash merge` (feature 브랜치의 작은 commit들이 main을 어지럽히지 않음)

## 4. Atomic Commit

### ✅ 좋은 commit
- 하나의 논리적 변경만 포함
- 빌드/테스트 통과 상태로 commit
- 메시지가 변경 내용을 정확히 설명

### ❌ 피할 것
- "WIP" / "stuff" / "asdf" 같은 commit
- 여러 무관한 변경을 한 commit에
- 빌드 깨진 상태로 commit

### 분할 도구
```bash
# 변경 일부만 stage
git add -p file.py     # patch mode — 헝크별 선택

# 이미 stage한 것 일부 제외
git reset -p file.py

# 여러 commit으로 분할 (rebase interactive)
git rebase -i HEAD~3   # 마지막 3개 commit 편집
# pick → edit로 변경, 저장
git reset HEAD~        # 변경 unstage
# 부분별 add + commit
git rebase --continue
```

## 5. 충돌 해결

```bash
# rebase 중 충돌 발생
git rebase main
# CONFLICT (content): Merge conflict in unit_actions/staging.py

# 충돌 해결 후
git add unit_actions/staging.py
git rebase --continue

# 포기하고 원상 복구
git rebase --abort
```

### 충돌 표시 형식
```python
<<<<<<< HEAD (현재 — 보통 main)
def place_at_staging(self, tool_id: str) -> ActionResult:
=======
def place_at_staging(self, tool_id: str, slot: int = 0) -> ActionResult:
>>>>>>> feat/multi-slot (들어오는 변경)
```

해결:
1. 두 변경을 모두 보고 의도 파악
2. 적절히 통합 또는 한쪽 선택
3. `<<<`, `===`, `>>>` 마커 모두 제거
4. 빌드/테스트 확인
5. `git add` + `git rebase --continue` (또는 `git commit`)

## 6. 안전한 History 수정

### 마지막 commit 수정 (push 전)
```bash
# 메시지만 수정
git commit --amend -m "fix(motion): joint limit 검증 수정"

# 파일 추가
git add forgotten_file.py
git commit --amend --no-edit
```

### Push 후 수정 (위험)
- 개인 브랜치에서만 허용
- 공유 브랜치는 절대 금지
```bash
git push --force-with-lease origin feat/my-feature
# --force-with-lease는 다른 사람의 push를 덮어쓰지 않음 (--force보다 안전)
```

### 절대 금지
- `main`에 force-push (`settings.json` deny + GitHub 보호 규칙 권장)
- 이미 머지된 commit 수정
- 다른 사람이 base로 사용 중인 brunch rebase

## 7. PR 작성

### 제목
commit과 동일 형식: `<type>(<scope>): <subject>`

### 본문 템플릿
```markdown
## 변경 요약
- 무엇을 변경했는가
- 왜 변경했는가

## 변경 사항
- [ ] interfaces/ToolStatus.msg 갱신
- [ ] db_core/client.py에 새 메서드 추가
- [ ] 단위 테스트 추가

## 테스트
- pytest 통과: ✅
- 통합 테스트: ✅
- 안전 회귀: ✅ (해당 시)

## 체크리스트
- [ ] .claude/rules/safety.md 위반 없음
- [ ] .claude/rules/engineering.md 준수
- [ ] CHANGELOG 갱신 (interfaces 변경 시)
- [ ] 시크릿 노출 없음

## 관련 이슈
Closes #N
```

### 리뷰어 호출
- 안전 코드 → `safety-reviewer` 에이전트
- interfaces 변경 → `interface-guardian` 에이전트
- 일반 → 팀원 1명 이상

## 8. 실수 복구

### 잘못 commit한 파일 되돌리기 (push 전)
```bash
# 마지막 commit 전체 취소 (변경은 unstaged 상태로 유지)
git reset HEAD~1

# 마지막 commit 완전 삭제 (변경 폐기 — 위험)
git reset --hard HEAD~1
```

### 시크릿을 commit했을 때 (긴급)
```bash
# 1. 즉시 키 revoke (가장 우선)

# 2. history에서 제거
git filter-repo --invert-paths --path .env  # filter-repo 권장 (git-filter-branch 보다 안전)
# 또는 BFG Repo-Cleaner

# 3. force-push (팀에 사전 알림)
git push --force-with-lease origin main
# 모든 팀원이 다시 clone하거나 rebase 필요
```

### 잘못 삭제한 commit 복구
```bash
git reflog                    # 모든 HEAD 이동 기록
git checkout <hash>           # 잃어버린 commit으로 이동
git branch recovery <hash>    # 복구 브랜치 생성
```

## 9. 흔한 함정

### ❌ `git add .` 무조건 사용
- `.env`, 임시 파일, 캐시 등 의도치 않은 파일 포함 가능
- ✅ 명시적: `git add path/to/file.py` 또는 `git add -p` (검토하며 추가)

### ❌ commit 메시지 한국어/영어 혼용
- 팀 컨벤션 통일 필요 — 이 프로젝트는 한국어 권장 (문서가 한국어)
- type/scope는 영어 고정

### ❌ 큰 PR (1000+ lines)
- 리뷰 품질 ↓, 머지 늦어짐
- ✅ 200~400 lines 단위로 분할

### ❌ 여러 변경을 한 commit에
```bash
# bad
git commit -m "Add staging, fix bug, update docs"

# good — 3개 commit
git commit -m "feat(unit_actions): place_at_staging 추가"
git commit -m "fix(motion): joint limit 오프바이원 버그"
git commit -m "docs(rules): engineering.md E-1 명확화"
```

## 10. 유용한 명령어

```bash
# 최근 변경 확인
git log --oneline -10
git log --all --graph --oneline

# 특정 파일 history
git log -p path/to/file.py
git blame path/to/file.py

# 누가 어떤 줄을 마지막으로 수정?
git blame -L 50,60 path/to/file.py

# 두 브랜치 비교
git diff main..feat/my-branch
git log main..feat/my-branch --oneline

# stash (임시 저장)
git stash push -m "wip: 실험 중"
git stash list
git stash pop

# 특정 commit만 다른 브랜치에 적용
git cherry-pick <hash>
```

## 11. 참고

- Conventional Commits: <https://www.conventionalcommits.org/ko/>
- Pro Git book: <https://git-scm.com/book/ko/v2>
- 프로젝트 룰: [`.claude/rules/process.md`](../rules/process.md)
