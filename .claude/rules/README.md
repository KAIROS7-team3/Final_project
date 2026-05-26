# 프로젝트 룰

이 폴더는 모든 코드 작성·리뷰·머지 작업에 적용되는 엔지니어링 및 운영 룰을 정의한다.

## 파일 구성

| 파일 | 내용 | 우선순위 |
|------|------|----------|
| [safety.md](safety.md) | 안전 invariant — 트랙 무관 절대 위반 금지 | 🔴 최우선 |
| [engineering.md](engineering.md) | 단위·좌표·의존성·에러/로깅·코딩 컨벤션 | 🟠 높음 |
| [process.md](process.md) | 테스트, CHANGELOG, 시크릿, 커밋 규칙 | 🟡 보통 |

## AI 에이전트 사용 지침

1. 코드 수정 전 관련 룰 파일 검토 필수
2. **룰 충돌 시 우선순위: `safety.md` > `engineering.md` > `process.md`**
3. `safety.md` 위반은 머지 차단 — `safety-reviewer` 에이전트가 자동 검사
4. 룰 자체 변경은 팀 합의 필요 (PR 리뷰 + CHANGELOG 갱신)

## 룰 추가/변경 절차

- 새 룰 제안 → PR로 `.claude/rules/` 변경 → 팀 리뷰 → 머지
- 변경 사항은 `CHANGELOG.md`(루트)에 한 줄 요약 기록
- 기존 코드와 충돌하면 마이그레이션 계획 포함
