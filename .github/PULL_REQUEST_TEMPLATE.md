## 변경 내용

<!-- 무엇을, 왜 변경했는지 간략히 -->

## 체크리스트

### 공통
- [ ] [`.claude/rules/safety.md`](../.claude/rules/safety.md) 위반 없음
- [ ] [`.claude/rules/engineering.md`](../.claude/rules/engineering.md) E-1~E-9 준수
- [ ] 시크릿/토큰 노출 없음 (P-3)
- [ ] 커밋 메시지 형식 확인 (P-4)

### 해당 항목만 체크
- [ ] 테스트 추가/갱신 (P-1) — 안전 critical은 필수
- [ ] CHANGELOG 갱신 (P-2) — interfaces/db_core/plc_core/unit_actions 변경 시
- [ ] `interface-guardian` 검토 완료 — interfaces/, db_core/, plc_core/, unit_actions/ 변경 시
- [ ] `safety-reviewer` 검토 완료 — 모션/VLA 출력/E-stop/SafetyValidator 변경 시
- [ ] AI 생성 코드·문서 명시 ("AI-assisted" 또는 "AI-generated")

## 테스트 방법

<!-- 리뷰어가 검증할 수 있는 방법 -->

## 관련 이슈 / ADR

<!-- closes #이슈번호 또는 ADR 번호 -->
