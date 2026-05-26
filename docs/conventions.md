# 프로젝트 컨벤션 + 수락 기준

> 이 프로젝트 한정 규칙. 일반 패턴은 `.claude/skills/`, 우선순위 규칙은 `.claude/rules/` 참조.

---

## 1. 개발 환경

| 항목 | 사양 |
|------|------|
| OS | Ubuntu 22.04 |
| 미들웨어 | ROS2 Humble (Track A/B) |
| Python | 3.10+ |
| STT | Whisper small (로컬) |
| 의도 LLM | Gemma 4 (로컬, Track A/B) |
| VLA | 미정 (Track C, 미결 #5) |
| DB | SQLite WAL (ADR-008) |
| PLC 프로토콜 | Modbus RTU via RS-485, LS Electric XBC-DR10E (ADR-009) |

---

## 2. 네이밍 컨벤션

tool_id 형식 및 config 키 형식 → [`.claude/rules/engineering.md` E-8](../.claude/rules/engineering.md)

### 파일 / 모듈 네이밍

| 대상 | 형식 | 예시 |
|------|------|------|
| Python 파일 | `snake_case.py` | `grasp_planner.py` |
| ROS2 msg | `PascalCase.msg` | `ToolStatus.msg` |
| config 파일 | `snake_case.yaml` | `staging_area.yaml` |
| BT 노드 클래스 | `PascalCase` | `LocalizeTool` |
| unit_actions 메서드 | `snake_case` | `place_at_staging()` |

### 커밋 메시지 형식

커밋 메시지 형식은 [`.claude/rules/process.md` P-4](../.claude/rules/process.md) 참조 (Conventional Commits 변형).

`interfaces/` 변경 커밋에는 반드시 `interfaces/CHANGELOG.md` 갱신 포함.

---

## 3. 설정 파일 위치

설정 파일 목록 및 하드코딩 금지 규칙 → [`.claude/rules/engineering.md` E-4](../.claude/rules/engineering.md)

---

## 4. 수락 기준

### 공통 (전 트랙)

- [ ] 공구 분류 정확도 ≥ 95% (실험실 조명 조건)
- [ ] Staging Area 거치 성공률 ≥ 98% (지정 좌표 ±5mm 이내)
- [ ] 슬롯 반납 오차 ≤ 5mm
- [ ] 모든 fetch/return/FOD 이벤트 DB에 타임스탬프와 함께 기록
- [ ] PLC LED가 상태 변경 후 500ms 이내에 반영
- [ ] FOD 알림이 공구 분실 후 30초 이내에 발생
- [ ] 부팅 시 YOLOv8 reconciliation 완료 후 정상 운영 시작
- [ ] systemd를 통해 Vector 16 HX 부팅 시 자동 시작

### Track A/B 전용

- [ ] Gemma 4 의도 정확도 ≥ 97%
- [ ] 불가 명령 (대출 중/FOD) 100% 차단
- [ ] 음성 → Staging Area 거치 완료 **≤ 10초**
- [ ] BT 통합 테스트 통과 (9종 × 3 사이클)

### Track C 전용

- [ ] 음성 → Staging Area 거치 완료 **≤ 13초** (모델 선정 후 확정)
- [ ] VLA action error rate ≤ 3%
- [ ] SafetyValidator가 joint limit/속도 한계 위반 trajectory **100% 차단**
- [ ] DB gate가 불가 명령 정상 차단

---

## 5. 검증 파이프라인

| 변경 대상 | 필수 검증 |
|-----------|-----------|
| `unit_actions/` | `python -m pytest unit_actions/tests/` |
| `interfaces/` (msg/srv/action) | `colcon build --packages-select interfaces` + `interfaces/CHANGELOG.md` 갱신 |
| DB 스키마 변경 | 마이그레이션 스크립트 포함 필수 |
| 모션 / VLA 코드 | `safety-reviewer` 에이전트 검토 필수 |
| BT 노드 | 골든 파일 회귀 테스트 (`colcon test`) |
| `interfaces/`, `db_core/`, `plc_core/`, `unit_actions/` API 변경 | `interface-guardian` 에이전트 검토 필수 |

---

## 6. 브랜치 / PR 규칙

- `git push`: 에이전트가 실행 시 사용자 확인 프롬프트 필수 — 어떤 브랜치·커밋을 push하는지 확인 후 승인. `git push --force`는 settings.json deny로 절대 금지.
- `interfaces/` 변경 PR은 반드시 `interface-guardian` 검토 후 병합.
- 안전 관련 코드(모션, VLA 출력, E-stop) 변경 PR은 `safety-reviewer` 검토 후 병합.

---

## 7. 상세 규칙 참조

| 주제 | 참조 |
|------|------|
| 안전 규칙 (S-1~S-9) | [`.claude/rules/safety.md`](../.claude/rules/safety.md) |
| 엔지니어링 규칙 (E-1~E-9) | [`.claude/rules/engineering.md`](../.claude/rules/engineering.md) |
| 프로세스 규칙 (P-1~P-7) | [`.claude/rules/process.md`](../.claude/rules/process.md) |
| Python 패턴 | [`.claude/skills/python-patterns.md`](../.claude/skills/python-patterns.md) |
| Git 컨벤션 | [`.claude/skills/git-conventions.md`](../.claude/skills/git-conventions.md) |
| 에러 처리 | [`.claude/skills/error-handling-patterns.md`](../.claude/skills/error-handling-patterns.md) |
| 설정 관리 | [`.claude/skills/config-management.md`](../.claude/skills/config-management.md) |
