---
name: code-review-checklist
description: >
  코드 리뷰 체크리스트 — PR 리뷰 절차, 심각도 분류, 코드 스멜 탐지,
  안전·보안·성능·가독성 관점 점검, 리뷰 코멘트 작성 패턴.
  PR 제출 전 셀프 리뷰, 팀 코드 리뷰 시 활성화.
when_to_use: >
  PR 리뷰 요청 전 셀프 체크, 코드 리뷰 수행, 리뷰 코멘트 작성,
  코드 스멜 탐지, 이 프로젝트의 .claude/rules/ 준수 여부 점검 시.
---

# 코드 리뷰 체크리스트

> 이 프로젝트의 `.claude/rules/` 폴더 기준. 안전 관련 코드는 [`safety-reviewer`](../agents/safety-reviewer.md) 에이전트 별도 수행.

## 1. 심각도 분류

| 레벨 | 의미 | 처리 |
|------|------|------|
| 🔴 **CRITICAL** | 병합 차단. 안전 위반, 데이터 손실, 보안 취약점 | 즉시 수정 필수 |
| 🟠 **HIGH** | 버그, 룰 위반, 주요 논리 오류 | 이번 PR에서 수정 |
| 🟡 **MEDIUM** | 성능, 가독성, 테스트 부족 | 수정 권장 |
| 🔵 **LOW** | 스타일, 선호도, 개선 제안 | 선택 적용 |
| 💬 **NIT** | 아주 사소한 것 (오타, 공백) | 자유 |

## 2. 안전 체크리스트 (이 프로젝트 전용)

> SafetyValidator 관련은 `safety-reviewer` 에이전트가 심층 분석. 여기서는 빠른 1차 점검.

- [ ] **S-1**: VLA 출력이 `SafetyValidator.check()` 통과 후 SDK 호출하는가?
- [ ] **S-2**: DB gate(`check_feasibility`)가 VLA/모션 호출 전 실행되는가?
- [ ] **S-3**: E-stop 경로에 예외 처리가 없어 차단되지 않는가? (응답 ≤ 500ms)
- [ ] **S-4**: SafetyWatchdog 비활성화 / 타임아웃 연장 코드가 없는가?
- [ ] **S-5**: 관절 속도/가속도 한계가 `config/robot_poses.yaml`에서 읽어오는가? (소프트 리밋 준수)
- [ ] **S-6**: v1.0에서 직접 핸드오버(사람 손 전달) 동작이 구현되지 않았는가?
- [ ] **S-7**: `is_moving=True` 구간에서 STT 추론 / 새 명령 수락이 차단되는가?
- [ ] **S-8**: `out`/`staged` 임계 시간 초과 시 `missing` → `fod_alert` 전이가 보장되는가?
- [ ] **S-9**: 부팅 reconciliation 완료 전 모든 명령이 거부되는가?
- [ ] **E-2**: `rclpy` import가 `db_core/`, `plc_core/`, `unit_actions/`, `track_c_vla.py`에 없는가?
- [ ] **E-5**: `SafetyError` 예외를 잡아서 무시하거나 복구하는 코드가 없는가?

## 3. 엔지니어링 체크리스트

### 단위 및 좌표 (E-1)
- [ ] 관절 값 단위가 rad인가? (degree 혼용 없음)
- [ ] 위치 단위가 m인가? (mm 혼용 없음)
- [ ] 함수 시그니처·변수명에 단위 명시 (`pos_m`, `angle_rad`, `timeout_s`)

### 설정 (E-4)
- [ ] 좌표, 임계값, 시간 상수가 코드에 하드코딩되지 않았는가?
- [ ] 새 파라미터는 `config/*.yaml`에 추가했는가?
- [ ] 시크릿(토큰, 비밀번호)이 코드나 yaml에 없는가? → `.env`

### 타입 및 인터페이스
- [ ] 공개 API에 타입 힌트가 있는가?
- [ ] `Any` 사용이 필요한 이유가 있는가?
- [ ] `Optional[X]` → `X | None` (Python 3.10+)

### 의존성 (E-2/E-3)
- [ ] `db_core/` ← `plc_core/` 역방향 의존이 없는가?
- [ ] `unit_actions/`에 ROS2 import가 없는가?
- [ ] 새 외부 라이브러리 추가 시 `requirements.txt` / `setup.py` 갱신했는가?

### 에러 처리 (E-5)
- [ ] `except Exception: pass` (silent swallow) 없음
- [ ] 예외에 충분한 컨텍스트 포함 (tool_id, joint, 값 등)
- [ ] 재시도 대상이 아닌 에러(안전, 한계 초과)에 retry 없음

### 로깅 (E-6)
- [ ] 레벨이 적절한가? (DEBUG=세부, INFO=흐름, WARNING=예상 실패, ERROR=비정상)
- [ ] 민감 정보(토큰, 비밀번호)가 로그에 없는가?
- [ ] f-string 로그 사용 금지 → `logger.info("msg %s", var)` 형식

## 4. 코드 스멜 탐지

### 복잡도
```
함수 길이 > 50줄      → 분리 고려
중첩 깊이 > 3         → 조기 반환 패턴 적용
인자 개수 > 5         → dataclass/TypedDict로 묶기
```

### 중복
```python
# ❌ 3곳에서 동일 변환
angle_deg = angle_rad * 180 / math.pi

# ✅ 유틸리티 함수 1개
def rad_to_deg(rad: float) -> float: ...
```

### 매직 넘버
```python
# ❌
if elapsed > 600:   # 600이 뭔지 모름

# ✅ config에서 읽거나 이름 있는 상수
FOD_TIMEOUT_S = config.fod.checkout_timeout_minutes * 60
if elapsed > FOD_TIMEOUT_S:
```

### Boolean 함정
```python
# ❌ 인자 의미 불명확
move(True, False, True)

# ✅ keyword argument
move(use_gripper=True, slow_mode=False, wait=True)
```

### 조기 반환 (가드 절)
```python
# ❌ 깊은 중첩
def fetch(tool_id):
    if tool_id in db:
        if db[tool_id].status == "available":
            if camera.ready():
                do_fetch()

# ✅ 조기 반환
def fetch(tool_id):
    if tool_id not in db:
        raise ToolNotFound(tool_id)
    if db[tool_id].status != "available":
        raise DBGateBlocked(tool_id, db[tool_id].status)
    if not camera.ready():
        raise PerceptionError("카메라 미준비")
    do_fetch()
```

## 5. 테스트 체크리스트 (P-1)

- [ ] 새 공개 함수에 테스트가 있는가?
- [ ] 엣지 케이스(빈 목록, None, 경계값)가 커버되는가?
- [ ] `unit_actions/` 변경 시 `pytest unit_actions/tests/` 통과하는가?
- [ ] 골든 파일이 변경된 BT 노드에 갱신됐는가?
- [ ] 테스트가 하드웨어에 의존하지 않는가? (mock 사용)
- [ ] 테스트 이름이 의도를 설명하는가? (`test_fetch_blocks_when_tool_missing`)

## 6. 보안 체크리스트

- [ ] SQL 쿼리에 파라미터 바인딩 사용 (f-string SQL 없음)
- [ ] 외부 입력(음성 명령 텍스트)을 `eval`, `exec`, `subprocess` shell=True에 전달 안 함
- [ ] 파일 경로 traversal 방어 (`..` 포함 경로 검증)
- [ ] `.env` 파일이 `.gitignore`에 포함돼 있는가?

## 7. 리뷰 코멘트 작성 패턴

### 명확한 제안 포함
```
❌ "이 코드 나쁜 것 같아요"

✅ [HIGH] `arm.move_joint(pos)` 호출 전 SafetyValidator 확인이 없습니다.
   .claude/rules/safety.md S-1에 따라 아래처럼 변경해주세요:
   ```python
   if not safety_validator.check(trajectory):
       raise SafetyError(...)
   arm.move_joint(pos)
   ```
```

### 질문형 (확인 필요 시)
```
[MEDIUM] `timeout=5.0`이 하드코딩돼 있는데, config/robot_poses.yaml에서
읽어오도록 의도한 건가요? (.claude/rules/engineering.md E-4)
```

### 칭찬 포함 (선택)
```
[NIT] 앞 PR의 피드백 잘 반영해주셨네요. 에러 컨텍스트가 훨씬 명확해졌습니다.
```

## 8. PR 제출 전 셀프 체크

```
□ git diff로 전체 변경 확인 (불필요한 파일 포함 여부)
□ 디버그 print / TODO 제거
□ 테스트 로컬 실행 완료
□ CHANGELOG 갱신 (interfaces/ 변경 시 interfaces/CHANGELOG.md)
□ 안전 관련 코드 → safety-reviewer 에이전트 검토 요청
□ interfaces/ 변경 → interface-guardian 에이전트 검토 요청
□ PR 설명에 "변경 이유" 포함
```

## 9. 참고

- 프로젝트 룰: [`.claude/rules/safety.md`](../rules/safety.md), [`.claude/rules/engineering.md`](../rules/engineering.md), [`.claude/rules/process.md`](../rules/process.md)
- 관련 에이전트: [`safety-reviewer`](../agents/safety-reviewer.md), [`interface-guardian`](../agents/interface-guardian.md)
- 관련 스킬: [`error-handling-patterns`](error-handling-patterns.md), [`pytest-patterns`](pytest-patterns.md)
