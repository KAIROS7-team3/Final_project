# 안전 룰 (Safety Invariants)

> 🔴 **트랙 무관 절대 위반 금지.** `safety-reviewer` 에이전트가 자동 검사한다.

## S-1. SafetyValidator 통과 필수 (Track C)

VLA 모델 출력은 **반드시** `SafetyValidator.check()`를 통과한 뒤 Doosan Python SDK에 전달한다.

```python
# ✅ 올바름
joint_traj, gripper_cmd = vla.infer(...)
if safety.check(joint_traj):
    arm.execute(joint_traj)

# ❌ 금지 — SafetyValidator 우회
joint_traj = vla.infer(...)
arm.execute(joint_traj)  # 위반: safety check 없음
```

검증 범위: joint limit, 속도/가속도 한계, Cartesian 작업공간, self-collision.

## S-2. DB Gate 우회 금지

`fetch` / `return` 명령은 DB `check_feasibility()` 통과 후에만 하드웨어 동작 시작.

- Track A/B: Gemma 4가 `CheckToolFeasibility` 서비스 호출
- Track C: `track_c_vla.py`가 `db_core.get_tool_status()` 직접 쿼리

DB 연결 실패 시 캐시 TTL(5분) 내에서만 동작. TTL 초과 → 모든 명령 거부.

## S-3. E-stop 경로 무결성

- `emergency_stop()`은 BT/VLA 상태와 무관하게 **항상 호출 가능**해야 함
- E-stop 응답 시간 ≤ 500ms (수락 기준)
- E-stop 후 시스템은 PLC 빨강 Solid + DB 로그 기록 후 정지 상태 유지 (자동 재시작 금지)

## S-4. SafetyWatchdog 무결성

- 하트비트 타임아웃 500ms 초과 시 자동 정지 트리거
- 워치독 비활성화 / 타임아웃 연장은 절대 금지
- 디버깅 목적이라도 production 코드에 `disable_watchdog()` 호출 금지

## S-5. Joint / 속도 한계

- 모든 joint command는 e0509 operational range 내 (소프트 리밋이 하드웨어 리밋보다 항상 좁음)
- 속도 오버라이드는 100% 이하로 클램프. 운영자 명시적 확인 없이 초과 금지
- Cartesian 속도 한계: 협동 모드 250mm/s 이하 (HRC 표준)

## S-6. 핸드오버 안전 조건 (팀 합의로 v1.0 구현 허용)

로봇이 사람 손에 직접 공구를 전달하는 동작을 허용한다. 단, 아래 조건을 모두 만족해야 한다.

- **속도 제한**: `place_on_hand` 실행 시 action scale 0.2 이하 강제
- **손 안정성 확인**: `/hand/pose` 변화량이 threshold 초과 시 즉시 abort → Staging Area fallback
- **Force 모니터링**: 접근 중 joint torque threshold 초과 시 즉시 정지
- **손 감지 실패 시**: Staging Area fallback으로 자동 전환 (하드오버 강제 금지)
- **손바닥 방향 확인**: `/hand/ready`가 True (손바닥 위 향함 + 안정적)일 때만 접근 시작

## S-7. 동작 중 음성 수신 차단

- `is_moving=True`일 때 STT 추론 / 새 명령 수락 금지
- 홈 복귀 + `is_moving=False` 전환 후 수신 재개
- 음성 E-stop("멈춰")은 v2.0+ 기능 — v1.0에서 구현하지 않음 (오인식 위험)

## S-8. FOD 상태 전이 무결성

  ### staged → missing (비전 기반, 상시 관제)
  - `staged` 공구가 탑뷰 카메라(`/vision/detections/top_view`)에서 **60초 연속 미감지**되면 `missing` 자동 전환
  - BT pick-from-staging이 정상 실행된 경우는 DB 전이가 선행되므로 오탐 없음
  - 노드 기동 직후에는 staged 공구에 60초 유예(grace) 부여 — 초기 미감지를 missing으로 오판하지 않음

  ### out → missing (타임아웃 기반)
  - `out` 상태가 `checkout_timeout_minutes`(기본 10분, `config/fod.yaml`)을 초과하면 `missing` 전환
  - operator가 공구를 들고 있는 동안은 탑뷰로 감지 불가 → 타임아웃으로만 판단

  ### missing → fod_alert
  - `missing` 상태가 30분 지속되면 `fod_alert` + PLC 노랑 점멸
  - 즉각 경보가 아닌 30분 유예를 두어 작업자가 정비 목적으로 가져간 정상 케이스를 구분

  ### 공통
  - `missing` 또는 `fod_alert` 상태의 공구는 모든 `fetch` 거부
  - 파라미터: `config/fod.yaml` (`staging_vision_timeout_s`, `checkout_timeout_minutes`, `missing_to_alert_seconds`)

## S-9. 부팅 시 reconciliation

- 부팅마다 YOLOv8로 슬롯 전체 스캔 → DB 상태와 비교
- 불일치 발견 시 자동 수정 금지 — 운영자에게 수동 확인 요청 + PLC 노랑 점멸
- 확인 완료 전까지 모든 명령 거부
