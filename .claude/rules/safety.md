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

## S-6. v1.0 핸드오버 금지

v1.0에서 로봇이 사람 손에 직접 공구를 전달하는 동작은 구현 금지.
모든 전달은 Staging Area 거치 방식으로만 수행. 직접 핸드오버는 v2.0+ 과제.

## S-7. 동작 중 음성 수신 차단

- `is_moving=True`일 때 STT 추론 / 새 명령 수락 금지
- 홈 복귀 + `is_moving=False` 전환 후 수신 재개
- 음성 E-stop("멈춰")은 v2.0+ 기능 — v1.0에서 구현하지 않음 (오인식 위험)

## S-8. FOD 상태 전이 무결성

- `out` 또는 `staged` 상태가 임계 시간(기본 10분, `config/fod.yaml`)을 초과하면 `missing` 자동 전환
- `missing` → 30초 내 `fod_alert` + PLC 노랑 점멸
- `missing` 또는 `fod_alert` 상태의 공구는 모든 `fetch` 거부

## S-9. 부팅 시 reconciliation

- 부팅마다 YOLOv8로 슬롯 전체 스캔 → DB 상태와 비교
- 불일치 발견 시 자동 수정 금지 — 운영자에게 수동 확인 요청 + PLC 노랑 점멸
- 확인 완료 전까지 모든 명령 거부
