# ADR — 안전

> 참조: [인덱스](index.md)
> 안전 룰 세부 사항 → [`.claude/rules/safety.md`](../../.claude/rules/safety.md)

---

## ADR-005: VLA 안전 경계 — SafetyValidator 필수

- **결정**: VLA 출력은 반드시 `SafetyValidator.check()`를 통과해야 하드웨어 접근 가능
- **이유**: HRC 환경에서 VLA action error 위험 차단
- **검증 범위**: joint limit, 속도/가속도 한계, Cartesian 작업공간 경계, self-collision
- **제약**: SafetyValidator 우회 코드 작성 금지 (S-1)

```python
# ✅ 올바름
joint_traj, gripper_cmd = vla.infer(...)
if safety.check(joint_traj):
    arm.execute(joint_traj)

# ❌ 금지
arm.execute(vla.infer(...))  # SafetyValidator 우회
```

---

## 미결 사항

| # | 항목 | 결정 시점 |
|---|------|----------|
| 7 | 안전 E-Stop v2.0 적용 시점 (음성 E-stop "멈춰" 등) | Phase 7 이후 |
| 33 | Wake word 감지 방식 (항상-on STT / 키워드 모델 / 물리 버튼) | Phase 4 전 |
