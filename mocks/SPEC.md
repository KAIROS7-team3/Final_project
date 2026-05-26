# SimulatedArm / SimulatedGripper / SimulatedCamera — 동작 명세

> Phase 0 HAL mock 구현(`hal/simulated/`)의 계약 문서.
> 실제 하드웨어 없이 BT·unit_actions·Track C 로직을 테스트할 수 있도록 결정론적 동작을 보장한다.

---

## SimulatedArm

| 메서드 | 입력 | 반환 | 사이드이펙트 |
|--------|------|------|-------------|
| `move_to_pose(pose, velocity_scale)` | Pose (base_link, m + quaternion), velocity_scale 0.0–1.0 | `True` | `_current_pose` 갱신, `_moving` 플래그 True→False |
| `move_to_pose` (E-stop 상태) | — | `False` | 없음 |
| `move_to_joint_positions(positions, velocity_scale)` | 6-element float list (rad), velocity_scale | `True` | `_joint_positions` 갱신 |
| `move_to_joint_positions` (joint limit 초과) | — | `False` | 없음, ERROR 로그 |
| `get_joint_states()` | — | `JointStates(positions, velocities=[0]*6, efforts=[0]*6)` | 없음 |
| `get_end_effector_pose()` | — | 마지막으로 이동한 `Pose` | 없음 |
| `emergency_stop()` | — | `None` | `_estop=True`, `_moving=False` |
| `is_moving()` | — | `bool` | 없음 |

**Joint limits**: `[-π, π]` 6축 모두. `move_to_joint_positions` 에서 초과 시 즉시 `False` 반환.

**초기 상태**: `_joint_positions=[0]*6`, `_current_pose=Pose(position=(0.5,0.0,0.5), quaternion=(0,0,0,1))`, `_moving=False`, `_estop=False`

---

## SimulatedGripper

| 메서드 | 입력 | 반환 | 사이드이펙트 |
|--------|------|------|-------------|
| `set_position(position, force)` | position 0.0–1.0, force N | `True` | `_position` 갱신, `_grasping = position > 0.8` |
| `set_position` (E-stop 상태) | — | `False` | 없음 |
| `set_position` (범위 초과) | position < 0 or > 1 | `False` | ERROR 로그 |
| `get_position()` | — | `float` (0.0–1.0) | 없음 |
| `is_grasping()` | — | `bool` | 없음 |
| `open()` | — | `bool` | `set_position(0.0)` 호출 |
| `close(force)` | — | `bool` | `set_position(1.0, force)` 호출 |
| `emergency_stop()` | — | `None` | `_estop=True` |

**초기 상태**: `_position=0.0`, `_grasping=False`, `_estop=False`

---

## SimulatedCamera

| 메서드 | 반환 | 비고 |
|--------|------|------|
| `start()` | `None` | `_streaming=True` |
| `stop()` | `None` | `_streaming=False` |
| `is_streaming()` | `bool` | |
| `get_rgb_frame()` | `np.zeros((480,640,3), uint8)` | start() 없이도 반환 |
| `get_depth_frame()` | `np.ones((480,640), float32)` | 1.0m uniform depth |
| `get_aligned_frames()` | `(rgb, depth)` | |
| `get_intrinsics()` | D455f 근사값 fx=fy=385, cx=320, cy=240, 640×480 | |
