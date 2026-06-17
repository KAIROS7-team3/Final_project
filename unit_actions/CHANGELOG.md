# Changelog — unit_actions

Keep a Changelog 형식. 함수 시그니처 변경 시 갱신 (P-2).

## [Unreleased]

### Added
- `toolbox_motion.Step`에 `marker: Optional[Literal["pick", "place"]]` 필드 추가
- `toolbox_motion.marked(step, marker)` 헬퍼 추가 — 시퀀스 빌더가 물리적
  집기/놓기 step을 표시. `marker`가 `"pick"`/`"place"`가 아니면 `ValueError`
  (오타로 인한 DB 전이 누락 방지). `tool_action_server`는 marker가 설정된
  step 실행 직후 action feedback(`phase=marker`)을 추가 발행하여,
  orchestrator가 BT 완료를 기다리지 않고 DB 상태를
  `in_slot<->out<->staged`로 즉시 전이시킨다. (`interfaces/CHANGELOG.md` 참조)
- `full_socket_fetch_seq()` / `full_socket_return_seq()`의 소켓 GRIP/RELEASE
  step에 각각 `"pick"` / `"place"` 마커 적용
- `visual_servoing.py` 신규 모듈 — `HandleServoController`(XZ 정렬), `ToolServoController`(XY 정렬), `ServoConfig`, `HandlePose`, `ToolPose`, `VelocityCommand` 추가. rclpy 의존성 없음(E-2).
- `vision_fetch_seq()` 추가 — 탑뷰 XY + 그리퍼 캠 VS 기반 공구 fetch 12단계 시퀀스. 좌표는 파라미터가 아닌 runner 토픽(/vision/tool_top_pose, /vision/tool_gripper_pose)에서 실시간 수신.
- `vision_return_seq()` 추가 — staging pick(VS) + slot place(VS) 2회 VS 구조 13단계 시퀀스. 좌표는 /vision/tool_top_pose, /vision/slot_top_pose 토픽에서 실시간 수신.
- `vision_drawer_open_seq(layer)` 추가 — 손잡이 XZ Visual Servoing 포함 서랍 열기 시퀀스(11단계). VISUAL_SERVO_XZ 스텝 포함.
- `vision_drawer_close_seq(layer)` 추가 — 손잡이 XZ Visual Servoing 포함 서랍 닫기 시퀀스(11단계). VISUAL_SERVO_XZ 스텝 포함.
- `StepKind` 신규 값 5종 추가: `VISUAL_SERVO_XZ`, `MOVE_L_TOP_XY`, `VISUAL_SERVO_XY`, `MOVE_L_TOOL_XYZ`, `MOVE_L_SLOT_XY`. 기존 값 순서 불변(하위 호환).

### Changed
- `drawer_open_seq(layer)` → `drawer_open_seq(layer, tool_pose=None)` — Optional `tool_pose` 파라미터 추가. 기존 호출부 하위 호환 유지.
- `drawer_close_seq(layer)` → `drawer_close_seq(layer, tool_pose=None)` — 동일.
- `approach_tool_seq(layer)` → `approach_tool_seq(layer, tool_pose=None)` — 동일.
- `fetch_from_drawer_seq(layer)` → `fetch_from_drawer_seq(layer, tool_pose=None)` — 동일.
- `return_to_drawer_seq(layer)` → `return_to_drawer_seq(layer, tool_pose=None)` — 동일.
- `StepKind.MOVE_L_SLOT_XY` 동작 변경 — 기존: `/vision/slot_top_pose` 토픽 XY 사용. 변경: `config/toolbox.yaml` `grasp_pose_base.x/y` (×1000 mm) 사용. vision_return_seq ⑨⑫번 slot 위 이동에 사용. runner의 `_slot_xy_map`에서 tool_id별로 로드.
- `StepKind.MOVE_L_STAGING_XYZ` 동작 변경 — Z 출처: 기존 `return_z_mm` → 변경 `staging_pickup_z_mm`. vision_return_seq ⑥번 staging 파지 하강 전용. ⑩번 slot 반납은 `MOVE_L_SLOT_XYZ`(return_z_mm)와 분리.
- `vision_return_seq()` 변경 — 토픽 `/vision/tool_gripper_pose` → `/vision/return/tool_gripper_pose` (PoseStamped). rz(yaw) 수신 추가. ⑨⑩⑫ slot XY를 yaml 고정값으로 변경.
- `ToolPose` (`visual_servoing.py`) — `rz: float = 0.0` 필드 추가. 기존 키워드 인수 호출 하위 호환.
- `VISION_FETCH_SCAN_J` → `VISION_FETCH_SCAN_J_DEG` 상수명 변경 (deg 단위 명시).
- `VISION_RETURN_SCAN_J` → `VISION_RETURN_SCAN_J_DEG` 상수명 변경 (deg 단위 명시).

### Added (feat/motion-drawer-v2)
- `StepKind.WAIT_VISION_RETURN_XY` — `/vision/return/tool_gripper_pose` 캐시 초기화 후 신규 수신 대기. vision_return_seq ④번.
- `VISION_RETURN_SCAN_J_DEG` 상수 추가 — return 전용 그리퍼 캠 스캔 자세 `[-24.60, 32.49, 50.78, 22.42, 105.63, -19.92]` deg.

### Deprecated
### Removed
### Fixed
### Security
