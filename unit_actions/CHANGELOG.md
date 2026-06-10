# Changelog — unit_actions

Keep a Changelog 형식. 함수 시그니처 변경 시 갱신 (P-2).

## [Unreleased]

### Added
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
