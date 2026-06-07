# Changelog — unit_actions

Keep a Changelog 형식. 함수 시그니처 변경 시 갱신 (P-2).

## [Unreleased]

### Added
- `vision_fetch_seq(vision_x, vision_y, vision_z)` 추가 — 비전 카메라(RealSense)에서 받은 XYZ 좌표로 공구를 집어 스테이징 영역에 전달하는 11단계 시퀀스. socket_fetch_seq()와 동일 구조, 3·4·6번 스텝 좌표를 비전 값으로 대체.
- `vision_return_seq(bottom_x, bottom_y, bottom_z, slot_x, slot_y, slot_z)` 추가 — 비전 좌표 기반 공구 반납 11단계 시퀀스. bottom 좌표(스테이징)는 3·4·6번, slot 좌표(공구함)는 7·8·10번 스텝에 사용.

### Changed
- `drawer_open_seq(layer)` → `drawer_open_seq(layer, tool_pose=None)` — Optional `tool_pose` 파라미터 추가. 기존 호출부 하위 호환 유지.
- `drawer_close_seq(layer)` → `drawer_close_seq(layer, tool_pose=None)` — 동일.
- `approach_tool_seq(layer)` → `approach_tool_seq(layer, tool_pose=None)` — 동일.
- `fetch_from_drawer_seq(layer)` → `fetch_from_drawer_seq(layer, tool_pose=None)` — 동일.
- `return_to_drawer_seq(layer)` → `return_to_drawer_seq(layer, tool_pose=None)` — 동일.
