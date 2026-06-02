# interfaces CHANGELOG

interfaces/ 패키지의 msg/srv/action 변경 이력.
형식: Keep a Changelog (process.md P-2)

---

## [Unreleased]

### Added
- `msg/MarkerMap.msg` — 탑뷰 ArUco 다중 마커 스캔 결과 메시지
  - `marker_ids[]`, `poses_robot[]` (geometry_msgs/Pose, m + quaternion), `place_zone_radius` (m), `calibrated`
  - MarkerScanNode → orchestrator BT ScanMarkers 연동용 (PR #22)
