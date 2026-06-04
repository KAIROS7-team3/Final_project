# interfaces CHANGELOG

interfaces/ 패키지의 msg/srv/action 변경 이력.
형식: Keep a Changelog (process.md P-2)

---

## [Unreleased]

### Added
- `srv/LogEvent.srv` — 상태 무변경 감사 이벤트 기록 서비스 (B1-1)
  - request: `tool_id`, `event_type`, `track`, `notes` / response: `success`, `message`
  - orchestrator S-7(is_moving) 가드가 드롭한 intent를 `tool_events('rejected')`로 남기기 위한 경로
  - `event_type`은 db_service_node에서 `'rejected'`로 제한 (임의 이벤트 위조 방지, B2-1 표면 최소화)
- `msg/MarkerMap.msg` — 탑뷰 ArUco 다중 마커 스캔 결과 메시지
  - `marker_ids[]`, `poses_robot[]` (geometry_msgs/Pose, m + quaternion), `place_zone_radius` (m), `calibrated`
  - MarkerScanNode → orchestrator BT ScanMarkers 연동용 (PR #22)
