# ADR — 인터페이스

> 참조: [인덱스](index.md)
> 인터페이스 전체 스키마 → [`docs/interfaces.md`](../interfaces.md)
> 변경 이력 → [`interfaces/CHANGELOG.md`](../../interfaces/CHANGELOG.md)

---

## ADR-012: 인터페이스 명명/구조 통일 (코드 구현 전 정리)

- **결정**: 코드 구현(Phase 0 ①) 시작 전 `interfaces/` 정의를 정리. 주요 변경:
  - 상태 스냅샷 메시지는 `Status` suffix로 통일 → `PLCState.msg` → `PLCStatus.msg`
  - `Intent.msg` 신설 → `/voice/intent`을 `std_msgs/String`(JSON) → `interfaces/Intent`로 교체
  - 단일 `UnitAction.action` 폐기 → 동작별 6개 액션으로 분리 (`MoveToPose`, `Grasp`, `Release`, `PlaceAtStaging`, `PickFromStaging`, `ReturnToSlot`)
  - `tool_id` 정규식 명시: `^[a-z][a-z0-9]*(_[a-z0-9]+)+$`
  - `event_type` enum 값 인터페이스 문서에 명시
  - `HandoverEvent.msg`를 v2.0+ 섹션으로 분리 (v1.0 API 표면 제외)
  - `system_state` enum의 `estop` → `e_stop` (snake_case 통일)
- **이유**:
  - **타입 안전성**: 단일 `UnitAction`은 string 디스크리미네이터 + tool_id만 전달, 액션별 파라미터 전달 경로 없어 런타임 검증 부담
  - **관측 가능성**: `std_msgs/String`(JSON) 의도 메시지는 rosbag 분석·시각화·필터링 불가
  - **일관성**: State/Status 혼용, 2-part/3-part tool_id 혼재가 신규 팀원 진입 장벽
- **트레이드오프**: 액션 6개 분리로 `.action` 파일 수 증가. 다만 `orchestrator/unit_action_server.py` 단일 노드가 다중 액션 서버를 호스팅하므로 런타임 구조는 동일.
- **타이밍**: 코드 구현 전이므로 마이그레이션 비용 없음.
