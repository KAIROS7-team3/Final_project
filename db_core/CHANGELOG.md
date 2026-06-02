# Changelog — db_core

Keep a Changelog 형식. `ToolRepository`/`DBClient` API 변경, 스키마 마이그레이션 시 갱신 (P-2).

## [Unreleased]

### Added
- `ToolRepository.__init__`에 `busy_timeout_ms` 파라미터 추가 (기본 5000) — 다중 프로세스(`db_service_node`+`fod_monitor_node`)의 WAL 락 경합 시 `SQLITE_BUSY` 즉시 실패 방지. 운영 값은 `config/runtime.yaml` (`db.busy_timeout_ms`).

### Changed
- `ToolRepository._connect()`가 연결 시 `SCHEMA_SQL`로 스키마를 부트스트랩한다. 신규/비어 있는 DB 파일에 대해 Track A/B 노드 단독 기동 시 `SchemaValidationError`로 DB Gate가 영구 차단되던 문제 해결. `CREATE TABLE IF NOT EXISTS`라 기존 DB에 멱등이며, 컬럼·FK 누락 검증(`_validate_schema`)은 그대로 유지. `DBClient.connect()`와 동일한 `SCHEMA_SQL`을 공유해 스키마 drift 방지.

### Notes
- 미해결(팀 합의 대기): DB Gate 이중 구현(`DBClient` vs `ToolRepository`) 단일화, `update_tool_status` 상태 전이 화이트리스트(S-8/S-9). interface-guardian·safety-reviewer 검토 결과 별도 PR로 분리.
