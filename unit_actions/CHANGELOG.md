# Changelog

모든 주요 변경사항은 이 파일에 기록한다 ([Keep a Changelog](https://keepachangelog.com/) 기반).

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

### Changed
### Deprecated
### Removed
### Fixed
### Security
