# CHANGELOG

> 루트 CHANGELOG. 룰 변경(`.claude/rules/*.md`)과 주요 아키텍처 결정만 기록한다.
> 패키지별 변경은 해당 패키지의 `CHANGELOG.md`에 기록한다.
> 형식: [Keep a Changelog](https://keepachangelog.com/ko/1.1.0/) 기반.

## [Unreleased]

### Added
- (예정)

### Known Issues / Technical Debt

- **[gripper] GripperCommand action 미사용 — service 경로 단독 운용 (추후 교체 검토)**
  - 현재 `sequence_engine._grip()`은 `GripperSetPosition` **service**를 호출한다.
  - `gripper_node`에 `GripperCommand` **action**이 존재하며 20초 대기 루프 + 파지 감지 피드백을 제공하지만, 시퀀스 실행 경로에서 사용되지 않는다.
  - service 경로는 settle 타임아웃(현재 2.5s) 이후 위치 미확인 상태로 success=True를 반환할 수 있어 파지 실패가 무음 전파될 위험이 있다.
  - **교체 범위**: `sequence_engine._grip()` → ActionClient 전환, `setup()` 서비스 대기 → action server 대기, 기존 service 경로 호출자(대시보드 standalone) 영향 확인 필요.
  - **관련 파일**: `ros2_ws/src/motion/motion/sequence_engine.py` `_grip()` / `ros2_ws/src/motion/motion/gripper_node.py` `_execute_callback()`

---

## [Phase 0 — 2차 정리] — 2026-05-27

### Changed
- **CycloneDDS 단일 머신 설정 → `ROS_DOMAIN_ID` 기반 격리로 대체**
  - rationale: Doosan 컨트롤러는 TCP(LAN) 별도 통신이라 DDS 격리와 무관. 루프백 전용 설정은 HP ProBook의 rqt 모니터링을 차단함
  - `.env.example`: `ROS_DOMAIN_ID=42` + `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` + `ROS_LOCALHOST_ONLY=0` 추가
  - `run.sh`: Track A/B 시작 시 ROS_DOMAIN_ID 누락 검증 + 환경변수 export
- **통합 빌드 주기: 주 1회 확정** (2026-05-27 결정)
- **Docker v1.0 계획에서 제외** — 네이티브 Ubuntu 22.04 + ROS2 Humble로 통합 운영
  - 영향: `robot-arm-project.md` 팀 구성·Phase 0·Phase 9·G0→1 게이트에서 Docker 항목 제거
  - 영향: `docs/hardware.md` 컨테이너 행을 "미사용" 으로 변경
  - 영향: `README.md` Phase 9 라인에서 "Docker Compose" → "의존성 lockfile"
  - 추후 배포 단순화 필요 시 재검토 가능
- **G0→1 게이트 조건 변경**: "Docker 빌드 통과" → "CI 단위 테스트 통과"

### Remaining (담당자에게 위임)
- 공구 이미지 데이터셋 수집 — **담당: C**
- 샘플 음성 파일 9종 — **담당: A**
- DB seed 데이터 — **담당: D**

---

## [Phase 0 — 1차 완료] — 2026-05-27

### Added
- `interfaces/` 패키지 v0.1.0 동결: msg 4종(ToolStatus, PLCStatus, RobotStatus, Intent), srv 2종(CheckToolFeasibility, UpdateToolStatus), action 6종(MoveToPose, Grasp, Release, PlaceAtStaging, PickFromStaging, ReturnToSlot)
- `hal/` 계층: `ArmInterface`, `GripperInterface`, `CameraInterface` 추상 클래스 + `SimulatedArm/Gripper/Camera` 스텁 + Doosan/RealSense 드라이버 골격
- `unit_actions/` 7개 모듈 스켈레톤 (grasp, move_to_pose, place_at_staging, pick_from_staging, release, return_to_slot, scan_workspace) + 단위 테스트
- `db_core/schema.py`: SQLite WAL 스키마 (tools, tool_events, operators, system_events)
- `track_c_vla.py` 골격 + `run.sh --track [A|B|C]` Track Selector
- `.github/workflows/ci.yml`: lint·rclpy-import-boundary·unit-test·safety-regression·config-yaml·secret-scan 6단계 파이프라인
- `mocks/SPEC.md`: SimulatedArm/Gripper/Camera 결정론적 동작 명세
- `config/` 6개 파일: staging_area.yaml, toolbox.yaml, hand_eye.yaml, robot_poses.yaml, fod.yaml, runtime.yaml
- `ros2_ws/src/` 7패키지 디렉토리 구조 + interfaces 패키지 빌드 가능 상태
- Doosan e0509: doosan-robot2 서브모듈 (URDF/XACRO, dsr_moveit_config_e0509, Gazebo/MuJoCo 씬)
