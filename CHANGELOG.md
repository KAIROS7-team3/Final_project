# CHANGELOG

> 루트 CHANGELOG. 룰 변경(`.claude/rules/*.md`)과 주요 아키텍처 결정만 기록한다.
> 패키지별 변경은 해당 패키지의 `CHANGELOG.md`에 기록한다.
> 형식: [Keep a Changelog](https://keepachangelog.com/ko/1.1.0/) 기반.

## [Unreleased]

### Added
- (예정)

---

## [Phase 0] — 2026-05-27

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

### Remaining (Phase 0 미완료)
- Docker 개발 컨테이너 (GPU 패스스루) — Dockerfile/docker-compose 미작성
- CycloneDDS 단일 머신 설정 파일 미작성
- 샘플 음성 파일 9종 + DB seed 데이터 미작성
- 통합 빌드 주기 미결정
- 공구 이미지 데이터셋 수집 진행 상황 미확인 (물리 작업)
