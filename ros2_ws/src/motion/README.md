# motion 패키지

Doosan e0509 + RH-P12-RN 그리퍼 통합 제어 패키지 (Track A/B).

## 구성

| 모듈 | 설명 |
|------|------|
| `motion/gripper_node.py` | RH-P12-RN 그리퍼 ROS2 노드. DRL/TCP 두 가지 transport 지원 |
| `motion/rviz_joint_state_merger_node.py` | 팔 joint_states + 그리퍼 pulse를 합쳐 RViz에 전달 |
| `motion/gripper_conversion.py` | 그리퍼 pulse ↔ rad 변환 유틸리티 |
| `config/gripper_node.yaml` | 그리퍼 노드 파라미터 |
| `launch/bringup_e0509_with_gripper.launch.py` | e0509 + 그리퍼 통합 bringup |
| `urdf/e0509_with_gripper.urdf` | 그리퍼 링크 포함 URDF |

## 실행

```bash
# 빌드
cd ~/Final_project/ros2_ws
colcon build --packages-select motion --symlink-install
source install/setup.bash

# 실물 로봇 bringup (IP 명시 필수)
ros2 launch motion bringup_e0509_with_gripper.launch.py \
  mode:=real model:=e0509 host:=<ROBOT_IP> robot_ip:=<ROBOT_IP>
```

## 그리퍼 명령

launch 실행 후 `[gripper] DRL 초기화 완료` 로그 확인 후 사용.

```bash
# 열기
ros2 topic pub --once /gripper/cmd_direct std_msgs/msg/String "{data: 'open'}"

# 닫기
ros2 topic pub --once /gripper/cmd_direct std_msgs/msg/String "{data: 'close'}"

# 커스텀 (pulse 0~700, current mA)
ros2 topic pub --once /gripper/cmd_direct std_msgs/msg/String "{data: 'custom 420 300'}"

# action (피드백 포함)
ros2 action send_goal /gripper/grasp interfaces/action/Grasp \
  "{tool_id: 'test', grasp_force: 300.0}"
```

## 상태 확인

```bash
# 그리퍼 pulse 상태 (close=700, open=0)
ros2 topic echo /gripper/state

# RViz 입력 (팔 + 그리퍼 합산)
ros2 topic echo /dsr01/joint_states_rviz
```

## 주요 파라미터 (gripper_node.yaml)

| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `command_transport` | `drl` | 명령 전송 방식: `drl` \| `tcp` |
| `robot_ns` | `dsr01` | 로봇 네임스페이스 |
| `pulse_open` | `0` | 열림 pulse |
| `pulse_closed_preset` | `700` | 닫힘 pulse |
| `init_current` | `400` | 초기화 전류 (mA) |
| `grip_current` | `300` | 파지 전류 (mA) |
| `gripper_command_action_enabled` | `true` | `/gripper/grasp` action 서버 활성 |
| `direct_cmd_topic_enabled` | `true` | `/gripper/cmd_direct` 토픽 활성 |

`robot_ip`는 launch 인자로만 전달 (`host:=<IP> robot_ip:=<IP>`). yaml에 하드코딩 금지.

## 아키텍처

```
/gripper/cmd_direct (String)
        │
        ▼
  gripper_node
        │  DRL transport
        ▼
  /dsr01/drl/drl_start (service)
        │
        ▼
  로봇 DRL → flange serial → Modbus RTU → RH-P12-RN
        │
        │  state (optimistic, DRL 모드에서는 명령 후 업데이트)
        ▼
  /gripper/state (JointState, name=gripper_joint)
        │
        ▼
  rviz_joint_state_merger
        │  pulse → rad 변환
        ▼
  /dsr01/joint_states_rviz → RViz
```

## 현재 상태 및 제한사항

- **DRL 모드**: 명령은 정상 동작. 상태는 실제 피드백 없이 명령값으로 업데이트 (optimistic).
- **TCP 모드**: DRL 서버 코드를 로봇에 배포해 TCP 소켓으로 양방향 통신. 실제 state feedback 가능하나 로봇 환경에 따라 소켓 서버 실행 여부 확인 필요.
- **초기 상태**: 노드 시작 시 실제 그리퍼 위치를 읽지 않아 pulse=0으로 초기화됨.

## TODO

- [x] **virtual 모드 대응**: `mode` 파라미터로 virtual 감지 → DRL/flange 초기화 생략 (에뮬레이터 블로킹 버그 수정)
  - launch 파일이 `mode:=virtual` 시 gripper_node에 `mode` 파라미터 전달
  - `_init_drl_server()` 진입 시 virtual이면 즉시 반환
- [ ] **초기 상태 읽기**: 노드 시작 시 DRL로 현재 그리퍼 위치(present_position)를 읽어 `_current_hz_pos` 초기화
  - `_init_drl_server` 완료 후 `_fc03(slave_id, REG_PRESENT_POSITION, 2)` 호출해 실제 pulse 값 반영
  - RViz에서 시작부터 실제 그리퍼 상태 표시 가능
