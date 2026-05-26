---
name: doosan-e0509
description: >
  Doosan Robotics e0509 협동 로봇팔 사양·관절 한계·작업공간·Safety Controller 연동·
  ROS2 드라이버(doosan-robot2)와 Python SDK 사용 패턴.
  로봇팔 제어 코드 작성, joint 명령 디버깅, workspace 설계, Safety 연동 시 활성화.
when_to_use: >
  Doosan e0509 관련 코드 작성, DSR controller 사용, Track C에서 Doosan Python SDK 직접 호출,
  joint limit / workspace 검증 시.
---

# Doosan e0509 협동 로봇팔 가이드

## 1. 하드웨어 사양

| 항목 | 값 |
|------|-----|
| 자유도 | 6 DOF (J1~J6) |
| 가반하중 | 5 kg |
| 도달거리 | 900 mm |
| 반복정밀도 | ±0.05 mm |
| 정격 속도 | TCP 1 m/s |
| 통신 | TCP/IP (제어), Modbus/EtherNet IP (보조) |
| 안전 등급 | ISO 10218-1 / ISO/TS 15066 (협동 로봇) |

## 2. 관절 한계 (소프트 리밋 권장값)

> 하드웨어 한계보다 좁게 설정. `config/robot_poses.yaml`에 정의.

| 관절 | 범위 (rad) | 비고 |
|------|-----------|------|
| J1 (base rotation) | ±π × 0.95 | 케이블 꼬임 방지 |
| J2 (shoulder) | -π × 0.5 ~ +π × 0.5 | 자세 제한 |
| J3 (elbow) | -π × 0.85 ~ +π × 0.85 | |
| J4 (wrist roll) | ±π × 0.95 | |
| J5 (wrist pitch) | ±π × 0.85 | 그리퍼 충돌 방지 |
| J6 (tool rotation) | ±π × 0.95 | |

```python
# rad 단위 — degree 사용 금지 (.claude/rules/engineering.md E-1)
JOINT_LIMITS_RAD = {
    1: (-2.985, 2.985),   # J1
    2: (-1.571, 1.571),   # J2
    # ...
}
```

## 3. Workspace 경계 (Cartesian)

```yaml
# config/workspace.yaml (예시)
robot_base_link:
  x: [-0.9, 0.9]       # m
  y: [-0.9, 0.9]
  z: [0.0, 1.3]
  # 테이블 충돌 방지: z 하한 = 테이블 표면 + 안전 마진
```

- Self-collision: doosan-robot2 URDF에 정의된 self_collision pairs 사용
- 환경 collision: v1.0에서는 작업공간 박스만, v2.0+에서 OctoMap 도입 검토

## 4. ROS2 드라이버 (Track A/B): `doosan-robot2`

### 패키지 설치
```bash
sudo apt install ros-humble-doosan-robot2
# 또는 source 빌드
cd ros2_ws/src && git clone https://github.com/doosan-robotics/doosan-robot2
```

### 노드 실행
```bash
ros2 launch dsr_bringup2 dsr_bringup2.launch.py \
    name:=e0509 \
    host:=192.168.137.100 \
    port:=12345 \
    model:=e0509 \
    mode:=real        # real / virtual
```

### 주요 토픽 / 서비스
| 인터페이스 | 타입 | 용도 |
|-----------|------|------|
| `/dsr/joint_states` | sensor_msgs/JointState | 현재 joint 상태 |
| `/dsr/state` | dsr_msgs2/RobotState | 로봇 전체 상태 |
| `/dsr/system/set_robot_mode` | dsr_msgs2/SetRobotMode | 모드 변경 (manual/auto) |
| `/dsr/motion/move_joint` | dsr_msgs2/MoveJoint | joint 공간 이동 |
| `/dsr/motion/move_line` | dsr_msgs2/MoveLine | Cartesian 직선 이동 |
| `/dsr/motion/stop` | dsr_msgs2/Stop | 비상 정지 |

### Action 패턴 (long-running motion)
```python
from rclpy.action import ActionClient
from dsr_msgs2.action import MoveJoint

client = ActionClient(node, MoveJoint, '/dsr/motion/move_joint_action')
goal = MoveJoint.Goal()
goal.pos = [0.0, 0.5, -1.2, 0.0, 0.7, 0.0]  # rad
goal.vel = 0.5  # 0.0~1.0 정규화
goal.acc = 0.5
goal.time = 0.0  # 0이면 vel/acc 사용
future = client.send_goal_async(goal)
```

## 5. Python SDK (Track C 전용): 직접 TCP/IP 제어

> ROS2 없이 Track C에서 직접 호출. 패키지명은 `dsr_robot2_python` 또는 동급 — 실제 설치 패키지 확인 필요.

```python
from doosan_sdk import DooSanArm  # 실제 import 경로는 패키지 docs 확인

arm = DooSanArm(host="192.168.137.100", port=12345, model="e0509")
arm.connect()
arm.set_robot_mode("automatic")

# Joint 공간 이동
arm.movej(pos=[0.0, 0.5, -1.2, 0.0, 0.7, 0.0], vel=0.5, acc=0.5)

# Cartesian 직선 이동 (tool frame 기준)
arm.movel(pos=[0.5, 0.0, 0.3, 0, np.pi, 0], vel=0.3, acc=0.3)

# Joint 상태 읽기
joints = arm.get_current_posj()  # [j1, ..., j6] rad

# 비상 정지
arm.stop()

arm.disconnect()
```

### Safety Controller 연동 (필수)
- Doosan 컨트롤러는 자체 충돌 감지 + 토크 모니터링 보유
- Python SDK 사용 시에도 이 기능은 활성 유지됨
- `set_robot_mode("manual")` 중에는 외부 명령 거부
- **티치 펜던트 E-Stop은 Python SDK 모드와 무관하게 항상 동작**

## 6. 좌표계 (Frame) 관리

```
world (가상, 고정)
  └── robot_base_link  ← config/*.yaml의 모든 좌표 기준
        └── link_1 ~ link_6 (joint chain)
              └── tool_link (TCP 위치, 그리퍼 장착 지점)
                    └── gripper_tip (RH-P12-RN 그리퍼 끝점)
                          └── camera_link (D455f, hand-eye 캘리브로 결정)
```

- 모든 `config/staging_area.yaml`, `config/toolbox.yaml` 좌표는 `robot_base_link` 기준
- 카메라 → 로봇 변환은 `config/hand_eye.yaml`에 명시

## 7. 흔한 함정

### ❌ rad/degree 혼용
```python
# 위험 — degree 인지 rad 인지 불명확
arm.movej(pos=[0, 30, -60, 0, 90, 0])
```
✅ 항상 rad 사용 + 변환은 명시적
```python
import numpy as np
JOINT_HOME_DEG = [0, 30, -60, 0, 90, 0]
JOINT_HOME_RAD = [np.deg2rad(d) for d in JOINT_HOME_DEG]
arm.movej(pos=JOINT_HOME_RAD, vel=0.3, acc=0.3)
```

### ❌ 속도 정규화 무시
- `vel=1.0`은 정격 속도 100%. 협동 모드에서는 안전 한계 초과 가능
- 권장: 일반 동작 `vel=0.3~0.5`, 정밀 동작 `vel=0.1~0.2`

### ❌ 비동기 이동 중 새 명령
- `movej`는 블로킹이지만 `movej_async`는 비블로킹
- 비동기 중 새 명령 → 미정의 동작. 반드시 `wait_motion_done()` 호출

### ❌ Reach 초과 시 silent fail
- Doosan SDK는 reach 초과 시 예외 발생. try/except 필수 (.claude/rules/engineering.md E-5)

## 8. 디버깅 도구

```bash
# 실제 로봇 없이 시뮬레이션
ros2 launch dsr_bringup2 dsr_bringup2.launch.py mode:=virtual

# Joint 상태 모니터링
ros2 topic echo /dsr/joint_states

# RViz로 URDF 시각화
ros2 launch dsr_description2 dsr_description.launch.py model:=e0509
```

## 9. 참고

- Doosan 공식: <https://www.doosanrobotics.com/>
- doosan-robot2 GitHub: <https://github.com/doosan-robotics/doosan-robot2>
- e0509 데이터시트: 공식 사이트 → Products → e-Series
- 프로젝트 룰: [`.claude/rules/safety.md`](../rules/safety.md), [`.claude/rules/engineering.md`](../rules/engineering.md)
