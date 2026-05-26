---
name: ros1
description: ROS1 (Noetic 등) 레거시 시스템 참조 및 ROS1→ROS2 마이그레이션 매핑. 본 프로젝트는 ROS2 Humble 전용이므로 외부 ROS1 코드와의 호환·이식 작업 시에만 활성화. ROS1 catkin/rospy/roscpp/roslaunch/tf/actionlib, ROS1-ROS2 bridge, rospy → rclpy 포팅 시 트리거.
---

# ROS1 참조 (마이그레이션 전용)

> **본 프로젝트는 ROS2 Humble만 사용한다.** 이 스킬은 외부 ROS1 코드와의 호환 또는 ROS1 자료를 ROS2로 포팅할 때만 참조한다.
> 전체 ROS1 best practice는 [`REFERENCE.md`](REFERENCE.md)에 보존되어 있다.

---

## 본 프로젝트에서의 사용 사례 (제한적)

| 상황 | 행동 |
|------|------|
| 외부 ROS1 패키지를 검토 | 이 스킬로 패턴 이해 후 ROS2 대응물 결정 |
| ROS1 튜토리얼/논문 코드 이식 | 아래 매핑 표 참조 |
| `realsense-ros` 등 ROS1 시절 자료 검색 | API 명칭 변경 확인 |

**ROS1 코드를 본 저장소에 추가하지 않는다.** `interfaces/`, `voice/`, `vision/`, `orchestrator/`, `motion/`, `db/`, `plc/`는 모두 ROS2 Humble.

---

## ROS1 ↔ ROS2 핵심 매핑

| 영역 | ROS1 | ROS2 |
|------|------|------|
| 빌드 도구 | `catkin_make`, `catkin build` | `colcon build` |
| 워크스페이스 | `catkin_ws/src` | `ros2_ws/src` |
| Python 클라이언트 | `rospy` | `rclpy` |
| C++ 클라이언트 | `roscpp` | `rclcpp` |
| 노드 생성 | `rospy.init_node('foo')` | `rclpy.init(); node = Node('foo')` |
| 구독 | `rospy.Subscriber(...)` | `node.create_subscription(...)` |
| 퍼블리시 | `rospy.Publisher(...)` | `node.create_publisher(...)` |
| 서비스 | `rospy.Service(...)` | `node.create_service(...)` |
| 액션 | `actionlib` | `rclpy.action` (내장) |
| Launch | XML `.launch` | Python `launch_*.py` |
| 파라미터 | `rospy.get_param('/ns/p')` | `node.declare_parameter('p').value` |
| 시간 | `rospy.Time.now()` | `node.get_clock().now()` |
| 미들웨어 | ROS1 자체 TCPROS | DDS (FastDDS/CycloneDDS) |
| Master | `roscore` 필수 | 분산 — Master 없음 |
| TF | `tf` (legacy), `tf2` | `tf2_ros` |
| Discovery | ROS_MASTER_URI | ROS_DOMAIN_ID |

---

## 흔한 포팅 함정

### Master vs DDS
ROS1은 중앙 `roscore` 의존. ROS2는 분산 discovery — 같은 `ROS_DOMAIN_ID`의 모든 노드가 서로 발견.
다중 머신 운영 시 ROS1은 `ROS_MASTER_URI`만 설정하면 됐지만, ROS2는 DDS 멀티캐스트 또는 Discovery Server 구성 필요.

### Callback 모델
- ROS1: 콜백은 별도 스레드. `rospy.spin()`은 메인 스레드 idle.
- ROS2: 단일/다중 Executor. 기본은 `rclpy.spin(node)`가 single-threaded — 콜백 직렬화.
- 동시 콜백이 필요하면 `MultiThreadedExecutor` 명시.

### 메시지 정의
- ROS1: `package_name/msg/Foo.msg` + `manifest.xml`
- ROS2: 동일 형식이나 `package.xml` (v3) + `CMakeLists.txt`에 `rosidl_generate_interfaces` 필요

### Launch
ROS1의 XML launch는 그대로 동작 안 함. ROS2는 Python launch — 조건부 로직 강력하지만 syntax 다름.

---

## ROS1-ROS2 Bridge (필요 시만)

레거시 ROS1 노드를 ROS2 스택에 임시 통합:
```bash
sudo apt install ros-humble-ros1-bridge
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```
**본 프로젝트는 ROS1 노드를 도입하지 않는다.** Bridge 사용 가능성은 v2.0+ 검토 사항 (현재 미정).

---

## 더 깊은 ROS1 자료가 필요하면

- 전체 ROS1 best practice → [`REFERENCE.md`](REFERENCE.md) (catkin, nodelets, dynamic reconfigure, pluginlib 등)
- 공식 ROS1 wiki: <https://wiki.ros.org/>
- 본 프로젝트의 ROS2 패턴 → [`../ros2/SKILL.md`](../ros2/SKILL.md)
- 원본 출처: <https://github.com/arpitg1304/robotics-agent-skills>
