# db package

Track A/B에서 공구 상태 DB를 ROS2 서비스로 노출하는 패키지다. `voice`,
`orchestrator`, 수동 테스트 노드는 이 패키지의 DB Gate를 통해서만
`fetch`/`return` 가능 여부를 확인해야 한다.

## Dependencies

ROS2/ament:

- ROS2 Humble
- `ament_python`
- `rclpy`
- `interfaces`

Python:

- Python 3.10
- `sqlite3` (Python standard library)
- `pytest` (test only)

Local runtime data:

- SQLite DB file, 기본값은 실행 위치 기준 `robot_arm.db`
- 실제 로봇 프로젝트에서는 보통
  `/home/thomas/Final_Project/robot_arm.db`를 지정해서 실행한다.

## Build

`interfaces` 서비스 타입이 먼저 필요하므로 `db`만 단독으로 빌드하지 말고
같이 빌드한다.

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select interfaces db
source install/setup.bash
```

## Test

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select db --event-handlers console_direct+
colcon test-result --verbose
```

Expected result:

```text
10 passed
```

## Run DB Service

```bash
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run db db_service_node --ros-args \
  -p db_path:=/home/thomas/Final_Project/robot_arm.db \
  -p operator_id:=operator_01
```

Provided services:

- `check_tool_feasibility`
- `update_tool_status`

## Manual Service Checks

Fetch 가능 여부 확인:

```bash
ros2 service call /check_tool_feasibility interfaces/srv/CheckToolFeasibility \
  "{intent: fetch, tool_id: spanner_16mm}"
```

Motion 완료를 가정한 상태 갱신:

```bash
ros2 service call /update_tool_status interfaces/srv/UpdateToolStatus \
  "{tool_id: spanner_16mm, new_status: out, event_type: fetch, track: A, notes: manual test}"
```

DB Gate 규칙상 이미 `out`인 공구를 다시 `fetch`하면 거부되어야 한다.

## Run FOD Monitor

`out`/`staged` 상태가 임계 시간을 넘으면 `missing`/`fod_alert`로 전이한다.
현장 테스트 중에는 시간을 짧게 낮춰 확인할 수 있다.

```bash
ros2 run db fod_monitor_node --ros-args \
  -p db_path:=/home/thomas/Final_Project/robot_arm.db \
  -p checkout_timeout_minutes:=10.0 \
  -p missing_to_alert_seconds:=30.0 \
  -p poll_interval_seconds:=5.0
```

## Voice-to-DB Simulation

하드웨어를 움직이지 않고 `/voice/intent`를 DB 상태 변경까지 연결해 보는
수동 테스트 노드다. 운영 제어 경로가 아니며, bring-up 확인 용도로만 쓴다.

Terminal 1:

```bash
ros2 run db db_service_node --ros-args \
  -p db_path:=/home/thomas/Final_Project/robot_arm.db
```

Terminal 2:

```bash
ros2 run db intent_status_simulator_node --ros-args \
  -p track:=A
```

Terminal 3:

```bash
ros2 topic pub --once /voice/intent interfaces/msg/Intent \
  "{intent_type: fetch, tool_id: spanner_16mm, confidence: 0.9, raw_utterance: '스패너 가져와'}"
```

Simulator도 `check_tool_feasibility`를 다시 호출하므로 DB Gate 우회 경로로
사용하면 안 된다.
