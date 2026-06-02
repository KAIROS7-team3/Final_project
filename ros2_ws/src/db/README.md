# db 패키지

`db` 패키지는 Track A/B에서 공구 상태 DB를 ROS2 서비스로 노출한다. 음성
노드, orchestrator, 수동 테스트 노드는 이 패키지의 DB Gate를 통해
`fetch`/`return` 명령 가능 여부를 확인해야 한다.

이 패키지는 로봇을 직접 움직이지 않는다. 공구 상태 조회, 명령 가능 여부 확인,
상태 갱신, FOD timeout 감시만 담당한다.

## 필요한 라이브러리

ROS2/ament 의존성:

- ROS2 Humble
- `ament_python`
- `rclpy`
- `interfaces`
- `std_msgs`

Python 의존성:

- Python 3.10
- `sqlite3` - Python 표준 라이브러리
- `pytest` - 테스트 실행용

런타임 데이터:

- SQLite DB 파일
- 기본값은 실행 위치 기준 `robot_arm.db`
- 현장/로컬 테스트에서는 보통
  `~/Final_Project/robot_arm.db`를 지정한다.

## 빌드 방법

`db` 패키지는 `interfaces`의 service 타입을 사용하므로 `interfaces`와 함께
빌드해야 한다.

```bash
cd ~/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select interfaces db
source install/setup.bash
```

## 테스트 방법

```bash
cd ~/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select db --event-handlers console_direct+
colcon test-result --verbose
```

정상 결과:

```text
10 passed
```

## DB 서비스 실행

```bash
cd ~/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run db db_service_node --ros-args \
  -p db_path:=$HOME/Final_Project/robot_arm.db \
  -p operator_id:=operator_01
```

제공 서비스:

- `/db/CheckToolFeasibility`
- `/db/UpdateToolStatus`

## 서비스 수동 테스트

공구를 가져올 수 있는지 확인:

```bash
ros2 service call /db/CheckToolFeasibility interfaces/srv/CheckToolFeasibility \
  "{intent: fetch, tool_id: spanner_16mm}"
```

motion 완료를 가정하고 공구 상태를 `out`으로 갱신:

```bash
ros2 service call /db/UpdateToolStatus interfaces/srv/UpdateToolStatus \
  "{tool_id: spanner_16mm, new_status: out, event_type: fetch, track: A, notes: manual test}"
```

DB Gate 규칙상 이미 `out` 상태인 공구를 다시 `fetch`하면 거부되어야 한다.

## FOD Monitor 실행

`fod_monitor_node`는 `out` 또는 `staged` 상태가 임계 시간을 넘었는지
주기적으로 확인한다. 시간이 초과되면 `missing` 또는 `fod_alert` 상태 전이를
DB에 기록하고 `/plc/system_state`에 `error`를 발행해 PLC 경고 표시를 요청한다.

```bash
ros2 run db fod_monitor_node --ros-args \
  -p db_path:=$HOME/Final_Project/robot_arm.db \
  -p checkout_timeout_minutes:=10.0 \
  -p missing_to_alert_seconds:=30.0 \
  -p poll_interval_seconds:=5.0
```

현장 테스트 중 빠르게 확인하려면 timeout 값을 짧게 낮춰 실행한다.

## 음성 명령에서 DB 상태 갱신까지 수동 확인

`intent_status_simulator_node`는 하드웨어를 움직이지 않고
`/voice/intent`를 DB 상태 변경까지 연결해 보는 테스트용 노드다. 운영 제어
경로가 아니며 bring-up 확인 용도로만 사용한다. DB Gate 통과 후에는
`/plc/system_state`에 `moving`을 보내고, DB update 성공 후 `idle`로 돌린다.
거부나 실패가 발생하면 `error`를 보낸다.

`voice` 패키지의 `rule_intent_node`까지 함께 켜면 PLC 상태는 다음 순서로 이어진다.

```text
/voice/raw_text 수신 -> inferring
DB Gate 통과 및 simulated DB update 시작 -> moving
DB update 성공 -> idle
DB Gate 거부/서비스 실패/update 실패 -> error
```

Terminal 1 - DB 서비스 실행:

```bash
ros2 run db db_service_node --ros-args \
  -p db_path:=$HOME/Final_Project/robot_arm.db
```

Terminal 2 - 시뮬레이터 실행:

```bash
ros2 run db intent_status_simulator_node --ros-args \
  -p track:=A
```

Terminal 3 - PLC 상태 확인:

```bash
ros2 topic echo /plc/system_state
```

Terminal 4 - intent 메시지를 직접 발행:

```bash
ros2 topic pub --once /voice/intent interfaces/msg/Intent \
  "{intent_type: fetch, tool_id: spanner_16mm, confidence: 0.9, raw_utterance: '스패너 가져와'}"
```

시뮬레이터도 `/db/CheckToolFeasibility`를 다시 호출한다. 따라서 DB Gate를
우회하는 경로로 사용하면 안 된다.

VOICE까지 포함해서 확인하려면 Terminal 4 대신 `rule_intent_node`를 실행하고
Terminal 5에서 `/voice/raw_text`를 보낸다.

```bash
ros2 run voice rule_intent_node --ros-args \
  -p require_wake_word:=true \
  -p wake_words:="[코봇]"

ros2 topic pub --once /voice/raw_text std_msgs/msg/String \
  "{data: '코봇 스패너 가져와'}"
```
