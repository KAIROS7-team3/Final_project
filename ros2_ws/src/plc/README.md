# plc

Track A/B용 ROS2 PLC 브리지 패키지.

이 패키지는 LS Electric XBC-DR14E PLC와 Modbus RTU로 통신하고, 프로젝트 표준
`/plc/status`와 현장 bring-up용 PLC 테스트 토픽을 제공한다. 상위 로봇/음성/DB
패키지는 가능한 한 PLC 메모리 주소를 직접 다루지 않고, 이후 semantic 상태 API를
통해 `idle`, `moving`, `error`, `e_stop` 같은 의미 단위로 PLC를 제어한다.

## 현재 범위

- Modbus RTU serial 연결
- P word register 주기 읽기
- M coil push-button pulse 제어
- M0100 reset pulse 제어
- PLC 상태 스냅샷 `/plc/status` 발행
- 선택형 watchdog heartbeat coil 감시
- 선택형 PLC E-stop input polling 및 latch

현재 raw M/P 토픽은 bring-up과 래더 검증용이다. 운영 로직에서 직접 의존하는
API로 확정하기 전에는 `interfaces` 변경 없이 이 surface를 유지한다.

Modbus 송수신 구현은 프로젝트 루트의 `plc_core`에 있으며, 이 ROS2 패키지는
parameter 로드, topic 변환, `/plc/status` publish를 담당하는 wrapper다.

## 연결 설정

기본 설정 파일: `config/xgb_plc.yaml`

| 항목 | 기본값 | 설명 |
|------|--------|------|
| `port` | `/dev/ttyUSB0` | USB-RS485 serial 장치 |
| `baudrate` | `115200` | 현장 검증 baudrate |
| `parity` | `"N"` | ROS2 parameter 타입 오인식을 막기 위해 문자열로 유지 |
| `stopbits` | `1` | Modbus RTU serial 설정 |
| `bytesize` | `8` | Modbus RTU serial 설정 |
| `device_id` | `1` | PLC slave ID |
| `pulse_duration_s` | `0.2` | push-button pulse ON 유지 시간 |
| `enable_watchdog` | `true` | PLC watchdog heartbeat 감시 사용 여부 |
| `watchdog_period_s` | `0.1` | heartbeat 샘플링 주기. 활성화 시 `0.1` 이하 유지 |
| `watchdog_timeout_s` | `0.5` | heartbeat stall 허용 한계 |
| `enable_estop_poll` | `false` | PLC E-stop input polling 사용 여부 |
| `estop_poll_period_s` | `0.1` | E-stop input polling 주기 |
| `db_path` | `robot_arm.db` | PLC 실패를 기록할 SQLite DB 경로 (절대 경로 권장: `db_path:=/path/to/robot_arm.db`) |

운영 배포 단계에서는 udev rule로 PLC serial 장치를 `/dev/plc`로 고정하는 것을
목표로 한다. 현재 개발/검증 기본값은 실제 성공한 `/dev/ttyUSB0`이다.

PLC 메모리 주소는 device label 중심 배열로 관리한다. 세 배열은 같은 index로
묶인다.

```yaml
start_coil_labels: ["M0000", "M0001", "M0002", "M0003", "M0004", "M0005"]
start_coil_addresses: [0, 1, 2, 3, 4, 5]
start_coil_outputs: ["P0040", "P0041", "P0042", "P0043", "P0043", "P0044"]
```

`reset_coil_label` / `reset_coil_address`, `read_register_label` /
`read_register_address`, `write_register_label` / `write_register_address`도 같은
방식으로 PLC 화면 표기와 Modbus address를 함께 기록한다.

## 주소 규칙

XG5000의 bit device 표기는 단순 10진수로 읽으면 안 된다. 예를 들어 `M0100`은
Modbus coil address `100`이 아니라 `0x100`, 즉 address `256`으로 매핑된다.

| PLC device | Modbus address | 용도 |
|------------|----------------|------|
| `M0000` | `0` | P0040 시작 버튼 pulse |
| `M0001` | `1` | P0041 시작 버튼 pulse |
| `M0002` | `2` | P0042 시작 버튼 pulse |
| `M0003` | `3` | e_stop / P0043 시작 버튼 pulse |
| `M0004` | `4` | error / P0043 시작 버튼 pulse |
| `M0005` | `5` | watchdog / P0044 시작 버튼 pulse |
| `M0050` | `80` | PLC 생성 watchdog heartbeat coil |
| `M0100` | `256` | P0040~P0044 자기유지 reset |
| `P000` | holding register `0` | word write 테스트 |
| `P020` | holding register `0` | word read 테스트 |

`M0100`은 래더의 reset 접점이다. `/plc_reset` 또는 `/plc_m100`에
`true`를 보내면 노드가 `M0100 ON -> pulse_duration_s 대기 -> M0100 OFF`를
수행한다.

현재 래더 이미지 기준으로 `M0100`은 reset이고, `M1000~M1005`는 내부 자기유지
bit다. ROS2에서 내부 자기유지 bit를 직접 쓰면 출력 회로 의미가 깨질 수 있으므로
직접 제어하지 않는다. Watchdog/E-stop은 전용 래더를 추가한 뒤 별도 device로
배정해야 한다.

## 토픽

### 상태 발행

| 토픽 | 타입 | 설명 |
|------|------|------|
| `/plc/status` | `interfaces/msg/PLCStatus` | 최신 PLC/system 상태 스냅샷 |
| `/plc/e_stop` | `std_msgs/msg/Bool` | PLC E-stop latch 상태. `true`면 상위 안전 경로가 즉시 정지해야 함 |
| `/plc_word_read` | `std_msgs/msg/Int32` | 주기적으로 읽은 P020 값 |

`/plc/status`와 `/plc/e_stop` QoS는 늦게 붙은 상위 safety/orchestrator도
마지막 상태를 즉시 받아야 하므로 Reliable + Transient Local, depth 1이다.

### bring-up 제어

| 토픽 | 타입 | 동작 |
|------|------|------|
| `/plc_bit_control` | `std_msgs/msg/Bool` | `M0000` pulse 또는 explicit OFF |
| `/plc_m0`~`/plc_m5` | `std_msgs/msg/Bool` | 대응 M coil pulse 또는 explicit OFF |
| `/plc_m_all` | `std_msgs/msg/Bool` | `M0000~M0005` batch pulse 또는 explicit OFF |
| `/plc_m100` | `std_msgs/msg/Bool` | `M0100` reset pulse 또는 explicit OFF |
| `/plc_reset` | `std_msgs/msg/Bool` | `M0100` reset pulse 또는 explicit OFF |
| `/plc/system_state` | `std_msgs/msg/String` | semantic 상태를 PLC 출력 패턴으로 적용 |
| `/plc_word_control` | `std_msgs/msg/Int32` | P000 word write |

`true` 입력은 latched ON이 아니라 push-button pulse다. `false` 입력은 stuck 상태
해제를 위한 explicit OFF로 처리한다.

## Semantic 상태 제어

상위 패키지는 PLC memory address를 직접 알 필요 없이 `/plc/system_state`에
프로젝트 표준 상태 문자열을 보낼 수 있다.

```bash
ros2 topic pub --once /plc/system_state std_msgs/msg/String "{data: moving}"
ros2 topic pub --once /plc/system_state std_msgs/msg/String "{data: error}"
ros2 topic pub --once /plc/system_state std_msgs/msg/String "{data: e_stop}"
ros2 topic pub --once /plc/system_state std_msgs/msg/String "{data: watchdog}"
```

허용 값은 `idle`, `listening`, `inferring`, `moving`, `e_stop`, `error`,
`watchdog`이다.
노드는 일반 상태 적용 전 `M0100` reset coil을 pulse해 기존 래치 출력을 끊고,
`system_state_output_labels`에 연결된 M coil을 push-button처럼 pulse한다.
`idle`은 `M0000`을 pulse해서 초록 상태를 유지하고, reset도 함께 수행한다.
`e_stop`은 reset을 선행하지 않고 `M0003`을 직접 ON 하며, latch 상태에서 자동
복구하지 않는다.
PLC 노드가 연결에 성공하면 기동 직후에도 `idle`을 한 번 적용해서 초록 상태를
바로 보이게 한다.

기본 매핑:

| 상태 | 출력 coil | 연결 출력 |
|------|-----------|-----------|
| `idle` | `M0000` | `P0040` |
| `listening` | `M0001` | `P0041` |
| `inferring` | `M0001` | `P0041` |
| `moving` | `M0002` | `P0042` |
| `e_stop` | `M0003` | `P0043` |
| `error` | `M0004` | `P0043` |
| `watchdog` | `M0005` | `P0044` |

현장 래더에서 출력 의미가 바뀌면
`ros2_ws/src/plc/config/xgb_plc.yaml`의 `system_state_output_labels`만 조정한다.

상위 음성/오케스트레이션 흐름에서는 다음처럼 사용한다.

| 상태 | 주 발행자 | 의미 |
|------|-----------|------|
| `listening` | `voice/whisper_node` | STT 원문이 들어와 음성 입력을 처리 중 |
| `inferring` | `voice`의 intent 분류 노드 | 명령 파싱과 DB Gate 확인 중 |
| `moving` | `motion/orchestrator` | 명령이 승인되어 동작 진행 중 |
| `idle` | `motion/orchestrator` | 동작/처리 완료 후 대기 |
| `error` | `voice`, `db`, `plc_node` | DB Gate 거부, 서비스 실패, PLC 통신 실패 등 |
| `e_stop` | plc_node | 비상 정지 상태 latch 및 출력 |
| `watchdog` | plc_node | watchdog 이상 상태 latch 및 출력 |

## Safety/운영 연동

6단계 safety hook은 현재 두 가지를 지원한다.

| 기능 | 파라미터 | 동작 |
|------|----------|------|
| Watchdog heartbeat | `enable_watchdog` | `watchdog_coil_address`를 읽어 `watchdog_timeout_s` 안에 변동이 있는지 검사 |
| E-stop input polling | `enable_estop_poll` | `estop_input_address`가 `true`면 `e_stop` latch 후 `/plc/e_stop`에 `true` publish |

기본 설정에서는 `M0050` heartbeat 감시가 켜져 있다. 이 신호는 `M0005`
상태 출력과 별개다.
`enable_watchdog=true`일 때는 `watchdog_coil_address`가 반드시 필요하고,
`watchdog_period_s`는 0.1초 이하, `watchdog_timeout_s`는 0.5초 이하만 허용한다.
운영 배선이 확정되면 `config/xgb_plc.yaml`의 주소를 실제 PLC 기준으로 바꾸고
아래처럼 실행한다.

```bash
ros2 launch plc plc.launch.py \
  port:=/dev/ttyUSB0 \
  enable_watchdog:=true \
  watchdog_coil_address:=80 \
  watchdog_period_s:=0.1 \
  watchdog_timeout_s:=0.5 \
  enable_estop_poll:=true \
  estop_input_address:=<전용_estop_input_address> \
  estop_poll_period_s:=0.1
```

E-stop input이 감지되면 노드는 `/plc/status`에 `system_state: e_stop`,
`led_color: red`, `led_mode: solid`를 발행하고 `/plc/e_stop`에 `true`를 발행한다.
E-stop latch 중에는 PLC 출력 변경 요청을 거부하며, 자동 복구하지 않는다.
상위 안전 계층 검증용 echo:

```bash
ros2 topic echo /plc/e_stop
```

## 빌드

```bash
cd <ros2_ws>
source /opt/ros/humble/setup.bash
colcon build --packages-select plc
source install/setup.bash
```

`interfaces`를 아직 빌드하지 않은 fresh workspace에서는 다음처럼 함께 빌드한다.

```bash
colcon build --packages-select interfaces plc
source install/setup.bash
```

## 실행

```bash
cd <ros2_ws>
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch plc plc.launch.py port:=/dev/ttyUSB0 baudrate:=115200 device_id:=1
```

pulse 시간을 현장에서 늘려 확인하려면:

```bash
ros2 launch plc plc.launch.py port:=/dev/ttyUSB0 baudrate:=115200 device_id:=1 pulse_duration_s:=0.5
```

## 런타임 Smoke Test

아래 절차는 실제 PLC와 serial 케이블이 연결된 상태에서 패키지 동작을
현장 smoke test처럼 확인할 때 사용한다. 각 단계는 "입력", "기대 동작",
"실패 판정"이 바로 보이도록 적었다.

### 1. 노드 실행

다른 터미널에서 workspace를 source한 뒤 PLC 노드를 띄운다.

```bash
cd <ros2_ws>
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch plc plc.launch.py port:=/dev/ttyUSB0 baudrate:=115200 device_id:=1
```

`/dev/ttyUSB0`가 아닌 경우 실제 장치로 바꾼다. 필요 시 watchdog이나 E-stop
감시를 끄려면 `enable_watchdog:=false` 또는 `enable_estop_poll:=false`를 명시한다.
DB를 다른 위치로 쓰려면 `db_path:=/absolute/path/to/robot_arm.db`를 같이 넘긴다.

#### watchdog on smoke test

기본 smoke test는 watchdog 감시를 켠 상태로 진행한다.

기대 동작:
- `M0050`이 실제 PLC 래더에서 0.2s ON / 0.2s OFF로 변한다.
- ROS2 노드는 `watchdog_period_s=0.1`, `watchdog_timeout_s=0.5` 기준으로 읽는다.
- `/plc/status`는 정상 상태를 유지하고 `/plc/e_stop`은 `false`다.

실패 판정:
- `M0050` 변화가 멈췄는데도 `watchdog` latch가 안 걸리면 실패다.
- `M0050`가 정상 토글 중인데 `watchdog` latch가 걸리면 샘플링 또는 timeout 설정이 잘못된 것이다.

#### watchdog off smoke test

감시 로직만 끄고 raw pulse와 reset을 확인하고 싶으면 아래처럼 실행한다.

```bash
ros2 launch plc plc.launch.py \
  port:=/dev/ttyUSB0 \
  baudrate:=115200 \
  device_id:=1 \
  enable_watchdog:=false \
  enable_estop_poll:=false
```

기대 동작:
- `/plc_m0`~`/plc_m5` pulse가 정상 동작한다.
- `/plc_reset`과 `/plc_m100`가 `M0100` reset으로 동작한다.
- watchdog 관련 latch나 상태 전이가 발생하지 않는다.

### 2. 상태 토픽 확인

다른 터미널에서 아래 토픽을 열어 둔다.

```bash
ros2 topic echo /plc/status
ros2 topic echo /plc_word_read
ros2 topic echo /plc/e_stop
```

정상이라면 `/plc/status`에는 `system_state`, `led_color`, `led_mode`가 주기적으로
갱신되고, `/plc_word_read`에는 P020 읽기 값이 들어온다.

### 3. M coil pulse 확인

`/plc_m0`부터 `/plc_m5`까지 한 번씩 보내서 각 M coil이 push-button처럼
잠깐 ON 되었다가 OFF 되는지 확인한다.

```bash
ros2 topic pub --once /plc_m0 std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /plc_m1 std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /plc_m2 std_msgs/msg/Bool "{data: true}"
```

노드 로그에서 `M0000 -> ON`, `M0000 -> OFF` 같은 순서가 보이면 pulse가 정상이다.
`true`는 latched ON이 아니라 momentary press여야 한다.

### 4. reset 동작 확인

`M0100` reset pulse를 한 번 보내서 래치 해제가 되는지 확인한다.

```bash
ros2 topic pub --once /plc_reset std_msgs/msg/Bool "{data: true}"
```

정상이라면 로그에 `M0100 -> ON` 다음 `M0100 -> OFF`가 찍히고,
`P0040~P0044` 출력이 해제된다.

### 5. semantic 상태 확인

PLC가 받는 의미 단위 상태도 smoke test에서 확인한다.

```bash
ros2 topic pub --once /plc/system_state std_msgs/msg/String "{data: idle}"
ros2 topic pub --once /plc/system_state std_msgs/msg/String "{data: moving}"
ros2 topic pub --once /plc/system_state std_msgs/msg/String "{data: error}"
```

확인 포인트:
- `idle`은 `M0000` pulse로 `P0040`이 반응하고, reset도 함께 수행한다.
- `moving`은 `M0002` pulse로 `P0042`가 반응해야 한다.
- `error`는 `M0004` pulse로 `P0043`가 반응해야 한다.

### 6. E-stop 확인

E-stop 입력 배선이 실제로 연결된 경우에만 확인한다.

```bash
ros2 topic echo /plc/e_stop
```

입력이 감지되면 `true`가 발행되고, `/plc/status`는 `system_state: e_stop`로
전환된다. E-stop latch 중에는 출력 변경 요청이 거부되어야 한다.

### 7. watchdog timeout 확인

watchdog 감시를 켠 상태에서 `M0050` 변화가 멈췄을 때 latch가 걸리는지 확인한다.
이 테스트는 실제 래더가 0.2s ON / 0.2s OFF라는 전제에서만 유효하다.

확인 방법:
- watchdog heartbeat가 잠시 멈추도록 PLC 쪽 래더를 정지하거나 heartbeat 분기를 차단한다.
- `/plc/status`에서 `watchdog`로 전환되는지 확인한다.
- latch 후에는 출력 변경 요청이 거부되는지 확인한다.
- heartbeat가 다시 살아나도 자동 복구되지 않는지 확인한다.

판정 기준:
- timeout 후 `/plc/status`가 `watchdog`가 아니면 실패다.
- timeout 후에도 출력이 계속 바뀌면 실패다.
- heartbeat 복구만으로 자동 해제되면 실패다.

## 테스트할 때 같이 쓰면 좋은 것들

아래 명령은 smoke test 중 원인을 빠르게 좁힐 때 유용하다.

### ROS2 상태 확인

```bash
ros2 node list
ros2 node info /plc_node
ros2 topic list
ros2 topic info /plc/status
ros2 topic hz /plc/status
ros2 topic echo --once /plc/status
```

- `node list`와 `node info`는 노드가 실제로 떠 있는지 확인할 때 쓴다.
- `topic info`는 누가 publish/subscribe 중인지 볼 때 쓴다.
- `topic hz`는 `/plc/status`가 기대 주기로 나오고 있는지 볼 때 쓴다.
- `echo --once`는 마지막 상태만 빠르게 보고 싶을 때 쓴다.

### 파라미터 확인

```bash
ros2 param list /plc_node
ros2 param get /plc_node enable_watchdog
ros2 param get /plc_node watchdog_coil_address
ros2 param get /plc_node watchdog_period_s
ros2 param get /plc_node watchdog_timeout_s
```

- watchdog 수치가 launch/YAML에서 제대로 들어왔는지 확인할 때 쓴다.
- `enable_estop_poll`과 `estop_input_address`도 같은 방식으로 확인할 수 있다.

### 입력 검증

```bash
ros2 topic pub --once /plc_m0 std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /plc_reset std_msgs/msg/Bool "{data: true}"
ros2 topic pub --once /plc/system_state std_msgs/msg/String "{data: moving}"
```

- `true` 입력은 latch가 아니라 momentary pulse인지 확인할 때 쓴다.
- `system_state`는 semantic 상태가 실제 출력 coil과 맞는지 볼 때 쓴다.

### 기록 남기기

```bash
ros2 bag record /plc/status /plc/e_stop /plc_word_read
```

- 현장에서 상태 전이를 다시 보고 싶을 때 유용하다.
- watchdog timeout이나 E-stop 발생 시점의 순서를 나중에 재확인할 수 있다.

### 코드 레벨 빠른 확인

```bash
python3 -m pytest plc_core/tests/test_modbus_client.py -q
python3 -m pytest ros2_ws/src/plc/test/test_plc_safety_contract.py -q
```

- `plc_core`는 Modbus read/write와 address mapping이 맞는지 볼 때 쓴다.
- `test_plc_safety_contract.py`는 watchdog, E-stop, DB logging, priority 규칙을 확인할 때 쓴다.

## 기본 테스트

다른 터미널에서:

```bash
cd <ros2_ws>
source /opt/ros/humble/setup.bash
source install/setup.bash
```

P0040 시작 pulse:

```bash
ros2 topic pub --once /plc_m0 std_msgs/msg/Bool "{data: true}"
```

P0040~P0043 reset:

```bash
ros2 topic pub --once /plc_reset std_msgs/msg/Bool "{data: true}"
```

동일한 reset을 M device 이름으로 테스트:

```bash
ros2 topic pub --once /plc_m100 std_msgs/msg/Bool "{data: true}"
```

정상이라면 launch 터미널 로그에 `M0100 -> ON` 이후 `M0100 -> OFF`가 출력되고,
PLC 출력 `P0040~P0044`가 꺼진다.

P020 읽기 확인:

```bash
ros2 topic echo /plc_word_read
```

상태 확인:

```bash
ros2 topic echo /plc/status
```

## 종료

launch를 실행한 터미널에서 `Ctrl+C`를 누른다. 노드는 종료 시 serial client를
닫는다.

## 문제 해결

### `/dev/ttyUSB0`가 없음

```bash
ls /dev/ttyUSB*
```

장치 번호가 다르면 launch에서 `port:=/dev/ttyUSB1`처럼 넘긴다. 운영 배포에서는
udev rule로 `/dev/plc` 심링크를 고정한다.

### 권한 오류

현재 사용자에 `dialout` 그룹이 필요하다. 그룹 변경 후에는 재로그인 또는 재부팅이
필요하다.

```bash
groups
sudo usermod -aG dialout "$USER"
```

당장 테스트만 해야 하면 임시로 권한을 열 수 있다.

```bash
sudo chmod 666 /dev/ttyUSB0
```

### reset이 안 됨

- `/plc_reset` 로그가 `M0100 -> ON`, `M0100 -> OFF`로 찍히는지 확인한다.
- 로그가 `M0010` 또는 address `16` 기준처럼 보이면 설정이 오래된 것이다.
- launch를 다시 빌드/source 했는지 확인한다.
- pulse가 너무 짧으면 `pulse_duration_s:=0.5`로 늘린다.

## 현재 코드 구조

| 경로 | 역할 |
|------|------|
| `plc_core/config.py` | ROS2 비의존 Modbus/address 설정 dataclass |
| `plc_core/modbus_client.py` | 실제 pymodbus 기반 coil/register client |
| `ros2_ws/src/plc/plc/plc_node.py` | ROS2 parameter/topic/status wrapper |
| `ros2_ws/src/plc/config/xgb_plc.yaml` | XG5000 device label과 Modbus address 매핑 |

## 다음 개발 방향

1. 상위 orchestrator/voice/db 흐름에서 `/plc/system_state`를 호출하게 연결한다.
2. 실제 현장 LED 색 배선이 확정되면 `system_state_output_labels`를 최종 값으로 고정한다.
3. E-stop 입력과 PLC watchdog heartbeat는 `plc_node`에서 이미 처리 중이므로, 실제 배선 주소와 smoke test 결과만 계속 갱신한다.

## 현재 상태 요약

- `plc_core`의 watchdog read 경로와 `plc_node`의 안전 우선순위, DB 기록, smoke test 문서는 반영됐다.
- `watchdog`, `reset`, `semantic_state`, `DB logging` 관련 단위 테스트는 추가했다.
- `E-stop 입력 실장`은 이번 범위에서 건너뛰었으므로, 실제 배선/활성화는 별도 작업이 필요하다.
- 코드와 문서 기준으로는 정리됐지만, 실제 PLC에 붙여 `M0050` heartbeat와 `watchdog timeout`을 한 번 더 최종 확인하는 현장 검증은 남아 있다.
