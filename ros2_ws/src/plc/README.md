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
- 선택형 watchdog heartbeat coil 갱신
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
| `enable_watchdog` | `false` | PLC watchdog heartbeat 사용 여부 |
| `watchdog_period_s` | `0.25` | heartbeat 주기. 활성화 시 `0.25` 이하 유지 |
| `enable_estop_poll` | `false` | PLC E-stop input polling 사용 여부 |
| `estop_poll_period_s` | `0.1` | E-stop input polling 주기 |

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
`idle`은 출력 coil 없이 reset만 수행한다.
`e_stop`은 reset을 선행하지 않고 `M0003`을 직접 ON 하며, latch 상태에서 자동
복구하지 않는다.

기본 매핑:

| 상태 | 출력 coil | 연결 출력 |
|------|-----------|-----------|
| `idle` | `none` | reset only |
| `listening` | `M0001` | `P0041` |
| `inferring` | `M0001` | `P0041` |
| `moving` | `M0002` | `P0042` |
| `e_stop` | `M0003` | `P0043` |
| `error` | `M0004` | `P0043` |
| `watchdog` | `M0005` | `P0044` |

현장 래더에서 출력 의미가 바뀌면
`ros2_ws/src/plc/config/xgb_plc.yaml`의 `system_state_output_labels`만 조정한다.

VOICE/DB 연동 흐름에서는 다음처럼 사용한다.

| 상태 | 주 발행자 | 의미 |
|------|-----------|------|
| `listening` | `voice/whisper_node` | STT 원문이 들어와 음성 입력을 처리 중 |
| `inferring` | `voice/rule_intent_node` | 명령 파싱과 DB Gate 확인 중 |
| `moving` | `db/intent_status_simulator_node` 또는 motion/orchestrator | 명령이 승인되어 동작 진행 중 |
| `idle` | `db/intent_status_simulator_node` 또는 motion/orchestrator | 동작/처리 완료 후 대기 |
| `error` | `voice`, `db`, `plc_node` | DB Gate 거부, 서비스 실패, PLC 통신 실패 등 |
| `e_stop` | safety/orchestrator 예정 | 비상 정지 상태 |
| `watchdog` | safety/orchestrator 예정 | watchdog 이상 상태 출력 |

## Safety/운영 연동

6단계 safety hook은 현재 두 가지를 지원한다.

| 기능 | 파라미터 | 동작 |
|------|----------|------|
| Watchdog heartbeat | `enable_watchdog` | `watchdog_coil_address`에 주기적으로 toggle write |
| E-stop input polling | `enable_estop_poll` | `estop_input_address`가 `true`면 `e_stop` latch 후 `/plc/e_stop`에 `true` publish |

기본값은 둘 다 `false`다. 여기서 watchdog heartbeat는 `M0005` 상태 출력과 별개다.
래더에서 heartbeat용 watchdog coil과 E-stop input 주소가 확정되기 전에는 기존
P0040~P0044 bring-up에 영향을 주지 않기 위해 비활성으로 둔다.
`enable_watchdog=true`일 때는 `watchdog_coil_address`가 반드시 필요하고,
`watchdog_period_s`는 0.25초 이하만 허용한다.
운영 배선이 확정되면 `config/xgb_plc.yaml`의 주소를 실제 PLC 기준으로 바꾸고
아래처럼 실행한다.

```bash
ros2 launch plc plc.launch.py \
  port:=/dev/ttyUSB0 \
  enable_watchdog:=true \
  watchdog_coil_address:=<전용_watchdog_M_address> \
  watchdog_period_s:=0.25 \
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
cd /home/thomas/Final_Project/ros2_ws
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
cd /home/thomas/Final_Project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch plc plc.launch.py port:=/dev/ttyUSB0 baudrate:=115200 device_id:=1
```

pulse 시간을 현장에서 늘려 확인하려면:

```bash
ros2 launch plc plc.launch.py port:=/dev/ttyUSB0 baudrate:=115200 device_id:=1 pulse_duration_s:=0.5
```

## 기본 테스트

다른 터미널에서:

```bash
cd /home/thomas/Final_Project/ros2_ws
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
3. E-stop 입력과 PLC watchdog heartbeat를 별도 topic/service로 추가한다.
