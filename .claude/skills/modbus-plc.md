---
name: modbus-plc
description: >
  PLC Modbus RTU/TCP 통신 가이드 — LED 상태 매핑(9종 공구), Modbus 패킷 형식,
  plc_core/ 클라이언트 패턴, watchdog, 비상 정지 연동.
  PLC 통신 구현, LED 상태 제어, plc_core/ 작성 시 활성화.
when_to_use: >
  PLC 통신 구현, LED 상태 매핑, Modbus RTU/TCP 패킷 작성,
  plc_core/ 클라이언트 코드 작성, watchdog 타이머 구현,
  E-stop PLC 연동 시.
---

# PLC Modbus 통신 가이드

> `plc_core/`는 ROS2 의존성 없는 순수 Python. Track A/B: `plc/` 패키지가 ROS2 래퍼 역할.
> 룰: [`.claude/rules/safety.md`](../rules/safety.md) S-3 (E-stop), S-4 (Watchdog), [`.claude/rules/engineering.md`](../rules/engineering.md) E-4 (설정).

## 1. 이 프로젝트의 PLC 역할

| 기능 | 설명 |
|------|------|
| **LED 상태 표시** | 공구함 슬롯별 LED (9종 × 최대 3색) |
| **E-stop 수신** | 물리 비상 정지 버튼 → PLC → ROS2 |
| **Watchdog** | PLC가 일정 시간 heartbeat 없으면 E-stop 자동 발동 |

## 2. LED 상태 매핑

```python
# plc_core/led_states.py
from enum import IntEnum

class LEDState(IntEnum):
    OFF    = 0  # 꺼짐 (공구 제자리, 대기)
    GREEN  = 1  # 초록 — fetch 대상 / 거치 완료
    YELLOW = 2  # 노랑 — staged / FOD 경고
    RED    = 3  # 빨강 — missing / 오류 / E-Stop
    WHITE  = 4  # 하양 — 이동 중 / STT 수음 / 추론 중

# 공구 슬롯 → Modbus 주소 매핑
# config/plc.yaml에서 로드 (하드코딩 금지)
SLOT_REGISTER_MAP: dict[str, int] = {}  # tool_id → coil address
```

```yaml
# config/plc.yaml
schema_version: 1
connection:
  mode: tcp          # tcp | rtu
  host: 192.168.1.10 # TCP 모드
  port: 502
  # rtu 모드 시:
  # port: /dev/ttyUSB0
  # baudrate: 9600
  # parity: N
  timeout_s: 1.0

watchdog:
  interval_s: 0.5    # heartbeat 전송 주기
  timeout_s: 2.0     # PLC watchdog 타임아웃

led_slots:
  screwdriver_phillips_small: 0   # coil 주소
  screwdriver_phillips_large: 1
  screwdriver_flathead_small: 2
  screwdriver_flathead_large: 3
  wrench_8mm:   4
  wrench_10mm:  5
  wrench_13mm:  6
  pliers_standard: 7
  pliers_long:     8
  watchdog_coil:   100   # heartbeat용
  estop_input:     200   # 디지털 입력 (읽기 전용)
```

## 3. Modbus 패킷 형식

### Function Code 정리

| FC | 이름 | 용도 |
|----|------|------|
| 0x01 | Read Coils | LED 상태 읽기 |
| 0x05 | Write Single Coil | LED 1개 제어 |
| 0x0F | Write Multiple Coils | 여러 LED 일괄 제어 |
| 0x02 | Read Discrete Inputs | E-stop 버튼 상태 읽기 |

### RTU 패킷 구조 (참고)
```
[Device Addr][FC][Start Addr Hi][Start Addr Lo][Data Hi][Data Lo][CRC Lo][CRC Hi]
     1byte   1B      1B            1B             1B       1B      1B      1B
```

## 4. pymodbus 클라이언트 (plc_core/)

```bash
pip install pymodbus
```

```python
# plc_core/plc_client.py
from pymodbus.client import ModbusTcpClient, ModbusSerialClient
from pymodbus.exceptions import ModbusException
import yaml
from pathlib import Path
from .led_states import LEDState

class PLCClient:
    def __init__(self, config_path: Path = Path("config/plc.yaml")):
        with config_path.open() as f:
            cfg = yaml.safe_load(f)

        conn = cfg["connection"]
        if conn["mode"] == "tcp":
            self._client = ModbusTcpClient(
                host=conn["host"],
                port=conn["port"],
                timeout=conn["timeout_s"],
            )
        else:
            self._client = ModbusSerialClient(
                port=conn["port"],
                baudrate=conn.get("baudrate", 9600),
                parity=conn.get("parity", "N"),
                timeout=conn["timeout_s"],
            )

        self._slots: dict[str, int] = cfg["led_slots"]
        self._watchdog_coil: int = cfg["led_slots"]["watchdog_coil"]
        self._estop_input: int = cfg["led_slots"]["estop_input"]

    def connect(self) -> bool:
        return self._client.connect()

    def close(self):
        self._client.close()

    def set_led(self, tool_id: str, state: LEDState) -> None:
        coil_addr = self._slots[tool_id]
        # FC 05: Write Single Coil
        # LEDState를 다중 비트 레지스터로 인코딩하는 경우
        # 실제 PLC 프로토콜에 따라 조정
        value = bool(state != LEDState.OFF)
        result = self._client.write_coil(coil_addr, value)
        if result.isError():
            raise PLCError(f"LED 쓰기 실패: {tool_id} state={state}")

    def set_led_batch(self, states: dict[str, LEDState]) -> None:
        """여러 슬롯 LED를 한 번에 설정 (FC 0F)."""
        if not states:
            return
        min_addr = min(self._slots[tid] for tid in states)
        max_addr = max(self._slots[tid] for tid in states)
        count = max_addr - min_addr + 1

        values = [False] * count
        for tool_id, state in states.items():
            idx = self._slots[tool_id] - min_addr
            values[idx] = state != LEDState.OFF

        result = self._client.write_coils(min_addr, values)
        if result.isError():
            raise PLCError("LED 일괄 쓰기 실패")

    def read_estop(self) -> bool:
        """E-stop 버튼 상태 읽기. True = 눌림."""
        result = self._client.read_discrete_inputs(self._estop_input, count=1)
        if result.isError():
            raise PLCError("E-stop 상태 읽기 실패")
        return result.bits[0]

    def heartbeat(self) -> None:
        """Watchdog heartbeat — 주기적으로 호출 필요."""
        self._client.write_coil(self._watchdog_coil, True)
```

## 5. Watchdog 구현

```python
# plc_core/watchdog.py
import threading
import time
import logging

logger = logging.getLogger(__name__)

class PLCWatchdog:
    def __init__(self, plc: PLCClient, interval_s: float = 0.5):
        self._plc = plc
        self._interval = interval_s
        self._active = False
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        self._active = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()
        logger.info("PLC watchdog 시작 (%.1fs 간격)", self._interval)

    def stop(self) -> None:
        self._active = False
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        while self._active:
            try:
                self._plc.heartbeat()
            except PLCError as e:
                logger.error("Watchdog heartbeat 실패: %s", e)
                # heartbeat 실패 시 PLC가 자체 E-stop 발동
            time.sleep(self._interval)
```

## 6. E-stop PLC 연동

```python
# plc/estop_monitor.py (Track A/B — ROS2 래퍼)
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class EStopMonitor(Node):
    def __init__(self, plc: PLCClient):
        super().__init__("estop_monitor")
        self._plc = plc
        self._pub = self.create_publisher(Bool, "/robot/estop", 10)
        self.create_timer(0.05, self._poll)   # 20Hz 폴링

    def _poll(self):
        try:
            pressed = self._plc.read_estop()
        except PLCError as e:
            # PLC 통신 실패 = 안전 불명 → E-stop으로 처리
            self.get_logger().error("E-stop 읽기 실패: %s", e)
            pressed = True

        msg = Bool()
        msg.data = pressed
        self._pub.publish(msg)
        if pressed:
            self.get_logger().warn("E-stop 감지!")
```

## 7. 상태 흐름 — LED와 DB 연동

```python
# orchestrator에서 사용 예
from plc_core import PLCClient, LEDState

def on_fetch_request(tool_id: str, plc: PLCClient):
    # 요청 공구 → 녹색
    plc.set_led(tool_id, LEDState.GREEN)

def on_fetch_complete(tool_id: str, plc: PLCClient):
    # 완료 → 황색 (staged)
    plc.set_led(tool_id, LEDState.YELLOW)

def on_return_complete(tool_id: str, plc: PLCClient):
    # 반납 완료 → 꺼짐
    plc.set_led(tool_id, LEDState.OFF)

def on_fod_alert(tool_id: str, plc: PLCClient):
    # FOD 경고 → 노랑 Flash
    plc.set_led(tool_id, LEDState.YELLOW)
```

## 8. 연결 관리 (Context Manager)

```python
from contextlib import contextmanager

@contextmanager
def plc_session(config_path=Path("config/plc.yaml")):
    client = PLCClient(config_path)
    watchdog = PLCWatchdog(client)
    try:
        if not client.connect():
            raise PLCError("PLC 연결 실패")
        watchdog.start()
        yield client
    finally:
        watchdog.stop()
        # 종료 시 모든 LED 꺼짐
        try:
            for tool_id in client._slots:
                if tool_id not in ("watchdog_coil", "estop_input"):
                    client.set_led(tool_id, LEDState.OFF)
        except PLCError:
            pass
        client.close()

# 사용
with plc_session() as plc:
    plc.set_led("screwdriver_phillips_small", LEDState.GREEN)
```

## 9. 흔한 함정

### ❌ Watchdog 없이 운영
- SW 크래시 → PLC가 로봇 상태 모름 → 안전 불명
- ✅ PLCWatchdog 필수 실행. heartbeat 중단 시 PLC가 자체 E-stop

### ❌ E-stop 통신 실패를 WARNING으로만 처리
- PLC 연결 끊김 = 안전 상태 불명 → 즉시 E-stop 처리
- ✅ `except PLCError: pressed = True` (안전 측으로 fail)

### ❌ LED 주소를 코드에 하드코딩
```python
client.write_coil(5, True)   # 5가 뭔지 모름
```
✅ `config/plc.yaml`의 `led_slots` 맵 사용

### ❌ plc_core/에 rclpy import
- `plc_core/`는 Track C에서도 사용 — ROS2 의존성 금지
- ✅ ROS2 래퍼는 `plc/` 패키지에만

### ❌ 연결 재시도 없이 실패 시 크래시
```python
if not client.connect():
    raise RuntimeError("연결 실패")   # 재시도 없음
```
✅ 지수 백오프로 재시도 (최대 3회)

## 10. 참고

- pymodbus: <https://pymodbus.readthedocs.io/>
- Modbus 프로토콜: <https://modbus.org/docs/Modbus_Application_Protocol_V1_1b3.pdf>
- 프로젝트 룰: [`.claude/rules/safety.md`](../rules/safety.md) S-3, S-4
- 관련 스킬: [`error-handling-patterns`](error-handling-patterns.md), [`config-management`](config-management.md)
- 설정 파일: `config/plc.yaml`
