"""ROS2 비의존 XBC-DR14E Modbus client."""

from __future__ import annotations

import time
from collections.abc import Callable
from typing import Any

from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException

from plc_core.client import PLCStatus
from plc_core.config import ModbusPLCConfig
from plc_core.states import STATE_LED_MAP, SystemState


class PLCError(RuntimeError):
    """PLC Modbus 작업 실패."""


class ModbusPLCClient:
    """XBC-DR14E Modbus RTU client.

    이 클래스는 ROS2를 import하지 않는다. Track A/B에서는 `plc_node`가 이 client를
    감싸고, Track C에서는 필요 시 직접 사용할 수 있다.
    """

    def __init__(
        self,
        config: ModbusPLCConfig,
        client_factory: Callable[..., Any] = ModbusSerialClient,
    ) -> None:
        self._config = config
        # ROS2 wrapper가 마지막으로 적용한 의미 상태를 /plc/status snapshot으로 재사용한다.
        self._current_state = SystemState.IDLE
        # pymodbus serial client 생성만 여기서 하고, 실제 연결은 connect()에서 연다.
        self._client = client_factory(
            port=config.port,
            baudrate=config.baudrate,
            parity=config.parity,
            stopbits=config.stopbits,
            bytesize=config.bytesize,
        )

    @property
    def config(self) -> ModbusPLCConfig:
        """현재 client 설정."""

        return self._config

    @property
    def connected(self) -> bool:
        """serial client가 연결 상태로 간주되는지 반환한다."""

        return bool(getattr(self._client, "connected", False))

    def connect(self) -> bool:
        """PLC serial 연결을 연다."""

        try:
            return bool(self._client.connect())
        except (ModbusException, OSError) as exc:
            raise PLCError(f"PLC connect failed: {exc}") from exc

    def close(self) -> None:
        """PLC serial 연결을 닫는다."""

        self._client.close()

    def read_register(self, address: int) -> int:
        """holding register 1개를 읽는다."""

        try:
            # FC03 Read Holding Registers. 현재 P020 read 확인에 사용한다.
            response = self._client.read_holding_registers(
                address=address,
                count=1,
                device_id=self._config.device_id,
            )
        except (ModbusException, OSError) as exc:
            raise PLCError(f"register read failed address={address}: {exc}") from exc

        self._raise_if_error(response, f"register read failed address={address}")
        return int(response.registers[0])

    def read_discrete_input(self, address: int) -> bool:
        """discrete input 1개를 읽는다."""

        try:
            # FC02 Read Discrete Inputs. E-stop input 전용 래더가 생기면 이 경로를 쓴다.
            response = self._client.read_discrete_inputs(
                address=address,
                count=1,
                device_id=self._config.device_id,
            )
        except (ModbusException, OSError) as exc:
            raise PLCError(
                f"discrete input read failed address={address}: {exc}"
            ) from exc

        self._raise_if_error(response, f"discrete input read failed address={address}")
        return bool(response.bits[0])

    def write_register(self, address: int, value: int) -> None:
        """holding register 1개에 값을 쓴다."""

        try:
            # FC06 Write Single Register. 현재 P000 word write bring-up에 사용한다.
            response = self._client.write_register(
                address=address,
                value=int(value),
                device_id=self._config.device_id,
            )
        except (ModbusException, OSError) as exc:
            raise PLCError(
                f"register write failed address={address} value={value}: {exc}"
            ) from exc

        self._raise_if_error(
            response,
            f"register write failed address={address} value={value}",
        )

    def write_coil(self, address: int, value: bool) -> None:
        """M coil 1개에 값을 쓴다."""

        try:
            # FC05 Write Single Coil. M0000~M0003 start와 M0010 reset pulse에 사용한다.
            response = self._client.write_coil(
                address=address,
                value=bool(value),
                device_id=self._config.device_id,
            )
        except (ModbusException, OSError) as exc:
            raise PLCError(
                f"coil write failed address={address} value={value}: {exc}"
            ) from exc

        self._raise_if_error(response, f"coil write failed address={address}")

    def pulse_coil(
        self,
        address: int,
        pulse_duration_s: float | None = None,
    ) -> None:
        """push-button처럼 coil을 잠깐 ON 했다가 OFF로 되돌린다."""

        duration_s = (
            self._config.pulse_duration_s
            if pulse_duration_s is None
            else pulse_duration_s
        )
        # 래더의 시작 입력은 push-button 접점이므로 ON으로 고정하지 않고 반드시 OFF로 복귀한다.
        self.write_coil(address, True)
        time.sleep(duration_s)
        self.write_coil(address, False)

    def write_start_coils(self, value: bool) -> None:
        """설정된 start coil 전체를 ON/OFF 처리한다."""

        addresses = self._config.start_coil_addresses
        if not self._config.start_coils_are_contiguous():
            # 주소가 띄엄띄엄이면 FC0F batch write 대신 안전하게 하나씩 쓴다.
            for address in addresses:
                self.write_coil(address, value)
            return

        values = [bool(value)] * len(addresses)
        try:
            # FC0F Write Multiple Coils. 현재 M0000~M0003이 연속이라 bring-up에 쓸 수 있다.
            response = self._client.write_coils(
                address=addresses[0],
                values=values,
                device_id=self._config.device_id,
            )
        except (ModbusException, OSError) as exc:
            raise PLCError(f"start coil batch write failed: {exc}") from exc

        self._raise_if_error(response, "start coil batch write failed")

    def pulse_start_coils(self, pulse_duration_s: float | None = None) -> None:
        """설정된 start coil 전체를 push-button처럼 pulse 처리한다."""

        duration_s = (
            self._config.pulse_duration_s
            if pulse_duration_s is None
            else pulse_duration_s
        )
        self.write_start_coils(True)
        time.sleep(duration_s)
        self.write_start_coils(False)

    def reset_outputs(self, pulse_duration_s: float | None = None) -> None:
        """reset coil을 pulse해 래더의 자기유지 출력을 끊는다."""

        self.pulse_coil(self._config.reset_coil_address, pulse_duration_s)

    def heartbeat(self) -> None:
        """PLC watchdog heartbeat coil을 ON으로 쓴다."""

        if self._config.watchdog_coil_address is None:
            raise PLCError("watchdog coil address is not configured")
        # 현재 래더에는 watchdog coil이 없다. 전용 coil이 설정된 경우에만 호출된다.
        self.write_coil(self._config.watchdog_coil_address, True)

    def read_estop(self) -> bool:
        """PLC E-stop discrete input을 읽는다. True는 E-stop 감지를 뜻한다."""

        if self._config.estop_input_address is None:
            raise PLCError("E-stop input address is not configured")
        # 현재 래더에는 E-stop input이 없다. 전용 입력이 설정된 경우에만 호출된다.
        return self.read_discrete_input(self._config.estop_input_address)

    def set_system_state(
        self,
        state: SystemState,
        *,
        apply_outputs: bool = True,
        reset_before_apply: bool = True,
    ) -> PLCStatus:
        """semantic system state를 PLC 출력 패턴과 상태 snapshot으로 적용한다.

        `apply_outputs=False`이면 실제 Modbus write 없이 내부 상태 snapshot만
        갱신한다. ROS2 wrapper가 연결 실패 상태를 알릴 때 사용할 수 있다.
        """

        if apply_outputs:
            output_labels = self._config.output_labels_for_state(state)
            if reset_before_apply:
                # 상태 전환 전 기존 자기유지 M0100~M0103을 끊기 위해 M0010을 먼저 pulse한다.
                self.reset_outputs()
            for label in output_labels:
                # 상위 상태는 M label로 매핑되고, 여기서 실제 Modbus address로 변환된다.
                self.pulse_coil(self._config.coil_address_for_label(label))

        self._current_state = state
        return self.get_status()

    def set_error(self, *, apply_outputs: bool = True) -> PLCStatus:
        """일반 오류 상태를 적용한다."""

        return self.set_system_state(SystemState.ERROR, apply_outputs=apply_outputs)

    def set_estop(self, *, apply_outputs: bool = True) -> PLCStatus:
        """E-stop 상태를 적용한다."""

        return self.set_system_state(SystemState.E_STOP, apply_outputs=apply_outputs)

    def get_status(self) -> PLCStatus:
        """마지막 semantic system state를 PLCStatus snapshot으로 반환한다."""

        color, mode = STATE_LED_MAP[self._current_state]
        return PLCStatus(
            led_color=color,
            led_mode=mode,
            system_state=self._current_state,
        )

    @staticmethod
    def _raise_if_error(response: Any, message: str) -> None:
        if response.isError():
            raise PLCError(message)
