from __future__ import annotations

import pytest

from plc_core.config import ModbusPLCConfig, PLCConfigError
from plc_core.modbus_client import ModbusPLCClient, PLCError
from plc_core.states import LEDColor, LEDMode, SystemState


class FakeResponse:
    def __init__(
        self,
        *,
        error: bool = False,
        registers: list[int] | None = None,
        bits: list[bool] | None = None,
    ) -> None:
        self.registers = registers or [0]
        self.bits = bits or [False]
        self._error = error

    def isError(self) -> bool:
        return self._error


class FakeSerialClient:
    def __init__(self, **kwargs) -> None:
        self.kwargs = kwargs
        self.connected = False
        self.calls: list[tuple[str, dict]] = []
        self.next_error = False

    def connect(self) -> bool:
        self.connected = True
        self.calls.append(("connect", {}))
        return True

    def close(self) -> None:
        self.connected = False
        self.calls.append(("close", {}))

    def read_holding_registers(self, **kwargs) -> FakeResponse:
        self.calls.append(("read_holding_registers", kwargs))
        return FakeResponse(error=self.next_error, registers=[123])

    def read_discrete_inputs(self, **kwargs) -> FakeResponse:
        self.calls.append(("read_discrete_inputs", kwargs))
        return FakeResponse(error=self.next_error, bits=[True])

    def read_coils(self, **kwargs) -> FakeResponse:
        self.calls.append(("read_coils", kwargs))
        return FakeResponse(error=self.next_error, bits=[True])

    def write_register(self, **kwargs) -> FakeResponse:
        self.calls.append(("write_register", kwargs))
        return FakeResponse(error=self.next_error)

    def write_coil(self, **kwargs) -> FakeResponse:
        self.calls.append(("write_coil", kwargs))
        return FakeResponse(error=self.next_error)

    def write_coils(self, **kwargs) -> FakeResponse:
        self.calls.append(("write_coils", kwargs))
        return FakeResponse(error=self.next_error)


@pytest.fixture
def config() -> ModbusPLCConfig:
    return ModbusPLCConfig(
        port="/dev/ttyUSB0",
        baudrate=115200,
        parity="N",
        stopbits=1,
        bytesize=8,
        device_id=1,
        start_coil_labels=(
            "M0000",
            "M0001",
            "M0002",
            "M0003",
            "M0004",
            "M0005",
        ),
        start_coil_addresses=(0, 1, 2, 3, 4, 5),
        start_coil_outputs=(
            "P0040",
            "P0041",
            "P0042",
            "P0043",
            "P0043",
            "P0044",
        ),
        reset_coil_label="M0100",
        reset_coil_address=256,
        read_register_label="P020",
        read_register_address=0,
        write_register_label="P000",
        write_register_address=0,
        pulse_duration_s=0.001,
        watchdog_coil_label="M0050",
        watchdog_coil_address=80,
        estop_input_label="P010",
        estop_input_address=10,
        system_state_outputs={
            SystemState.IDLE: ("M0000",),
            SystemState.LISTENING: ("M0001",),
            SystemState.INFERRING: ("M0001",),
            SystemState.MOVING: ("M0002",),
            SystemState.E_STOP: ("M0003",),
            SystemState.ERROR: ("M0004",),
            SystemState.WATCHDOG: ("M0005",),
        },
    )


@pytest.fixture
def fake_client() -> FakeSerialClient:
    return FakeSerialClient()


def test_config_rejects_mismatched_start_coil_arrays() -> None:
    with pytest.raises(PLCConfigError):
        ModbusPLCConfig(
            port="/dev/ttyUSB0",
            baudrate=115200,
            parity="N",
            stopbits=1,
            bytesize=8,
            device_id=1,
            start_coil_labels=("M0000",),
            start_coil_addresses=(0, 1),
            start_coil_outputs=("P0040",),
            reset_coil_label="M0100",
            reset_coil_address=256,
            read_register_label="P020",
            read_register_address=0,
            write_register_label="P000",
            write_register_address=0,
            pulse_duration_s=0.2,
        )


def test_m_topic_suffix_uses_xg5000_hex_bit_label() -> None:
    assert ModbusPLCConfig.m_topic_suffix("M0010") == "16"
    assert ModbusPLCConfig.m_topic_suffix("M0100") == "256"


def test_parse_system_state_outputs_supports_empty_and_comma_labels() -> None:
    outputs = ModbusPLCConfig.parse_system_state_outputs(
        ("idle", "moving", "error"),
        ("M0000", "M0001,M0002", "M0004"),
    )

    assert outputs[SystemState.IDLE] == ("M0000",)
    assert outputs[SystemState.MOVING] == ("M0001", "M0002")
    assert outputs[SystemState.ERROR] == ("M0004",)


def test_config_rejects_unknown_semantic_output_label(
    config: ModbusPLCConfig,
) -> None:
    with pytest.raises(PLCConfigError):
        ModbusPLCConfig(
            **{
                **config.__dict__,
                "system_state_outputs": {SystemState.ERROR: ("M0099",)},
            }
        )


def test_connect_builds_serial_client_with_config(config: ModbusPLCConfig) -> None:
    created: list[FakeSerialClient] = []

    def factory(**kwargs) -> FakeSerialClient:
        client = FakeSerialClient(**kwargs)
        created.append(client)
        return client

    plc = ModbusPLCClient(config, client_factory=factory)

    assert plc.connect() is True
    assert created[0].kwargs == {
        "port": "/dev/ttyUSB0",
        "baudrate": 115200,
        "parity": "N",
        "stopbits": 1,
        "bytesize": 8,
    }


def test_read_register_uses_device_id(config: ModbusPLCConfig) -> None:
    fake_client = FakeSerialClient()
    plc = ModbusPLCClient(config, client_factory=lambda **_: fake_client)

    value = plc.read_register(config.read_register_address)

    assert value == 123
    assert fake_client.calls[-1] == (
        "read_holding_registers",
        {"address": 0, "count": 1, "device_id": 1},
    )


def test_read_estop_uses_configured_discrete_input(config: ModbusPLCConfig) -> None:
    fake_client = FakeSerialClient()
    plc = ModbusPLCClient(config, client_factory=lambda **_: fake_client)

    assert plc.read_estop() is True
    assert fake_client.calls[-1] == (
        "read_discrete_inputs",
        {"address": 10, "count": 1, "device_id": 1},
    )


def test_watchdog_read_uses_configured_watchdog_coil(
    config: ModbusPLCConfig,
) -> None:
    fake_client = FakeSerialClient()
    plc = ModbusPLCClient(config, client_factory=lambda **_: fake_client)

    assert plc.read_watchdog() is True

    assert fake_client.calls[-1] == (
        "read_coils",
        {"address": 80, "count": 1, "device_id": 1},
    )


def test_heartbeat_alias_reads_configured_watchdog_coil(
    config: ModbusPLCConfig,
) -> None:
    fake_client = FakeSerialClient()
    plc = ModbusPLCClient(config, client_factory=lambda **_: fake_client)

    assert plc.heartbeat() is True

    assert fake_client.calls[-1] == (
        "read_coils",
        {"address": 80, "count": 1, "device_id": 1},
    )


def test_watchdog_read_requires_configured_watchdog_coil(
    config: ModbusPLCConfig,
) -> None:
    no_watchdog = ModbusPLCConfig(
        **{
            **config.__dict__,
            "watchdog_coil_label": None,
            "watchdog_coil_address": None,
        }
    )
    plc = ModbusPLCClient(no_watchdog, client_factory=lambda **_: FakeSerialClient())

    with pytest.raises(PLCError):
        plc.read_watchdog()


def test_read_coil_raises_plc_error_on_modbus_failure(
    config: ModbusPLCConfig,
) -> None:
    fake_client = FakeSerialClient()
    fake_client.next_error = True
    plc = ModbusPLCClient(config, client_factory=lambda **_: fake_client)

    with pytest.raises(PLCError):
        plc.read_coil(config.watchdog_coil_address)


def test_pulse_coil_writes_on_then_off(config: ModbusPLCConfig) -> None:
    fake_client = FakeSerialClient()
    plc = ModbusPLCClient(config, client_factory=lambda **_: fake_client)

    plc.pulse_coil(config.reset_coil_address)

    assert fake_client.calls[-2:] == [
        ("write_coil", {"address": 256, "value": True, "device_id": 1}),
        ("write_coil", {"address": 256, "value": False, "device_id": 1}),
    ]


def test_set_system_state_resets_then_pulses_configured_output(
    config: ModbusPLCConfig,
) -> None:
    fake_client = FakeSerialClient()
    plc = ModbusPLCClient(config, client_factory=lambda **_: fake_client)

    status = plc.set_system_state(SystemState.MOVING)

    assert status.system_state == SystemState.MOVING
    assert status.led_color == LEDColor.RED
    assert status.led_mode == LEDMode.PULSE
    assert fake_client.calls[-4:] == [
        ("write_coil", {"address": 256, "value": True, "device_id": 1}),
        ("write_coil", {"address": 256, "value": False, "device_id": 1}),
        ("write_coil", {"address": 2, "value": True, "device_id": 1}),
        ("write_coil", {"address": 2, "value": False, "device_id": 1}),
    ]


def test_set_estop_does_not_reset_before_output(config: ModbusPLCConfig) -> None:
    fake_client = FakeSerialClient()
    plc = ModbusPLCClient(config, client_factory=lambda **_: fake_client)

    status = plc.set_estop()

    assert status.system_state == SystemState.E_STOP
    assert fake_client.calls[-2:] == [
        ("write_coil", {"address": 3, "value": True, "device_id": 1}),
        ("write_coil", {"address": 3, "value": False, "device_id": 1}),
    ]


def test_set_system_state_can_update_status_without_modbus_writes(
    config: ModbusPLCConfig,
) -> None:
    fake_client = FakeSerialClient()
    plc = ModbusPLCClient(config, client_factory=lambda **_: fake_client)

    status = plc.set_error(apply_outputs=False)

    assert status.system_state == SystemState.ERROR
    assert status.led_color == LEDColor.RED
    assert fake_client.calls == []


def test_write_start_coils_batches_contiguous_addresses(config: ModbusPLCConfig) -> None:
    fake_client = FakeSerialClient()
    plc = ModbusPLCClient(config, client_factory=lambda **_: fake_client)

    plc.write_start_coils(True)

    assert fake_client.calls[-1] == (
        "write_coils",
        {
            "address": 0,
            "values": [True, True, True, True, True, True],
            "device_id": 1,
        },
    )


def test_write_start_coils_falls_back_for_non_contiguous_addresses(
    config: ModbusPLCConfig,
) -> None:
    non_contiguous = ModbusPLCConfig(
        **{
            **config.__dict__,
            "start_coil_addresses": (0, 2),
            "start_coil_labels": ("M0000", "M0002"),
            "start_coil_outputs": ("P0040", "P0042"),
            "system_state_outputs": {
                SystemState.IDLE: ("M0000",),
                SystemState.MOVING: ("M0002",),
            },
        }
    )
    fake_client = FakeSerialClient()
    plc = ModbusPLCClient(non_contiguous, client_factory=lambda **_: fake_client)

    plc.write_start_coils(True)

    assert fake_client.calls[-2:] == [
        ("write_coil", {"address": 0, "value": True, "device_id": 1}),
        ("write_coil", {"address": 2, "value": True, "device_id": 1}),
    ]


def test_modbus_error_raises_plc_error(config: ModbusPLCConfig) -> None:
    fake_client = FakeSerialClient()
    fake_client.next_error = True
    plc = ModbusPLCClient(config, client_factory=lambda **_: fake_client)

    with pytest.raises(PLCError):
        plc.write_register(config.write_register_address, 7)
