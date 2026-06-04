"""PLC Modbus 연결과 XG5000 device address 설정."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

from plc_core.states import SystemState


def _empty_system_state_outputs() -> dict[SystemState, tuple[str, ...]]:
    """기본 semantic 상태는 실제 출력 coil을 건드리지 않게 둔다."""

    return {state: () for state in SystemState}


@dataclass(frozen=True)
class ModbusPLCConfig:
    """ROS2에 의존하지 않는 PLC Modbus 설정 값."""

    # Serial 통신 기본값이다. ROS2 node는 launch/YAML parameter에서 이 값을 채운다.
    port: str
    baudrate: int
    parity: str
    stopbits: int
    bytesize: int
    device_id: int
    # M0000~M0005 같은 시작 버튼 coil을 label/address/output 세 배열로 관리한다.
    # 세 배열은 같은 index끼리 하나의 PLC 회로를 뜻한다.
    start_coil_labels: tuple[str, ...]
    start_coil_addresses: tuple[int, ...]
    start_coil_outputs: tuple[str, ...]
    # M0100은 현재 래더의 reset 접점이다. ON pulse로 자기유지를 끊는다.
    reset_coil_label: str
    reset_coil_address: int
    # P word register는 bring-up 확인용 read/write 테스트 지점이다.
    read_register_label: str
    read_register_address: int
    write_register_label: str
    write_register_address: int
    # Start/reset coil은 latch가 아니라 push-button처럼 짧게 눌렀다 떼는 방식으로 쓴다.
    pulse_duration_s: float
    # M0005는 상태 출력용 watchdog 입력이다. heartbeat watchdog/E-stop hook은
    # 전용 래더를 추가한 뒤에만 주소를 채운다.
    watchdog_coil_label: str | None = None
    watchdog_coil_address: int | None = None
    estop_input_label: str | None = None
    estop_input_address: int | None = None
    # 상위 ROS2 패키지는 PLC 주소 대신 idle/moving/error 같은 의미 상태만 보낸다.
    system_state_outputs: Mapping[SystemState | str, Sequence[str] | str] = field(
        default_factory=_empty_system_state_outputs
    )

    def __post_init__(self) -> None:
        # label/address/output 배열이 어긋나면 다른 출력이 켜질 수 있으므로 즉시 거부한다.
        if not (
            len(self.start_coil_labels)
            == len(self.start_coil_addresses)
            == len(self.start_coil_outputs)
        ):
            raise PLCConfigError(
                "start_coil_labels, start_coil_addresses, "
                "start_coil_outputs must have the same length"
            )
        if not self.start_coil_labels:
            raise PLCConfigError("at least one start coil must be configured")
        if self.pulse_duration_s <= 0.0:
            raise PLCConfigError("pulse_duration_s must be positive")
        # reset은 현재 래더상 M device coil이어야 한다.
        if not self.reset_coil_label.upper().startswith("M"):
            raise PLCConfigError("reset_coil_label must be an M device label")
        # watchdog은 쓰기용 coil이므로 M device만 허용한다. 현재 기본값은 비활성이다.
        if self.watchdog_coil_label and not self.watchdog_coil_label.upper().startswith(
            "M"
        ):
            raise PLCConfigError("watchdog_coil_label must be an M device label")
        if self.watchdog_coil_label and self.watchdog_coil_address is None:
            raise PLCConfigError("watchdog_coil_address is required")
        if self.watchdog_coil_address is not None and not self.watchdog_coil_label:
            raise PLCConfigError("watchdog_coil_label is required")
        if self.estop_input_label and self.estop_input_address is None:
            raise PLCConfigError("estop_input_address is required")
        if self.estop_input_address is not None and not self.estop_input_label:
            raise PLCConfigError("estop_input_label is required")
        # 시작 입력도 모두 M device여야 한다. P 출력은 직접 쓰지 않는다.
        for label in self.start_coil_labels:
            if not label.upper().startswith("M"):
                raise PLCConfigError(f"start coil label must be an M device: {label}")
        normalized_outputs = self._normalize_system_state_outputs(
            self.system_state_outputs
        )
        configured_labels = {label.upper() for label in self.start_coil_labels}
        # semantic state가 미등록 coil을 가리키면 런타임에서 엉뚱한 출력을 누르게 된다.
        for state, labels in normalized_outputs.items():
            for label in labels:
                if label.upper() not in configured_labels:
                    raise PLCConfigError(
                        f"system_state_outputs[{state.value}] references "
                        f"unknown start coil label: {label}"
                    )
        object.__setattr__(self, "system_state_outputs", normalized_outputs)

    @property
    def first_start_coil_address(self) -> int:
        """기존 `/plc_bit_control`이 사용하는 첫 start coil address."""

        return self.start_coil_addresses[0]

    @property
    def first_start_coil_label(self) -> str:
        """기존 `/plc_bit_control`이 사용하는 첫 start coil label."""

        return self.start_coil_labels[0]

    @property
    def first_start_output_label(self) -> str:
        """첫 start coil과 연결된 출력 label."""

        return self.start_coil_outputs[0]

    def start_coils_are_contiguous(self) -> bool:
        """Batch write 가능하도록 start coil 주소가 연속인지 반환한다."""

        return all(
            next_address == address + 1
            for address, next_address in zip(
                self.start_coil_addresses,
                self.start_coil_addresses[1:],
            )
        )

    def coil_address_for_label(self, label: str) -> int:
        """XG5000 M label에 대응하는 configured Modbus coil address를 반환한다."""

        normalized = label.upper()
        for coil_label, address in zip(
            self.start_coil_labels,
            self.start_coil_addresses,
        ):
            if coil_label.upper() == normalized:
                return address
        raise PLCConfigError(f"unknown start coil label: {label}")

    def output_labels_for_state(self, state: SystemState) -> tuple[str, ...]:
        """semantic system state에 연결된 start coil label 목록을 반환한다."""

        return self.system_state_outputs[state]

    @staticmethod
    def parse_system_state_outputs(
        state_names: Sequence[str],
        output_label_specs: Sequence[str],
    ) -> dict[SystemState, tuple[str, ...]]:
        """ROS2 flat parameter 2개를 semantic output mapping으로 변환한다.

        `output_label_specs`의 각 항목은 `M0000,M0001`처럼 comma-separated
        label을 받을 수 있다. 빈 문자열, `none`, `-`는 출력 없음으로 처리한다.
        """

        if len(state_names) != len(output_label_specs):
            raise PLCConfigError(
                "system_state_names and system_state_output_labels "
                "must have the same length"
            )

        outputs: dict[SystemState, tuple[str, ...]] = {}
        for raw_state, raw_labels in zip(state_names, output_label_specs):
            try:
                # YAML/launch에서는 문자열로 들어오므로 enum으로 강제 변환한다.
                state = SystemState(str(raw_state))
            except ValueError as exc:
                raise PLCConfigError(f"unsupported system state: {raw_state}") from exc

            label_spec = str(raw_labels).strip()
            # reset만 하고 어떤 출력도 선택하지 않는 상태를 표현할 수 있게 둔다.
            if label_spec.lower() in {"", "-", "none"}:
                outputs[state] = ()
                continue

            outputs[state] = tuple(
                label.strip().upper()
                for label in label_spec.split(",")
                if label.strip()
            )

        return outputs

    @staticmethod
    def m_topic_suffix(label: str) -> str:
        """XG5000 M label을 bring-up topic suffix로 변환한다.

        예: `M0003` -> `3`, `M0010` -> `16`, `M0100` -> `256`.
        """

        normalized = label.upper()
        if not normalized.startswith("M"):
            raise PLCConfigError(f"unsupported M coil label: {label}")
        # XG5000 bit label은 16진 표기처럼 취급한다. M0010은 10이 아니라 0x10=16이다.
        return str(int(normalized[1:], 16))

    @staticmethod
    def coil_display_label(coil_label: str, output_label: str) -> str:
        """로그에서 coil과 연결 output을 함께 보여준다."""

        return f"{coil_label} ({output_label})"

    @staticmethod
    def _normalize_system_state_outputs(
        outputs: Mapping[SystemState | str, Sequence[str] | str],
    ) -> dict[SystemState, tuple[str, ...]]:
        normalized = _empty_system_state_outputs()
        for raw_state, raw_labels in outputs.items():
            try:
                state = (
                    raw_state
                    if isinstance(raw_state, SystemState)
                    else SystemState(str(raw_state))
                )
            except ValueError as exc:
                raise PLCConfigError(f"unsupported system state: {raw_state}") from exc

            if isinstance(raw_labels, str):
                labels = tuple(
                    label.strip().upper()
                    for label in raw_labels.split(",")
                    if label.strip()
                )
            else:
                labels = tuple(str(label).strip().upper() for label in raw_labels)
            normalized[state] = labels
        return normalized


class PLCConfigError(ValueError):
    """PLC 설정 값이 유효하지 않을 때 발생하는 예외."""
