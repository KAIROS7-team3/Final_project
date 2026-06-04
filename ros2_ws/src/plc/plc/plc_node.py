"""LS Electric XBC-DR14E용 ROS2 Modbus RTU bridge.

이 노드는 프로젝트 `plc_core`의 Modbus client를 ROS2 topic surface로 노출한다.
bring-up용 raw M/P topic을 유지하면서 `/plc/status`도 발행해 운영자 UI/상위
노드가 PLC 상태를 볼 수 있게 한다.
"""

from __future__ import annotations

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

import rclpy
from interfaces.msg import PLCStatus as PLCStatusMsg
from plc_core import (
    ModbusPLCClient,
    ModbusPLCConfig,
    PLCError,
    PLCStatus as CorePLCStatus,
    SystemState,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String


@dataclass(frozen=True)
class PlcNodeConfig:
    """ROS2 node runtime 설정."""

    modbus: ModbusPLCConfig
    read_period_s: float
    connect_retry_count: int
    connect_retry_backoff_s: float
    enable_watchdog: bool
    watchdog_period_s: float
    enable_estop_poll: bool
    estop_poll_period_s: float


class XgbRos2ModbusNode(Node):
    """XBC-DR14E Modbus RTU 배선 테스트 노드의 ROS2 래퍼."""

    def __init__(self) -> None:
        super().__init__("plc_node")
        self._config = self._load_config()
        self._plc = ModbusPLCClient(self._config.modbus)
        # E-stop은 자동 복구하지 않는다. true가 되면 운영자가 노드를 재시작해야 한다.
        self._estop_latched = threading.Event()
        self._pulse_timers: list[object] = []

        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # PLC 상태는 늦게 붙은 모니터도 마지막 안전 상태를 즉시 받아야 한다.
        self._status_pub = self.create_publisher(
            PLCStatusMsg,
            "/plc/status",
            status_qos,
        )
        self._word_pub = self.create_publisher(Int32, "/plc_word_read", 10)
        # 상위 safety/orchestrator가 PLC 물리 E-stop 감지를 받을 수 있는 최소 topic이다.
        estop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._estop_pub = self.create_publisher(Bool, "/plc/e_stop", estop_qos)
        self._publish_estop(False)

        # 아래 topic들은 standalone 스크립트와 맞춘 bring-up 호환 surface다.
        # bool true 입력은 latch가 아니라 짧은 push-button pulse처럼 처리한다.
        self.create_subscription(
            Bool,
            "/plc_bit_control",
            self.bit_control_callback,
            10,
        )
        self._m_bit_subscriptions = [
            self.create_subscription(
                Bool,
                f"/plc_m{ModbusPLCConfig.m_topic_suffix(label)}",
                lambda msg, addr=address, coil_label=label, output_label=output: (
                    self.m_bit_callback(
                        msg,
                        addr,
                        ModbusPLCConfig.coil_display_label(
                            coil_label,
                            output_label,
                        ),
                    )
                ),
                10,
            )
            for label, address, output in zip(
                self._config.modbus.start_coil_labels,
                self._config.modbus.start_coil_addresses,
                self._config.modbus.start_coil_outputs,
            )
        ]
        self.create_subscription(Bool, "/plc_m_all", self.m_all_callback, 10)
        # /plc_m100과 /plc_reset은 같은 M0100 reset pulse를 가리킨다.
        self.create_subscription(Bool, "/plc_m100", self.reset_callback, 10)
        self.create_subscription(Bool, "/plc_reset", self.reset_callback, 10)
        self.create_subscription(
            String,
            "/plc/system_state",
            self.system_state_callback,
            10,
        )
        self.create_subscription(
            Int32, "/plc_word_control", self.word_control_callback, 10
        )

        self._connect_with_retry()
        self.create_timer(
            max(self._config.connect_retry_backoff_s, 1.0),
            self.reconnect_timer_callback,
        )
        self.create_timer(self._config.read_period_s, self.read_timer_callback)
        # M0005는 상태 출력용 watchdog 입력이다. heartbeat/E-stop hook은 별도
        # device가 확정되기 전까지 기본값 false다.
        # 전용 래더와 주소가 확정된 뒤 launch parameter로 켠다.
        if self._config.enable_watchdog:
            self.create_timer(
                self._config.watchdog_period_s,
                self.watchdog_timer_callback,
            )
        if self._config.enable_estop_poll:
            self.create_timer(
                self._config.estop_poll_period_s,
                self.estop_poll_timer_callback,
            )

    def _load_config(self) -> PlcNodeConfig:
        """ROS2 parameter를 선언하고 core/node 설정 객체로 고정한다."""

        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("parity", "N")
        self.declare_parameter("stopbits", 1)
        self.declare_parameter("bytesize", 8)
        self.declare_parameter("device_id", 1)
        self.declare_parameter(
            "start_coil_labels",
            ["M0000", "M0001", "M0002", "M0003", "M0004", "M0005"],
        )
        self.declare_parameter("start_coil_addresses", [0, 1, 2, 3, 4, 5])
        self.declare_parameter(
            "start_coil_outputs",
            ["P0040", "P0041", "P0042", "P0043", "P0043", "P0044"],
        )
        self.declare_parameter("reset_coil_label", "M0100")
        self.declare_parameter("reset_coil_address", 256)
        self.declare_parameter("read_register_label", "P020")
        self.declare_parameter("read_register_address", 0)
        self.declare_parameter("write_register_label", "P000")
        self.declare_parameter("write_register_address", 0)
        # 현재 래더 이미지에는 heartbeat watchdog/E-stop device가 없다.
        # 빈 label + -1 address는 safety hook 비활성 상태를 뜻한다.
        self.declare_parameter("watchdog_coil_label", "")
        self.declare_parameter("watchdog_coil_address", -1)
        self.declare_parameter("estop_input_label", "")
        self.declare_parameter("estop_input_address", -1)
        self.declare_parameter(
            "system_state_names",
            [
                "idle",
                "listening",
                "inferring",
                "moving",
                "e_stop",
                "error",
                "watchdog",
            ],
        )
        self.declare_parameter(
            "system_state_output_labels",
            ["none", "M0001", "M0001", "M0002", "M0003", "M0004", "M0005"],
        )
        self.declare_parameter("read_period_s", 1.0)
        self.declare_parameter("connect_retry_count", 3)
        self.declare_parameter("connect_retry_backoff_s", 0.5)
        self.declare_parameter("pulse_duration_s", 0.2)
        self.declare_parameter("enable_watchdog", False)
        self.declare_parameter("watchdog_period_s", 0.25)
        self.declare_parameter("enable_estop_poll", False)
        self.declare_parameter("estop_poll_period_s", 0.1)

        start_coil_labels = tuple(
            str(label) for label in self.get_parameter("start_coil_labels").value
        )
        start_coil_addresses = tuple(
            int(address) for address in self.get_parameter("start_coil_addresses").value
        )
        start_coil_outputs = tuple(
            str(output) for output in self.get_parameter("start_coil_outputs").value
        )
        system_state_names = tuple(
            str(state) for state in self.get_parameter("system_state_names").value
        )
        system_state_output_labels = tuple(
            str(labels)
            for labels in self.get_parameter("system_state_output_labels").value
        )
        watchdog_coil_label = str(self.get_parameter("watchdog_coil_label").value)
        watchdog_coil_address = int(self.get_parameter("watchdog_coil_address").value)
        estop_input_label = str(self.get_parameter("estop_input_label").value)
        estop_input_address = int(self.get_parameter("estop_input_address").value)
        # 세 배열이 어긋나면 topic 이름과 실제 출력이 달라지므로 node 시작 전에 막는다.
        if not (
            len(start_coil_labels)
            == len(start_coil_addresses)
            == len(start_coil_outputs)
        ):
            raise ValueError(
                "start_coil_labels, start_coil_addresses, "
                "start_coil_outputs must have the same length"
            )

        enable_watchdog = bool(self.get_parameter("enable_watchdog").value)
        watchdog_period_s = float(self.get_parameter("watchdog_period_s").value)
        enable_estop_poll = bool(self.get_parameter("enable_estop_poll").value)

        modbus_config = ModbusPLCConfig(
            port=str(self.get_parameter("port").value),
            baudrate=int(self.get_parameter("baudrate").value),
            parity=str(self.get_parameter("parity").value),
            stopbits=int(self.get_parameter("stopbits").value),
            bytesize=int(self.get_parameter("bytesize").value),
            device_id=int(self.get_parameter("device_id").value),
            start_coil_labels=start_coil_labels,
            start_coil_addresses=start_coil_addresses,
            start_coil_outputs=start_coil_outputs,
            reset_coil_label=str(self.get_parameter("reset_coil_label").value),
            reset_coil_address=int(self.get_parameter("reset_coil_address").value),
            read_register_label=str(self.get_parameter("read_register_label").value),
            read_register_address=int(
                self.get_parameter("read_register_address").value
            ),
            write_register_label=str(self.get_parameter("write_register_label").value),
            write_register_address=int(
                self.get_parameter("write_register_address").value
            ),
            pulse_duration_s=float(self.get_parameter("pulse_duration_s").value),
            # -1은 launch/YAML에서 "주소 미설정"을 표현하기 위한 sentinel이다.
            watchdog_coil_label=watchdog_coil_label or None,
            watchdog_coil_address=(
                watchdog_coil_address if watchdog_coil_address >= 0 else None
            ),
            estop_input_label=estop_input_label or None,
            estop_input_address=(
                estop_input_address if estop_input_address >= 0 else None
            ),
            system_state_outputs=ModbusPLCConfig.parse_system_state_outputs(
                system_state_names,
                system_state_output_labels,
            ),
        )
        if enable_watchdog:
            if modbus_config.watchdog_coil_address is None:
                raise ValueError("enable_watchdog requires watchdog_coil_address")
            if watchdog_period_s > 0.25:
                raise ValueError("watchdog_period_s must be <= 0.25 when enabled")
        if enable_estop_poll and modbus_config.estop_input_address is None:
            raise ValueError("enable_estop_poll requires estop_input_address")

        return PlcNodeConfig(
            modbus=modbus_config,
            read_period_s=float(self.get_parameter("read_period_s").value),
            connect_retry_count=int(self.get_parameter("connect_retry_count").value),
            connect_retry_backoff_s=float(
                self.get_parameter("connect_retry_backoff_s").value
            ),
            enable_watchdog=enable_watchdog,
            watchdog_period_s=watchdog_period_s,
            enable_estop_poll=enable_estop_poll,
            estop_poll_period_s=float(self.get_parameter("estop_poll_period_s").value),
        )

    def _connect_with_retry(self) -> bool:
        """PLC serial 연결을 재시도한다.

        현장에서는 udev/USB 준비 타이밍 때문에 첫 connect가 실패할 수 있다.
        제한된 횟수만 backoff를 두고 재시도한 뒤, 실패 상태를 `/plc/status`에
        빨간 점멸로 알린다.
        """

        for attempt in range(1, self._config.connect_retry_count + 1):
            try:
                if self._plc.connect():
                    self.get_logger().info(
                        f"XBC-DR14E connected on {self._config.modbus.port} "
                        f"baudrate={self._config.modbus.baudrate} "
                        f"device_id={self._config.modbus.device_id}"
                    )
                    self._set_and_publish_system_state(
                        SystemState.IDLE,
                        apply_outputs=False,
                    )
                    return True
            except PLCError as exc:
                self.get_logger().error(f"PLC connect attempt {attempt} failed: {exc}")

            if attempt < self._config.connect_retry_count:
                time.sleep(self._config.connect_retry_backoff_s * attempt)

        self.get_logger().error(
            f"PLC connect failed on {self._config.modbus.port}; "
            "check wiring and serial port"
        )
        self._set_and_publish_system_state(SystemState.ERROR, apply_outputs=False)
        return False

    def _ensure_connected(self) -> bool:
        """요청 처리 전 연결 상태를 확인한다.

        Timer/subscription callback에서 backoff sleep이 발생하면 watchdog/E-stop poll이
        굶을 수 있으므로 재연결은 별도 reconnect timer에서만 수행한다.
        """

        if self._plc.connected:
            return True
        self.get_logger().warning("PLC client is not connected; command skipped")
        return False

    def reconnect_timer_callback(self) -> None:
        """연결이 끊긴 경우 단일 connect 시도만 수행한다."""

        if self._plc.connected or self._estop_latched.is_set():
            return
        try:
            if self._plc.connect():
                self.get_logger().info("PLC reconnect succeeded")
                self._set_and_publish_system_state(
                    SystemState.IDLE,
                    apply_outputs=False,
                )
        except PLCError as exc:
            self.get_logger().warning(f"PLC reconnect failed: {exc}")

    def _outputs_allowed(self) -> bool:
        """E-stop latch 중에는 PLC 출력 변경 요청을 거부한다."""

        if not self._estop_latched.is_set():
            return True
        self.get_logger().error("PLC output command rejected because E-stop is latched")
        self._publish_estop(True)
        self._set_and_publish_system_state(SystemState.E_STOP, apply_outputs=False)
        return False

    def _publish_estop(self, asserted: bool) -> None:
        """Publish PLC E-stop latch state."""

        message = Bool()
        message.data = bool(asserted)
        self._estop_pub.publish(message)

    def _publish_status(self, status: CorePLCStatus) -> None:
        """프로젝트 표준 PLCStatus 메시지를 publish한다."""

        msg = PLCStatusMsg()
        msg.led_color = status.led_color.value
        msg.led_mode = status.led_mode.value
        msg.system_state = status.system_state.value
        self._status_pub.publish(msg)

    def _set_and_publish_system_state(
        self,
        state: SystemState,
        *,
        apply_outputs: bool,
        reset_before_apply: bool = True,
    ) -> None:
        """core semantic 상태를 갱신하고 ROS2 PLCStatus로 publish한다."""

        try:
            if apply_outputs and not self._apply_system_state_outputs(
                state,
                reset_before_apply=reset_before_apply,
            ):
                status = self._plc.set_error(apply_outputs=False)
                self._publish_status(status)
                return
            status = self._plc.set_system_state(
                state,
                apply_outputs=False,
            )
        except PLCError as exc:
            self.get_logger().error(f"PLC semantic state apply failed: {exc}")
            status = self._plc.set_error(apply_outputs=False)
        self._publish_status(status)

    def _apply_system_state_outputs(
        self,
        state: SystemState,
        *,
        reset_before_apply: bool,
    ) -> bool:
        """semantic 상태에 매핑된 PLC 출력을 non-blocking pulse로 적용한다."""

        output_labels = self._config.modbus.output_labels_for_state(state)
        if state == SystemState.E_STOP:
            for label in output_labels:
                if not self._write_label_coil(label, True):
                    return False
            return True

        if reset_before_apply:
            if not self._write_coil_raw(
                self._config.modbus.reset_coil_address,
                True,
                self._config.modbus.reset_coil_label,
            ):
                return False

            def apply_after_reset() -> None:
                self._write_coil_raw(
                    self._config.modbus.reset_coil_address,
                    False,
                    self._config.modbus.reset_coil_label,
                )
                for label in output_labels:
                    self._pulse_label_for_state(label)

            self._schedule_once(self._config.modbus.pulse_duration_s, apply_after_reset)
            return True

        for label in output_labels:
            self._pulse_label_for_state(label)
        return True

    def _write_label_coil(self, label: str, value: bool) -> bool:
        """설정 label에 해당하는 coil을 직접 쓴다."""

        return self._write_coil_raw(
            self._config.modbus.coil_address_for_label(label),
            value,
            label,
        )

    def _pulse_label_for_state(self, label: str) -> None:
        """semantic state용 start coil pulse를 timer release로 처리한다."""

        address = self._config.modbus.coil_address_for_label(label)
        if not self._write_coil_raw(address, True, label):
            return
        self._schedule_coil_release(address, label)

    def _schedule_once(self, delay_s: float, callback: Callable[[], None]) -> None:
        """rclpy timer로 1회성 후속 작업을 예약한다."""

        timer_ref: dict[str, object] = {}

        def run_once() -> None:
            try:
                callback()
            finally:
                timer = timer_ref["timer"]
                timer.cancel()
                if timer in self._pulse_timers:
                    self._pulse_timers.remove(timer)

        timer = self.create_timer(max(delay_s, 0.001), run_once)
        timer_ref["timer"] = timer
        self._pulse_timers.append(timer)

    def _schedule_coil_release(
        self,
        address: int,
        label: str,
        *,
        release_state: SystemState | None = None,
    ) -> None:
        """pulse ON 이후 OFF write를 timer callback으로 예약한다."""

        def release() -> None:
            self._write_coil_raw(address, False, label)
            if release_state is not None and not self._estop_latched.is_set():
                self._set_and_publish_system_state(release_state, apply_outputs=False)

        self._schedule_once(self._config.modbus.pulse_duration_s, release)

    def _write_coil_raw(self, address: int, value: bool, label: str) -> bool:
        """Latch 검사 없이 실제 coil write만 수행한다.

        예약된 pulse release와 E-stop 직접 출력처럼 latch를 우회해야 하는 경우에만
        사용한다.
        """

        if not self._ensure_connected():
            return False
        try:
            self._plc.write_coil(address, value)
        except PLCError as exc:
            self.get_logger().error(f"{label} control failed: {exc}")
            self._set_and_publish_system_state(SystemState.ERROR, apply_outputs=False)
            return False

        status = "ON" if value else "OFF"
        self.get_logger().info(f"{label} -> {status}")
        return True

    def read_timer_callback(self) -> None:
        """주기적으로 PLC word register를 읽어 `/plc_word_read`에 발행한다."""

        if not self._ensure_connected():
            return

        try:
            register_value = self._plc.read_register(
                self._config.modbus.read_register_address
            )
        except PLCError as exc:
            self.get_logger().error(f"PLC word read failed: {exc}")
            self._set_and_publish_system_state(SystemState.ERROR, apply_outputs=False)
            return

        msg = Int32()
        msg.data = register_value
        self._word_pub.publish(msg)
        self.get_logger().info(
            f"{self._config.modbus.read_register_label} read value={register_value}"
        )
        self._publish_status(self._plc.get_status())

    def watchdog_timer_callback(self) -> None:
        """PLC watchdog heartbeat coil을 주기적으로 갱신한다."""

        if not self._ensure_connected():
            return

        try:
            # heartbeat는 같은 값 반복 write가 아니라 toggle로 보낸다.
            self._plc.heartbeat()
        except PLCError as exc:
            self.get_logger().error(f"PLC watchdog heartbeat failed: {exc}")
            self._set_and_publish_system_state(SystemState.ERROR, apply_outputs=False)

    def estop_poll_timer_callback(self) -> None:
        """PLC E-stop input을 polling하고 감지 시 latch 상태로 전환한다."""

        if self._estop_latched.is_set():
            self._publish_estop(True)
            self._set_and_publish_system_state(SystemState.E_STOP, apply_outputs=False)
            return
        if not self._ensure_connected():
            return

        try:
            estop_pressed = self._plc.read_estop()
        except PLCError as exc:
            self.get_logger().error(f"PLC E-stop input read failed: {exc}")
            self._set_and_publish_system_state(SystemState.ERROR, apply_outputs=False)
            return

        if estop_pressed:
            # E-stop은 safety invariant라서 상태를 latch하고 자동으로 다시 idle로 돌리지 않는다.
            self.get_logger().error("PLC E-stop input asserted")
            self._estop_latched.set()
            self._publish_estop(True)
            self._set_and_publish_system_state(
                SystemState.E_STOP,
                apply_outputs=True,
                reset_before_apply=False,
            )

    def _write_m_coil(self, address: int, value: bool, label: str) -> bool:
        """M coil 하나를 쓰고 결과를 상태 topic에 반영한다."""

        if not self._outputs_allowed():
            return False
        if not self._ensure_connected():
            return False

        try:
            self._plc.write_coil(address, value)
        except PLCError as exc:
            self.get_logger().error(f"{label} control failed: {exc}")
            self._set_and_publish_system_state(SystemState.ERROR, apply_outputs=False)
            return False

        status = "ON" if value else "OFF"
        self.get_logger().info(f"{label} -> {status}")
        self._set_and_publish_system_state(
            SystemState.MOVING if value else SystemState.IDLE,
            apply_outputs=False,
        )
        return True

    def _pulse_m_coil(self, address: int, label: str) -> None:
        """push-button처럼 coil을 잠깐 ON 했다가 OFF로 되돌린다."""

        if not self._write_m_coil(address, True, label):
            return
        self._schedule_coil_release(address, label, release_state=SystemState.IDLE)

    def bit_control_callback(self, msg: Bool) -> None:
        """기존 `/plc_bit_control` topic을 M0000 pulse 제어로 처리한다."""

        address = self._config.modbus.first_start_coil_address
        label = ModbusPLCConfig.coil_display_label(
            self._config.modbus.first_start_coil_label,
            self._config.modbus.first_start_output_label,
        )
        if msg.data:
            # true는 "누름" 요청이다. 고정 ON이 아니라 ON->OFF pulse로 처리한다.
            self._pulse_m_coil(address, label)
            return
        self._write_m_coil(address, False, label)

    def m_bit_callback(self, msg: Bool, address: int, label: str) -> None:
        """`/plc_m0`~`/plc_mN` 입력을 해당 M coil 제어로 변환한다."""

        if msg.data:
            # 각 M topic도 실제 버튼처럼 짧게 누르는 방식이다.
            self._pulse_m_coil(address, label)
            return
        self._write_m_coil(address, False, label)

    def m_all_callback(self, msg: Bool) -> None:
        """전체 M coil을 한 번에 pulse 또는 OFF 처리한다."""

        if not msg.data:
            self._write_m_all(False)
            return

        if self._write_m_all(True):
            # 전체 테스트도 latch가 아니라 일괄 push-button pulse로 처리한다.
            self._schedule_once(
                self._config.modbus.pulse_duration_s,
                lambda: self._write_m_all(False, bypass_latch=True),
            )

    def reset_callback(self, msg: Bool) -> None:
        """M0100 리셋 접점을 push-button처럼 잠깐 ON 처리한다.

        현재 래더에서 M0100은 reset 접점으로, ON인 동안 내부 자기유지 회로를
        끊어 P0040~P0044 출력을 reset한다.
        `false` 입력은 명시적 release/OFF로 처리해 테스트 중 stuck 상태를 풀 수
        있게 한다.
        """

        label = self._config.modbus.reset_coil_label
        if msg.data:
            self._pulse_m_coil(self._config.modbus.reset_coil_address, label)
            return
        self._write_m_coil(self._config.modbus.reset_coil_address, False, label)

    def system_state_callback(self, msg: String) -> None:
        """`/plc/system_state`를 semantic 상태 기반 PLC 출력으로 적용한다."""

        try:
            state = SystemState(msg.data.strip())
        except ValueError:
            self.get_logger().error(f"Unsupported PLC system_state: {msg.data}")
            self._set_and_publish_system_state(SystemState.ERROR, apply_outputs=False)
            return

        if state == SystemState.E_STOP:
            self._estop_latched.set()
            self._publish_estop(True)
            # 외부 safety/orchestrator가 e_stop을 명령한 경우에는 연결되어 있으면
            # 현재 e_stop 출력 패턴(M0003)을 실제 PLC에도 적용한다.
            self._set_and_publish_system_state(
                SystemState.E_STOP,
                apply_outputs=self._plc.connected,
                reset_before_apply=False,
            )
            return
        if not self._outputs_allowed():
            return
        if not self._ensure_connected():
            return

        self._set_and_publish_system_state(state, apply_outputs=True)

    def _write_m_all(self, value: bool, *, bypass_latch: bool = False) -> bool:
        """설정된 M coil 범위 전체를 batch write한다."""

        if not bypass_latch and not self._outputs_allowed():
            return False
        if not self._ensure_connected():
            return False

        try:
            self._plc.write_start_coils(value)
        except PLCError as exc:
            self.get_logger().error(f"M coil batch control failed: {exc}")
            self._set_and_publish_system_state(SystemState.ERROR, apply_outputs=False)
            return False

        status = "ON" if value else "OFF"
        self.get_logger().info(
            f"{self._config.modbus.start_coil_labels[0]}~"
            f"{self._config.modbus.start_coil_labels[-1]} -> {status}"
        )
        self._set_and_publish_system_state(
            SystemState.MOVING if value else SystemState.IDLE,
            apply_outputs=False,
        )
        return True

    def word_control_callback(self, msg: Int32) -> None:
        """`/plc_word_control` 값을 PLC word register에 쓴다."""

        if not self._outputs_allowed():
            return
        if not self._ensure_connected():
            return

        target_value = int(msg.data)
        try:
            self._plc.write_register(
                self._config.modbus.write_register_address,
                target_value,
            )
        except PLCError as exc:
            self.get_logger().error(f"PLC word write failed: {exc}")
            self._set_and_publish_system_state(SystemState.ERROR, apply_outputs=False)
            return

        self.get_logger().info(
            f"{self._config.modbus.write_register_label} write value={target_value}"
        )
        self._set_and_publish_system_state(SystemState.MOVING, apply_outputs=False)

    def close(self) -> None:
        """노드 종료 시 serial client를 닫는다."""

        self._plc.close()


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = XgbRos2ModbusNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
