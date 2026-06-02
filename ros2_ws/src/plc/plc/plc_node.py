"""LS Electric XBC-DR14E용 ROS2 Modbus RTU bridge.

이 노드는 프로젝트 `plc_core`의 Modbus client를 ROS2 topic surface로 노출한다.
bring-up용 raw M/P topic을 유지하면서 `/plc/status`도 발행해 운영자 UI/상위
노드가 PLC 상태를 볼 수 있게 한다.
"""

from __future__ import annotations

import time
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
from rclpy.qos import QoSProfile, ReliabilityPolicy
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
        self._estop_latched = False

        status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        # PLC 상태는 최신 값 하나만 중요하므로 depth=1 BEST_EFFORT로 충분하다.
        self._status_pub = self.create_publisher(
            PLCStatusMsg,
            "/plc/status",
            status_qos,
        )
        self._word_pub = self.create_publisher(Int32, "/plc_word_read", 10)
        # 상위 safety/orchestrator가 PLC 물리 E-stop 감지를 받을 수 있는 최소 topic이다.
        self._estop_pub = self.create_publisher(Bool, "/plc/e_stop", 1)
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
        # /plc_m10과 /plc_reset은 같은 M0010 reset pulse를 가리킨다.
        self.create_subscription(Bool, "/plc_m10", self.reset_callback, 10)
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
        self.create_timer(self._config.read_period_s, self.read_timer_callback)
        # 현재 래더에는 watchdog/E-stop device가 없으므로 기본값은 false다.
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
            "start_coil_labels", ["M0000", "M0001", "M0002", "M0003"]
        )
        self.declare_parameter("start_coil_addresses", [0, 1, 2, 3])
        self.declare_parameter(
            "start_coil_outputs", ["P0040", "P0041", "P0042", "P0043"]
        )
        self.declare_parameter("reset_coil_label", "M0010")
        self.declare_parameter("reset_coil_address", 16)
        self.declare_parameter("read_register_label", "P020")
        self.declare_parameter("read_register_address", 0)
        self.declare_parameter("write_register_label", "P000")
        self.declare_parameter("write_register_address", 0)
        # 현재 래더 이미지에는 watchdog/E-stop device가 없다.
        # 빈 label + -1 address는 safety hook 비활성 상태를 뜻한다.
        self.declare_parameter("watchdog_coil_label", "")
        self.declare_parameter("watchdog_coil_address", -1)
        self.declare_parameter("estop_input_label", "")
        self.declare_parameter("estop_input_address", -1)
        self.declare_parameter(
            "system_state_names",
            ["idle", "listening", "inferring", "moving", "error", "e_stop"],
        )
        self.declare_parameter(
            "system_state_output_labels",
            ["M0000", "M0001", "M0001", "M0002", "M0003", "M0003"],
        )
        self.declare_parameter("read_period_s", 1.0)
        self.declare_parameter("connect_retry_count", 3)
        self.declare_parameter("connect_retry_backoff_s", 0.5)
        self.declare_parameter("pulse_duration_s", 0.2)
        self.declare_parameter("enable_watchdog", False)
        self.declare_parameter("watchdog_period_s", 0.5)
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

        return PlcNodeConfig(
            modbus=modbus_config,
            read_period_s=float(self.get_parameter("read_period_s").value),
            connect_retry_count=int(self.get_parameter("connect_retry_count").value),
            connect_retry_backoff_s=float(
                self.get_parameter("connect_retry_backoff_s").value
            ),
            enable_watchdog=bool(self.get_parameter("enable_watchdog").value),
            watchdog_period_s=float(self.get_parameter("watchdog_period_s").value),
            enable_estop_poll=bool(self.get_parameter("enable_estop_poll").value),
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
        """요청 처리 전 연결 상태를 확인하고 필요하면 재연결한다."""

        if self._plc.connected:
            return True
        # serial 장치가 늦게 뜨는 경우가 있어 매 요청에서 제한된 재연결을 허용한다.
        self.get_logger().warning("PLC client is not connected; retrying connection")
        return self._connect_with_retry()

    def _outputs_allowed(self) -> bool:
        """E-stop latch 중에는 PLC 출력 변경 요청을 거부한다."""

        if not self._estop_latched:
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
    ) -> None:
        """core semantic 상태를 갱신하고 ROS2 PLCStatus로 publish한다."""

        try:
            status = self._plc.set_system_state(
                state,
                apply_outputs=apply_outputs,
            )
        except PLCError as exc:
            self.get_logger().error(f"PLC semantic state apply failed: {exc}")
            status = self._plc.set_error(apply_outputs=False)
        self._publish_status(status)

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
            # 현재 구현은 toggle이 아니라 true 반복 write다. 전용 래더가 이를 감시해야 한다.
            self._plc.heartbeat()
        except PLCError as exc:
            self.get_logger().error(f"PLC watchdog heartbeat failed: {exc}")
            self._set_and_publish_system_state(SystemState.ERROR, apply_outputs=False)

    def estop_poll_timer_callback(self) -> None:
        """PLC E-stop input을 polling하고 감지 시 latch 상태로 전환한다."""

        if self._estop_latched:
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
            self._estop_latched = True
            self._publish_estop(True)
            self._set_and_publish_system_state(SystemState.E_STOP, apply_outputs=False)

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
        time.sleep(self._config.modbus.pulse_duration_s)
        self._write_m_coil(address, False, label)

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
            time.sleep(self._config.modbus.pulse_duration_s)
            self._write_m_all(False)

    def reset_callback(self, msg: Bool) -> None:
        """M0010 리셋 접점을 push-button처럼 잠깐 ON 처리한다.

        이미지 (4)의 래더에서 M0010은 맨 위 공통 NC 접점으로, ON인 동안
        M0100~M0103 자기유지 회로를 끊어 P0040~P0043 출력을 reset한다.
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
            self._estop_latched = True
            self._publish_estop(True)
            # 외부 safety/orchestrator가 e_stop을 명령한 경우에는 연결되어 있으면
            # 현재 error/e_stop 출력 패턴(M0003)을 실제 PLC에도 적용한다.
            self._set_and_publish_system_state(
                SystemState.E_STOP,
                apply_outputs=self._plc.connected,
            )
            return
        if not self._outputs_allowed():
            return
        if not self._ensure_connected():
            return

        self._set_and_publish_system_state(state, apply_outputs=True)

    def _write_m_all(self, value: bool) -> bool:
        """설정된 M coil 범위 전체를 batch write한다."""

        if not self._outputs_allowed():
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
