"""FOD timeout을 주기적으로 검사해 DB 상태 전이를 적용하는 노드."""

from __future__ import annotations

from datetime import timedelta

import rclpy
from db_core.repository import ToolRepository
from rclpy.node import Node


class FodMonitorNode(Node):
    """Polls DB and marks overdue out/staged tools as missing/fod_alert."""

    def __init__(self) -> None:
        super().__init__("fod_monitor_node")
        self.declare_parameter("db_path", "robot_arm.db")
        self.declare_parameter("operator_id", "operator_01")
        self.declare_parameter("checkout_timeout_minutes", 10.0)
        self.declare_parameter("missing_to_alert_seconds", 30.0)
        self.declare_parameter("poll_interval_seconds", 5.0)

        # FOD 정책 시간은 parameter로 열어 두어 현장 테스트 중 조정할 수 있게 한다.
        self._repository = ToolRepository(
            self.get_parameter("db_path").get_parameter_value().string_value,
            self.get_parameter("operator_id").get_parameter_value().string_value,
        )
        poll_interval = (
            self.get_parameter("poll_interval_seconds").get_parameter_value().double_value
        )
        self.create_timer(poll_interval, self._poll)

    def _poll(self) -> None:
        """Apply S-8 timeout transitions and log every changed tool."""

        try:
            updates = self._repository.mark_checkout_timeouts(
                checkout_timeout=timedelta(
                    minutes=self.get_parameter("checkout_timeout_minutes")
                    .get_parameter_value()
                    .double_value
                ),
                alert_grace=timedelta(
                    seconds=self.get_parameter("missing_to_alert_seconds")
                    .get_parameter_value()
                    .double_value
                ),
            )
        except Exception as exc:
            self.get_logger().error(f"FOD monitor poll failed: {exc}")
            return
        for update in updates:
            self.get_logger().warning(
                f"FOD transition tool_id={update.tool_id}: "
                f"{update.previous_status} -> {update.new_status}"
            )


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FodMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
