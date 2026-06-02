"""DB Gate와 공구 상태 갱신을 ROS2 service로 노출하는 노드."""

from __future__ import annotations

import rclpy
from db_core.repository import ToolRepository
from interfaces.srv import CheckToolFeasibility, UpdateToolStatus
from rclpy.node import Node


class DbServiceNode(Node):
    """Thin ROS2 wrapper around ToolRepository."""

    def __init__(self) -> None:
        super().__init__("db_service_node")
        self.declare_parameter("db_path", "robot_arm.db")
        self.declare_parameter("operator_id", "operator_01")
        # 동시 쓰기(fod_monitor_node)와의 WAL 락 경합 대기 시간 (config/runtime.yaml).
        self.declare_parameter("busy_timeout_ms", 5000)
        # DB 접근 로직은 repository에 모아 ROS2와 순수 DB 코드를 분리한다.
        self._repository = ToolRepository(
            self.get_parameter("db_path").get_parameter_value().string_value,
            self.get_parameter("operator_id").get_parameter_value().string_value,
            busy_timeout_ms=self.get_parameter("busy_timeout_ms")
            .get_parameter_value()
            .integer_value,
        )
        self.create_service(
            CheckToolFeasibility,
            "/db/CheckToolFeasibility",
            self._handle_check_tool_feasibility,
        )
        self.create_service(
            UpdateToolStatus,
            "/db/UpdateToolStatus",
            self._handle_update_tool_status,
        )

    def _handle_check_tool_feasibility(
        self,
        request: CheckToolFeasibility.Request,
        response: CheckToolFeasibility.Response,
    ) -> CheckToolFeasibility.Response:
        """DB Gate: fetch/return 명령이 현재 DB 상태에서 가능한지 확인한다."""

        result = self._repository.check_feasibility(request.intent, request.tool_id)
        response.feasible = result.feasible
        response.reason = result.reason
        if not result.feasible:
            self.get_logger().warning(
                f"DB gate rejected intent={request.intent} "
                f"tool_id={request.tool_id}: {result.reason}"
            )
        return response

    def _handle_update_tool_status(
        self,
        request: UpdateToolStatus.Request,
        response: UpdateToolStatus.Response,
    ) -> UpdateToolStatus.Response:
        """Motion 완료 후 호출되어 tools snapshot과 tool_events를 함께 갱신한다."""

        result = self._repository.update_tool_status(
            tool_id=request.tool_id,
            new_status=request.new_status,
            event_type=request.event_type,
            track=request.track,
            notes=request.notes,
        )
        response.success = result.success
        response.message = result.message
        if not result.success:
            self.get_logger().error(
                f"status update failed tool_id={request.tool_id}: {result.message}"
            )
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DbServiceNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
