"""DB Gate와 공구 상태 갱신을 ROS2 service로 노출하는 노드."""

from __future__ import annotations

import rclpy
from db_core.repository import ToolRepository
from interfaces.srv import CheckToolFeasibility, UpdateToolStatus
from rclpy.node import Node


class DbServiceNode(Node):
    """`ToolRepository`를 ROS2 service로 노출하는 얇은 래퍼.

    DB 규칙은 `db_core`에 두고, 이 노드는 request/response 변환과 ROS2 logging만
    담당한다. 이렇게 해야 DB Gate를 ROS2 밖의 단위 테스트에서도 동일하게 검증할 수 있다.
    """

    def __init__(self) -> None:
        super().__init__("db_service_node")
        self.declare_parameter("db_path", "robot_arm.db")
        self.declare_parameter("operator_id", "operator_01")
        # DB 접근 로직은 repository에 모아 ROS2와 순수 DB 코드를 분리한다.
        # `operator_id`는 rejected/error event를 추적할 때 사용한다.
        self._repository = ToolRepository(
            self.get_parameter("db_path").get_parameter_value().string_value,
            self.get_parameter("operator_id").get_parameter_value().string_value,
        )
        self.create_service(
            CheckToolFeasibility,
            "/db/CheckToolFeasibility",
            self._handle_check_tool_feasibility,
        )
        # 상태 변경은 motion 완료 후 호출되어야 한다. voice node가 직접 호출하면
        # 실제 동작 전 DB 상태만 바뀌는 안전 문제가 생길 수 있다.
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

        # fetch/return 요청은 항상 이 service를 먼저 통과해야 한다.
        # feasible=False이면 downstream motion으로 보내지 않는 것이 계약이다.
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

        # Repository가 event log와 tools snapshot을 같은 transaction으로 갱신한다.
        # 노드는 service response에 성공 여부와 사람이 읽을 수 있는 실패 사유만 담는다.
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
