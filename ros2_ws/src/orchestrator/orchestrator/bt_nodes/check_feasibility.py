"""CheckFeasibility BT 노드 — /db/CheckToolFeasibility 서비스 호출."""
import py_trees
import py_trees_ros
import rclpy

from interfaces.srv import CheckToolFeasibility

from orchestrator.blackboard import (
    KEY_ACTIVE_TOOL_ID,
    KEY_FEASIBILITY_REASON,
    KEY_INTENT,
)


class CheckFeasibility(py_trees_ros.service_clients.FromBlackboard):
    """Blackboard의 intent + active_tool_id로 DB Gate를 통과하는지 확인한다.

    SUCCESS: feasible=True
    FAILURE: feasible=False (feasibility_reason에 사유 기록)
    """

    def __init__(self, name: str = "CheckFeasibility"):
        super().__init__(
            name=name,
            service_type=CheckToolFeasibility,
            service_name="/db/CheckToolFeasibility",
            key_request=KEY_ACTIVE_TOOL_ID,
            key_response=KEY_FEASIBILITY_REASON,
        )
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(
            key=KEY_INTENT, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=KEY_ACTIVE_TOOL_ID, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=KEY_FEASIBILITY_REASON, access=py_trees.common.Access.WRITE
        )

    def initialise(self) -> None:
        # TODO(Phase 5a): 서비스 요청 객체 구성 및 전송
        raise NotImplementedError("Phase 5a에서 구현")

    def update(self) -> py_trees.common.Status:
        # TODO(Phase 5a): 서비스 응답 확인 후 SUCCESS/FAILURE 반환
        raise NotImplementedError("Phase 5a에서 구현")
