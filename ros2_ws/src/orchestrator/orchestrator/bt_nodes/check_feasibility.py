"""CheckFeasibility BT 노드 — /db/CheckToolFeasibility 서비스로 DB Gate를 확인한다 (S-2)."""
import py_trees

from orchestrator.blackboard import (
    KEY_ACTIVE_TOOL_ID,
    KEY_FEASIBILITY_REASON,
    KEY_INTENT,
)


class CheckFeasibility(py_trees.behaviour.Behaviour):
    """Blackboard의 intent + active_tool_id로 /db/CheckToolFeasibility를 호출한다.

    SUCCESS: 응답 feasible=True
    FAILURE: 응답 feasible=False — feasibility_reason에 사유 기록
    RUNNING: 서비스 응답 대기 중

    Phase 5a에서 rclpy 서비스 클라이언트를 노드로부터 주입받아 구현한다.
    request: {intent, tool_id} (Blackboard 두 키에서 조립)
    response: {feasible, reason}
    """

    # Phase 5a 주의: fetch/return 서브트리에 각각 CheckFeasibility를 추가할 때
    # 반드시 고유한 name을 지정할 것 (예: "CheckFeasibility_fetch", "CheckFeasibility_return").
    # 같은 name으로 두 인스턴스를 만들면 Blackboard 클라이언트 이름이 충돌하고
    # KEY_FEASIBILITY_REASON WRITE 등록이 중복되어 단일-작성자 보장이 깨진다 (E-9).
    def __init__(self, name: str = "CheckFeasibility"):
        super().__init__(name=name)
        self.blackboard = self.attach_blackboard_client(name=name)
        self.blackboard.register_key(key=KEY_INTENT, access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            key=KEY_ACTIVE_TOOL_ID, access=py_trees.common.Access.READ
        )
        self.blackboard.register_key(
            key=KEY_FEASIBILITY_REASON, access=py_trees.common.Access.WRITE
        )

    def update(self) -> py_trees.common.Status:
        # TODO(Phase 5a): 서비스 요청 전송 → 응답의 feasible로 SUCCESS/FAILURE 결정
        return py_trees.common.Status.FAILURE
