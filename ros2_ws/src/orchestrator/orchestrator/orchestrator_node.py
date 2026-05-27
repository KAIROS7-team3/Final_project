"""orchestrator_node — Behavior Tree를 실행하는 메인 ROS2 노드.

/voice/intent 토픽을 구독해 BT를 tick한다.
Track A/B 전용.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from interfaces.msg import Intent

from orchestrator.bt_nodes.fetch_tool import build_fetch_subtree
from orchestrator.bt_nodes.return_tool import build_return_subtree


class OrchestratorNode(Node):
    """BT 루트 트리를 보유하고 intent 수신 시 tick한다."""

    def __init__(self) -> None:
        super().__init__("orchestrator_node")

        # TODO(Phase 5a): py_trees_ros.trees.BehaviourTree로 교체
        self._fetch_tree = build_fetch_subtree()
        self._return_tree = build_return_subtree()

        # interfaces.md 명세: /voice/intent = Reliable / depth 1 (S-7: 최신 명령만 수신)
        _qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        self._intent_sub = self.create_subscription(
            Intent,
            "/voice/intent",
            self._on_intent,
            _qos,
        )
        self.get_logger().info("[OrchestratorNode] ready — listening on /voice/intent")

    def _on_intent(self, msg: Intent) -> None:
        """S-7: is_moving 중 신규 명령 무시."""
        # TODO(Phase 5a): Blackboard에서 is_moving 읽기
        self.get_logger().info(
            "[OrchestratorNode] intent received - type=%s tool_id=%s",
            msg.intent_type,
            msg.tool_id,
        )
        # TODO(Phase 5a): Blackboard에 intent/active_tool_id 쓰기 후 BT tick


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = OrchestratorNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
