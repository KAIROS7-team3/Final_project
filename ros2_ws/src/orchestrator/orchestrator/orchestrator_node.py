"""orchestrator_node — Behavior Tree를 실행하는 메인 ROS2 노드.

/voice/intent, /vision/tracked_poses, /vision/scene_context를 구독한다.
Track A/B 전용.
"""
import py_trees
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from interfaces.msg import Intent, RobotStatus

from orchestrator.blackboard import (
    KEY_ACTIVE_TOOL_ID,
    KEY_INTENT,
    KEY_TOOL_POSE,
)
from orchestrator.bt_nodes.fetch_tool import build_fetch_subtree
from orchestrator.bt_nodes.return_tool import build_return_subtree

# interfaces.md §4 QoS
_QOS_INTENT = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
_QOS_TRACKED_POSES = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
_QOS_SCENE_CONTEXT = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
_QOS_ROBOT_STATUS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)


class OrchestratorNode(Node):
    """BT 루트 트리를 보유하고 intent 수신 시 tick한다."""

    def __init__(self) -> None:
        super().__init__("orchestrator_node")

        # Blackboard 단일 작성자 클라이언트 (E-9)
        self._bb = py_trees.blackboard.Client(name="OrchestratorNode")
        self._bb.register_key(key=KEY_INTENT, access=py_trees.common.Access.WRITE)
        self._bb.register_key(key=KEY_ACTIVE_TOOL_ID, access=py_trees.common.Access.WRITE)
        self._bb.register_key(key=KEY_TOOL_POSE, access=py_trees.common.Access.WRITE)

        # TODO(Phase 5a): py_trees_ros.trees.BehaviourTree로 교체
        self._fetch_tree = build_fetch_subtree()
        self._return_tree = build_return_subtree()

        # 노드 로컬 캐시 — blackboard READ 클라이언트 없이 tracked_poses 콜백에서 사용
        self._active_tool_id: str = ""
        self._latest_scene_context: str | None = None
        self._is_moving: bool = False  # S-7: /robot/status에서 갱신

        self._robot_status_sub = self.create_subscription(
            RobotStatus, "/robot/status", self._on_robot_status, _QOS_ROBOT_STATUS
        )
        self._intent_sub = self.create_subscription(
            Intent, "/voice/intent", self._on_intent, _QOS_INTENT
        )
        self._tracked_poses_sub = self.create_subscription(
            Detection3DArray,
            "/vision/tracked_poses",
            self._on_tracked_poses,
            _QOS_TRACKED_POSES,
        )
        self._scene_context_sub = self.create_subscription(
            String, "/vision/scene_context", self._on_scene_context, _QOS_SCENE_CONTEXT
        )

        self.get_logger().info(
            "[OrchestratorNode] ready — listening on /voice/intent, "
            "/vision/tracked_poses, /vision/scene_context"
        )

    def _on_robot_status(self, msg: RobotStatus) -> None:
        """S-7: is_moving 플래그 갱신 — True 동안 신규 명령 차단."""
        self._is_moving = msg.is_moving

    def _on_intent(self, msg: Intent) -> None:
        """S-7: is_moving=True 이면 즉시 반환 (이동 중 신규 명령 차단)."""
        if self._is_moving:
            self.get_logger().warn(
                "[OrchestratorNode] intent ignored — robot is moving (S-7): "
                "type=%s tool_id=%s",
                msg.intent_type,
                msg.tool_id,
            )
            return
        self.get_logger().info(
            "[OrchestratorNode] intent received - type=%s tool_id=%s",
            msg.intent_type,
            msg.tool_id,
        )
        self._active_tool_id = msg.tool_id
        self._bb.intent = msg.intent_type
        self._bb.active_tool_id = msg.tool_id
        # tool_id 변경 시 이전 포즈 무효화
        self._bb.tool_pose = None
        # TODO(Phase 5a): Blackboard 갱신 후 BT tick

    def _on_tracked_poses(self, msg: Detection3DArray) -> None:
        """확정 트랙 중 active_tool_id 매칭 포즈를 Blackboard에 기록."""
        if not self._active_tool_id:
            return

        for det in msg.detections:
            if not det.results:
                continue
            if det.results[0].hypothesis.class_id != self._active_tool_id:
                continue

            pose = det.results[0].pose.pose
            self._bb.tool_pose = {
                "position": [pose.position.x, pose.position.y, pose.position.z],
                "quaternion": [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                "frame": msg.header.frame_id or "robot_base_link",
            }
            self.get_logger().debug(
                "[OrchestratorNode] tool_pose updated - tool_id=%s frame=%s",
                self._active_tool_id,
                msg.header.frame_id,
            )
            break

    def _on_scene_context(self, msg: String) -> None:
        """최신 scene context JSON 캐시 (Phase 5a: Gemma 4 프롬프트 주입 시 사용)."""
        self._latest_scene_context = msg.data


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
