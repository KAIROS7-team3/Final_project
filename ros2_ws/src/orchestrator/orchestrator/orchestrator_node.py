"""orchestrator_node — Behavior Tree를 실행하는 메인 ROS2 노드.

/voice/intent, /vision/tracked_poses, /vision/scene_context를 구독한다.
Track A/B 전용.
"""
import py_trees
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from interfaces.msg import Intent, RobotStatus
from interfaces.srv import LogEvent

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

# S-7 거부 감사 로그의 동시 in-flight 요청 상한. is_moving 동안 intent가 폭주해도
# 미해결 future가 무한히 쌓여 executor를 굶기지 않도록 막는다(안전 콜백 starvation
# 방지). 상한 도달 시 추가 거부는 기록을 생략한다 — 가드 자체는 영향 없음.
_MAX_PENDING_S7_LOGS = 16


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

        # B1-1: S-7으로 드롭한 intent를 DB 감사 로그(tool_events 'rejected')에 남긴다.
        # ⚠️ 불변식: 이 경로는 반드시 fire-and-forget이어야 한다. main()이 기본
        # SingleThreadedExecutor(rclpy.spin)로 도므로, 구독 콜백 안에서 future를
        # 동기 대기(spin_until_future_complete / future.done() busy-wait 등)하면
        # executor가 응답을 처리하지 못해 노드 전체가 데드락된다 — E-stop·home
        # 복귀 토픽까지 막힌다. 동기 대기를 추가하려면 MultiThreadedExecutor로의
        # 전환과 안전 재검토가 선행돼야 한다(safety-reviewer F2 HIGH).
        # callback group은 향후 MTE 전환 대비용으로 분리해 둔다.
        self._log_event_cli = self.create_client(
            LogEvent,
            "/db/LogEvent",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self._pending_s7_logs = 0  # in-flight 로그 요청 수 (starvation 방지 캡)

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
            self._log_s7_rejection(msg)
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

    def _log_s7_rejection(self, msg: Intent) -> None:
        """S-7으로 드롭한 intent를 best-effort로 DB 감사 로그에 남긴다 (B1-1).

        안전 가드는 이미 적용됐고, 여기서는 관측만 한다. DB 서비스가 떠 있지
        않거나 호출이 실패해도 절대 블록하거나 예외를 올리지 않는다 — 로깅
        실패가 S-7 차단 자체를 무력화해선 안 된다. 따라서 본문 전체를 방어적으로
        감싸 어떤 예외도 호출자(_on_intent의 return 경로)로 전파되지 않게 한다.
        """
        try:
            if not self._log_event_cli.service_is_ready():
                self.get_logger().debug(
                    "[OrchestratorNode] LogEvent service unavailable — "
                    "S-7 rejection not persisted (tool_id=%s)",
                    msg.tool_id,
                )
                return
            if self._pending_s7_logs >= _MAX_PENDING_S7_LOGS:
                # 미해결 요청이 상한에 도달 — executor starvation 방지를 위해 생략.
                self.get_logger().warning(
                    "[OrchestratorNode] S-7 rejection log skipped — "
                    "%d in-flight (tool_id=%s)",
                    self._pending_s7_logs,
                    msg.tool_id,
                )
                return
            req = LogEvent.Request()
            req.tool_id = msg.tool_id
            req.event_type = "rejected"
            req.track = ""
            req.notes = f"S-7: robot moving; intent={msg.intent_type}"
            future = self._log_event_cli.call_async(req)
            self._pending_s7_logs += 1
            future.add_done_callback(self._on_log_event_done)
        except Exception as exc:  # noqa: BLE001 - 로깅이 가드를 막아선 안 됨
            self.get_logger().warning(
                "[OrchestratorNode] S-7 rejection log dispatch failed: %s", exc
            )

    def _on_log_event_done(self, future) -> None:
        """call_async 완료 콜백 — 실패는 경고 로그로만 남긴다 (비차단)."""
        self._pending_s7_logs = max(0, self._pending_s7_logs - 1)
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001 - 로깅 실패가 노드를 죽이면 안 됨
            self.get_logger().warning(
                "[OrchestratorNode] S-7 rejection log call failed: %s", exc
            )
            return
        if not result.success:
            self.get_logger().warning(
                "[OrchestratorNode] S-7 rejection log rejected: %s", result.message
            )


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
