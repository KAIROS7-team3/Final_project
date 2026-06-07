"""orchestrator_node — Behavior Tree를 실행하는 메인 ROS2 노드.

/voice/intent, /vision/tracked_poses, /vision/scene_context를 구독한다.
Track A/B 전용.
"""
from __future__ import annotations

import threading

import py_trees
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from interfaces.action import PlaceAtStaging, ReturnToSlot
from interfaces.msg import Intent, RobotStatus
from interfaces.srv import CheckToolFeasibility, LogEvent, UpdateToolStatus

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

_MAX_PENDING_S7_LOGS = 16
_STATUS_AFTER_FETCH = "staged"
_STATUS_AFTER_RETURN = "in_slot"


class OrchestratorNode(Node):
    """BT 루트 트리를 보유하고 intent 수신 시 tick한다."""

    def __init__(self) -> None:
        super().__init__("orchestrator_node")

        # 콜백 그룹
        self._normal_cbg = ReentrantCallbackGroup()
        self._log_cbg = MutuallyExclusiveCallbackGroup()

        # Blackboard 단일 작성자 클라이언트 (E-9)
        self._bb = py_trees.blackboard.Client(name="OrchestratorNode")
        self._bb.register_key(key=KEY_INTENT, access=py_trees.common.Access.WRITE)
        self._bb.register_key(key=KEY_ACTIVE_TOOL_ID, access=py_trees.common.Access.WRITE)
        self._bb.register_key(key=KEY_TOOL_POSE, access=py_trees.common.Access.WRITE)

        # ── 서비스 클라이언트 ──────────────────────────────────────────────
        self._feasibility_cli = self.create_client(
            CheckToolFeasibility,
            "/db/CheckToolFeasibility",
            callback_group=self._normal_cbg,
        )
        self._update_status_cli = self.create_client(
            UpdateToolStatus,
            "/db/UpdateToolStatus",
            callback_group=self._normal_cbg,
        )
        self._log_event_cli = self.create_client(
            LogEvent,
            "/db/LogEvent",
            callback_group=self._log_cbg,
        )

        # ── 액션 클라이언트 ───────────────────────────────────────────────
        self._place_cli = ActionClient(
            self,
            PlaceAtStaging,
            "place_at_staging",
            callback_group=self._normal_cbg,
        )
        self._return_cli = ActionClient(
            self,
            ReturnToSlot,
            "return_to_slot",
            callback_group=self._normal_cbg,
        )

        # ── BT 서브트리 조립 (클라이언트 주입) ───────────────────────────
        self._fetch_tree = build_fetch_subtree(
            feasibility_client=self._feasibility_cli,
            place_at_staging_client=self._place_cli,
        )
        self._return_tree = build_return_subtree(
            feasibility_client=self._feasibility_cli,
            return_to_slot_client=self._return_cli,
        )
        self._fetch_tree.setup(timeout=5)
        self._return_tree.setup(timeout=5)

        # ── 상태 ─────────────────────────────────────────────────────────
        self._active_tool_id: str = ""
        self._latest_scene_context: str | None = None
        self._is_moving: bool = False
        self._bt_lock = threading.Lock()
        self._pending_s7_logs = 0
        self._s7_log_lock = threading.Lock()  # MTE에서 _pending_s7_logs 보호

        # ── /plc/system_state 발행자 ──────────────────────────────────────
        self._plc_pub = self.create_publisher(String, "/plc/system_state", 1)

        # ── 구독 ─────────────────────────────────────────────────────────
        self._robot_status_sub = self.create_subscription(
            RobotStatus, "/robot/status", self._on_robot_status, _QOS_ROBOT_STATUS,
            callback_group=self._normal_cbg,
        )
        self._intent_sub = self.create_subscription(
            Intent, "/voice/intent", self._on_intent, _QOS_INTENT,
            callback_group=self._normal_cbg,
        )
        self._tracked_poses_sub = self.create_subscription(
            Detection3DArray,
            "/vision/tracked_poses",
            self._on_tracked_poses,
            _QOS_TRACKED_POSES,
            callback_group=self._normal_cbg,
        )
        self._scene_context_sub = self.create_subscription(
            String, "/vision/scene_context", self._on_scene_context, _QOS_SCENE_CONTEXT,
            callback_group=self._normal_cbg,
        )

        self.get_logger().info(
            "[OrchestratorNode] ready — listening on /voice/intent"
        )

    def _on_robot_status(self, msg: RobotStatus) -> None:
        self._is_moving = msg.is_moving

    def _on_intent(self, msg: Intent) -> None:
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
        self._bb.tool_pose = None

        threading.Thread(
            target=self._tick_bt,
            args=(msg.intent_type, msg.tool_id),
            daemon=True,
        ).start()

    def _tick_bt(self, intent_type: str, tool_id: str) -> None:
        """BT를 실행 전용 스레드에서 tick한다. blocking OK."""
        if not self._bt_lock.acquire(blocking=False):
            self.get_logger().warn(
                "[OrchestratorNode] BT 이미 실행 중 — intent 무시: type=%s", intent_type
            )
            return
        try:
            if intent_type == "fetch":
                tree = self._fetch_tree
            elif intent_type == "return":
                tree = self._return_tree
            else:
                self.get_logger().warn(
                    "[OrchestratorNode] 알 수 없는 intent: %s", intent_type
                )
                return

            self.get_logger().info(
                "[OrchestratorNode] BT tick 시작: intent=%s tool_id=%s",
                intent_type, tool_id,
            )
            self._set_plc("moving")
            status = tree.tick_once()

            if status == py_trees.common.Status.SUCCESS:
                self.get_logger().info(
                    "[OrchestratorNode] BT 성공: intent=%s tool_id=%s",
                    intent_type, tool_id,
                )
                self._set_plc("idle")
                self._update_tool_status_after(intent_type, tool_id)
            else:
                self.get_logger().error(
                    "[OrchestratorNode] BT 실패: intent=%s tool_id=%s status=%s",
                    intent_type, tool_id, status,
                )
                self._set_plc("error")
        except Exception as exc:
            self.get_logger().error("[OrchestratorNode] BT tick 예외: %s", exc)
            self._set_plc("error")
        finally:
            self._bt_lock.release()

    def _update_tool_status_after(self, intent_type: str, tool_id: str) -> None:
        if not self._update_status_cli.service_is_ready():
            self.get_logger().warn("[OrchestratorNode] UpdateToolStatus 서비스 미준비")
            return
        req = UpdateToolStatus.Request()
        req.tool_id = tool_id
        req.event_type = intent_type
        req.track = "A"
        if intent_type == "fetch":
            req.new_status = _STATUS_AFTER_FETCH
            req.notes = "fetch 완료 — BT 성공"
        else:
            req.new_status = _STATUS_AFTER_RETURN
            req.notes = "return 완료 — BT 성공"
        self._update_status_cli.call_async(req)

    def _set_plc(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._plc_pub.publish(msg)

    def _on_tracked_poses(self, msg: Detection3DArray) -> None:
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
                    pose.orientation.x, pose.orientation.y,
                    pose.orientation.z, pose.orientation.w,
                ],
                "frame": msg.header.frame_id or "robot_base_link",
            }
            break

    def _on_scene_context(self, msg: String) -> None:
        self._latest_scene_context = msg.data

    def _log_s7_rejection(self, msg: Intent) -> None:
        try:
            if not self._log_event_cli.service_is_ready():
                return
            with self._s7_log_lock:
                if self._pending_s7_logs >= _MAX_PENDING_S7_LOGS:
                    return
                self._pending_s7_logs += 1
            req = LogEvent.Request()
            req.tool_id = msg.tool_id
            req.event_type = "rejected"
            req.track = ""
            req.notes = f"S-7: robot moving; intent={msg.intent_type}"
            future = self._log_event_cli.call_async(req)
            future.add_done_callback(self._on_log_event_done)
        except Exception as exc:
            self.get_logger().warning(
                "[OrchestratorNode] S-7 rejection log dispatch failed: %s", exc
            )

    def _on_log_event_done(self, future) -> None:
        with self._s7_log_lock:
            self._pending_s7_logs = max(0, self._pending_s7_logs - 1)
        try:
            result = future.result()
        except Exception as exc:
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
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
