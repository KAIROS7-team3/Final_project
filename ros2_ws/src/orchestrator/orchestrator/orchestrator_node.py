"""orchestrator_node — Behavior Tree를 실행하는 메인 ROS2 노드.

/voice/intent, /vision/tracked_poses, /vision/scene_context, /vision/tool_gripper_pose를 구독한다.
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
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, Detection3DArray

from interfaces.action import ExecutePhase
from interfaces.msg import Intent, RobotStatus
from interfaces.srv import CheckToolFeasibility, LogEvent, UpdateToolStatus

from orchestrator.blackboard import (
    KEY_ACTIVE_TOOL_ID,
    KEY_INTENT,
    KEY_TOOL_POSE,
)
from orchestrator.bt_nodes.fetch_tool import build_fetch_subtree
from orchestrator.bt_nodes.return_tool import build_return_subtree
from orchestrator.bt_nodes.scan_layer import build_scan_subtree

# interfaces.md §4 QoS
_QOS_INTENT = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
_QOS_TRACKED_POSES = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
_QOS_SCENE_CONTEXT = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
_QOS_ROBOT_STATUS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
_QOS_GRIPPER_POSE = QoSProfile(depth=10, reliability=QoSReliabilityPolicy.BEST_EFFORT)

_MAX_PENDING_S7_LOGS = 16
_UPDATE_STATUS_TIMEOUT_S = 5.0


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
        self._exec_phase_cli = ActionClient(
            self,
            ExecutePhase,
            "execute_phase",
            callback_group=self._normal_cbg,
        )

        # ── BT 서브트리 조립 (클라이언트 주입) ───────────────────────────
        # on_pick/on_place: action feedback(phase="pick"/"place")에 맞춰
        # DB 상태를 물리적 집기/놓기 시점에 즉시 전이시킨다 (BT 완료 대기 X).
        self._fetch_tree = build_fetch_subtree(
            feasibility_client=self._feasibility_cli,
            execute_phase_client=self._exec_phase_cli,
            publish_status_fn=self._publish_status,
            set_plc_fn=self._set_plc,
            log_error_fn=self._log_error_event,
            on_pick=self._on_fetch_pick,
            on_place=self._on_fetch_place,
            layer_id=1,
            max_fetch_attempts=3,
        )
        self._return_tree = build_return_subtree(
            feasibility_client=self._feasibility_cli,
            execute_phase_client=self._exec_phase_cli,
            publish_status_fn=self._publish_status,
            set_plc_fn=self._set_plc,
            log_error_fn=self._log_error_event,
            on_pick=self._on_return_pick,
            on_place=self._on_return_place,
            layer_id=1,
            max_return_attempts=2,
        )
        self._fetch_tree.setup(timeout=5)
        self._return_tree.setup(timeout=5)

        # ── 스캔 공유 버퍼 (CollectAndSave BT 노드와 공유) ───────────────
        self._scan_pose_buf: list[dict] = []
        self._scan_pose_buf_lock = threading.Lock()

        _scan_kwargs = dict(
            execute_phase_client=self._exec_phase_cli,
            pose_buf=self._scan_pose_buf,
            buf_lock=self._scan_pose_buf_lock,
            publish_status_fn=self._publish_status,
            set_plc_fn=self._set_plc,
            log_error_fn=self._log_error_event,
        )
        self._scan_tree_l1 = build_scan_subtree(layer_id=1, **_scan_kwargs)
        self._scan_tree_l2 = build_scan_subtree(layer_id=2, **_scan_kwargs)
        self._scan_tree_l1.setup(timeout=5)
        self._scan_tree_l2.setup(timeout=5)

        # ── 상태 ─────────────────────────────────────────────────────────
        self._active_tool_id: str = ""
        self._latest_scene_context: str | None = None
        self._is_moving: bool = False
        self._bt_lock = threading.Lock()
        self._pending_s7_logs = 0
        self._s7_log_lock = threading.Lock()  # MTE에서 _pending_s7_logs 보호

        # ── /plc/system_state 발행자 ──────────────────────────────────────
        self._plc_pub = self.create_publisher(String, "/plc/system_state", 1)

        # ── /robot/status 발행자 (is_moving 발행권 orchestrator 소유) ─────
        self._status_pub = self.create_publisher(
            RobotStatus, "/robot/status", _QOS_ROBOT_STATUS
        )

        # ── 구독 ─────────────────────────────────────────────────────────
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
        self._gripper_pose_sub = self.create_subscription(
            PoseStamped, "/vision/tool_gripper_pose",
            self._on_gripper_pose, _QOS_GRIPPER_POSE,
            callback_group=self._normal_cbg,
        )
        self._gripper_det_sub = self.create_subscription(
            Detection2DArray, "/vision/detections/gripper",
            self._on_gripper_det, _QOS_GRIPPER_POSE,
            callback_group=self._normal_cbg,
        )
        self._latest_gripper_det: Detection2DArray | None = None
        self._latest_gripper_det_stamp: float = 0.0

        self.get_logger().info(
            "[OrchestratorNode] ready — listening on /voice/intent"
        )

    def _publish_status(self, is_moving: bool) -> None:
        """is_moving 상태를 /robot/status에 발행한다. (is_moving 발행권 소유)"""
        self._is_moving = is_moving
        msg = RobotStatus()
        msg.is_moving = is_moving
        self._status_pub.publish(msg)

    def _log_error_event(self, tool_id: str, notes: str) -> None:
        """FaultHandler용 DB 에러 이벤트 로그 (동기, 별도 스레드에서 호출됨)."""
        self._call_update_status(tool_id, "error", "error", notes)

    def _on_intent(self, msg: Intent) -> None:
        if self._is_moving:
            self.get_logger().warning(
                "[OrchestratorNode] intent ignored — robot is moving (S-7): "
                f"type={msg.intent_type} tool_id={msg.tool_id}"
            )
            self._log_s7_rejection(msg)
            return
        self.get_logger().info(
            f"[OrchestratorNode] intent received - type={msg.intent_type} "
            f"tool_id={msg.tool_id}"
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
            self.get_logger().warning(
                f"[OrchestratorNode] BT 이미 실행 중 — intent 무시: type={intent_type}"
            )
            return
        try:
            if intent_type == "fetch":
                tree = self._fetch_tree
            elif intent_type == "return":
                tree = self._return_tree
            elif intent_type == "scan":
                layer = int(tool_id) if tool_id in ("1", "2") else 1
                tree = self._scan_tree_l1 if layer == 1 else self._scan_tree_l2
            elif intent_type == "home":
                self._run_home_intent()
                return
            elif intent_type in ("scan_pose", "scan_pose_fetch"):
                self._run_single_phase_intent("scan_pose_fetch", timeout=30.0)
                return
            elif intent_type == "scan_pose_return":
                self._run_single_phase_intent("scan_pose_return", timeout=30.0)
                return
            elif intent_type == "stage_pick_test":
                self._run_single_phase_intent("stage_pick_test", tool_id=tool_id, timeout=60.0)
                return
            else:
                self.get_logger().warning(
                    f"[OrchestratorNode] 알 수 없는 intent: {intent_type}"
                )
                return

            self.get_logger().info(
                f"[OrchestratorNode] BT tick 시작: intent={intent_type} tool_id={tool_id}"
            )
            tree.tick_once()
            # tick_once()는 void — 루트 노드 status에서 실제 결과 읽기
            status = getattr(tree, 'root', tree).status

            if status == py_trees.common.Status.SUCCESS:
                self.get_logger().info(
                    f"[OrchestratorNode] BT 성공: intent={intent_type} tool_id={tool_id}"
                )
            else:
                self.get_logger().error(
                    f"[OrchestratorNode] BT 실패: intent={intent_type} "
                    f"tool_id={tool_id} status={status}"
                )
        except Exception as exc:
            self.get_logger().error(f"[OrchestratorNode] BT tick 예외: {exc}")
            self._set_plc("error")
            self._publish_status(False)  # 예외 시 is_moving 안전 해제
        finally:
            self._bt_lock.release()

    def _on_fetch_pick(self) -> None:
        """fetch: 슬롯에서 공구를 집는 순간 (action feedback phase="pick")."""
        self._dispatch_update_status(
            self._active_tool_id, "fetch", "out", "fetch pick - 슬롯에서 집어듦"
        )

    def _on_fetch_place(self) -> None:
        """fetch: Staging Area에 거치하는 순간 (action feedback phase="place")."""
        self._dispatch_update_status(
            self._active_tool_id, "fetch", "staged", "fetch place - Staging Area에 거치"
        )

    def _on_return_pick(self) -> None:
        """return: Staging Area에서 공구를 집는 순간 (action feedback phase="pick")."""
        self._dispatch_update_status(
            self._active_tool_id, "return", "out", "return pick - Staging에서 집어듦"
        )

    def _on_return_place(self) -> None:
        """return: 슬롯에 반납하는 순간 (action feedback phase="place")."""
        self._dispatch_update_status(
            self._active_tool_id, "return", "in_slot", "return place - 슬롯에 반납"
        )

    def _run_home_intent(self) -> None:
        """home intent: is_moving True → ExecutePhase(home) → is_moving False."""
        self._run_single_phase_intent("home", timeout=60.0)

    def _run_single_phase_intent(self, phase: str, tool_id: str = "", timeout: float = 60.0) -> None:
        """단일 phase ExecutePhase 목표를 is_moving 게이팅과 함께 실행."""
        self._publish_status(True)
        try:
            goal = ExecutePhase.Goal()
            goal.phase = phase
            goal.tool_id = tool_id
            goal.layer_id = 0

            done = threading.Event()
            result_holder: list = []

            def _on_goal(fut):
                gh = fut.result()
                if not gh.accepted:
                    result_holder.append(None)
                    done.set()
                    return
                gh.get_result_async().add_done_callback(
                    lambda f: (result_holder.append(f.result()), done.set())
                )

            self._exec_phase_cli.send_goal_async(goal).add_done_callback(_on_goal)
            if not done.wait(timeout=timeout):
                self.get_logger().error(f"[OrchestratorNode] {phase} 타임아웃")
                return
            if result_holder and result_holder[0] and result_holder[0].result.success:
                self.get_logger().info(f"[OrchestratorNode] {phase} 완료")
            else:
                self.get_logger().error(f"[OrchestratorNode] {phase} 실패")
        except Exception as exc:
            self.get_logger().error(f"[OrchestratorNode] {phase} 예외: {exc}")
        finally:
            self._publish_status(False)

    def _dispatch_update_status(
        self, tool_id: str, event_type: str, new_status: str, notes: str
    ) -> None:
        """_call_update_status를 별도 daemon thread에서 실행한다 (E-9).

        이 메서드는 action feedback 콜백(rclpy executor 스레드)에서
        동기 호출되므로, 최대 5초까지 블로킹되는 _call_update_status를
        직접 호출하면 그 시간 동안 같은 콜백 그룹의 다른 콜백 처리가 지연된다.
        모션 진행 중인 시점이라 fire-and-forget으로 분리한다.
        """
        def _run() -> None:
            if not self._call_update_status(tool_id, event_type, new_status, notes):
                self._set_plc("error")

        threading.Thread(target=_run, daemon=True).start()

    def _call_update_status(
        self, tool_id: str, event_type: str, new_status: str, notes: str
    ) -> bool:
        if not self._update_status_cli.service_is_ready():
            self.get_logger().error(
                "[OrchestratorNode] UpdateToolStatus 서비스 미준비 - "
                f"tool_id={tool_id} new_status={new_status}"
            )
            return False
        req = UpdateToolStatus.Request()
        req.tool_id = tool_id
        req.event_type = event_type
        req.track = "A"
        req.new_status = new_status
        req.notes = notes

        future = self._update_status_cli.call_async(req)
        done = threading.Event()
        result_holder: list = []

        def _on_done(f) -> None:
            try:
                result_holder.append(f.result())
            except Exception as exc:
                self.get_logger().error(
                    f"[OrchestratorNode] UpdateToolStatus 호출 예외 - tool_id={tool_id} "
                    f"new_status={new_status}: {exc}"
                )
            finally:
                done.set()

        future.add_done_callback(_on_done)
        if not done.wait(timeout=_UPDATE_STATUS_TIMEOUT_S):
            self.get_logger().error(
                "[OrchestratorNode] UpdateToolStatus 타임아웃 - "
                f"tool_id={tool_id} new_status={new_status}"
            )
            return False
        if not result_holder or not result_holder[0].success:
            message = result_holder[0].message if result_holder else "no response"
            self.get_logger().error(
                "[OrchestratorNode] UpdateToolStatus 실패 - "
                f"tool_id={tool_id} new_status={new_status} message={message}"
            )
            return False
        self.get_logger().info(
            f"[OrchestratorNode] UpdateToolStatus 성공 - tool_id={tool_id} "
            f"new_status={new_status}"
        )
        return True

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

    def _on_gripper_det(self, msg: Detection2DArray) -> None:
        """최신 그리퍼 detection을 보관 (gripper_pose와 timestamp 매칭용)."""
        self._latest_gripper_det = msg
        self._latest_gripper_det_stamp = (
            msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        )

    def _on_gripper_pose(self, msg: PoseStamped) -> None:
        """스캔 중 그리퍼캠 XY를 버퍼에 누적. CollectAndSave BT 노드가 소비한다.

        gripper_marker_scan_node가 frame_id="tool:<tool_id>" 형식으로 발행하면
        해당 값을 직접 사용해 공구별 pose를 정확히 그룹핑한다.
        구 형식(base_link frame_id)은 detection 캐시에서 best 공구 ID를 추출한다.
        """
        frame_id = msg.header.frame_id
        if frame_id.startswith("tool:"):
            tool_id = frame_id[5:]
            if not tool_id:
                return
        else:
            det = self._latest_gripper_det
            if det is None or not det.detections:
                return
            pose_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if abs(pose_stamp - self._latest_gripper_det_stamp) > 0.15:
                return  # 타임스탬프 슬롭 초과 — 페어 스킵
            best = max(det.detections, key=lambda d: d.results[0].hypothesis.score)
            tool_id = best.results[0].hypothesis.class_id
            if not tool_id:
                return

        entry = {
            "tool_id": tool_id,
            "x": msg.pose.position.x,
            "y": msg.pose.position.y,
        }
        with self._scan_pose_buf_lock:
            self._scan_pose_buf.append(entry)

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
                f"[OrchestratorNode] S-7 rejection log dispatch failed: {exc}"
            )

    def _on_log_event_done(self, future) -> None:
        with self._s7_log_lock:
            self._pending_s7_logs = max(0, self._pending_s7_logs - 1)
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().warning(
                f"[OrchestratorNode] S-7 rejection log call failed: {exc}"
            )
            return
        if not result.success:
            self.get_logger().warning(
                f"[OrchestratorNode] S-7 rejection log rejected: {result.message}"
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
