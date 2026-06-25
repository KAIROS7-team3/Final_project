"""tool_action_server.py
────────────────────────
ExecutePhase 영구 모션 액션 서버 (Track A BT 오케스트레이션 대상).

단일 ExecutePhase 액션으로 PlaceAtStaging / ReturnToSlot 을 대체한다.
phase 문자열로 시퀀스 선택 → SequenceEngine 이 DSR 서비스 실행.

phase 매핑:
  open_drawer  → drawer_open_seq(layer_id)
  fetch        → fixed_fetch_seq()       (비전-free, toolbox.yaml 고정좌표)
  return       → vision_return_seq()     (그리퍼 캠 비전 기반 staging 픽업)
  close_drawer → drawer_close_seq(layer_id)
  home         → home_seq()

서비스:
  ~/home          — 홈 복귀 (E-stop 래치·동작 중 거부)
  ~/estop         — DSR 즉시 정지 + 래치 (S-3)
  ~/estop_reset   — 래치 해제 (운영자 명시 확인 후)
  ~/open_toolbox  — 서랍 열기 단독 (layer 1 고정)
  ~/close_toolbox — 서랍 닫기 단독 (layer 1 고정)

설계:
  - SequenceEngine 이 DSR 클라이언트·비전 구독·E-stop 감시를 모두 소유
  - 액션 서버는 phase 매핑 + goal 수명 관리 담당. /robot/status 발행은 orchestrator 소유 (SetMoving BT 노드).
  - E-stop: engine.request_stop() → engine.estop_triggered 래치 → goal 거부
  - ~/estop_reset: engine.reset_estop() 호출 (운영자 명시 확인 후)
"""

from __future__ import annotations

import math
import os
import pathlib
import sys
import threading
import time
from pathlib import Path
from typing import Any, Optional

import yaml
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from scipy.spatial.transform import Rotation

from dsr_msgs2.srv import ConfigCreateTcp, MoveLine, MoveJoint, MoveStop, SetCurrentTcp
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from interfaces.action import PlaceAtStaging, PlaceOnHand, ReturnToSlot
from interfaces.msg import RobotStatus
from interfaces.srv import GripperSetPosition, LogEvent
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from interfaces.action import ExecutePhase
from motion.sequence_engine import SequenceEngine

from unit_actions.toolbox_motion import (
    drawer_close_seq,
    drawer_open_seq,
    drawer_open_seq_v2,
    drawer_close_seq_v2,
    scan_layer_seq,
    fixed_fetch_seq,
    full_socket_fetch_seq,
    full_socket_return_seq,
    handover_fetch_seq,
    handover_fetch_handle_first_seq,
    handover_place_only_seq,
    handover_abort_to_staging_seq,
    home_seq,
    vision_return_seq,
    stage_pick_test_seq,
)
from unit_actions.visual_servoing import ToolPose  # noqa: E402

# DSR 모션 상수 (핸드오버 직접 DSR 서비스 호출용)
DR_BASE = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

# 그리퍼 파지력 (TaskWriter SubRoutine 실측) — pulse > PULSE_RELEASE 일 때만 인가
_GRIP_CURRENT: int = 400

_VALID_PHASES = frozenset({
    "open_drawer", "fetch", "return", "close_drawer", "home",
    "open_drawer_scan", "close_drawer_scan",   # 스캔 전용 v2 서랍 시퀀스
    "scan_pose", "scan_pose_fetch", "scan_pose_return",
    "stage_pick_test",
})
_DEFAULT_TCP_NAME   = "GripperDA_v1"
_DEFAULT_CONFIG_PATH = str(
    Path(os.environ["FINAL_PROJECT_ROOT"]) / "config" / "toolbox.yaml"
    if os.environ.get("FINAL_PROJECT_ROOT")
    else Path(__file__).resolve().parents[5] / "config" / "toolbox.yaml"
)
_STANDALONE_LAYER   = 1   # ~/open_toolbox · ~/close_toolbox 기본 layer

# 핸드오버 속도 (S-6: approach_action_scale=0.2 기준)
_HANDOVER_VEL_L: float = 5.0    # mm/s  (10.0 × 0.5)
_HANDOVER_ACC_L: float = 20.0
_HANDOVER_VEL_R: float = 2.5    # deg/s
_HANDOVER_ACC_R: float = 10.0
_HANDOVER_PRE_VEL_L: float = VEL_L / 3.0   # pre_approach: 일반 속도 1/3 (30mm 여유 충돌 방지)
_HANDOVER_PRE_ACC_L: float = ACC_L / 3.0
_HANDOVER_PRE_VEL_R: float = VEL_R / 3.0
_HANDOVER_PRE_ACC_R: float = ACC_R / 3.0

# 핸드오버 rx/ry — fetch/place와 동일한 tool_approach_ori 기준 (RX=180, RY=180)
_HANDOVER_RX: float = 180.0
_HANDOVER_RY: float = 180.0


def _load_yaml_cfg(filename: str) -> dict:
    """레포 루트에서 config/<filename> 을 찾아 로드한다."""
    here = pathlib.Path(__file__).resolve()
    for p in here.parents:
        cfg = p / "config" / filename
        if cfg.exists():
            return yaml.safe_load(cfg.read_text())
    return {}


class ExecutePhaseServer(Node):
    """ExecutePhase 영구 액션 서버 — SequenceEngine 위임."""

    def __init__(self) -> None:
        super().__init__("tool_action_server")  # 노드명 유지 (launch 호환)

        # ── 파라미터 ────────────────────────────────────────────────────────
        self.declare_parameter("robot_ns",    "dsr01")
        self.declare_parameter("tcp_name",    _DEFAULT_TCP_NAME)
        self.declare_parameter("config_path", _DEFAULT_CONFIG_PATH)
        self.declare_parameter("mode",        "virtual")

        ns   = self.get_parameter("robot_ns").get_parameter_value().string_value
        tcp  = self.get_parameter("tcp_name").get_parameter_value().string_value
        cfg  = self.get_parameter("config_path").get_parameter_value().string_value
        mode = self.get_parameter("mode").get_parameter_value().string_value

        # ── 콜백 그룹 분리 ──────────────────────────────────────────────────
        self._normal_cbg = ReentrantCallbackGroup()
        self._estop_cbg  = MutuallyExclusiveCallbackGroup()

        # ── 동시 goal 방지 락 ────────────────────────────────────────────────
        self._action_lock = threading.Lock()

        # ── DSR 서비스 클라이언트 (핸드오버 직접 호출용) ─────────────────────
        p = ns  # 핸드오버 direct-call 클라이언트는 robot_ns prefix 사용
        self._movel_cli = self.create_client(
            MoveLine, f"{p}/motion/move_line", callback_group=self._normal_cbg
        )
        self._movej_cli = self.create_client(
            MoveJoint, f"{p}/motion/move_joint", callback_group=self._normal_cbg
        )
        self._stop_cli = self.create_client(
            MoveStop, f"{p}/motion/move_stop", callback_group=self._estop_cbg
        )
        self._create_tcp_cli = self.create_client(
            ConfigCreateTcp, f"{p}/tcp/config_create_tcp", callback_group=self._normal_cbg
        )
        self._set_tcp_cli = self.create_client(
            SetCurrentTcp, f"{p}/tcp/set_current_tcp", callback_group=self._normal_cbg
        )
        self._gripper_cli = self.create_client(
            GripperSetPosition, "/gripper/set_position", callback_group=self._normal_cbg
        )
        self._log_event_cli = self.create_client(
            LogEvent, "/db/LogEvent", callback_group=self._normal_cbg
        )
        # 핸드오버 클라이언트 서비스 대기
        for cli, name in [
            (self._movel_cli, "move_line"),
            (self._movej_cli, "move_joint"),
            (self._gripper_cli, "gripper/set_position"),
        ]:
            if not cli.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f"[TAS] {name} 없음 — bringup 먼저 실행")
                raise RuntimeError(f"{name} 서비스 없음")
            self.get_logger().info(f"[TAS] {name} 연결됨")

        # ── SequenceEngine 초기화 ────────────────────────────────────────────
        self._engine = SequenceEngine(
            self,
            robot_ns=ns,
            tcp_name=tcp,
            config_path=cfg,
            mode=mode,
        )
        self._engine.setup(wait_timeout_sec=15.0)

        # ── 발행자 ──────────────────────────────────────────────────────────
        self._plc_pub = self.create_publisher(String, "/plc/system_state", 1)

        # ── 핸드오버 상태 ────────────────────────────────────────────────────
        self._estop_latch = threading.Event()  # E-stop 래치 (_wait_motion_complete 전용)

        # ── 핸드오버 config ───────────────────────────────────────────────
        self._h_cfg: dict = _load_yaml_cfg("handover.yaml").get("handover", {})
        self._toolbox_cfg: dict = _load_yaml_cfg("toolbox.yaml")

        # ── 핸드오버 손 상태 ──────────────────────────────────────────────
        self._hand_lock = threading.Lock()
        self._hand_pose: Optional[PoseStamped] = None
        self._hand_ready: bool = False
        self._current_tool_id: str = ""
        self._hand_approach_pos: Optional[list] = None
        self._current_goal_handle = None

        # DSR joint velocity 모니터 (모션 완료 감지용)
        self._joint_vel_lock = threading.Lock()
        self._joint_velocities: list[float] = []
        self.create_subscription(
            JointState, f"/{ns}/joint_states", self._on_joint_states, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, "/hand/pose", self._on_hand_pose, qos_profile_sensor_data
        )
        self.create_subscription(
            Bool, "/hand/ready", self._on_hand_ready, qos_profile_sensor_data
        )

        # ── 비전 그리퍼 캠 상태 (handover fetch 공통) ────────────────────────
        self._tool_gripper_lock = threading.Lock()
        self._latest_tool_gripper: ToolPose = ToolPose()
        self._grip_taken: bool = False
        vm = self._toolbox_cfg.get("vision_motion", {})
        self._tool_approach_z_mm: float = float(vm.get("tool_approach_z_mm", 234.0))
        self._tool_approach_ori: list = list(vm.get("tool_approach_ori", [180.0, 180.0, 90.0]))
        lim = vm.get("workspace_limits", {})
        self._vis_x_min = float((lim.get("x") or [50.0, 800.0])[0])
        self._vis_x_max = float((lim.get("x") or [50.0, 800.0])[1])
        self._vis_y_min = float((lim.get("y") or [-600.0, 600.0])[0])
        self._vis_y_max = float((lim.get("y") or [-600.0, 600.0])[1])
        self._vis_z_min = float((lim.get("z") or [-31.0, 700.0])[0])
        self._vis_z_max = float((lim.get("z") or [-31.0, 700.0])[1])
        self._grasp_z_map: dict[str, float] = {
            t["tool_id"]: float(t["grasp_z_mm"])
            for t in self._toolbox_cfg.get("tools", [])
            if "grasp_z_mm" in t
        }
        self.create_subscription(
            PoseStamped,
            "/vision/tool_gripper_pose",
            self._on_tool_gripper_pose,
            qos_profile_sensor_data,
        )

        # ── ExecutePhase 액션 서버 ───────────────────────────────────────────
        self._action_server = ActionServer(
            self,
            ExecutePhase,
            "execute_phase",
            execute_callback=self._on_execute_phase,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._normal_cbg,
        )
        self._handover_server = ActionServer(
            self,
            PlaceOnHand,
            "place_on_hand",
            execute_callback=self._on_place_on_hand,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._normal_cbg,
        )
        self._handover_test_server = ActionServer(
            self,
            PlaceOnHand,
            "place_on_hand_test",
            execute_callback=self._on_place_on_hand_test,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._normal_cbg,
        )

        # ── 서비스 서버 ─────────────────────────────────────────────────────
        self._home_srv = self.create_service(
            Trigger, "~/home", self._on_home, callback_group=self._normal_cbg
        )
        self._estop_srv = self.create_service(
            Trigger, "~/estop", self._on_estop, callback_group=self._estop_cbg
        )
        self._estop_reset_srv = self.create_service(
            Trigger, "~/estop_reset", self._on_estop_reset, callback_group=self._estop_cbg
        )
        self._open_srv = self.create_service(
            Trigger, "~/open_toolbox", self._on_open_toolbox, callback_group=self._normal_cbg
        )
        self._close_srv = self.create_service(
            Trigger, "~/close_toolbox", self._on_close_toolbox, callback_group=self._normal_cbg
        )
        self._open_srv_l2 = self.create_service(
            Trigger, "~/open_toolbox_l2", self._on_open_toolbox_l2, callback_group=self._normal_cbg
        )
        self._close_srv_l2 = self.create_service(
            Trigger, "~/close_toolbox_l2", self._on_close_toolbox_l2, callback_group=self._normal_cbg
        )

        # ── TCP 초기 설정 타이머 ─────────────────────────────────────────────
        self._tcp_timer = self.create_timer(
            2.0, self._tcp_setup_once, callback_group=self._normal_cbg
        )

        self.get_logger().info("[TAS] ExecutePhase 서버 준비 완료")

    # ── TCP 설정 ─────────────────────────────────────────────────────────────

    def _tcp_setup_once(self) -> None:
        self._tcp_timer.cancel()
        self._engine.set_tcp()

    def _set_plc(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._plc_pub.publish(msg)

    # ── 액션 goal / cancel 핸들러 ────────────────────────────────────────────

    def _goal_callback(self, goal_request) -> GoalResponse:
        if self._engine.estop_triggered:
            self.get_logger().warn("[TAS] goal 거부 — E-stop 래치 활성")
            return GoalResponse.REJECT
        phase = getattr(goal_request, "phase", "")
        if phase not in _VALID_PHASES:
            self.get_logger().warn(f"[TAS] goal 거부 — 알 수 없는 phase: {phase!r}")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    # ── ExecutePhase 실행 ────────────────────────────────────────────────────

    def _on_execute_phase(self, goal_handle) -> ExecutePhase.Result:
        result  = ExecutePhase.Result()
        req     = goal_handle.request
        phase   = req.phase
        tool_id = req.tool_id
        layer_id = req.layer_id

        if not self._action_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = "다른 동작 진행 중"
            return result

        try:
            self.get_logger().info(
                f"[TAS] phase={phase} tool_id={tool_id!r} layer_id={layer_id}"
            )
            self._engine.set_tcp()
            # PLC 상태 관리는 orchestrator BT(SetMoving)가 전담한다.
            # 여기서 _set_plc를 호출하면 phase 완료마다 M0100 reset이 발생해
            # 다음 phase 시작 전 P040~P044가 순간 꺼지는 문제가 생긴다.

            steps = self._resolve_phase(phase, tool_id, layer_id)
            if steps is None:
                goal_handle.abort()
                result.success = False
                result.message = f"알 수 없는 phase: {phase!r}"
                return result

            def feedback_cb(step_name: str, progress: float) -> None:
                fb = ExecutePhase.Feedback()
                fb.phase    = step_name
                fb.progress = progress
                goal_handle.publish_feedback(fb)

            ok = self._engine.run_sequence(
                steps,
                tool_id=tool_id,
                cancel_check=lambda: goal_handle.is_cancel_requested,
                feedback_cb=feedback_cb,
            )

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "취소됨"
            elif ok:
                goal_handle.succeed()
                result.success = True
                result.message = f"{phase} 완료"
                self.get_logger().info(f"[TAS] {phase} 완료: tool_id={tool_id!r}")
            else:
                self._engine.stop_motion()
                goal_handle.abort()
                result.success = False
                result.message = f"{phase} 실패"
                self.get_logger().error(f"[TAS] {phase} 실패: tool_id={tool_id!r}")

        except Exception as exc:
            self.get_logger().error(f"[TAS] phase={phase} 예외: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
        finally:
            self._action_lock.release()

        return result

    # ── phase → 시퀀스 매핑 ──────────────────────────────────────────────────

    def _resolve_phase(
        self, phase: str, tool_id: str, layer_id: int
    ) -> Optional[list]:
        if phase == "open_drawer":
            return drawer_open_seq_v2(layer_id)
        if phase == "fetch":
            return fixed_fetch_seq()
        if phase == "return":
            return vision_return_seq(scan_j_deg=self._engine._return_scan_j_deg)
        if phase == "close_drawer":
            return drawer_close_seq_v2(layer_id)
        if phase == "home":
            return home_seq()
        if phase == "open_drawer_scan":
            return drawer_open_seq_v2(layer_id)
        if phase == "close_drawer_scan":
            return drawer_close_seq_v2(layer_id)
        if phase in ("scan_pose", "scan_pose_fetch"):
            self._engine._load_toolbox_config()
            return scan_layer_seq(getattr(self._engine, "_fetch_scan_j_deg", None))
        if phase == "scan_pose_return":
            self._engine._load_toolbox_config()
            return scan_layer_seq(getattr(self._engine, "_return_scan_j_deg", None))
        if phase == "stage_pick_test":
            self._engine._load_toolbox_config()
            return stage_pick_test_seq(scan_j_deg=getattr(self._engine, "_return_scan_j_deg", None))
        return None

        try:
            self.get_logger().info(f"[TAS] return 시작: tool_id={tool_id}")
            self._set_tcp()
            self._publish_status(is_moving=True)
            self._set_plc("moving")

            steps = full_socket_return_seq()
            ok = self._run_sequence(steps, goal_handle, action_class=ReturnToSlot)

            if ok and not goal_handle.is_cancel_requested:
                goal_handle.succeed()
                result.success = True
                result.message = f"return 완료: {tool_id}"
                self._set_plc("idle")
                self.get_logger().info(f"[TAS] return 완료: tool_id={tool_id}")
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "취소됨"
            else:
                goal_handle.abort()
                result.success = False
                result.message = f"return 실패: {tool_id}"
                self._set_plc("error")
        except Exception as exc:
            self.get_logger().error(f"[TAS] return 예외: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            self._set_plc("error")
        finally:
            self._publish_status(is_moving=False)
            self._action_lock.release()

        return result

    # ── 핸드오버 콜백 ───────────────────────────────────────────────────────

    def _on_joint_states(self, msg: JointState) -> None:
        with self._joint_vel_lock:
            self._joint_velocities = list(msg.velocity)

    def _wait_motion_complete(
        self, timeout: float = 60.0, still_thresh: float = 0.01, moving_thresh: float = 0.05
    ) -> bool:
        """joint velocity 기반 모션 완료 대기 — 시작 감지 후 멈춤 감지 2단계.
        sync_type=1로 이미 완료된 경우 즉시 True 반환."""
        deadline = time.monotonic() + timeout

        # 초기 대기: 5mm/s 저속 모션 가속 시간(acc=20mm/s² → ramp=0.25s) 고려
        # 0.1s는 로봇이 아직 가속 전이라 vel≈0 → 완료로 오인하므로 0.5s로 연장
        time.sleep(0.5)
        with self._joint_vel_lock:
            vels = self._joint_velocities
        if vels and max(abs(v) for v in vels) < still_thresh:
            return True

        # 1단계: 로봇이 실제로 움직이기 시작할 때까지 대기
        while time.monotonic() < deadline:
            if self._estop_latch.is_set():
                return False
            with self._joint_vel_lock:
                vels = self._joint_velocities
            if vels and max(abs(v) for v in vels) > moving_thresh:
                break
            time.sleep(0.02)
        else:
            self.get_logger().warn("[TAS] _wait_motion_complete: 움직임 미감지 timeout")
            return False

        # 2단계: 움직임이 멈출 때까지 대기
        while time.monotonic() < deadline:
            if self._estop_latch.is_set():
                return False
            with self._joint_vel_lock:
                vels = self._joint_velocities
            if vels and max(abs(v) for v in vels) < still_thresh:
                return True
            time.sleep(0.05)

        self.get_logger().warn("[TAS] _wait_motion_complete: 정지 timeout")
        return False

    def _on_hand_pose(self, msg: PoseStamped) -> None:
        with self._hand_lock:
            self._hand_pose = msg

    def _on_hand_ready(self, msg: Bool) -> None:
        with self._hand_lock:
            self._hand_ready = msg.data

    def _get_hand_state(self) -> tuple[Optional[PoseStamped], bool]:
        with self._hand_lock:
            return self._hand_pose, self._hand_ready

    # ── 핸드오버 액션 핸들러 ────────────────────────────────────────────────

    def _on_place_on_hand(self, goal_handle) -> PlaceOnHand.Result:
        result = PlaceOnHand.Result()
        tool_id = goal_handle.request.tool_id

        if not self._action_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = "다른 동작 진행 중"
            return result

        try:
            self.get_logger().info(f"[TAS] place_on_hand 시작: tool_id={tool_id}")
            self._set_tcp()
            self._publish_status(is_moving=True)
            self._set_plc("moving")
            self._current_tool_id = tool_id
            self._hand_approach_pos = None
            self._grip_taken = False

            handover_type = self._get_handover_type(tool_id)
            steps = (
                handover_fetch_handle_first_seq()
                if handover_type == "handle_first"
                else handover_fetch_seq()
            )
            self.get_logger().info(f"[TAS] handover_type={handover_type} seq={len(steps)}단계")

            ok = self._run_sequence_handover(steps, goal_handle)

            # 공구를 집은 뒤 핸드오버가 abort됐으면 staging에 거치
            if not ok and self._grip_taken and not goal_handle.is_cancel_requested:
                self.get_logger().warn(
                    f"[TAS] place_on_hand abort: grip_taken=True → staging 거치 시도"
                )
                staging_steps = handover_abort_to_staging_seq()
                ok_staging = self._run_sequence(staging_steps, goal_handle, PlaceOnHand)
                if ok_staging:
                    self._grip_taken = False  # staging 완료, 공구 내려놓음
                    ok = True
                    result.message = f"place_on_hand staging fallback 완료: {tool_id}"
                    self._log_event_async("fetch", f"place_on_hand staging fallback: tool_id={tool_id}")
                    self.get_logger().info(f"[TAS] staging fallback 완료: tool_id={tool_id}")
                else:
                    self.get_logger().error(
                        f"[TAS] staging fallback 실패 — 공구 미거치: tool_id={tool_id}"
                    )

            if ok and not goal_handle.is_cancel_requested:
                goal_handle.succeed()
                result.success = True
                if not result.message:
                    result.message = f"place_on_hand 완료: {tool_id}"
                self._set_plc("idle")
                if "staging fallback" not in result.message:
                    self._log_event_async("fetch", f"place_on_hand 성공: tool_id={tool_id}")
                self.get_logger().info(f"[TAS] place_on_hand 완료: tool_id={tool_id}")
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "취소됨"
                self._log_event_async("rejected", f"place_on_hand 취소: tool_id={tool_id}")
            else:
                goal_handle.abort()
                result.success = False
                result.message = f"place_on_hand 실패: {tool_id}"
                self._log_event_async("error", f"place_on_hand 실패: tool_id={tool_id}")
                self._set_plc("error")
        except Exception as exc:
            self.get_logger().error(f"[TAS] place_on_hand 예외: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            self._log_event_async("error", f"place_on_hand 예외: tool_id={tool_id} err={exc}")
            self._set_plc("error")
        finally:
            self._current_tool_id = ""
            self._hand_approach_pos = None
            if self._grip_taken:
                # 공구 파지 후 abort — 추가 모션 금지, 운영자 수동 확인 필요
                self.get_logger().error(
                    "[TAS] 공구 파지 후 abort — 모션 정지, 운영자 확인 필요"
                )
                self._set_plc("error")
                # is_moving=False 미발행 → STT 차단 유지 (운영자 리셋 전까지)
            else:
                self._publish_status(is_moving=False)
            self._grip_taken = False
            self._action_lock.release()

        return result

    def _on_place_on_hand_test(self, goal_handle) -> PlaceOnHand.Result:
        """테스트용 — 공구를 이미 쥔 상태에서 손 전달만 실행 (①~⑨ 스킵)."""
        result = PlaceOnHand.Result()
        tool_id = goal_handle.request.tool_id

        if not self._action_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = "다른 동작 진행 중"
            return result

        try:
            self.get_logger().info(f"[TAS] place_on_hand_test 시작: tool_id={tool_id}")
            self._set_tcp()
            self._publish_status(is_moving=True)
            self._set_plc("moving")
            self._current_tool_id = tool_id
            self._hand_approach_pos = None
            self._grip_taken = True  # 이미 공구 파지 상태로 간주

            handle_first = self._get_handover_type(tool_id) == "handle_first"
            steps = handover_place_only_seq(handle_first=handle_first)
            self.get_logger().info(
                f"[TAS] handover_type={'handle_first' if handle_first else 'direct'} "
                f"place-only seq={len(steps)}단계"
            )

            ok = self._run_sequence_handover(steps, goal_handle)

            if ok and not goal_handle.is_cancel_requested:
                goal_handle.succeed()
                result.success = True
                result.message = f"place_on_hand_test 완료: {tool_id}"
                self._set_plc("idle")
                self._log_event_async("fetch", f"place_on_hand_test 성공: tool_id={tool_id}")
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "취소됨"
                self._log_event_async("rejected", f"place_on_hand_test 취소: tool_id={tool_id}")
            else:
                goal_handle.abort()
                result.success = False
                result.message = f"place_on_hand_test 실패: {tool_id}"
                self._log_event_async("error", f"place_on_hand_test 실패: tool_id={tool_id}")
                self._set_plc("error")
        except Exception as exc:
            self.get_logger().error(f"[TAS] place_on_hand_test 예외: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            self._log_event_async("error", f"place_on_hand_test 예외: tool_id={tool_id} err={exc}")
            self._set_plc("error")
        finally:
            self._current_tool_id = ""
            self._hand_approach_pos = None
            if self._grip_taken:
                self.get_logger().error(
                    "[TAS] place_on_hand_test: 공구 파지 후 abort — 모션 정지, 운영자 확인 필요"
                )
                self._set_plc("error")
                # is_moving=False 미발행 → STT 차단 유지
            else:
                self._publish_status(is_moving=False)
            self._grip_taken = False
            self._action_lock.release()

        return result

    # ── 핸드오버 헬퍼 ───────────────────────────────────────────────────────

    def _get_handover_type(self, tool_id: str) -> str:
        for tool in self._toolbox_cfg.get("tools", []):
            if tool.get("tool_id") == tool_id:
                return tool.get("handover_type", "direct")
        return "direct"

    def _get_tool_length_mm(self, tool_id: str) -> float:
        for tool in self._toolbox_cfg.get("tools", []):
            if tool.get("tool_id") == tool_id:
                return tool.get("dimensions", {}).get("length", 0.15) * 1000.0
        return 150.0

    def _quat_to_robot_rz(self, qx: float, qy: float, qz: float, qw: float) -> float:
        """손 쿼터니언 → Doosan rz (deg). handover.yaml 캘리브레이션 적용."""
        raw_yaw = float(Rotation.from_quat([qx, qy, qz, qw]).as_euler("xyz", degrees=True)[2])
        rz = (
            (raw_yaw - self._h_cfg.get("rz_cal_hand_yaw", 77.0))
            * self._h_cfg.get("rz_sign", -1.0)
            + self._h_cfg.get("rz_cal_robot_rz", 8.39)
            + 180.0
        )
        # [-180, 180] 범위로 정규화
        if rz > 180.0:
            rz -= 360.0
        elif rz < -180.0:
            rz += 360.0
        return rz

    # ── 비전 그리퍼 캠 콜백 ─────────────────────────────────────────────────

    def _on_tool_gripper_pose(self, msg: PoseStamped) -> None:
        """handover fetch ⑤: /vision/tool_gripper_pose 수신 — toolbox_seq_runner 동일 패턴."""
        q = msg.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        pca_theta = math.degrees(math.atan2(siny_cosp, cosy_cosp))
        rz = pca_theta - 90.0
        if rz < -180.0:
            rz += 360.0
        with self._tool_gripper_lock:
            self._latest_tool_gripper = ToolPose(
                x=msg.pose.position.x * 1000.0,
                y=msg.pose.position.y * 1000.0,
                z=msg.pose.position.z * 1000.0,
                rz=rz,
                valid=True,
            )

    def _check_vision_coords(self, label: str, x: float, y: float, z: float) -> bool:
        if x == 0.0 and y == 0.0 and z == 0.0:
            self.get_logger().error(f"[TAS] {label} 좌표 미설정 (0,0,0) — 실행 거부")
            return False
        for val, lo, hi, axis in [
            (x, self._vis_x_min, self._vis_x_max, "x"),
            (y, self._vis_y_min, self._vis_y_max, "y"),
            (z, self._vis_z_min, self._vis_z_max, "z"),
        ]:
            if not (lo <= val <= hi):
                self.get_logger().error(
                    f"[TAS] {label} {axis}={val:.1f}mm 범위 초과 [{lo}, {hi}] — 실행 거부"
                )
                return False
        return True

    def _exec_wait_vision_gripper_xy(self, timeout_sec: float = 5.0) -> bool:
        """WAIT_VISION_TOP_XY: 캐시 초기화 후 신규 /vision/tool_gripper_pose 수신 대기."""
        with self._tool_gripper_lock:
            self._latest_tool_gripper = ToolPose(valid=False)
        self.get_logger().info("  [WAIT_VISION] 그리퍼 캠 좌표 대기 중...")
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if self._estop_latch.is_set():
                return False
            if self._current_goal_handle is not None and self._current_goal_handle.is_cancel_requested:
                return False
            with self._tool_gripper_lock:
                if self._latest_tool_gripper.valid:
                    p = self._latest_tool_gripper
                    self.get_logger().info(
                        f"  [WAIT_VISION] 수신 완료 → ({p.x:.1f}, {p.y:.1f}) rz={p.rz:.1f}°"
                    )
                    return True
            time.sleep(0.05)
        self.get_logger().error("  [WAIT_VISION] 타임아웃 — /vision/tool_gripper_pose 확인 필요")
        return False

    def _exec_move_l_top_xy(self) -> bool:
        """MOVE_L_TOP_XY: 그리퍼 캠 XY + rz + approach_z 로 이동 (fetch ⑥⑨)."""
        with self._tool_gripper_lock:
            pose = ToolPose(**vars(self._latest_tool_gripper))
        if not pose.valid:
            self.get_logger().error("  [TOP_XY] 그리퍼 캠 좌표 미수신")
            return False
        if not math.isfinite(pose.rz) or not (-185.0 <= pose.rz <= 185.0):
            self.get_logger().error(f"  [TOP_XY] rz 비정상값 거부: {pose.rz!r}°")
            return False
        if not self._check_vision_coords("TOP_XY", pose.x, pose.y, self._tool_approach_z_mm):
            return False
        ori = list(self._tool_approach_ori)
        ori[2] = pose.rz
        pos = [pose.x, pose.y, self._tool_approach_z_mm] + ori
        from unit_actions.toolbox_motion import Step as _Step  # 로컬 import (순환 방지)
        step = _Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=VEL_L, acc=ACC_L)
        self.get_logger().info(
            f"  [TOP_XY] → ({pose.x:.1f}, {pose.y:.1f}, {self._tool_approach_z_mm:.1f}) rz={pose.rz:.1f}°"
        )
        return self._movel(step, DR_MV_MOD_ABS)

    def _exec_move_l_tool_xyz(self) -> bool:
        """MOVE_L_TOOL_XYZ: 그리퍼 캠 XY + rz + grasp_z 로 공구 위치 하강 (fetch ⑦)."""
        with self._tool_gripper_lock:
            pose = ToolPose(**vars(self._latest_tool_gripper))
        if not pose.valid:
            self.get_logger().error("  [TOOL_XYZ] 그리퍼 캠 좌표 미수신")
            return False
        grasp_z = self._grasp_z_map.get(self._current_tool_id)
        if grasp_z is not None:
            z = grasp_z
            self.get_logger().info(
                f"  [TOOL_XYZ] Z={z:.2f}mm (toolbox.yaml, tool_id={self._current_tool_id})"
            )
        else:
            z = pose.z
            self.get_logger().warn(
                f"  [TOOL_XYZ] tool_id={self._current_tool_id!r} grasp_z_mm 미등록 — 그리퍼 캠 Z 사용: {z:.2f}mm"
            )
        if not self._check_vision_coords("TOOL_XYZ", pose.x, pose.y, z):
            return False
        ori = list(self._tool_approach_ori)
        ori[2] = pose.rz
        pos = [pose.x, pose.y, z] + ori
        from unit_actions.toolbox_motion import Step as _Step
        step = _Step(kind=StepKind.MOVE_L_ABS, pose=pos, vel=VEL_L, acc=ACC_L)
        self.get_logger().info(
            f"  [TOOL_XYZ] → ({pose.x:.1f}, {pose.y:.1f}, {z:.2f}) rz={pose.rz:.1f}°"
        )
        return self._movel(step, DR_MV_MOD_ABS)

    # ── 핸드오버 스텝 실행 ──────────────────────────────────────────────────

    def _wait_hand_pose(self, timeout_override: float | None = None) -> bool:
        """WAIT_HAND_POSE: /hand/ready=True 될 때까지 대기. 타임아웃 시 False (staging fallback).

        timeout_override=0: 현재 상태만 즉시 확인 (페치 완료 후 순간 체크용).
        _hand_approach_pos가 설정된 상태(⑪ 접근 직전 재확인)에서는 손 이동 거리도 검사.
        lock_update_distance_m 초과 시 즉시 abort (S-6 손 안정성 확인).
        """
        if timeout_override == 0.0:
            pose, ready = self._get_hand_state()
            if pose is not None and ready:
                return True
            self.get_logger().warn("[TAS] WAIT_HAND_POSE 즉시 체크: 손 미감지 — staging fallback")
            return False
        timeout = timeout_override if timeout_override is not None else self._h_cfg.get("detection_timeout_s", 5.0)
        threshold_mm = self._h_cfg.get("lock_update_distance_m", 0.03) * 1000.0
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._estop_latch.is_set():
                return False
            pose, ready = self._get_hand_state()
            if pose is not None and ready:
                # ⑪ 재확인 단계: approach 위치 대비 손 이동 거리 검사
                if self._hand_approach_pos is not None:
                    dx = pose.pose.position.x * 1000.0 - self._hand_approach_pos[0]
                    dy = pose.pose.position.y * 1000.0 - self._hand_approach_pos[1]
                    dz = pose.pose.position.z * 1000.0 - self._hand_approach_pos[2]
                    dist_mm = math.sqrt(dx * dx + dy * dy + dz * dz)
                    if dist_mm > threshold_mm:
                        self.get_logger().warn(
                            f"[TAS] 손 이동 {dist_mm:.1f}mm > {threshold_mm:.0f}mm — abort (S-6)"
                        )
                        return False
                return True
            time.sleep(0.1)
        self.get_logger().warn("[TAS] WAIT_HAND_POSE 타임아웃 — staging fallback")
        return False

    def _move_hand_rz_approach(self, handle_first: bool) -> bool:
        """MOVE_L_HAND_RZ_APPROACH(_HANDLE): rz 적용 후 손바닥 위 approach_height 위치로 이동."""
        pose, ready = self._get_hand_state()
        if pose is None or not ready:
            self.get_logger().warn("[TAS] 손 감지 없음 — abort")
            return False

        o = pose.pose.orientation
        rz = self._quat_to_robot_rz(o.x, o.y, o.z, o.w)
        # robot_base_link 기준 실제 손 방향 각도 (오프셋 계산 전용)
        raw_yaw = float(
            Rotation.from_quat([o.x, o.y, o.z, o.w]).as_euler("xyz", degrees=True)[2]
        )

        # hand_node는 m 단위 → DSR은 mm
        x_mm = pose.pose.position.x * 1000.0
        y_mm = pose.pose.position.y * 1000.0
        z_mm = pose.pose.position.z * 1000.0

        # S-6: 손 Z 상한 검사 — 초과 시 DSR 알람/연결 끊김 방지
        hand_z_max_mm = self._h_cfg.get("hand_z_max_m", 0.17) * 1000.0
        if z_mm > hand_z_max_mm:
            self.get_logger().error(
                f"[TAS] 손 Z {z_mm:.1f}mm > 상한 {hand_z_max_mm:.1f}mm — abort (S-6)"
            )
            return False

        approach_h_mm = self._h_cfg.get("approach_height_m", 0.05) * 1000.0
        x_mm += self._h_cfg.get("approach_x_offset_m", 0.0) * 1000.0
        y_mm += self._h_cfg.get("approach_y_offset_m", 0.0) * 1000.0

        if handle_first:
            R_mat = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
            # 1) palm center 보정: wrist 포함으로 손목 쪽 쏠림 → 중지 방향(Y컬럼)으로 먼저 보정
            finger_off_mm = self._h_cfg.get("finger_offset_m", 0.02) * 1000.0
            x_mm += float(R_mat[0, 1]) * finger_off_mm
            y_mm += float(R_mat[1, 1]) * finger_off_mm
            # 2) 손잡이 방향 = X컬럼 (새끼→검지)으로 tool_length/2 오프셋
            offset_mm = self._get_tool_length_mm(self._current_tool_id) / 2.0
            x_mm += float(R_mat[0, 0]) * offset_mm
            y_mm += float(R_mat[1, 0]) * offset_mm

        # place 단계에서 재사용할 위치 저장 (approach_height 제외)
        self._hand_approach_pos = [x_mm, y_mm, z_mm, _HANDOVER_RX, _HANDOVER_RY, rz]

        # ── 사전 이동: approach XY로 이동하되 Z는 손 Z + approach_height + 50mm ──
        pre_pos = [x_mm, y_mm, z_mm + approach_h_mm + 30.0, _HANDOVER_RX, _HANDOVER_RY, rz]
        self.get_logger().info(
            f"[TAS] hand_pre_approach ({'handle' if handle_first else 'direct'}) "
            f"pos={[round(v, 1) for v in pre_pos]}"
        )
        req = MoveLine.Request()
        req.pos = [float(v) for v in pre_pos]
        req.vel = [_HANDOVER_PRE_VEL_L, _HANDOVER_PRE_VEL_R]  # 일반 속도 1/3
        req.acc = [_HANDOVER_PRE_ACC_L, _HANDOVER_PRE_ACC_R]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 1
        fut = self._movel_cli.call_async(req)
        if not self._wait_future(fut, timeout=60.0):
            self.get_logger().error("[TAS] hand_pre_approach 타임아웃")
            return False
        res = fut.result()
        if not (res and res.success):
            self.get_logger().error("[TAS] hand_pre_approach 실패")
            return False
        self._wait_motion_complete(timeout=60.0)  # 실제 모션 완료 대기

        # pre_approach 이동 중 손이 치워졌을 수 있으므로 재확인 (S-6)
        _, ready_after = self._get_hand_state()
        if not ready_after:
            self.get_logger().warn("[TAS] pre_approach 후 손 감지 소실 — abort (S-6)")
            return False

        # ── 본 approach: XY 유지, Z만 approach 높이로 하강 ──
        dsr_pos = [x_mm, y_mm, z_mm + approach_h_mm, _HANDOVER_RX, _HANDOVER_RY, rz]
        self.get_logger().info(
            f"[TAS] hand_approach ({'handle' if handle_first else 'direct'}) "
            f"pos={[round(v, 1) for v in dsr_pos]}"
        )

        req = MoveLine.Request()
        req.pos = [float(v) for v in dsr_pos]
        req.vel = [_HANDOVER_VEL_L, _HANDOVER_VEL_R]
        req.acc = [_HANDOVER_ACC_L, _HANDOVER_ACC_R]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 1
        fut = self._movel_cli.call_async(req)
        if not self._wait_future(fut, timeout=60.0):
            self.get_logger().error("[TAS] hand_approach 타임아웃")
            return False
        res = fut.result()
        if not (res and res.success):
            return False
        # 10mm/s 저속 모션 — 낮은 임계값으로 감지
        self._wait_motion_complete(timeout=60.0, moving_thresh=0.01, still_thresh=0.005)
        return True

    def _move_hand_place(self, handle_first: bool = False) -> bool:
        """MOVE_L_HAND_PLACE(_HANDLE): approach 단계에서 저장한 손바닥 높이로 Z 수직 하강."""
        if self._hand_approach_pos is None:
            self.get_logger().error("[TAS] _hand_approach_pos 없음 — approach 먼저 실행 필요")
            return False

        if handle_first:
            place_z_offset_mm = self._h_cfg.get("place_z_offset_handle_first_m",
                                                  self._h_cfg.get("place_z_offset_m", 0.0)) * 1000.0
        else:
            place_z_offset_mm = self._h_cfg.get("place_z_offset_m", 0.0) * 1000.0
        dsr_pos = list(self._hand_approach_pos)
        dsr_pos[2] += place_z_offset_mm
        self.get_logger().info(
            f"[TAS] hand_place pos={[round(v, 1) for v in dsr_pos]}"
        )

        req = MoveLine.Request()
        req.pos = [float(v) for v in dsr_pos]
        req.vel = [_HANDOVER_VEL_L, _HANDOVER_VEL_R]
        req.acc = [_HANDOVER_ACC_L, _HANDOVER_ACC_R]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 1
        fut = self._movel_cli.call_async(req)
        if not self._wait_future(fut, timeout=60.0):
            self.get_logger().error("[TAS] hand_place 타임아웃")
            return False
        res = fut.result()
        # 10mm/s 저속 모션 — 낮은 임계값으로 감지
        self._wait_motion_complete(timeout=60.0, moving_thresh=0.01, still_thresh=0.005)
        time.sleep(0.3)
        return bool(res and res.success)

    # ── 서비스 핸들러 ───────────────────────────────────────────────────────

    def _on_home(self, _req, response: Trigger.Response) -> Trigger.Response:
        # ⚠️ S-7: 이 서비스는 BT 외부(유지보수·긴급 복귀)에서만 호출할 것.
        # BT가 실행 중이 아닐 때만 안전하다 — 호출 중 /robot/status가 갱신되지 않으므로
        # voice 게이트가 열린 채로 로봇이 움직일 수 있다.
        if self._engine.estop_triggered:
            response.success = False
            response.message = "E-stop 래치 — estop_reset 후 재시도"
            return response
        if not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "다른 동작 진행 중"
            return response
        try:
            self.get_logger().info("[TAS] home 시작")
            self._engine.set_tcp()
            self._set_plc("moving")
            ok = self._engine.run_sequence(home_seq())
            if ok:
                response.success = True
                response.message = "home 완료"
                self._set_plc("idle")
            else:
                response.success = False
                response.message = "home 실패"
                self._set_plc("error")
        except Exception as exc:
            response.success = False
            response.message = str(exc)
        finally:
            self._action_lock.release()
        return response

    def _on_open_toolbox(self, _req, response: Trigger.Response) -> Trigger.Response:
        return self._run_service_seq(
            drawer_open_seq_v2(_STANDALONE_LAYER), "open_toolbox_l1", response
        )

    def _on_close_toolbox(self, _req, response: Trigger.Response) -> Trigger.Response:
        return self._run_service_seq(
            drawer_close_seq_v2(_STANDALONE_LAYER), "close_toolbox_l1", response
        )

    def _on_open_toolbox_l2(self, _req, response: Trigger.Response) -> Trigger.Response:
        return self._run_service_seq(
            drawer_open_seq_v2(1), "open_toolbox_l2", response
        )

    def _on_close_toolbox_l2(self, _req, response: Trigger.Response) -> Trigger.Response:
        return self._run_service_seq(
            drawer_close_seq_v2(1), "close_toolbox_l2", response
        )

    def _run_service_seq(
        self, seq: list, label: str, response: Trigger.Response
    ) -> Trigger.Response:
        if self._engine.estop_triggered:
            response.success = False
            response.message = "E-stop 래치 — estop_reset 후 재시도"
            return response
        if not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "다른 동작 진행 중"
            return response
        try:
            self.get_logger().info(f"[TAS] {label} 시작")
            self._engine.set_tcp()
            self._set_plc("moving")
            ok = self._engine.run_sequence(seq)
            if ok:
                response.success = True
                response.message = f"{label} 완료"
                self._set_plc("idle")
            else:
                response.success = False
                response.message = f"{label} 실패"
                self._set_plc("error")
        except Exception as exc:
            self.get_logger().error(f"[TAS] {label} 예외: {exc}")
            response.success = False
            response.message = str(exc)
            self._set_plc("error")
        finally:
            self._action_lock.release()
        return response

    def _on_estop(self, _req, response: Trigger.Response) -> Trigger.Response:
        """S-3: E-stop — DSR 즉시 정지 + 래치. 고우선 콜백 그룹에서 실행."""
        self.get_logger().error("[TAS] E-STOP 요청 수신")
        self._estop_latch.set()
        self._engine.request_stop()
        self._publish_status(is_moving=False)
        self._set_plc("e_stop")

        # DSR 즉시 정지
        try:
            if self._stop_cli.service_is_ready():
                req = MoveStop.Request()
                req.stop_type = 0  # 감속 정지
                self._stop_cli.call_async(req)
                self.get_logger().error("[TAS] move_stop 요청 완료")
            else:
                self.get_logger().error("[TAS] move_stop 서비스 미준비")
        except Exception as exc:
            self.get_logger().error(f"[TAS] move_stop 예외: {exc}")

        # DB 로그 (best-effort)
        self._log_event_async("e_stop", "E-stop 버튼 트리거")

        response.success = True
        response.message = "E-stop 래치 활성 — estop_reset 으로만 해제"
        return response

    def _on_estop_reset(self, _req, response: Trigger.Response) -> Trigger.Response:
        """E-stop 래치 해제 — 운영자 명시적 확인 후만 허용."""
        self.get_logger().warn("[TAS] E-stop 래치 해제")
        grip_was_taken = self._grip_taken
        if grip_was_taken:
            self.get_logger().warn("[TAS] estop_reset: grip_taken 잔존 — 공구 수동 확인 후 재호출 필요")
        self._engine.reset_estop()
        self._estop_latch.clear()
        self._grip_taken = False
        if grip_was_taken:
            self._set_plc("error")
        else:
            self._set_plc("idle")
            self._publish_status(is_moving=False)
        self._log_event_async("e_stop_reset", "운영자 E-stop 래치 해제")
        response.success = True
        response.message = "E-stop 래치 해제 완료"
        return response

    # ── 시퀀스 실행 ─────────────────────────────────────────────────────────

    def _run_sequence(
        self, steps: list, goal_handle=None, action_class=None
    ) -> bool:
        total = len(steps)
        for i, step in enumerate(steps):
            if self._estop_latch.is_set():
                self.get_logger().warn(f"[TAS] E-stop 래치 — step {i+1} 중단")
                return False
            if goal_handle is not None and goal_handle.is_cancel_requested:
                return False

            self.get_logger().info(f"  step {i+1}/{total}: {step.kind.name}")

            if goal_handle is not None and action_class is not None:
                fb = action_class.Feedback()
                fb.phase = step.kind.name
                fb.progress = float(i) / float(total)
                goal_handle.publish_feedback(fb)

            ok = self._exec_step(step)
            if not ok:
                self.get_logger().error(f"  step {i+1} 실패 — 중단")
                return False

            # step.marker("pick"/"place")가 설정된 step은 실행 직후 추가
            # feedback을 발행한다 — orchestrator가 이를 받아 DB 상태를
            # 물리적 집기/놓기 시점에 맞춰 전이시킨다.
            if goal_handle is not None and action_class is not None and step.marker:
                fb = action_class.Feedback()
                fb.phase = step.marker
                fb.progress = float(i + 1) / float(total)
                goal_handle.publish_feedback(fb)
        return True

    def _run_sequence_handover(self, steps: list, goal_handle) -> bool:
        """PlaceOnHand 전용 시퀀스 실행 — GRIP_TOOL/릴리즈 시점에 _grip_taken 추적."""
        self._current_goal_handle = goal_handle
        total = len(steps)
        for i, step in enumerate(steps):
            if self._estop_latch.is_set():
                self.get_logger().warn(f"[TAS] E-stop 래치 — step {i+1} 중단")
                return False
            if goal_handle.is_cancel_requested:
                return False

            self.get_logger().info(f"  step {i+1}/{total}: {step.kind.name}")

            fb = PlaceOnHand.Feedback()
            fb.phase = step.kind.name
            fb.progress = float(i) / float(total)
            goal_handle.publish_feedback(fb)

            ok = self._exec_step(step)

            # GRIP 스텝 결과로 _grip_taken 갱신 (PlaceOnHand 전용)
            if step.kind == StepKind.GRIP and ok and step.pulse is not None:
                if step.pulse >= 600:
                    self._grip_taken = True   # 공구 파지 성공
                else:
                    self._grip_taken = False  # 그리퍼 열기 성공 (릴리즈)

            if not ok:
                self.get_logger().error(f"  step {i+1} 실패 — 중단")
                return False

            if step.marker:
                fb = PlaceOnHand.Feedback()
                fb.phase = step.marker
                fb.progress = float(i + 1) / float(total)
                goal_handle.publish_feedback(fb)
        return True

    def _exec_step(self, step) -> bool:
        if step.kind == StepKind.MOVE_L_ABS:
            return self._movel(step, DR_MV_MOD_ABS)
        elif step.kind == StepKind.MOVE_L_REL:
            return self._movel(step, DR_MV_MOD_REL)
        elif step.kind in (StepKind.MOVE_J_ABS, StepKind.MOVE_J_REL):
            return self._movej(step)
        elif step.kind == StepKind.GRIP:
            return self._grip(step)
        elif step.kind == StepKind.WAIT:
            time.sleep(step.sec or 0.5)
            return True
        elif step.kind == StepKind.WAIT_VISION_TOP_XY:
            return self._exec_wait_vision_gripper_xy()
        elif step.kind == StepKind.MOVE_L_TOP_XY:
            return self._exec_move_l_top_xy()
        elif step.kind == StepKind.MOVE_L_TOOL_XYZ:
            return self._exec_move_l_tool_xyz()
        elif step.kind == StepKind.WAIT_HAND_POSE:
            return self._wait_hand_pose(timeout_override=step.sec)
        elif step.kind == StepKind.MOVE_L_HAND_RZ_APPROACH:
            return self._move_hand_rz_approach(handle_first=False)
        elif step.kind == StepKind.MOVE_L_HAND_PLACE:
            return self._move_hand_place(handle_first=False)
        elif step.kind == StepKind.MOVE_L_HAND_RZ_APPROACH_HANDLE:
            return self._move_hand_rz_approach(handle_first=True)
        elif step.kind == StepKind.MOVE_L_HAND_PLACE_HANDLE:
            return self._move_hand_place(handle_first=True)
        self.get_logger().warn(f"  알 수 없는 StepKind: {step.kind}")
        return False

    # ── 모션 헬퍼 (threading.Event 패턴 — executor 데드락 방지) ──────────────

    def _wait_future(self, fut: Any, timeout: float) -> bool:
        """future 완료를 blocking 없이 대기. spin_until_future_complete 사용 금지."""
        done = threading.Event()
        fut.add_done_callback(lambda _: done.set())
        return done.wait(timeout=timeout)

    def _movel(self, step, mode: int) -> bool:
        req = MoveLine.Request()
        req.pos = [float(v) for v in step.pose]
        # MoveLine vel/acc는 [병진, 회전] 2원소. Step은 병진 스칼라만 보유하므로
        # 회전 성분은 TaskWriter 실측 상수(VEL_R/ACC_R)로 고정한다.
        req.vel = [step.vel or VEL_L, VEL_R]
        req.acc = [step.acc or ACC_L, ACC_R]
        req.time = 0.0
        req.radius = 0.0
        req.ref = DR_BASE
        req.mode = mode
        req.blend_type = 0
        req.sync_type = 0
        fut = self._movel_cli.call_async(req)
        if not self._wait_future(fut, timeout=30.0):
            self.get_logger().error(f"  move_line 타임아웃: pos={step.pose}")
            return False
        res = fut.result()
        time.sleep(0.2)
        ok = bool(res and res.success)
        if not ok:
            self.get_logger().error(f"  move_line 실패: pos={step.pose}")
        return ok

    def _movej(self, step) -> bool:
        req = MoveJoint.Request()
        req.pos = [float(v) for v in step.pose]
        req.vel = step.vel or 12.0
        req.acc = step.acc or 20.0
        req.time = 0.0
        req.radius = 0.0
        req.mode = DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type = 0
        fut = self._movej_cli.call_async(req)
        if not self._wait_future(fut, timeout=20.0):
            self.get_logger().error(f"  move_joint 타임아웃: pos={step.pose}")
            return False
        res = fut.result()
        time.sleep(0.2)
        ok = bool(res and res.success)
        if not ok:
            self.get_logger().error(f"  move_joint 실패: pos={step.pose}")
        return ok

    def _grip(self, step) -> bool:
        pulse = step.pulse if step.pulse is not None else 0
        current = _GRIP_CURRENT if pulse >= PULSE_RELEASE else 0
        req = GripperSetPosition.Request()
        req.position = pulse
        req.current = current
        req.timeout_sec = 0.0
        fut = self._gripper_cli.call_async(req)
        if not self._wait_future(fut, timeout=5.0):
            self.get_logger().error("  gripper 타임아웃")
            return False
        res = fut.result()
        if res is None or not res.success:
            msg = res.message if res else "timeout"
            self.get_logger().error(f"  gripper 실패: {msg}")
            return False
        time.sleep(0.1)
        return True

    def _log_event_async(self, event_type: str, notes: str) -> None:
        try:
            if not self._log_event_cli.service_is_ready():
                return
            req = LogEvent.Request()
            req.tool_id = ""
            req.event_type = event_type
            req.track = "A"
            req.notes = notes
            self._log_event_cli.call_async(req)
        except Exception:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = ExecutePhaseServer()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    except RuntimeError as e:
        print(f"[TAS] 초기화 실패: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
