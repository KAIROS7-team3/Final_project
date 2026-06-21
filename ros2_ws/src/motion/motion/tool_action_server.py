"""tool_action_server.py
────────────────────────
영구 모션 액션 서버 — PlaceAtStaging(fetch) / ReturnToSlot(return) 액션 +
home / estop / estop_reset 서비스.

TCP GripperDA_v1 Z+160 상주 설정. (이전 데모 실패 원인 = TCP 미적용)
/robot/status (RobotStatus) 단일 발행자 → S-7 명령차단 복구.
E-stop: move_stop + 래치 (servo-off 미적용, S-3).

설계 원칙:
  - MultiThreadedExecutor + ReentrantCallbackGroup (일반) /
    MutuallyExclusiveCallbackGroup (E-stop, 고우선 독립)
  - _movel/_movej/_grip 은 threading.Event 패턴 사용
    (spin_until_future_complete 로 executor 데드락 방지)
  - E-stop 래치: set 이후 모든 신규 액션/home 즉시 거부
  - unit_actions/toolbox_motion 은 ROS2 의존 없음 — sys.path 로 직접 import
"""

from __future__ import annotations

import math
import os
import pathlib
import sys
import threading
import time
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
from interfaces.action import PlaceAtStaging, PlaceOnHand, ReturnToSlot
from interfaces.msg import RobotStatus
from interfaces.srv import GripperSetPosition, LogEvent
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def _add_unit_actions_to_path() -> None:
    """레포 루트에서 unit_actions/ 를 sys.path에 추가한다 (toolbox_seq_runner 동일 패턴)."""
    candidates: list[str] = []
    env_root = os.environ.get("FINAL_PROJECT_ROOT")
    if env_root:
        candidates.append(env_root)
    here = os.path.dirname(os.path.abspath(__file__))
    while True:
        candidates.append(here)
        parent = os.path.dirname(here)
        if parent == here:
            break
        here = parent
    for root in candidates:
        if os.path.isdir(os.path.join(root, "unit_actions")):
            if root not in sys.path:
                sys.path.insert(0, root)
            return
    raise RuntimeError(
        "unit_actions 경로를 찾을 수 없습니다. "
        "FINAL_PROJECT_ROOT 환경변수로 레포 루트를 지정하세요."
    )


_add_unit_actions_to_path()

from unit_actions.toolbox_motion import (  # noqa: E402
    ACC_L,
    ACC_R,
    PULSE_RELEASE,
    StepKind,
    VEL_L,
    VEL_R,
    drawer_close_seq,
    drawer_open_seq,
    full_socket_fetch_seq,
    full_socket_return_seq,
    handover_fetch_seq,
    handover_fetch_handle_first_seq,
    home_seq,
)
from unit_actions.visual_servoing import ToolPose  # noqa: E402

# 그리퍼 파지력 (TaskWriter SubRoutine 실측) — pulse > PULSE_RELEASE 일 때만 인가
_GRIP_CURRENT: int = 400

DR_BASE = 0
DR_MV_MOD_ABS = 0
DR_MV_MOD_REL = 1

_QOS_ROBOT_STATUS = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)
_TCP_NAME = "GripperDA_v1"
_TCP_POS = [0.0, 0.0, 160.0, 0.0, 0.0, 0.0]

# socket_19mm 보관 서랍 (full_socket_fetch_seq/full_socket_return_seq 와 동일 layer)
_TOOLBOX_LAYER = 1

# 핸드오버 속도 (S-6: approach_action_scale=0.2 기준)
_HANDOVER_VEL_L: float = 10.0   # mm/s  (VEL_L 50 × 0.2)
_HANDOVER_ACC_L: float = 40.0
_HANDOVER_VEL_R: float = 5.0    # deg/s
_HANDOVER_ACC_R: float = 20.0

# 핸드오버 고정 rx/ry (hand_orientation_test.py 실측 — gripper down, 홈 기준)
_HANDOVER_RX: float = 8.39
_HANDOVER_RY: float = -180.0


def _load_yaml_cfg(filename: str) -> dict:
    """레포 루트에서 config/<filename> 을 찾아 로드한다."""
    here = pathlib.Path(__file__).resolve()
    for p in here.parents:
        cfg = p / "config" / filename
        if cfg.exists():
            return yaml.safe_load(cfg.read_text())
    return {}


class ToolActionServer(Node):
    """PlaceAtStaging(fetch) / ReturnToSlot(return) 영구 액션 서버."""

    def __init__(self) -> None:
        super().__init__("tool_action_server")

        self.declare_parameter("robot_ns", "dsr01")
        ns = self.get_parameter("robot_ns").get_parameter_value().string_value
        p = f"/{ns}"

        # ── 콜백 그룹 분리 ─────────────────────────────────────────────────
        self._normal_cbg = ReentrantCallbackGroup()        # 액션·home
        self._estop_cbg = MutuallyExclusiveCallbackGroup()  # E-stop (독립)

        # ── DSR 서비스 클라이언트 ──────────────────────────────────────────
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

        # ── 필수 서비스 대기 (시작 시 연결 확인) ──────────────────────────
        for cli, name in [
            (self._movel_cli, "move_line"),
            (self._movej_cli, "move_joint"),
            (self._gripper_cli, "gripper/set_position"),
        ]:
            if not cli.wait_for_service(timeout_sec=15.0):
                self.get_logger().error(f"[TAS] {name} 없음 — bringup 먼저 실행")
                raise RuntimeError(f"{name} 서비스 없음")
            self.get_logger().info(f"[TAS] {name} 연결됨")

        # ── /robot/status 발행자 ──────────────────────────────────────────
        self._status_pub = self.create_publisher(
            RobotStatus, "/robot/status", _QOS_ROBOT_STATUS
        )
        # ── /plc/system_state 발행자 ──────────────────────────────────────
        self._plc_pub = self.create_publisher(String, "/plc/system_state", 1)

        # ── 상태 ──────────────────────────────────────────────────────────
        self._estop_latch = threading.Event()  # E-stop 래치 (set 후 reset 전까지 모든 동작 거부)
        self._action_lock = threading.Lock()  # 동시 액션 방지

        # ── 핸드오버 config ───────────────────────────────────────────────
        self._h_cfg: dict = _load_yaml_cfg("handover.yaml").get("handover", {})
        self._toolbox_cfg: dict = _load_yaml_cfg("toolbox.yaml")

        # ── 핸드오버 손 상태 ──────────────────────────────────────────────
        self._hand_lock = threading.Lock()
        self._hand_pose: Optional[PoseStamped] = None
        self._hand_ready: bool = False
        self._current_tool_id: str = ""
        self._hand_approach_pos: Optional[list] = None  # approach 단계에서 저장, place에서 재사용

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

        # ── 액션 서버 ─────────────────────────────────────────────────────
        self._place_server = ActionServer(
            self,
            PlaceAtStaging,
            "place_at_staging",
            execute_callback=self._on_place_at_staging,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self._normal_cbg,
        )
        self._return_server = ActionServer(
            self,
            ReturnToSlot,
            "return_to_slot",
            execute_callback=self._on_return_to_slot,
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

        # ── 서비스 서버 ───────────────────────────────────────────────────
        self._home_srv = self.create_service(
            Trigger, "~/home", self._on_home, callback_group=self._normal_cbg
        )
        self._estop_srv = self.create_service(
            Trigger, "~/estop", self._on_estop, callback_group=self._estop_cbg
        )
        self._estop_reset_srv = self.create_service(
            Trigger, "~/estop_reset", self._on_estop_reset, callback_group=self._estop_cbg
        )
        self._open_toolbox_srv = self.create_service(
            Trigger, "~/open_toolbox", self._on_open_toolbox, callback_group=self._normal_cbg
        )
        self._close_toolbox_srv = self.create_service(
            Trigger, "~/close_toolbox", self._on_close_toolbox, callback_group=self._normal_cbg
        )

        # ── TCP 초기 설정 타이머 ──────────────────────────────────────────
        self._tcp_timer = self.create_timer(
            2.0, self._tcp_setup_once, callback_group=self._normal_cbg
        )

        self._publish_status(is_moving=False)
        self.get_logger().info("[TAS] tool_action_server 준비 완료")

    # ── TCP 설정 ────────────────────────────────────────────────────────────

    def _tcp_setup_once(self) -> None:
        self._tcp_timer.cancel()
        self._set_tcp()

    def _set_tcp(self) -> None:
        """TCP 등록 + 활성화 (fire-and-forget, chamjo 동일 패턴)."""
        try:
            if self._create_tcp_cli.service_is_ready():
                req = ConfigCreateTcp.Request()
                req.name = _TCP_NAME
                req.pos = _TCP_POS
                self._create_tcp_cli.call_async(req)
                self.get_logger().info(f"[TAS] TCP 등록 요청: {_TCP_NAME}")
            else:
                self.get_logger().warn("[TAS] config_create_tcp 미준비 — 건너뜀")
            if self._set_tcp_cli.service_is_ready():
                req2 = SetCurrentTcp.Request()
                req2.name = _TCP_NAME
                self._set_tcp_cli.call_async(req2)
                self.get_logger().info(f"[TAS] TCP 활성화 요청: {_TCP_NAME}")
            else:
                self.get_logger().warn("[TAS] set_current_tcp 미준비 — 건너뜀")
            time.sleep(0.3)
        except Exception as exc:
            self.get_logger().warn(f"[TAS] TCP 설정 예외 (무시): {exc}")

    # ── /robot/status 발행 ──────────────────────────────────────────────────

    def _publish_status(self, is_moving: bool) -> None:
        msg = RobotStatus()
        msg.is_moving = is_moving
        self._status_pub.publish(msg)

    def _set_plc(self, state: str) -> None:
        msg = String()
        msg.data = state
        self._plc_pub.publish(msg)

    # ── 액션 goal/cancel 핸들러 ─────────────────────────────────────────────

    def _goal_callback(self, _goal_request) -> GoalResponse:
        if self._estop_latch.is_set():
            self.get_logger().warn("[TAS] goal 거부 — E-stop 래치 활성")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, _goal_handle) -> CancelResponse:
        return CancelResponse.ACCEPT

    # ── 액션 실행 ───────────────────────────────────────────────────────────

    def _on_place_at_staging(self, goal_handle) -> PlaceAtStaging.Result:
        """fetch: 서랍 열기 → 소켓 파지 → 스테이징 거치 → 서랍 닫기."""
        result = PlaceAtStaging.Result()
        tool_id = goal_handle.request.tool_id

        if not self._action_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = "다른 동작 진행 중"
            return result

        try:
            self.get_logger().info(f"[TAS] fetch 시작: tool_id={tool_id}")
            self._set_tcp()
            self._publish_status(is_moving=True)
            self._set_plc("moving")

            steps = full_socket_fetch_seq()
            ok = self._run_sequence(steps, goal_handle, action_class=PlaceAtStaging)

            if ok and not goal_handle.is_cancel_requested:
                goal_handle.succeed()
                result.success = True
                result.message = f"fetch 완료: {tool_id}"
                self._set_plc("idle")
                self.get_logger().info(f"[TAS] fetch 완료: tool_id={tool_id}")
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "취소됨"
            else:
                goal_handle.abort()
                result.success = False
                result.message = f"fetch 실패: {tool_id}"
                self._set_plc("error")
        except Exception as exc:
            self.get_logger().error(f"[TAS] fetch 예외: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
            self._set_plc("error")
        finally:
            self._publish_status(is_moving=False)
            self._action_lock.release()

        return result

    def _on_return_to_slot(self, goal_handle) -> ReturnToSlot.Result:
        """return: 서랍 열기 → 스테이징 픽업 → 슬롯 반납 → 서랍 닫기."""
        result = ReturnToSlot.Result()
        tool_id = goal_handle.request.tool_id

        if not self._action_lock.acquire(blocking=False):
            goal_handle.abort()
            result.success = False
            result.message = "다른 동작 진행 중"
            return result

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

            ok = self._run_sequence(steps, goal_handle, action_class=PlaceOnHand)

            if ok and not goal_handle.is_cancel_requested:
                goal_handle.succeed()
                result.success = True
                result.message = f"place_on_hand 완료: {tool_id}"
                self._set_plc("idle")
                self.get_logger().info(f"[TAS] place_on_hand 완료: tool_id={tool_id}")
            elif goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = "취소됨"
            else:
                goal_handle.abort()
                result.success = False
                result.message = f"place_on_hand 실패: {tool_id}"
                self._set_plc("error")
        except Exception as exc:
            self.get_logger().error(f"[TAS] place_on_hand 예외: {exc}")
            goal_handle.abort()
            result.success = False
            result.message = str(exc)
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
        return (
            (raw_yaw - self._h_cfg.get("rz_cal_hand_yaw", 77.0))
            * self._h_cfg.get("rz_sign", -1.0)
            + self._h_cfg.get("rz_cal_robot_rz", 8.39)
        )

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
            (z, 0.0, 700.0, "z"),
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

    def _wait_hand_pose(self) -> bool:
        """WAIT_HAND_POSE: /hand/ready=True 될 때까지 대기. 타임아웃 시 False (staging fallback)."""
        timeout = self._h_cfg.get("detection_timeout_s", 5.0)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._estop_latch.is_set():
                return False
            pose, ready = self._get_hand_state()
            if pose is not None and ready:
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

        # hand_node는 m 단위 → DSR은 mm
        x_mm = pose.pose.position.x * 1000.0
        y_mm = pose.pose.position.y * 1000.0
        z_mm = pose.pose.position.z * 1000.0
        approach_h_mm = self._h_cfg.get("approach_height_m", 0.10) * 1000.0

        if handle_first:
            # finger_dir 방향(rz 각도)으로 tool_length/4 오프셋
            offset_mm = self._get_tool_length_mm(self._current_tool_id) / 4.0
            rz_rad = math.radians(rz)
            x_mm += math.cos(rz_rad) * offset_mm
            y_mm += math.sin(rz_rad) * offset_mm

        # place 단계에서 재사용할 위치 저장 (approach_height 제외)
        self._hand_approach_pos = [x_mm, y_mm, z_mm, _HANDOVER_RX, _HANDOVER_RY, rz]

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
        req.sync_type = 0
        fut = self._movel_cli.call_async(req)
        if not self._wait_future(fut, timeout=30.0):
            self.get_logger().error("[TAS] hand_approach 타임아웃")
            return False
        res = fut.result()
        time.sleep(0.2)
        return bool(res and res.success)

    def _move_hand_place(self) -> bool:
        """MOVE_L_HAND_PLACE(_HANDLE): approach 단계에서 저장한 손바닥 높이로 Z 수직 하강."""
        if self._hand_approach_pos is None:
            self.get_logger().error("[TAS] _hand_approach_pos 없음 — approach 먼저 실행 필요")
            return False

        dsr_pos = self._hand_approach_pos
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
        req.sync_type = 0
        fut = self._movel_cli.call_async(req)
        if not self._wait_future(fut, timeout=30.0):
            self.get_logger().error("[TAS] hand_place 타임아웃")
            return False
        res = fut.result()
        time.sleep(0.2)
        return bool(res and res.success)

    # ── 서비스 핸들러 ───────────────────────────────────────────────────────

    def _on_home(self, _req, response: Trigger.Response) -> Trigger.Response:
        if self._estop_latch.is_set():
            response.success = False
            response.message = "E-stop 래치 — estop_reset 후 재시도"
            return response
        if not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "다른 동작 진행 중"
            return response
        try:
            self.get_logger().info("[TAS] home 시작")
            self._set_tcp()
            self._publish_status(is_moving=True)
            self._set_plc("moving")
            ok = self._run_sequence(home_seq())
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
            self._publish_status(is_moving=False)
            self._action_lock.release()
        return response

    def _on_open_toolbox(self, _req, response: Trigger.Response) -> Trigger.Response:
        """공구함 서랍 열기 (단독 sub-task — fetch/return 없이 서랍만 개방).

        전제조건: 서랍이 닫힌 상태(LAYER{n}_APPROACH 진입 가능)여야 한다.
        이미 열린 서랍에 호출하면 GRIP_BOX 단계에서 충돌 위험 — 운영자가
        대시보드에서 서랍 상태를 육안 확인 후 호출할 것.
        """
        return self._run_toolbox_seq(
            seq=drawer_open_seq(_TOOLBOX_LAYER),
            label="open_toolbox",
            response=response,
        )

    def _on_close_toolbox(self, _req, response: Trigger.Response) -> Trigger.Response:
        """공구함 서랍 닫기 (단독 sub-task — fetch/return 없이 서랍만 폐쇄).

        전제조건: 서랍이 열린 상태(팔이 LAYER{n}_INNER 부근)여야 한다.
        이미 닫힌 서랍에 호출하면 충돌 위험 — 운영자가 대시보드에서
        서랍 상태를 육안 확인 후 호출할 것.
        """
        return self._run_toolbox_seq(
            seq=drawer_close_seq(_TOOLBOX_LAYER),
            label="close_toolbox",
            response=response,
        )

    def _run_toolbox_seq(
        self, seq: list, label: str, response: Trigger.Response
    ) -> Trigger.Response:
        if self._estop_latch.is_set():
            response.success = False
            response.message = "E-stop 래치 — estop_reset 후 재시도"
            return response
        if not self._action_lock.acquire(blocking=False):
            response.success = False
            response.message = "다른 동작 진행 중"
            return response
        try:
            self.get_logger().info(f"[TAS] {label} 시작")
            self._set_tcp()
            self._publish_status(is_moving=True)
            self._set_plc("moving")
            ok = self._run_sequence(seq)
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
            self._publish_status(is_moving=False)
            self._action_lock.release()
        return response

    def _on_estop(self, _req, response: Trigger.Response) -> Trigger.Response:
        """E-stop: move_stop + 래치. 고우선 콜백 그룹(estop_cbg)에서 실행."""
        self.get_logger().error("[TAS] E-STOP 요청 수신")
        self._estop_latch.set()
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
        self._estop_latch.clear()
        self._set_plc("idle")
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

    def _exec_step(self, step) -> bool:
        if step.kind == StepKind.MOVE_L_ABS:
            return self._movel(step, DR_MV_MOD_ABS)
        elif step.kind == StepKind.MOVE_L_REL:
            return self._movel(step, DR_MV_MOD_REL)
        elif step.kind in (StepKind.MOVE_J_ABS, StepKind.MOVE_J_REL):
            return self._movej(step)
        elif step.kind == StepKind.GRIP:
            ok = self._grip(step)
            if ok and step.pulse is not None:
                if step.pulse >= 600:
                    self._grip_taken = True   # 공구 파지 성공
                else:
                    self._grip_taken = False  # 그리퍼 열기 성공 (릴리즈)
            return ok
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
            return self._wait_hand_pose()
        elif step.kind == StepKind.MOVE_L_HAND_RZ_APPROACH:
            return self._move_hand_rz_approach(handle_first=False)
        elif step.kind == StepKind.MOVE_L_HAND_PLACE:
            return self._move_hand_place()
        elif step.kind == StepKind.MOVE_L_HAND_RZ_APPROACH_HANDLE:
            return self._move_hand_rz_approach(handle_first=True)
        elif step.kind == StepKind.MOVE_L_HAND_PLACE_HANDLE:
            return self._move_hand_place()
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
        current = _GRIP_CURRENT if pulse > PULSE_RELEASE else 0
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
        node = ToolActionServer()
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
