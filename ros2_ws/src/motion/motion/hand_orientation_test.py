"""hand_orientation_test.py — 손 방향(rz) 추적 검증 테스트 노드.

홈자세에서 rz만 손 방향에 맞게 회전하는지 확인한다.
위치는 TEST_POSE_XYZ 고정, 방향의 rz만 /hand/pose 에서 읽어 적용.

사용법:
  ros2 run motion hand_orientation_test

키보드:
  g + Enter → 손 rz 로 회전 이동
  h + Enter → 홈 복귀
  q + Enter → 종료
"""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from dsr_msgs2.srv import MoveLine, MoveJoint

from scipy.spatial.transform import Rotation

_HANDOVER_CFG = next(
    p for p in Path(__file__).resolve().parents if (p / "config" / "handover.yaml").exists()
) / "config" / "handover.yaml"


def _load_rz_cal() -> tuple[float, float, float]:
    """(rz_cal_hand_yaw, rz_cal_robot_rz, rz_sign) 로드. 파일 없으면 기본값."""
    try:
        cfg = yaml.safe_load(_HANDOVER_CFG.read_text())["handover"]
        return (
            float(cfg.get("rz_cal_hand_yaw", 0.0)),
            float(cfg.get("rz_cal_robot_rz", 0.0)),
            float(cfg.get("rz_sign", 1.0)),
        )
    except Exception:
        return 0.0, 0.0, 1.0


_RZ_CAL_HAND_YAW, _RZ_CAL_ROBOT_RZ, _RZ_SIGN = _load_rz_cal()


# ── 고정 테스트 위치 (홈 근처 안전 지점) ─────────────────────────────────────
# [x, y, z, rx, ry, rz]  mm / deg — rz는 손 방향으로 덮어씀
TEST_POSE_XYZ = [373.0, 0.0, 245.0]   # mm, base_link 기준
TEST_RX = 8.39                          # deg — 홈 자세 기준 rx (실측)
TEST_RY = -180.0                        # deg — 홈 자세 기준 ry (gripper down)

HOME_J_DEG   = [0.0, 0.0, 90.0, 0.0, 90.0, 0.0]
VEL_L, ACC_L = 20.0, 40.0   # mm/s — 테스트용 느린 속도
VEL_R, ACC_R = 10.0, 20.0   # deg/s
ROBOT_NS     = "dsr01"


def _quat_to_rz(qx: float, qy: float, qz: float, qw: float) -> float:
    """쿼터니언 → Doosan rz (deg).

    rz = (hand_yaw - rz_cal_hand_yaw) * rz_sign + rz_cal_robot_rz
    캘리브레이션 값은 config/handover.yaml 의 rz_cal_* 키로 조정.
    """
    raw_yaw = float(Rotation.from_quat([qx, qy, qz, qw]).as_euler("xyz", degrees=True)[2])
    return (raw_yaw - _RZ_CAL_HAND_YAW) * _RZ_SIGN + _RZ_CAL_ROBOT_RZ


class HandOrientationTestNode(Node):

    def __init__(self) -> None:
        super().__init__("hand_orientation_test")

        self.declare_parameter("robot_ns", ROBOT_NS)
        ns = self.get_parameter("robot_ns").get_parameter_value().string_value
        p  = f"/{ns}"

        self._hand_pose: Optional[PoseStamped] = None
        self._hand_ready: bool = False
        self._lock = threading.Lock()

        self.create_subscription(PoseStamped, "/hand/pose",  self._on_pose,  qos_profile_sensor_data)
        self.create_subscription(Bool,        "/hand/ready", self._on_ready, qos_profile_sensor_data)

        self._movel_cli = self.create_client(MoveLine,  f"{p}/motion/move_line")
        self._movej_cli = self.create_client(MoveJoint, f"{p}/motion/move_joint")

        self.get_logger().info("[OrientTest] move_line 대기...")
        if not self._movel_cli.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("move_line 서비스 없음 — bringup 확인")
        self.get_logger().info("[OrientTest] 연결됨")

        self.create_timer(1.0, self._print_status)

        t = threading.Thread(target=self._keyboard_loop, daemon=True)
        t.start()

        self.get_logger().info(
            "[OrientTest] 준비\n"
            "  손을 카메라에 보여준 후\n"
            "    g + Enter : rz 적용해서 이동\n"
            "    h + Enter : 홈 복귀\n"
            "    q + Enter : 종료"
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._hand_pose = msg

    def _on_ready(self, msg: Bool) -> None:
        with self._lock:
            self._hand_ready = msg.data

    def _print_status(self) -> None:
        with self._lock:
            pose  = self._hand_pose
            ready = self._hand_ready
        if pose is None:
            self.get_logger().info("[OrientTest] 손 감지 없음")
            return
        o  = pose.pose.orientation
        raw_yaw = float(Rotation.from_quat([o.x, o.y, o.z, o.w]).as_euler("xyz", degrees=True)[2])
        rz = _quat_to_rz(o.x, o.y, o.z, o.w)
        self.get_logger().info(
            f"[OrientTest] ready={ready}  "
            f"raw_yaw={raw_yaw:+.1f}  rz={rz:+.1f} deg  "
            f"(cal: hand_yaw={_RZ_CAL_HAND_YAW} sign={_RZ_SIGN:+.0f} robot_rz={_RZ_CAL_ROBOT_RZ})"
        )

    def _keyboard_loop(self) -> None:
        while rclpy.ok():
            try:
                key = input().strip().lower()
            except EOFError:
                break
            if   key == "g": self._do_orient()
            elif key == "h": self._do_home()
            elif key == "q":
                rclpy.shutdown()
                break

    def _do_orient(self) -> None:
        with self._lock:
            pose  = self._hand_pose
            ready = self._hand_ready

        if pose is None:
            self.get_logger().warn("[OrientTest] 손 감지 없음")
            return

        o  = pose.pose.orientation
        rz = _quat_to_rz(o.x, o.y, o.z, o.w)

        dsr_pos = [*TEST_POSE_XYZ, TEST_RX, TEST_RY, rz]

        self.get_logger().info(
            f"[OrientTest] → 이동\n"
            f"  pos = {TEST_POSE_XYZ} mm\n"
            f"  rx={TEST_RX:.2f}  ry={TEST_RY:.2f}  rz={rz:+.2f} deg  ← 손 yaw 적용\n"
            f"  /hand/ready={ready}  quat=({o.x:.3f},{o.y:.3f},{o.z:.3f},{o.w:.3f})"
        )

        req = MoveLine.Request()
        req.pos       = [float(v) for v in dsr_pos]
        req.vel       = [VEL_L, VEL_R]
        req.acc       = [ACC_L, ACC_R]
        req.time      = 0.0
        req.radius    = 0.0
        req.ref       = 0   # DR_BASE
        req.mode      = 0   # DR_MV_MOD_ABS
        req.blend_type = 0
        req.sync_type  = 0

        fut = self._movel_cli.call_async(req)
        deadline = time.monotonic() + 15.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        if fut.done() and fut.result() and fut.result().success:
            self.get_logger().info(f"[OrientTest] ✓ 완료 — rz={rz:+.1f} deg 적용됨")
        else:
            self.get_logger().error("[OrientTest] ✗ 실패 또는 타임아웃")

    def _do_home(self) -> None:
        self.get_logger().info("[OrientTest] → 홈 복귀")
        req = MoveJoint.Request()
        req.pos       = [float(v) for v in HOME_J_DEG]
        req.vel       = 20.0
        req.acc       = 30.0
        req.time      = 0.0
        req.radius    = 0.0
        req.mode      = 0
        req.blend_type = 0
        req.sync_type  = 0

        fut = self._movej_cli.call_async(req)
        deadline = time.monotonic() + 15.0
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.05)

        ok = fut.done() and fut.result() and fut.result().success
        self.get_logger().info("[OrientTest] " + ("✓ 홈 완료" if ok else "✗ 홈 실패"))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandOrientationTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
