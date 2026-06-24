"""toolbox_scan_node.py
════════════════════════════════════════════════════════════════
서랍 열기 → 스캔 자세 이동 → 그리퍼 캠 검출 수집 → 공구 XY 좌표 파일 저장.

사용법:
  ros2 run motion toolbox_scan --ros-args -p layer_id:=1   # 1층 (아랫층)
  ros2 run motion toolbox_scan --ros-args -p layer_id:=2   # 2층 (윗층)

출력:
  - 콘솔: tool_id별 평균 XY (BASE frame, m / mm)
  - 파일: config/toolbox_scan_YYYYMMDD_HHMMSS_layer<N>.yaml
    → toolbox.yaml 의 grasp_pose_base.x/y 에 수동 반영 후 커밋

전제:
  - tool_action_server 가 동작 중 (execute_phase 액션 서버)
  - gripper_marker_scan_node 가 동작 중 (/vision/tool_gripper_pose 발행)
  - yolo_node (gripper) 가 동작 중 (/vision/detections/gripper 발행)
"""
from __future__ import annotations

import os
import sys
import threading
import time
import yaml
from collections import defaultdict
from datetime import datetime
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from vision_msgs.msg import Detection2DArray

from interfaces.action import ExecutePhase

_DEFAULT_CONFIG = "/home/kg/assistant/config/toolbox.yaml"
_SCAN_COLLECT_SEC = 5.0   # 수집 시간 (초)
_MOVEJ_TIMEOUT    = 15.0  # MOVEJ 대기 타임아웃 (초)
_MIN_SAMPLES      = 3     # tool_id당 최소 유효 샘플 수
# 검출-포즈 타임스탬프 최대 허용 차이 (초) — ApproximateTimeSynchronizer slop과 유사
_STAMP_SLOP_SEC   = 0.15
_VEL_J = 30.0
_ACC_J = 60.0


class ToolboxScanNode(Node):
    """서랍 열기 → 스캔 자세 → 공구 XY 수집 → 결과 저장."""

    def __init__(self) -> None:
        super().__init__("toolbox_scan_node")

        self.declare_parameter("layer_id",        1)
        self.declare_parameter("robot_ns",        "dsr01")
        self.declare_parameter("scan_duration",   _SCAN_COLLECT_SEC)
        self.declare_parameter("config_path",     _DEFAULT_CONFIG)

        self._layer_id   = self.get_parameter("layer_id").get_parameter_value().integer_value
        self._layer_idx  = self._layer_id - 1    # 1-indexed → 0-indexed (ExecutePhase)
        robot_ns         = self.get_parameter("robot_ns").get_parameter_value().string_value
        self._scan_dur   = self.get_parameter("scan_duration").get_parameter_value().double_value
        self._cfg_path   = self.get_parameter("config_path").get_parameter_value().string_value

        if self._layer_id not in (1, 2):
            self.get_logger().error(f"[scan] layer_id={self._layer_id} 는 1 또는 2만 허용")
            raise ValueError("layer_id must be 1 or 2")

        self._scan_j_deg = self._load_scan_pose()

        # ── DSR MOVEJ 클라이언트 ─────────────────────────────────────────────
        self._movej_cli = self.create_client(
            MoveJoint, f"/{robot_ns}/motion/move_joint"
        )

        # ── ExecutePhase 액션 클라이언트 ────────────────────────────────────
        self._exec_cli = ActionClient(self, ExecutePhase, "execute_phase")

        # ── 비전 구독 ───────────────────────────────────────────────────────
        # Detection2DArray: 최신 유지 (타임스탬프 비교용)
        self._latest_det: Optional[Detection2DArray] = None
        self._det_stamp: float = 0.0
        self._det_lock = threading.Lock()

        # 수집 버퍼: {tool_id: [(x_m, y_m), ...]}
        self._buf: dict[str, list[tuple[float, float]]] = defaultdict(list)
        self._collecting = False

        self.create_subscription(
            Detection2DArray,
            "/vision/detections/gripper",
            self._on_detection,
            10,
        )
        self.create_subscription(
            PoseStamped,
            "/vision/tool_gripper_pose",
            self._on_pose,
            10,
        )

        self.get_logger().info(
            f"[scan] layer_id={self._layer_id} (idx={self._layer_idx}) "
            f"scan_j={self._scan_j_deg} dur={self._scan_dur}s"
        )

    # ── 설정 로딩 ──────────────────────────────────────────────────────────

    def _load_scan_pose(self) -> list[float]:
        try:
            with open(self._cfg_path, encoding="utf-8") as f:
                cfg = yaml.safe_load(f)
            j = cfg["vision_motion"]["gripper_cam_scan"]["fetch_j_deg"]
            self.get_logger().info(f"[scan] 스캔 자세 로드: {j}")
            return list(j)
        except Exception as e:
            self.get_logger().warn(f"[scan] toolbox.yaml 로드 실패, 기본값 사용: {e}")
            return [-30.1, 15.5, 74.7, 20.9, 101.2, -27.8]

    # ── 비전 콜백 ──────────────────────────────────────────────────────────

    def _on_detection(self, msg: Detection2DArray) -> None:
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._det_lock:
            self._latest_det = msg
            self._det_stamp  = stamp

    def _on_pose(self, msg: PoseStamped) -> None:
        if not self._collecting:
            return
        pose_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        with self._det_lock:
            det   = self._latest_det
            d_stamp = self._det_stamp

        if det is None or not det.detections:
            return
        if abs(pose_stamp - d_stamp) > _STAMP_SLOP_SEC:
            return  # 타임스탬프 불일치 — 페어 스킵

        best = max(det.detections, key=lambda d: d.results[0].hypothesis.score)
        tool_id = best.results[0].hypothesis.class_id
        if not tool_id:
            return

        x = msg.pose.position.x
        y = msg.pose.position.y
        self._buf[tool_id].append((x, y))

    # ── DSR / 액션 헬퍼 ───────────────────────────────────────────────────

    def _movej(self, j_deg: list[float]) -> bool:
        if not self._movej_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("[scan] move_joint 서비스 없음")
            return False
        req = MoveJoint.Request()
        req.pos       = [float(v) for v in j_deg]
        req.vel       = _VEL_J
        req.acc       = _ACC_J
        req.time      = 0.0
        req.radius    = 0.0
        req.mode      = 0   # ABS
        req.blend_type = 0
        req.sync_type  = 0
        fut = self._movej_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=_MOVEJ_TIMEOUT)
        res = fut.result()
        if res is None or not res.success:
            self.get_logger().error(f"[scan] MOVEJ 실패: {j_deg}")
            return False
        time.sleep(0.5)
        return True

    def _exec_phase(self, phase: str, layer_id: int = 0) -> bool:
        if not self._exec_cli.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"[scan] execute_phase 서버 없음 ({phase})")
            return False
        goal = ExecutePhase.Goal()
        goal.phase    = phase
        goal.tool_id  = ""
        goal.layer_id = layer_id
        done = threading.Event()
        result_holder: list = []

        def _on_result(future):
            result_holder.append(future.result())
            done.set()

        def _on_goal(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f"[scan] goal 거부됨: {phase}")
                done.set()
                return
            gh.get_result_async().add_done_callback(_on_result)

        self._exec_cli.send_goal_async(goal).add_done_callback(_on_goal)
        done.wait(timeout=120.0)

        if not result_holder:
            self.get_logger().error(f"[scan] {phase} 타임아웃")
            return False
        res = result_holder[0].result
        if not res.success:
            self.get_logger().error(f"[scan] {phase} 실패: {res.message}")
            return False
        self.get_logger().info(f"[scan] {phase} 완료")
        return True

    # ── 메인 스캔 시퀀스 ──────────────────────────────────────────────────

    def run(self) -> None:
        self.get_logger().info(f"[scan] ── {self._layer_id}층 스캔 시작 ──")

        # ① 서랍 열기
        if not self._exec_phase("open_drawer", layer_id=self._layer_idx):
            self.get_logger().error("[scan] 서랍 열기 실패 — 중단")
            return

        # ② 스캔 자세로 이동
        self.get_logger().info(f"[scan] 스캔 자세 이동 중: {self._scan_j_deg}")
        if not self._movej(self._scan_j_deg):
            self.get_logger().error("[scan] 스캔 자세 이동 실패 — 서랍 닫고 중단")
            self._exec_phase("close_drawer", layer_id=self._layer_idx)
            return

        # ③ 비전 데이터 수집
        self.get_logger().info(f"[scan] {self._scan_dur}초 수집 중...")
        self._collecting = True
        time.sleep(self._scan_dur)
        self._collecting = False

        # ④ 홈 복귀
        self._exec_phase("home")

        # ⑤ 서랍 닫기
        self._exec_phase("close_drawer", layer_id=self._layer_idx)

        # ⑥ 결과 집계 및 저장
        self._finalize()

    def _finalize(self) -> None:
        if not self._buf:
            self.get_logger().warn("[scan] 검출된 공구 없음 — 비전 노드 동작 확인 필요")
            return

        results: dict[str, dict] = {}
        for tool_id, samples in self._buf.items():
            if len(samples) < _MIN_SAMPLES:
                self.get_logger().warn(
                    f"[scan] {tool_id}: 샘플 부족 ({len(samples)} < {_MIN_SAMPLES}) — 제외"
                )
                continue
            avg_x = sum(s[0] for s in samples) / len(samples)
            avg_y = sum(s[1] for s in samples) / len(samples)
            results[tool_id] = {
                "x_m": round(avg_x, 5),
                "y_m": round(avg_y, 5),
                "samples": len(samples),
                "x_mm": round(avg_x * 1000.0, 2),
                "y_mm": round(avg_y * 1000.0, 2),
            }

        if not results:
            self.get_logger().warn("[scan] 유효 샘플 없음")
            return

        # 콘솔 출력
        self.get_logger().info("═" * 60)
        self.get_logger().info(f"[scan] {self._layer_id}층 스캔 결과")
        self.get_logger().info("─" * 60)
        for tid, r in results.items():
            self.get_logger().info(
                f"  {tid:<30s} x={r['x_m']:.5f}m ({r['x_mm']:.2f}mm) "
                f"y={r['y_m']:.5f}m ({r['y_mm']:.2f}mm)  [n={r['samples']}]"
            )
        self.get_logger().info("═" * 60)

        # 파일 저장
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_path = os.path.join(
            os.path.dirname(self._cfg_path),
            f"toolbox_scan_{ts}_layer{self._layer_id}.yaml",
        )
        out = {
            "scan_meta": {
                "layer_id": self._layer_id,
                "timestamp": ts,
                "scan_duration_sec": self._scan_dur,
                "note": "grasp_pose_base.x/y 에 수동 반영 후 커밋",
            },
            "tools": [
                {
                    "tool_id": tid,
                    "grasp_pose_base": {"x": r["x_m"], "y": r["y_m"]},
                    "samples": r["samples"],
                }
                for tid, r in results.items()
            ],
        }
        with open(out_path, "w", encoding="utf-8") as f:
            yaml.dump(out, f, allow_unicode=True, default_flow_style=False)
        self.get_logger().info(f"[scan] 결과 저장: {out_path}")
        self.get_logger().info(
            "[scan] ⚠️  toolbox.yaml 의 grasp_pose_base.x/y 에 수동 반영 후 커밋하세요."
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor()
    try:
        node = ToolboxScanNode()
        executor.add_node(node)

        # spin + run을 별도 스레드로 분리
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()

        node.run()

    except (ValueError, RuntimeError) as e:
        if node:
            node.get_logger().error(f"[scan] 초기화 실패: {e}")
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        if node:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except RuntimeError:
            pass


if __name__ == "__main__":
    main()
