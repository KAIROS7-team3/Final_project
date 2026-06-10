"""Scene context builder — Track A/B용 (Track C 미사용).

Subscribe : /vision/tracked_poses  (vision_msgs/Detection3DArray)
Publish   : /vision/scene_context  (std_msgs/String, JSON)
            /vision/slot_top_pose  (geometry_msgs/PointStamped)
              — 빈 슬롯 중 첫 번째의 base_link XYZ [m]. 빈 슬롯 없으면 발행 중단.

Gemma 4 의도 분류 노드(voice/)와 orchestrator(BT)가 이 JSON을 소비한다.
tool 위치를 toolbox.yaml 슬롯 좌표와 대조해 슬롯 점유 정보를 생성한다.

scene JSON 구조:
{
  "stamp":         "<ISO8601 UTC>",
  "calibrated":    bool,       # hand_eye.yaml 캘리브레이션 여부
  "tools_visible": [
    {
      "tool_id":    str,
      "confidence": float,
      "position":   {"x": m, "y": m, "z": m},  # base_link [m]
      "slot":       [row, col] | null            # 슬롯 미할당이면 null
    }, ...
  ],
  "slots_occupied": [[row, col], ...],
  "slots_empty":    [[row, col], ...],
  "summary":        str        # Gemma 4 프롬프트용 한 줄 요약
}
"""
from __future__ import annotations

import json
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from vision_msgs.msg import Detection3DArray

from vision.hand_eye_loader import HandEyeNotCalibratedError, load_transform

# interfaces.md §4
_QOS_BEST_EFFORT_5 = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)
_QOS_RELIABLE_1 = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE)

_TOOLBOX_PATH = Path("config/toolbox.yaml")
_HAND_EYE_PATH = Path("config/hand_eye.yaml")
_SLOT_ASSIGN_MARGIN = 1.5   # 슬롯 반경의 몇 배까지 허용 (1.5 × 반 슬롯 크기)


@dataclass
class _SlotInfo:
    row: int
    col: int
    center: np.ndarray   # base_link [m]
    half_w: float        # x 반폭 [m]
    half_h: float        # y 반폭 [m]


def _load_slots() -> list[_SlotInfo]:
    with _TOOLBOX_PATH.open() as f:
        cfg = yaml.safe_load(f)["toolbox"]

    ox = cfg["origin"]["x"]
    oy = cfg["origin"]["y"]
    oz = cfg["origin"]["z"]
    sw = cfg["slot_size"]["x"]
    sh = cfg["slot_size"]["y"]
    rows = cfg["grid_rows"]
    cols = cfg["grid_cols"]

    slots = []
    for r in range(rows):
        for c in range(cols):
            cx = ox + c * sw + sw / 2
            cy = oy + r * sh + sh / 2
            slots.append(_SlotInfo(
                row=r, col=c,
                center=np.array([cx, cy, oz]),
                half_w=sw / 2,
                half_h=sh / 2,
            ))
    return slots


def _assign_slot(pos: np.ndarray, slots: list[_SlotInfo]) -> list[int] | None:
    """가장 가까운 슬롯 반환. 허용 범위 밖이면 None."""
    if not slots:
        return None
    dists = [np.linalg.norm(pos[:2] - s.center[:2]) for s in slots]
    idx = int(np.argmin(dists))
    s = slots[idx]
    threshold = _SLOT_ASSIGN_MARGIN * max(s.half_w, s.half_h)
    if dists[idx] <= threshold:
        return [s.row, s.col]
    return None


class ContextBuilder(Node):
    """tracked_poses → scene JSON 발행 노드."""

    def __init__(self) -> None:
        super().__init__("context_builder")

        self._slots = _load_slots()
        self._all_slots = [[s.row, s.col] for s in self._slots]
        self._calibrated = self._check_calibration()

        # interfaces.md §4: Best Effort / depth 5
        self.create_subscription(
            Detection3DArray, "/vision/tracked_poses", self._on_poses, _QOS_BEST_EFFORT_5
        )
        # interfaces.md §4: Reliable / depth 1 (orchestrator 소비)
        self._pub = self.create_publisher(String, "/vision/scene_context", _QOS_RELIABLE_1)
        self._slot_top_pub = self.create_publisher(
            PointStamped, "/vision/slot_top_pose", _QOS_BEST_EFFORT_5
        )

        self.get_logger().info(
            f"[context_builder] ready - slots={len(self._slots)} "
            f"calibrated={self._calibrated}"
        )
        if not self._calibrated:
            self.get_logger().warn(
                "[context_builder] hand-eye 미캘리브레이션 — "
                "슬롯 할당이 부정확할 수 있음. Phase 1 캘리브레이션 완료 후 재기동 권장."
            )

    def _check_calibration(self) -> bool:
        try:
            load_transform(_HAND_EYE_PATH)
            return True
        except (HandEyeNotCalibratedError, FileNotFoundError):
            return False

    def _on_poses(self, msg: Detection3DArray) -> None:
        tools_visible = []
        occupied: list[list[int]] = []

        for det in msg.detections:
            tool_id = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            pos = np.array([
                det.results[0].pose.pose.position.x,
                det.results[0].pose.pose.position.y,
                det.results[0].pose.pose.position.z,
            ])

            slot = _assign_slot(pos, self._slots)
            if slot and slot not in occupied:
                occupied.append(slot)

            tools_visible.append({
                "tool_id": tool_id,
                "confidence": round(score, 3),
                "position": {
                    "x": round(float(pos[0]), 4),
                    "y": round(float(pos[1]), 4),
                    "z": round(float(pos[2]), 4),
                },
                "slot": slot,
            })

        empty = [s for s in self._all_slots if s not in occupied]
        self._publish_slot_top_pose(msg.header, empty)
        summary = self._build_summary(tools_visible, empty)

        scene: dict[str, Any] = {
            "stamp": datetime.now(timezone.utc).isoformat(),
            "calibrated": self._calibrated,
            "tools_visible": tools_visible,
            "slots_occupied": occupied,
            "slots_empty": empty,
            "summary": summary,
        }

        out = String()
        out.data = json.dumps(scene, ensure_ascii=False)
        self._pub.publish(out)

        self.get_logger().debug(
            f"[context_builder] scene published - visible={len(tools_visible)} "
            f"occupied={len(occupied)} empty={len(empty)}"
        )

    @staticmethod
    def _build_summary(tools: list[dict], empty_slots: list[list[int]]) -> str:
        if not tools:
            return f"공구함 비어 있음 (빈 슬롯 {len(empty_slots)}개)"
        parts = []
        for t in tools:
            slot_str = f"슬롯({t['slot'][0]},{t['slot'][1]})" if t["slot"] else "슬롯 미할당"
            parts.append(f"{t['tool_id']}@{slot_str}")
        return ", ".join(parts)


def main() -> None:
    rclpy.init()
    node = ContextBuilder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
