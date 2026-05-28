"""멀티 오브젝트 트래커 노드 (Track A/B).

Subscribe : /vision/tool_poses    (vision_msgs/Detection3DArray)
Publish   : /vision/tracked_poses (vision_msgs/Detection3DArray)

공구함은 정적 환경이므로 간단한 class-aware Euclidean 트래커를 사용한다.
  - 동일 tool_id + 거리 기준으로 검출-트랙 연결
  - EMA로 위치 평활화 (진동 억제)
  - min_hits 연속 검출 후 트랙 확정 (false positive 제거)
  - max_misses 연속 미검출 시 트랙 소멸 (파지 후 사라진 공구 처리)

파라미터: config/vision.yaml tracker 섹션
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from vision_msgs.msg import Detection3D, Detection3DArray

_QOS_BEST_EFFORT_5 = QoSProfile(
    depth=5,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

_CONFIG_PATH = Path("config/vision.yaml")


@dataclass
class _Track:
    tool_id: str
    position: np.ndarray        # base_link 좌표 [m], EMA 평활화 값
    score: float
    hits: int = 1               # 연속 검출 횟수
    misses: int = 0             # 연속 미검출 횟수
    confirmed: bool = False
    _last_det: Detection3D | None = field(default=None, repr=False)

    def update(self, pos: np.ndarray, score: float, alpha: float, det: Detection3D) -> None:
        self.position = alpha * pos + (1.0 - alpha) * self.position
        self.score = score
        self.hits += 1
        self.misses = 0
        self._last_det = det

    def mark_missed(self) -> None:
        self.misses += 1
        self.hits = 0


class TrackerNode(Node):
    """class-aware Euclidean 트래커 — 공구함 정적 환경 특화."""

    def __init__(self) -> None:
        super().__init__("tracker_node")

        cfg = self._load_cfg()
        self._min_hits: int = cfg["min_hits"]
        self._max_misses: int = cfg["max_misses"]
        self._max_dist: float = cfg["max_match_dist_m"]
        self._alpha: float = cfg["ema_alpha"]

        # tool_id → 트랙 목록 (동일 공구가 여러 슬롯에 있을 수 없으나, 일반성을 위해 list)
        self._tracks: dict[str, list[_Track]] = {}

        # interfaces.md §4: Best Effort / depth 5
        self.create_subscription(
            Detection3DArray, "/vision/tool_poses", self._on_poses, _QOS_BEST_EFFORT_5
        )
        self._pub = self.create_publisher(
            Detection3DArray, "/vision/tracked_poses", _QOS_BEST_EFFORT_5
        )

        self.get_logger().info(
            f"[tracker_node] ready - min_hits={self._min_hits} "
            f"max_misses={self._max_misses} max_dist={self._max_dist}m"
        )

    @staticmethod
    def _load_cfg() -> dict:
        with _CONFIG_PATH.open() as f:
            return yaml.safe_load(f)["tracker"]

    def _on_poses(self, msg: Detection3DArray) -> None:
        # 이번 프레임에서 매칭된 트랙 집합
        matched: set[int] = set()

        for det in msg.detections:
            tool_id = det.results[0].hypothesis.class_id
            score = det.results[0].hypothesis.score
            pos = np.array([
                det.bbox.center.position.x,
                det.bbox.center.position.y,
                det.bbox.center.position.z,
            ])

            tracks = self._tracks.setdefault(tool_id, [])
            best_idx, best_dist = self._find_closest(tracks, pos)

            if best_idx is not None and best_dist < self._max_dist:
                t = tracks[best_idx]
                t.update(pos, score, self._alpha, det)
                if not t.confirmed and t.hits >= self._min_hits:
                    t.confirmed = True
                    self.get_logger().info(f"[tracker_node] track confirmed - tool_id={tool_id}")
                matched.add((tool_id, best_idx))
            else:
                tracks.append(
                    _Track(tool_id=tool_id, position=pos.copy(), score=score, _last_det=det)
                )
                matched.add((tool_id, len(tracks) - 1))

        # 매칭 안 된 트랙 miss 처리
        for tool_id, tracks in self._tracks.items():
            for idx, t in enumerate(tracks):
                if (tool_id, idx) not in matched:
                    t.mark_missed()

        # 소멸 트랙 제거
        for tool_id in list(self._tracks):
            self._tracks[tool_id] = [
                t for t in self._tracks[tool_id] if t.misses <= self._max_misses
            ]
            if not self._tracks[tool_id]:
                del self._tracks[tool_id]

        # 확정 트랙만 발행
        out = Detection3DArray()
        out.header = msg.header
        for tracks in self._tracks.values():
            for t in tracks:
                if t.confirmed and t._last_det is not None:
                    smoothed = self._smooth_det(t)
                    out.detections.append(smoothed)

        self._pub.publish(out)

        if out.detections:
            self.get_logger().debug(
                f"[tracker_node] tracked={len(out.detections)} "
                + ", ".join(d.results[0].hypothesis.class_id for d in out.detections)
            )

    @staticmethod
    def _find_closest(
        tracks: list[_Track], pos: np.ndarray
    ) -> tuple[int | None, float]:
        if not tracks:
            return None, float("inf")
        dists = [np.linalg.norm(t.position - pos) for t in tracks]
        idx = int(np.argmin(dists))
        return idx, dists[idx]

    def _smooth_det(self, t: _Track) -> Detection3D:
        """EMA 평활화된 위치를 마지막 Detection3D에 반영해 반환."""
        det = t._last_det
        det.bbox.center.position.x = float(t.position[0])
        det.bbox.center.position.y = float(t.position[1])
        det.bbox.center.position.z = float(t.position[2])
        if det.results:
            det.results[0].pose.pose.position.x = float(t.position[0])
            det.results[0].pose.pose.position.y = float(t.position[1])
            det.results[0].pose.pose.position.z = float(t.position[2])
            det.results[0].hypothesis.score = t.score
        return det


def main() -> None:
    rclpy.init()
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
