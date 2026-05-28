"""YOLOv11s 공구 검출 노드 (Track A/B).

Subscribe : /d455f/color/image_raw        (sensor_msgs/Image)
Publish   : /vision/detections            (vision_msgs/Detection2DArray)
            /vision/debug/annotated       (sensor_msgs/Image, debug 시에만)

config/vision.yaml의 yolo.model_path가 null이면 추론 없이 대기.
Phase 2 파인튜닝 완료 후 model_path 기입 → 재기동으로 활성화.
"""
from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

_QOS_BEST_EFFORT_10 = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

_CONFIG_PATH = Path("config/vision.yaml")


def _load_cfg() -> dict:
    with _CONFIG_PATH.open() as f:
        return yaml.safe_load(f)


class YoloNode(Node):
    """YOLOv11s 기반 9종 공구 탑뷰 검출 노드."""

    def __init__(self) -> None:
        super().__init__("yolo_node")

        cfg = _load_cfg()
        yolo_cfg = cfg["yolo"]
        debug_cfg = cfg["debug"]

        self._conf: float = yolo_cfg["confidence_threshold"]
        self._iou: float = yolo_cfg["iou_threshold"]
        self._device: str = yolo_cfg["device"]
        self._class_names: list[str] = yolo_cfg["class_names"]
        self._publish_debug: bool = debug_cfg["publish_annotated_image"]

        self._model = self._load_model(yolo_cfg.get("model_path"))
        self._bridge = CvBridge()

        # interfaces.md §4: Best Effort / depth 10
        self._det_pub = self.create_publisher(
            Detection2DArray, "/vision/detections", _QOS_BEST_EFFORT_10
        )

        if self._publish_debug:
            self._debug_pub = self.create_publisher(Image, "/vision/debug/annotated", 1)
        else:
            self._debug_pub = None

        self.create_subscription(
            Image, "/d455f/color/image_raw", self._on_image, qos_profile_sensor_data
        )

        if self._model is None:
            self.get_logger().warn(
                "[yolo_node] model_path not set — inference disabled. "
                "Phase 2 파인튜닝 완료 후 config/vision.yaml model_path 기입 후 재기동."
            )
        else:
            self.get_logger().info(
                f"[yolo_node] ready - classes={len(self._class_names)} "
                f"conf={self._conf} device={self._device}"
            )

    def _load_model(self, model_path: str | None):
        if model_path is None:
            return None
        try:
            from ultralytics import YOLO
            model = YOLO(model_path)
            self.get_logger().info(f"[yolo_node] model loaded - path={model_path}")
            return model
        except Exception as e:
            self.get_logger().error(f"[yolo_node] model load failed - path={model_path} error={e}")
            return None

    def _on_image(self, msg: Image) -> None:
        if self._model is None:
            return

        rgb = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results = self._model(
            rgb,
            conf=self._conf,
            iou=self._iou,
            device=self._device,
            verbose=False,
        )

        det_array = self._build_detection_array(msg, results[0])
        self._det_pub.publish(det_array)

        if self._debug_pub is not None:
            annotated = results[0].plot()
            debug_msg = self._bridge.cv2_to_imgmsg(annotated, encoding="bgr8")
            debug_msg.header = msg.header
            self._debug_pub.publish(debug_msg)

    def _build_detection_array(self, img_msg: Image, result) -> Detection2DArray:
        array = Detection2DArray()
        array.header = img_msg.header

        if result.boxes is None:
            return array

        boxes = result.boxes
        for i in range(len(boxes)):
            cls_idx = int(boxes.cls[i].item())
            score = float(boxes.conf[i].item())
            xyxy = boxes.xyxy[i].cpu().numpy()

            tool_id = (
                self._class_names[cls_idx]
                if cls_idx < len(self._class_names)
                else f"unknown_{cls_idx}"
            )

            det = Detection2D()
            det.header = img_msg.header

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = tool_id
            hyp.hypothesis.score = score
            det.results.append(hyp)

            cx = float((xyxy[0] + xyxy[2]) / 2)
            cy = float((xyxy[1] + xyxy[3]) / 2)
            w = float(xyxy[2] - xyxy[0])
            h = float(xyxy[3] - xyxy[1])

            det.bbox.center.position.x = cx
            det.bbox.center.position.y = cy
            det.bbox.size_x = w
            det.bbox.size_y = h

            array.detections.append(det)

        if array.detections:
            self.get_logger().debug(
                f"[yolo_node] detected {len(array.detections)} tools - "
                + ", ".join(
                    f"{d.results[0].hypothesis.class_id}({d.results[0].hypothesis.score:.2f})"
                    for d in array.detections
                )
            )

        return array


def main() -> None:
    rclpy.init()
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
