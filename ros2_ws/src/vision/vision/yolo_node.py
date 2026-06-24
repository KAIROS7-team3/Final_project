"""YOLOv11 공구 검출/세그멘테이션 노드 (Track A/B).

탑뷰(D455f)와 그리퍼(C270) 각각 별도 인스턴스로 기동.
런치 파라미터 camera_type으로 시점을 구분한다.
  - camera_type=top_view : top_view_model_path 로드, /d455f/color/image_raw 구독
  - camera_type=gripper  : gripper_model_path  로드, /c270/image_raw 구독

세그멘테이션 모델(v3-seg 등) 사용 시 마스크 무게중심을 검출 중심점으로 사용.
detection 모델 사용 시 bbox 중심점을 그대로 사용 (하위 호환).

Subscribe : image_topic 파라미터 (기본 /d455f/color/image_raw)
Publish   : /vision/detections/<camera_type>  (vision_msgs/Detection2DArray)
            /vision/debug/annotated       (sensor_msgs/Image, debug 시에만)
            /vision/debug/mask            (sensor_msgs/Image, seg 모델 + debug 시에만)

config/vision.yaml의 해당 model_path가 null이면 추론 없이 대기.
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

_QOS_BEST_EFFORT_10 = QoSProfile(
    depth=10,
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
)

_CONFIG_PATH = Path("config/vision.yaml")

_CAMERA_TOPICS = {
    "top_view": "/d455f/color/image_raw",
    "gripper":  "/c270/image_raw",
}
_MODEL_KEYS = {
    "top_view": "top_view_model_path",
    "gripper":  "gripper_model_path",
}


def _load_cfg() -> dict:
    with _CONFIG_PATH.open() as f:
        return yaml.safe_load(f)


class YoloNode(Node):
    """YOLOv11s 기반 6종 공구 검출 노드 (탑뷰/그리퍼 공용)."""

    def __init__(self) -> None:
        super().__init__("yolo_node")

        camera_type: str = self.declare_parameter("camera_type", "top_view").value
        if camera_type not in _CAMERA_TOPICS:
            raise ValueError(
                f"[yolo_node] 알 수 없는 camera_type='{camera_type}'. "
                "top_view 또는 gripper 지정 필요."
            )

        cfg = _load_cfg()
        yolo_cfg = cfg["yolo"]
        debug_cfg = cfg["debug"]

        self._conf: float = yolo_cfg["confidence_threshold"]
        self._iou: float = yolo_cfg["iou_threshold"]
        device_override = self.declare_parameter("device", "").value
        self._device: str = device_override if device_override else yolo_cfg["device"]
        self._class_names: list[str] = yolo_cfg["class_names"]
        self._publish_debug: bool = debug_cfg["publish_annotated_image"]
        self._camera_type: str = camera_type

        model_path = yolo_cfg.get(_MODEL_KEYS[camera_type])
        self._model = self._load_model(model_path)

        image_topic: str = _CAMERA_TOPICS[camera_type]

        # interfaces.md §4: Best Effort / depth 10
        # camera_type별 토픽 분리 — 동시 기동 시 top_view/gripper 검출 결과 혼용 방지
        self._det_pub = self.create_publisher(
            Detection2DArray, f"/vision/detections/{camera_type}", _QOS_BEST_EFFORT_10
        )

        # gripper 타입일 때 최고 신뢰도 검출의 이진 마스크 발행 (PCA theta용)
        self._gripper_mask_pub = (
            self.create_publisher(Image, "/vision/masks/gripper", _QOS_BEST_EFFORT_10)
            if camera_type == "gripper"
            else None
        )

        if self._publish_debug:
            self._debug_pub = self.create_publisher(
                Image, f"/vision/debug/{camera_type}/annotated", 1
            )
            self._mask_pub = self.create_publisher(
                Image, f"/vision/debug/{camera_type}/mask", 1
            )
        else:
            self._debug_pub = None
            self._mask_pub = None

        self.create_subscription(
            Image, image_topic, self._on_image, qos_profile_sensor_data
        )

        if self._model is None:
            self.get_logger().warn(
                f"[yolo_node] camera_type={camera_type} model_path not set — inference disabled. "
                f"Phase 2 파인튜닝 완료 후 config/vision.yaml {_MODEL_KEYS[camera_type]} 기입 후 재기동."
            )
        else:
            self.get_logger().info(
                f"[yolo_node] ready - camera_type={camera_type} topic={image_topic} "
                f"classes={len(self._class_names)} conf={self._conf} device={self._device}"
            )

    @staticmethod
    def _imgmsg_to_bgr(msg: Image) -> np.ndarray:
        enc = msg.encoding.lower()
        if enc == "bgr8":
            return np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
        if enc == "rgb8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3).copy()
            return arr[:, :, ::-1]
        if enc == "mono8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width)
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
        if enc in ("yuv422_yuy2", "yuyv"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 2)
            return cv2.cvtColor(arr, cv2.COLOR_YUV2BGR_YUYV)
        raise ValueError(f"[yolo_node] 지원하지 않는 encoding: {enc}")

    @staticmethod
    def _bgr_to_imgmsg(arr: np.ndarray, encoding: str = "bgr8") -> Image:
        msg = Image()
        msg.height, msg.width = arr.shape[:2]
        msg.encoding = encoding
        msg.step = arr.shape[1] * (arr.shape[2] if arr.ndim == 3 else 1)
        msg.data = arr.tobytes()
        return msg

    @staticmethod
    def _mono_to_imgmsg(arr: np.ndarray) -> Image:
        msg = Image()
        msg.height, msg.width = arr.shape
        msg.encoding = "mono8"
        msg.step = arr.shape[1]
        msg.data = arr.tobytes()
        return msg

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

        rgb = self._imgmsg_to_bgr(msg)
        results = self._model(
            rgb,
            conf=self._conf,
            iou=self._iou,
            device=self._device,
            verbose=False,
        )

        det_array = self._build_detection_array(msg, results[0])
        self._det_pub.publish(det_array)

        if self._gripper_mask_pub is not None and results[0].boxes is not None and results[0].masks is not None:
            _h, _w = rgb.shape[:2]
            for _i in range(len(results[0].boxes.cls)):
                if _i >= len(results[0].masks.xy):
                    continue
                _xy = results[0].masks.xy[_i]
                if len(_xy) == 0:
                    continue
                _mask = np.zeros((_h, _w), dtype=np.uint8)
                cv2.fillPoly(_mask, [_xy.astype(np.int32).reshape((-1, 1, 2))], 255)
                _cls_idx = int(results[0].boxes.cls[_i].item())
                _class_id = results[0].names.get(_cls_idx, "") if results[0].names else ""
                _mask_msg = self._mono_to_imgmsg(_mask)
                _mask_msg.header.stamp = msg.header.stamp
                _mask_msg.header.frame_id = _class_id
                self._gripper_mask_pub.publish(_mask_msg)

        if self._debug_pub is not None:
            annotated = results[0].plot()
            debug_msg = self._bgr_to_imgmsg(annotated, "bgr8")
            debug_msg.header = msg.header
            self._debug_pub.publish(debug_msg)

            if self._mask_pub is not None and results[0].masks is not None:
                mask_vis = self._build_mask_image(rgb, results[0])
                mask_msg = self._bgr_to_imgmsg(mask_vis, "bgr8")
                mask_msg.header = msg.header
                self._mask_pub.publish(mask_msg)

    def _mask_centroid(self, mask_xy: np.ndarray) -> tuple[float, float] | None:
        """마스크 픽셀 좌표 배열(N×2) → 무게중심 (cx, cy). 빈 마스크면 None."""
        if mask_xy is None or len(mask_xy) == 0:
            return None
        return float(mask_xy[:, 0].mean()), float(mask_xy[:, 1].mean())

    def _build_detection_array(self, img_msg: Image, result) -> Detection2DArray:
        array = Detection2DArray()
        array.header = img_msg.header

        if result.boxes is None:
            return array

        boxes = result.boxes
        masks = result.masks  # 세그 모델이면 Masks 객체, detection 모델이면 None

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

            # 세그 모델: 마스크 무게중심을 중심점으로 사용 (bbox 중심보다 정확)
            # detection 모델: bbox 중심 fallback
            cx = cy = None
            if masks is not None and i < len(masks.xy):
                cx, cy = self._mask_centroid(masks.xy[i]) or (None, None)

            if cx is None or cy is None:
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
            src = "mask" if masks is not None else "bbox"
            self.get_logger().debug(
                f"[yolo_node] detected {len(array.detections)} tools ({src} centroid) - "
                + ", ".join(
                    f"{d.results[0].hypothesis.class_id}({d.results[0].hypothesis.score:.2f})"
                    for d in array.detections
                )
            )

        return array

    def _build_best_mask(
        self, img: np.ndarray, result
    ) -> tuple[np.ndarray, str] | tuple[None, None]:
        """최고 신뢰도 검출의 이진 마스크(mono8)와 class_id 반환. 마스크 없으면 (None, None)."""
        if result.boxes is None or result.masks is None:
            return None, None
        best_i = int(result.boxes.conf.argmax().item())
        if best_i >= len(result.masks.xy):
            return None, None
        xy = result.masks.xy[best_i]
        if len(xy) == 0:
            return None, None
        h, w = img.shape[:2]
        mask = np.zeros((h, w), dtype=np.uint8)
        pts = xy.astype(np.int32).reshape((-1, 1, 2))
        cv2.fillPoly(mask, [pts], 255)
        cls_idx = int(result.boxes.cls[best_i].item())
        class_id = result.names.get(cls_idx, "") if result.names else ""
        return mask, class_id

    def _build_mask_image(self, img: np.ndarray, result) -> np.ndarray:
        """마스크 윤곽선 + 무게중심점을 원본 이미지에 오버레이해서 반환."""
        vis = img.copy()
        if result.masks is None:
            return vis

        colors = [
            (0, 255, 0), (0, 128, 255), (255, 0, 128),
            (255, 255, 0), (0, 255, 255), (255, 0, 255),
        ]
        for i, xy in enumerate(result.masks.xy):
            if len(xy) == 0:
                continue
            color = colors[i % len(colors)]
            pts = xy.astype(np.int32).reshape((-1, 1, 2))
            cv2.polylines(vis, [pts], isClosed=True, color=color, thickness=2)
            cx, cy = self._mask_centroid(xy)
            cv2.circle(vis, (int(cx), int(cy)), 6, color, -1)
            cls_idx = int(result.boxes.cls[i].item())
            label = (
                self._class_names[cls_idx]
                if cls_idx < len(self._class_names)
                else f"cls{cls_idx}"
            )
            cv2.putText(vis, label, (int(cx) + 8, int(cy) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        return vis


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
