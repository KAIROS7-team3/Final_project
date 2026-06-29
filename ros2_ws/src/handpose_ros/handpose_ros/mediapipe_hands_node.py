"""MediaPipe 손 감지 → /hands/detections (handpose_interfaces/Hands) 발행 노드.

Subscribe:
  <image_topic>  (sensor_msgs/Image, bgr8)  기본: /d455f/color/image_raw

Publish:
  /hands/detections             (handpose_interfaces/Hands)

Parameters:
  image_topic             (str,   default /d455f/color/image_raw)  컬러 토픽
  flip_image              (bool,  default False)   이미지 좌우 반전
  min_detection_confidence (float, default 0.7)   MediaPipe 감지 신뢰도 하한
  max_num_hands           (int,   default 4)       최대 감지 손 개수
"""
from __future__ import annotations

import cv2
import mediapipe as mp
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from handpose_interfaces.msg import HandLandmarks, Hands

try:
    from cv_bridge import CvBridge
except (ImportError, AttributeError):
    CvBridge = None  # fallback: manual decode (numpy version mismatch with cv_bridge)

_QOS_SENSOR = qos_profile_sensor_data
_QOS_BE = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT)


def _img_msg_to_bgr(msg: Image) -> np.ndarray:
    if CvBridge is not None:
        return CvBridge().imgmsg_to_cv2(msg, desired_encoding="bgr8")
    # manual fallback for bgr8 / rgb8 / bgra8 / rgba8
    dtype = np.uint8
    channels = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4}.get(msg.encoding, 3)
    img = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(msg.height, msg.width, channels)
    if msg.encoding in ("rgb8", "rgba8"):
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    return img


class MediapipeHandsNode(Node):
    def __init__(self) -> None:
        super().__init__("mediapipe_hands_node")

        self.declare_parameter("image_topic", "/d455f/color/image_raw")
        self.declare_parameter("flip_image", False)
        self.declare_parameter("min_detection_confidence", 0.7)
        self.declare_parameter("max_num_hands", 4)

        image_topic = self.get_parameter("image_topic").get_parameter_value().string_value
        flip = self.get_parameter("flip_image").value
        min_conf = float(self.get_parameter("min_detection_confidence").value)
        max_hands = int(self.get_parameter("max_num_hands").value)

        self._flip = flip
        self._mp_hands = mp.solutions.hands.Hands(
            static_image_mode=False,
            max_num_hands=max_hands,
            min_detection_confidence=min_conf,
            min_tracking_confidence=0.5,
        )

        self._pub = self.create_publisher(Hands, "/hands/detections", _QOS_BE)
        self.create_subscription(
            Image, image_topic,
            self._on_image, _QOS_SENSOR,
        )

        self.get_logger().info(
            f"[mediapipe_hands] 초기화 완료 — image_topic={image_topic} flip={flip} "
            f"min_conf={min_conf} max_hands={max_hands}"
        )

    def _on_image(self, msg: Image) -> None:
        try:
            bgr = _img_msg_to_bgr(msg)
        except Exception as e:
            self.get_logger().warn(f"[mediapipe_hands] 이미지 변환 실패: {e}")
            return

        if self._flip:
            bgr = cv2.flip(bgr, 1)

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        h_px, w_px = rgb.shape[:2]
        results = self._mp_hands.process(rgb)

        hands_msg = Hands()
        hands_msg.header = Header()
        hands_msg.header.stamp = msg.header.stamp
        hands_msg.header.frame_id = msg.header.frame_id

        if results.multi_hand_landmarks and results.multi_handedness:
            for lm_list, handedness in zip(
                results.multi_hand_landmarks, results.multi_handedness
            ):
                info = handedness.classification[0]
                hl = HandLandmarks()
                hl.label = info.label       # "Left" or "Right"
                hl.score = float(info.score)

                # canonical landmarks: pixel (u, v, z_norm)
                canon: list[float] = []
                for lm in lm_list.landmark:
                    canon.append(float(lm.x * w_px))
                    canon.append(float(lm.y * h_px))
                    canon.append(float(lm.z))
                hl.landmarks_canon = canon

                # world landmarks [m]
                world: list[float] = []
                if results.multi_hand_world_landmarks:
                    idx = list(results.multi_hand_landmarks).index(lm_list)
                    if idx < len(results.multi_hand_world_landmarks):
                        for wlm in results.multi_hand_world_landmarks[idx].landmark:
                            world.append(float(wlm.x))
                            world.append(float(wlm.y))
                            world.append(float(wlm.z))
                if not world:
                    world = [0.0] * 63
                hl.landmarks_world = world

                hands_msg.hands.append(hl)

        self._pub.publish(hands_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MediapipeHandsNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
