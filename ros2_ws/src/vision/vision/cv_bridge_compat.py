"""cv_bridge 대체 구현 — NumPy 2.x 호환.

cv_bridge C 확장(.so) 없이 sensor_msgs/Image ↔ numpy array 변환.
CvBridge 클래스와 동일한 인터페이스 제공.

지원 encoding:
  imgmsg_to_cv2: passthrough, 16UC1, 32FC1, mono8, mono16,
                 bgr8, rgb8, bgra8, rgba8
  cv2_to_imgmsg: bgr8, rgb8, mono8, mono16, 16UC1
"""
from __future__ import annotations

import numpy as np
from sensor_msgs.msg import Image

_ENC_DTYPE: dict[str, tuple[type, int]] = {
    "mono8":       (np.uint8,   1),
    "mono16":      (np.uint16,  1),
    "16uc1":       (np.uint16,  1),
    "32fc1":       (np.float32, 1),
    "bgr8":        (np.uint8,   3),
    "rgb8":        (np.uint8,   3),
    "bgra8":       (np.uint8,   4),
    "rgba8":       (np.uint8,   4),
}

_BGR_ORDER = {"bgr8", "bgra8"}
_RGB_ORDER = {"rgb8", "rgba8"}


def _decode(data: bytes, enc: str, h: int, w: int) -> np.ndarray:
    """raw bytes + encoding → HxWxC (or HxW for 1ch) numpy array.

    enc는 소문자 정규화된 값을 받는다.
    """
    if enc not in _ENC_DTYPE:
        raise ValueError(f"[cv_bridge_compat] 지원하지 않는 encoding: {enc}")
    dtype, ch = _ENC_DTYPE[enc]
    arr = np.frombuffer(data, dtype=dtype)
    if ch == 1:
        return arr.reshape(h, w)
    return arr.reshape(h, w, ch)


class CvBridge:
    """sensor_msgs/Image ↔ numpy array 변환 (cv_bridge 호환 인터페이스).

    cv_bridge C 확장을 import하지 않으므로 NumPy 2.x에서도 동작.
    """

    def imgmsg_to_cv2(
        self,
        img_msg: Image,
        desired_encoding: str = "passthrough",
    ) -> np.ndarray:
        """sensor_msgs/Image → numpy array.

        desired_encoding:
          "passthrough" — 원본 dtype 그대로 반환
          "bgr8"        — uint8 BGR 3채널 반환 (rgb8 소스는 자동 변환)
          기타 encoding — 해당 dtype/채널로 reshape만 수행
        """
        enc = img_msg.encoding.lower().replace(" ", "")
        h, w = img_msg.height, img_msg.width

        if desired_encoding == "passthrough":
            if enc not in _ENC_DTYPE:
                raise ValueError(f"[cv_bridge_compat] passthrough 불가 encoding: {enc}")
            return _decode(img_msg.data, enc, h, w)

        dst = desired_encoding.lower()

        # 소스 디코딩
        src = _decode(img_msg.data, enc, h, w)

        # 채널 수 동일 → 채널 순서만 교환 (RGB↔BGR)
        if dst == "bgr8":
            if enc in _RGB_ORDER:
                return src[..., :3][..., ::-1].copy()
            if enc in _BGR_ORDER:
                return src[..., :3].copy()
            if enc in ("mono8", "mono16", "16UC1", "32FC1"):
                return src  # 단채널 그대로
        if dst == "rgb8":
            if enc in _BGR_ORDER:
                return src[..., :3][..., ::-1].copy()
            if enc in _RGB_ORDER:
                return src[..., :3].copy()

        # 동일 encoding 요청
        if dst == enc:
            return src.copy()

        raise ValueError(
            f"[cv_bridge_compat] 변환 미지원: {enc} → {desired_encoding}"
        )

    def cv2_to_imgmsg(
        self,
        cvim: np.ndarray,
        encoding: str = "passthrough",
        header=None,
    ) -> Image:
        """numpy array → sensor_msgs/Image.

        encoding "passthrough": ndim/dtype에서 자동 추론.
        """
        msg = Image()
        if header is not None:
            msg.header = header

        if encoding == "passthrough":
            if cvim.ndim == 2:
                if cvim.dtype == np.uint8:
                    enc = "mono8"
                elif cvim.dtype == np.uint16:
                    enc = "mono16"
                elif cvim.dtype == np.float32:
                    enc = "32FC1"
                else:
                    enc = "mono8"
            elif cvim.ndim == 3 and cvim.shape[2] == 3:
                enc = "bgr8"
            elif cvim.ndim == 3 and cvim.shape[2] == 4:
                enc = "bgra8"
            else:
                raise ValueError(f"[cv_bridge_compat] 자동 추론 불가: shape={cvim.shape}")
        else:
            enc = encoding.lower()

        msg.encoding = enc
        if cvim.ndim == 2:
            msg.height, msg.width = cvim.shape
            msg.step = msg.width * cvim.itemsize
        else:
            msg.height, msg.width, ch = cvim.shape
            msg.step = msg.width * ch * cvim.itemsize

        msg.data = cvim.tobytes()
        msg.is_bigendian = False
        return msg
