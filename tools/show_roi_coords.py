"""카메라 토픽 구독 → OpenCV 창에서 마우스 클릭/호버 시 픽셀 좌표 출력."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import qos_profile_sensor_data
import cv2
import numpy as np
import sys

TOPIC = "/d455f/d455f/color/image_raw"
_mouse_pos = [0, 0]
_click_points: list[tuple[int, int]] = []


def _mouse_cb(event, x, y, flags, param):
    _mouse_pos[0], _mouse_pos[1] = x, y
    if event == cv2.EVENT_LBUTTONDOWN:
        _click_points.append((x, y))
        print(f"[CLICK] x={x}, y={y}")


class CoordViewer(Node):
    def __init__(self):
        super().__init__("roi_coord_viewer")
        self._frame = None
        self.create_subscription(Image, TOPIC, self._cb, qos_profile_sensor_data)
        cv2.namedWindow("ROI Coord Viewer", cv2.WINDOW_NORMAL)
        cv2.setMouseCallback("ROI Coord Viewer", _mouse_cb)
        self.get_logger().info(f"구독 중: {TOPIC}")
        self.get_logger().info("클릭 → 좌표 출력 / q → 종료 / c → 클릭 목록 초기화")

    def _cb(self, msg: Image):
        h, w = msg.height, msg.width
        if msg.encoding == "rgb8":
            arr = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
            frame = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif msg.encoding == "bgr8":
            arr = np.frombuffer(msg.data, np.uint8).reshape(h, w, 3)
            frame = arr.copy()
        else:
            return
        self._frame = frame

    def spin_once_and_show(self):
        rclpy.spin_once(self, timeout_sec=0.03)
        if self._frame is None:
            return True

        disp = self._frame.copy()

        # 클릭 포인트 표시
        for i, (cx, cy) in enumerate(_click_points):
            cv2.drawMarker(disp, (cx, cy), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
            cv2.putText(disp, f"P{i+1}({cx},{cy})", (cx + 5, cy - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 현재 마우스 위치 십자선
        mx, my = _mouse_pos
        h, w = disp.shape[:2]
        cv2.line(disp, (mx, 0), (mx, h), (255, 255, 0), 1)
        cv2.line(disp, (0, my), (w, my), (255, 255, 0), 1)
        cv2.putText(disp, f"({mx}, {my})", (mx + 5, my - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)

        # 해상도 표시
        cv2.putText(disp, f"res: {w}x{h}", (10, h - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (150, 150, 150), 1)

        # 클릭 목록 요약
        if _click_points:
            xs = [p[0] for p in _click_points]
            ys = [p[1] for p in _click_points]
            summary = (f"roi_x_min={min(xs)} roi_x_max={max(xs)} "
                       f"roi_y_min={min(ys)} roi_y_max={max(ys)}")
            cv2.putText(disp, summary, (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        cv2.imshow("ROI Coord Viewer", disp)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            return False
        if key == ord('c'):
            _click_points.clear()
            print("[INFO] 클릭 목록 초기화")
        return True


def main():
    rclpy.init()
    node = CoordViewer()
    try:
        while main_loop(node):
            pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


def main_loop(node):
    return node.spin_once_and_show()


if __name__ == "__main__":
    main()
