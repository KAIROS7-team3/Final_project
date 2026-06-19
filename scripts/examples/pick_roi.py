"""탑뷰 카메라에서 마우스 클릭으로 ROI 좌표를 뽑는 스크립트.

사용법:
    python3 scripts/examples/pick_roi.py

조작:
    마우스 좌클릭 2번  → 사각형 ROI 지정 (1번째: 좌상단, 2번째: 우하단)
    r                  → ROI 초기화
    s                  → 좌표 출력 + config/vision.yaml 붙여넣기용 텍스트 출력
    q / ESC            → 종료

출력 좌표는 config/vision.yaml의 roi 섹션에 바로 붙여 쓸 수 있는 형식으로 나옵니다.
"""
from __future__ import annotations

import logging
import sys
from pathlib import Path

import cv2
import numpy as np
import pyrealsense2 as rs

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

_WIN = "ROI Picker  [click x2=rect  r=reset  s=save  q=quit]"

# 클릭 상태
_clicks: list[tuple[int, int]] = []
_roi_final: tuple[int, int, int, int] | None = None  # (x1, y1, x2, y2)


def _on_mouse(event, x, y, flags, param) -> None:
    global _clicks, _roi_final

    if event == cv2.EVENT_LBUTTONDOWN:
        if len(_clicks) >= 2:
            _clicks.clear()
            _roi_final = None

        _clicks.append((x, y))
        logger.info("클릭 %d: (%d, %d)", len(_clicks), x, y)

        if len(_clicks) == 2:
            x1, y1 = _clicks[0]
            x2, y2 = _clicks[1]
            # 정규화: 항상 좌상→우하 순서
            _roi_final = (min(x1, x2), min(y1, y2), max(x1, x2), max(y1, y2))
            logger.info("ROI 확정: x1=%d y1=%d x2=%d y2=%d", *_roi_final)


def _print_roi(roi: tuple[int, int, int, int]) -> None:
    x1, y1, x2, y2 = roi
    w = x2 - x1
    h = y2 - y1
    print("\n" + "=" * 50)
    print(f"[ROI 좌표]  x1={x1}  y1={y1}  x2={x2}  y2={y2}")
    print(f"[크기]      w={w}  h={h}")
    print()
    print("── config/vision.yaml 붙여넣기용 ──")
    print(f"  roi:")
    print(f"    x1: {x1}")
    print(f"    y1: {y1}")
    print(f"    x2: {x2}")
    print(f"    y2: {y2}")
    print("=" * 50 + "\n")


def main() -> None:
    global _clicks, _roi_final

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    logger.info("D455f 스트림 시작...")
    pipeline.start(cfg)
    logger.info("스트림 시작 완료")

    cv2.namedWindow(_WIN, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(_WIN, 960, 720)
    cv2.waitKey(1)  # Qt 윈도우 핸들러 초기화 대기
    cv2.setMouseCallback(_WIN, _on_mouse)

    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            img = np.asanyarray(color_frame.get_data())
            vis = img.copy()

            # 첫 번째 클릭 점 표시
            if len(_clicks) >= 1:
                cv2.circle(vis, _clicks[0], 5, (0, 255, 0), -1)
                cv2.putText(vis, f"P1 {_clicks[0]}", (_clicks[0][0] + 8, _clicks[0][1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

            # ROI 사각형 표시
            if _roi_final is not None:
                x1, y1, x2, y2 = _roi_final
                # ROI 외부 어둡게
                overlay = vis.copy()
                overlay[:y1, :] = (overlay[:y1, :] * 0.35).astype(np.uint8)
                overlay[y2:, :] = (overlay[y2:, :] * 0.35).astype(np.uint8)
                overlay[y1:y2, :x1] = (overlay[y1:y2, :x1] * 0.35).astype(np.uint8)
                overlay[y1:y2, x2:] = (overlay[y1:y2, x2:] * 0.35).astype(np.uint8)
                vis = overlay

                cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                label = f"ROI ({x1},{y1}) - ({x2},{y2})  {x2-x1}x{y2-y1}px"
                cv2.putText(vis, label, (x1, max(y1 - 8, 12)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            elif len(_clicks) == 1:
                cv2.putText(vis, "두 번째 점을 클릭하세요 (우하단)",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)
            else:
                cv2.putText(vis, "ROI 좌상단을 클릭하세요",
                            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 255), 2)

            cv2.imshow(_WIN, vis)
            key = cv2.waitKey(1) & 0xFF

            if key in (ord("q"), 27):
                break
            elif key == ord("r"):
                _clicks.clear()
                _roi_final = None
                logger.info("ROI 초기화")
            elif key == ord("s"):
                if _roi_final is not None:
                    _print_roi(_roi_final)
                else:
                    logger.warning("ROI가 아직 지정되지 않았습니다. 클릭 2번으로 먼저 지정하세요.")

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        pipeline.stop()
        logger.info("종료")


if __name__ == "__main__":
    main()
