"""ArUco 마커 탐지 예제 — 탑뷰(D455f) 또는 웹캠.

⚠️  현재 테스트 중인 기능입니다. 시나리오 확정 전 참고용으로만 사용하세요.

탑뷰에서 ArUco 마커 ID와 2D 좌표를 추출한다.
추후 마커 ID → place zone 매핑, 그리퍼 캠 정밀 스캔과 연계 예정.

사전 조건:
    pip install opencv-contrib-python pyrealsense2

사용법:
    # D455f로 테스트 (권장)
    python3 scripts/examples/example_aruco_detection.py --camera realsense

    # 웹캠으로 테스트
    python3 scripts/examples/example_aruco_detection.py --camera webcam --device 0
"""
from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np


# ArUco 딕셔너리 — 4x4, 50개 패턴 (scripts/aruco_markers/ 와 동일 규격)
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
ARUCO_PARAMS = cv2.aruco.DetectorParameters()
DETECTOR = cv2.aruco.ArucoDetector(ARUCO_DICT, ARUCO_PARAMS)


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="ArUco 마커 탐지 예제 (테스트 중)")
    p.add_argument("--camera", default="realsense", choices=["realsense", "webcam"],
                   help="카메라 종류 (기본값: realsense)")
    p.add_argument("--device", type=int, default=0,
                   help="웹캠 device 인덱스 (기본값: 0, webcam 선택 시만 사용)")
    return p.parse_args()


def _detect_and_draw(frame: np.ndarray) -> tuple[np.ndarray, list[int]]:
    """ArUco 마커 탐지 후 ID 목록과 annotated 이미지 반환."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = DETECTOR.detectMarkers(gray)

    annotated = frame.copy()
    detected_ids: list[int] = []

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(annotated, corners, ids)
        for i, marker_id in enumerate(ids.flatten()):
            detected_ids.append(int(marker_id))
            # 마커 중심 좌표 계산
            cx = int(corners[i][0][:, 0].mean())
            cy = int(corners[i][0][:, 1].mean())
            cv2.putText(
                annotated, f"ID:{marker_id} ({cx},{cy})",
                (cx - 30, cy - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
            )

    return annotated, detected_ids


def _run_realsense() -> None:
    try:
        import pyrealsense2 as rs
    except ImportError:
        print("[ERROR] pyrealsense2 미설치 — pip install pyrealsense2")
        sys.exit(1)

    pipeline = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    pipeline.start(cfg)

    print("[INFO] D455f 스트림 시작 | q: 종료")
    try:
        while True:
            frames = pipeline.wait_for_frames(timeout_ms=5000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            annotated, ids = _detect_and_draw(frame)

            if ids:
                print(f"\r[DETECT] 마커 ID: {ids}          ", end="")
            else:
                print(f"\r[DETECT] 마커 없음               ", end="")

            cv2.imshow("ArUco detection (D455f)", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()
        print("\n[INFO] 종료")


def _run_webcam(device: int) -> None:
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        print(f"[ERROR] 웹캠 device={device} 열기 실패")
        sys.exit(1)

    print(f"[INFO] 웹캠 device={device} 스트림 시작 | q: 종료")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        annotated, ids = _detect_and_draw(frame)

        if ids:
            print(f"\r[DETECT] 마커 ID: {ids}          ", end="")
        else:
            print(f"\r[DETECT] 마커 없음               ", end="")

        cv2.imshow("ArUco detection (webcam)", annotated)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    print("\n[INFO] 종료")


def main() -> None:
    args = _parse_args()
    if args.camera == "realsense":
        _run_realsense()
    else:
        _run_webcam(args.device)


if __name__ == "__main__":
    main()
