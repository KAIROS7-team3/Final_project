"""그리퍼 캠 C270 이미지 수집 스크립트 (학습 데이터용).

s 키를 누를 때마다 원본 이미지 1장 저장.
저장 위치: scripts/dataset/gripper_raw_images/

사용법:
    python3 scripts/dataset/collect_gripper_images.py

키 조작:
    s — 현재 프레임 원본 저장
    q — 종료
"""
from __future__ import annotations

import logging
import time
from pathlib import Path

import cv2

logging.basicConfig(format="[%(levelname)s] %(message)s", level=logging.INFO)
logger = logging.getLogger(__name__)

_SAVE_DIR = Path.home() / "gripper_cam"


def _find_c270() -> str | None:
    """v4l2-ctl로 C270 장치 경로 자동 탐색."""
    import subprocess
    try:
        out = subprocess.check_output(["v4l2-ctl", "--list-devices"], text=True, stderr=subprocess.DEVNULL)
    except FileNotFoundError:
        return None
    current_device = None
    for line in out.splitlines():
        if "C270" in line:
            current_device = "found"
        elif current_device == "found" and "/dev/video" in line:
            return line.strip()
    return None


def main() -> None:
    _SAVE_DIR.mkdir(parents=True, exist_ok=True)

    device = _find_c270()
    if device is None:
        logger.error("C270 카메라를 찾을 수 없습니다. USB 연결 확인 필요")
        return
    logger.info("C270 감지: %s", device)
    cap = cv2.VideoCapture(device)
    if not cap.isOpened():
        logger.error("C270 카메라 열기 실패: %s", device)
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    saved_count = 0
    logger.info("준비 완료 | 저장 위치: %s", _SAVE_DIR)
    logger.info("s: 저장  q: 종료")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue

        # 화면 상단에 저장 카운트 표시
        display = frame.copy()
        cv2.putText(display, f"saved: {saved_count}  |  s:save  q:quit",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("C270 - Gripper Image Collect", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("s"):
            ts        = time.strftime("%Y%m%d_%H%M%S_%f")[:19]
            save_path = _SAVE_DIR / f"gripper_{ts}.jpg"
            cv2.imwrite(str(save_path), frame)  # 원본 저장 (바운딩박스 없음)
            saved_count += 1
            logger.info("[%d장] 저장: %s", saved_count, save_path.name)
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    logger.info("종료 | 총 %d장 저장 → %s", saved_count, _SAVE_DIR)


if __name__ == "__main__":
    main()
