#!/usr/bin/env python3
"""D455f 연결 상태 즉시 검증 스크립트 (ROS2 불필요).

사용:
    python3 scripts/verify_camera.py              # 검증만
    python3 scripts/verify_camera.py --save       # 스냅샷 저장
    python3 scripts/verify_camera.py --save --frames 5

출력:
    - 카메라 시리얼 번호, 펌웨어 버전
    - RGB / Depth 해상도 및 FPS
    - Intrinsics (fx, fy, cx, cy)
    - Depth 통계 (유효 비율, min/mean/max) — 탑뷰 작업 높이 확인용
    - 스냅샷: verify_rgb.png, verify_depth_colormap.png (--save 시)
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

try:
    import pyrealsense2 as rs
except ImportError:
    print("[ERROR] pyrealsense2 미설치 — pip install pyrealsense2")
    sys.exit(1)

_DEPTH_SCALE = 0.001  # D455f 기본값 (1 raw unit = 1mm)
_DEPTH_CLIP_MAX_M = 3.0  # 탑뷰 작업 범위 상한


def main() -> None:
    parser = argparse.ArgumentParser(description="D455f 연결 검증")
    parser.add_argument("--save", action="store_true", help="스냅샷 이미지 저장")
    parser.add_argument("--frames", type=int, default=30, help="수집할 워밍업 프레임 수 (기본 30)")
    args = parser.parse_args()

    ctx = rs.context()
    devices = ctx.query_devices()
    if len(devices) == 0:
        print("[FAIL] RealSense 장치를 찾을 수 없습니다.")
        print("       USB 3.x 포트 연결 및 udev 규칙 설치 확인:")
        print("       sudo bash scripts/udev/install_udev_rules.sh")
        sys.exit(1)

    dev = devices[0]
    serial = dev.get_info(rs.camera_info.serial_number)
    firmware = dev.get_info(rs.camera_info.firmware_version)
    print(f"[OK] 장치 감지: {dev.get_info(rs.camera_info.name)}")
    print(f"     시리얼: {serial} / 펌웨어: {firmware}")

    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)

    profile = pipeline.start(config)

    depth_sensor = profile.get_device().first_depth_sensor()
    depth_sensor.set_option(rs.option.visual_preset, 3)  # HighAccuracy
    actual_depth_scale = depth_sensor.get_depth_scale()

    align = rs.align(rs.stream.color)

    print(f"\n스트림 워밍업 중 ({args.frames} 프레임)...", end="", flush=True)

    rgb_img = depth_m = None
    color_intrinsics = None

    try:
        for i in range(args.frames):
            frames = pipeline.wait_for_frames(timeout_ms=2000)
            aligned = align.process(frames)

            color_frame = aligned.get_color_frame()
            depth_frame = aligned.get_depth_frame()

            if not color_frame or not depth_frame:
                continue

            rgb_img = np.asanyarray(color_frame.get_data())
            depth_raw = np.asanyarray(depth_frame.get_data())
            depth_m = depth_raw.astype(np.float32) * actual_depth_scale

            if color_intrinsics is None:
                color_intrinsics = (
                    aligned.get_color_frame()
                    .profile.as_video_stream_profile()
                    .intrinsics
                )
            print(".", end="", flush=True)
    finally:
        pipeline.stop()

    print(" 완료\n")

    if rgb_img is None or depth_m is None:
        print("[FAIL] 프레임 수신 실패 — 카메라 재연결 후 재시도")
        sys.exit(1)

    h, w = rgb_img.shape[:2]
    print(f"[스트림]")
    print(f"  RGB   : {w}x{h} @ 30fps")
    print(f"  Depth : 848x480 → aligned {w}x{h}")

    print(f"\n[Intrinsics]")
    if color_intrinsics:
        print(f"  fx={color_intrinsics.fx:.2f}  fy={color_intrinsics.fy:.2f}")
        print(f"  cx={color_intrinsics.ppx:.2f}  cy={color_intrinsics.ppy:.2f}")

    valid = depth_m[(depth_m > 0) & (depth_m < _DEPTH_CLIP_MAX_M)]
    zero_ratio = 1.0 - len(valid) / depth_m.size

    print(f"\n[Depth 통계] (탑뷰 작업 범위 0~{_DEPTH_CLIP_MAX_M}m)")
    print(f"  유효 비율 : {1 - zero_ratio:.1%}")
    if len(valid) > 0:
        print(f"  min  : {valid.min():.3f} m")
        print(f"  mean : {valid.mean():.3f} m  ← 카메라 탑뷰 설치 높이 근사")
        print(f"  max  : {valid.max():.3f} m")
    else:
        print("  [WARN] 유효 depth 없음 — 조명·거리 확인")

    if zero_ratio > 0.5:
        print(f"\n[WARN] zero-depth 비율 {zero_ratio:.1%} > 50% — 조명 문제 또는 USB 연결 확인")
    elif zero_ratio > 0.3:
        print(f"\n[INFO] zero-depth 비율 {zero_ratio:.1%} — 금속 공구 IR 반사 정상 범위")

    if args.save:
        out_dir = Path(".")
        cv2.imwrite(str(out_dir / "verify_rgb.png"), rgb_img)

        depth_vis = np.clip(depth_m, 0, _DEPTH_CLIP_MAX_M)
        depth_norm = (depth_vis / _DEPTH_CLIP_MAX_M * 255).astype(np.uint8)
        depth_colormap = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
        cv2.imwrite(str(out_dir / "verify_depth_colormap.png"), depth_colormap)

        print(f"\n[저장] verify_rgb.png, verify_depth_colormap.png")

    print("\n[결과]")
    passed = len(valid) > 0 and zero_ratio <= 0.5
    if passed:
        print("  ✓ D455f 스트림 정상 — Phase 1 카메라 bring-up 완료 조건 충족")
        print("  다음: ros2 launch vision realsense_bringup.launch.py 으로 ROS2 스트림 검증")
    else:
        print("  ✗ 스트림 이상 — 위 경고 메시지 확인 후 재시도")
        sys.exit(1)


if __name__ == "__main__":
    main()
