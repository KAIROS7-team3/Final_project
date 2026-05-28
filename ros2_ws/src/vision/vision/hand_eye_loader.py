"""Hand-eye 캘리브레이션 결과 로더 + 좌표 변환 유틸리티.

config/hand_eye.yaml에서 T_camera_to_base를 로드해 4×4 변환 행렬로 반환.
YOLOv11s + depth로 구한 카메라 좌표를 로봇 베이스 좌표로 변환할 때 사용.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml
from scipy.spatial.transform import Rotation


class HandEyeNotCalibratedError(RuntimeError):
    pass


def load_transform(path: Path = Path("config/hand_eye.yaml")) -> np.ndarray:
    """4×4 변환 행렬 반환: camera_color_optical_frame → base_link.

    config/hand_eye.yaml의 transformation 필드가 TBD(null)이면
    HandEyeNotCalibratedError 발생 — 캘리브레이션 완료 전 호출 금지.
    """
    with path.open() as f:
        cfg = yaml.safe_load(f)

    meta = cfg.get("metadata", {})
    calib_date = meta.get("calibration_date")
    if calib_date is None:
        raise HandEyeNotCalibratedError(
            f"hand_eye.yaml에 캘리브레이션 결과가 없습니다. "
            f"Phase 1 캘리브레이션 후 {path} 갱신 필요."
        )

    t_cfg = cfg["transformation"]
    rot = t_cfg["rotation"]
    trans = t_cfg["translation"]

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = Rotation.from_quat(
        [rot["x"], rot["y"], rot["z"], rot["w"]]
    ).as_matrix()
    T[:3, 3] = [trans["x"], trans["y"], trans["z"]]
    return T


def camera_to_base(
    point_camera_m: np.ndarray,
    T: np.ndarray,
) -> np.ndarray:
    """카메라 좌표 → 로봇 베이스 좌표 변환.

    Args:
        point_camera_m: (3,) float64, 카메라 좌표계 [m]
        T: load_transform()이 반환한 4×4 행렬

    Returns:
        (3,) float64, base_link 좌표계 [m]
    """
    p_hom = np.append(point_camera_m, 1.0)
    return (T @ p_hom)[:3]
