"""unit tests — vision.hand_eye_loader (ROS2-free)."""
import textwrap
from pathlib import Path

import numpy as np
import pytest

import conftest  # noqa: F401 — activates ROS2 stubs

from vision.hand_eye_loader import HandEyeNotCalibratedError, camera_to_base, load_transform


# ---------- fixtures ----------

@pytest.fixture()
def uncalibrated_yaml(tmp_path: Path) -> Path:
    """calibration_date: null → HandEyeNotCalibratedError 기대."""
    p = tmp_path / "hand_eye.yaml"
    p.write_text(textwrap.dedent("""\
        schema_version: 1
        transformation:
          rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
          translation: {x: 0.0, y: 0.0, z: 0.0}
        metadata:
          calibration_date: null
    """))
    return p


@pytest.fixture()
def identity_yaml(tmp_path: Path) -> Path:
    """identity transform — 카메라 == 베이스 (테스트용)."""
    p = tmp_path / "hand_eye.yaml"
    p.write_text(textwrap.dedent("""\
        schema_version: 1
        transformation:
          rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
          translation: {x: 0.0, y: 0.0, z: 0.0}
        metadata:
          calibration_date: "2026-05-27"
          sample_count: 25
          reprojection_error_px: 0.42
    """))
    return p


@pytest.fixture()
def offset_yaml(tmp_path: Path) -> Path:
    """순수 평행 이동 변환 — 회전 없이 translation만 (x=1, y=2, z=3)."""
    p = tmp_path / "hand_eye.yaml"
    p.write_text(textwrap.dedent("""\
        schema_version: 1
        transformation:
          rotation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
          translation: {x: 1.0, y: 2.0, z: 3.0}
        metadata:
          calibration_date: "2026-05-27"
    """))
    return p


# ---------- load_transform tests ----------

def test_load_transform_raises_when_uncalibrated(uncalibrated_yaml: Path) -> None:
    with pytest.raises(HandEyeNotCalibratedError):
        load_transform(uncalibrated_yaml)


def test_load_transform_returns_4x4(identity_yaml: Path) -> None:
    T = load_transform(identity_yaml)
    assert T.shape == (4, 4)
    assert T.dtype == np.float64


def test_load_transform_identity(identity_yaml: Path) -> None:
    T = load_transform(identity_yaml)
    np.testing.assert_allclose(T, np.eye(4), atol=1e-9)


def test_load_transform_translation(offset_yaml: Path) -> None:
    T = load_transform(offset_yaml)
    np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0], atol=1e-9)
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-9)


def test_load_transform_file_not_found() -> None:
    with pytest.raises(FileNotFoundError):
        load_transform(Path("/nonexistent/hand_eye.yaml"))


# ---------- camera_to_base tests ----------

def test_camera_to_base_identity(identity_yaml: Path) -> None:
    T = load_transform(identity_yaml)
    pt_cam = np.array([0.1, 0.2, 0.5])
    pt_base = camera_to_base(pt_cam, T)
    np.testing.assert_allclose(pt_base, pt_cam, atol=1e-9)


def test_camera_to_base_offset(offset_yaml: Path) -> None:
    T = load_transform(offset_yaml)
    pt_cam = np.array([0.0, 0.0, 0.0])
    pt_base = camera_to_base(pt_cam, T)
    np.testing.assert_allclose(pt_base, [1.0, 2.0, 3.0], atol=1e-9)


def test_camera_to_base_returns_3d(identity_yaml: Path) -> None:
    T = load_transform(identity_yaml)
    result = camera_to_base(np.array([1.0, 2.0, 3.0]), T)
    assert result.shape == (3,)
