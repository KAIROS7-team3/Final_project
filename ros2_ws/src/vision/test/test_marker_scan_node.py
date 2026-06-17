"""unit tests — vision.gripper_marker_scan_node (ROS2-free, PR #49 재검토 반영분).

대상:
  - _quat_to_rot          : 쿼터니언 → 회전행렬
  - _select_drawer_marker : 서랍 마커 단일 선택 가드 (Medium-2)
  - MarkerScanNode._pixel_to_cam / _cam_to_tcp : 순수 좌표 변환
  - MarkerScanNode._lookup_base_from_gripper   : TF 조회 실패 시 fail-safe(None)
"""
from types import SimpleNamespace
from unittest.mock import MagicMock

import numpy as np
import pytest

import conftest  # noqa: F401 — activates ROS2 stubs

from vision.gripper_marker_scan_node import (
    MarkerScanNode,
    _quat_to_rot,
    _select_drawer_marker,
)
from tf2_ros import LookupException


# ---------- _quat_to_rot ----------

def test_quat_to_rot_identity() -> None:
    R = _quat_to_rot(0.0, 0.0, 0.0, 1.0)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-9)


def test_quat_to_rot_normalizes_input() -> None:
    # 정규화되지 않은 쿼터니언도 정규 회전행렬을 반환해야 함
    R = _quat_to_rot(0.0, 0.0, 0.0, 2.0)
    np.testing.assert_allclose(R, np.eye(3), atol=1e-9)


# ---------- _select_drawer_marker ----------

def test_select_drawer_marker_single_valid() -> None:
    corners_list = [np.zeros((1, 4, 2)), np.ones((1, 4, 2))]
    ids = np.array([[5], [1]])  # ID 5 무효, ID 1 유효(layer_1)
    result = _select_drawer_marker(corners_list, ids)
    assert result is not None
    corners, marker_id = result
    assert marker_id == 1
    np.testing.assert_array_equal(corners, corners_list[1])


def test_select_drawer_marker_none_valid() -> None:
    corners_list = [np.zeros((1, 4, 2))]
    ids = np.array([[7]])
    assert _select_drawer_marker(corners_list, ids) is None


def test_select_drawer_marker_ambiguous_skips() -> None:
    """layer_0/layer_1 마커가 동시에 보이면 어느 쪽 Z인지 알 수 없으므로 스킵."""
    corners_list = [np.zeros((1, 4, 2)), np.ones((1, 4, 2))]
    ids = np.array([[0], [1]])
    assert _select_drawer_marker(corners_list, ids) is None


# ---------- MarkerScanNode 순수 변환 메서드 (인스턴스 생성 없이 unbound 호출) ----------

def _fake_self(**overrides) -> SimpleNamespace:
    base = dict(
        _cam_matrix=np.array([
            [500.0, 0.0, 320.0],
            [0.0, 500.0, 240.0],
            [0.0, 0.0, 1.0],
        ]),
        _hand_eye_R=np.eye(3),
        _hand_eye_t=np.array([0.0, 0.033, 0.0825]),
    )
    base.update(overrides)
    return SimpleNamespace(**base)


def test_pixel_to_cam_principal_point_maps_to_zero_xy() -> None:
    fake = _fake_self()
    p = MarkerScanNode._pixel_to_cam(fake, 320.0, 240.0, 0.5)
    np.testing.assert_allclose(p, [0.0, 0.0, 0.5], atol=1e-9)


def test_pixel_to_cam_offset_scales_with_depth() -> None:
    fake = _fake_self()
    p = MarkerScanNode._pixel_to_cam(fake, 320.0 + 50.0, 240.0, 1.0)
    # (u-cx)*Z/fx = 50*1.0/500.0 = 0.1
    np.testing.assert_allclose(p, [0.1, 0.0, 1.0], atol=1e-9)


def test_cam_to_tcp_applies_hand_eye_translation() -> None:
    fake = _fake_self()
    p_tcp = MarkerScanNode._cam_to_tcp(fake, np.array([0.0, 0.0, 0.0]))
    np.testing.assert_allclose(p_tcp, fake._hand_eye_t, atol=1e-9)


def test_cam_to_tcp_applies_rotation() -> None:
    # 90도 yaw 등 임의 회전이 적용되는지 확인 (R != I)
    R = np.array([
        [0.0, -1.0, 0.0],
        [1.0,  0.0, 0.0],
        [0.0,  0.0, 1.0],
    ])
    fake = _fake_self(_hand_eye_R=R, _hand_eye_t=np.zeros(3))
    p_tcp = MarkerScanNode._cam_to_tcp(fake, np.array([1.0, 0.0, 0.0]))
    np.testing.assert_allclose(p_tcp, [0.0, 1.0, 0.0], atol=1e-9)


# ---------- _lookup_base_from_gripper: TF 실패 시 fail-safe ----------

def test_lookup_base_from_gripper_returns_none_on_tf_failure() -> None:
    fake = SimpleNamespace(
        _base_frame="base_link",
        _gripper_frame="link_6",
        _tf_buf=MagicMock(
            lookup_transform=MagicMock(side_effect=LookupException("no tf"))
        ),
        get_logger=MagicMock(return_value=MagicMock()),
    )
    result = MarkerScanNode._lookup_base_from_gripper(fake)
    assert result is None
    fake.get_logger().warning.assert_called_once()


def test_lookup_base_from_gripper_composes_transform() -> None:
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        )
    )
    fake = SimpleNamespace(
        _base_frame="base_link",
        _gripper_frame="link_6",
        _tf_buf=MagicMock(lookup_transform=MagicMock(return_value=transform)),
        get_logger=MagicMock(return_value=MagicMock()),
    )
    T = MarkerScanNode._lookup_base_from_gripper(fake)
    assert T is not None
    np.testing.assert_allclose(T[:3, :3], np.eye(3), atol=1e-9)
    np.testing.assert_allclose(T[:3, 3], [1.0, 2.0, 3.0], atol=1e-9)
