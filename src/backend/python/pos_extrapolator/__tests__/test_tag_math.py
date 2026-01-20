from __future__ import annotations

import math
from dataclasses import dataclass
from typing import cast

import numpy as np
from numpy.typing import NDArray
import pytest

import backend.python.pos_extrapolator.preparers.util.tag_math as tm


ArrF64 = NDArray[np.float64]


def _Rt_to_T(R: ArrF64, t: ArrF64) -> ArrF64:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T


def _Rz(deg: float) -> ArrF64:
    th = math.radians(deg)
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64)


def _assert_unit_quat_xyzw(q: ArrF64, atol: float = 1e-10) -> None:
    assert q.shape == (4,)
    assert np.isfinite(q).all()
    assert float(np.linalg.norm(q)) == pytest.approx(1.0, abs=atol)


def _quat_same_rotation(q1: ArrF64, q2: ArrF64, atol: float = 1e-10) -> None:
    # q and -q represent the same rotation; compare via |dot| ~ 1
    _assert_unit_quat_xyzw(q1, atol=atol)
    _assert_unit_quat_xyzw(q2, atol=atol)
    assert abs(float(np.dot(q1, q2))) == pytest.approx(1.0, abs=atol)


def test_quat_xyzw_from_R_identity_is_unit_and_w_positive():
    q = tm._quat_xyzw_from_R(np.eye(3, dtype=np.float64))
    _assert_unit_quat_xyzw(q)
    # Identity rotation should be (0,0,0,1)
    assert float(q[0]) == pytest.approx(0.0, abs=1e-12)
    assert float(q[1]) == pytest.approx(0.0, abs=1e-12)
    assert float(q[2]) == pytest.approx(0.0, abs=1e-12)
    assert float(q[3]) == pytest.approx(1.0, abs=1e-12)


@pytest.mark.parametrize("deg", [90.0, 180.0, -90.0, 45.0, 135.0])
def test_quat_xyzw_from_R_known_z_rotations_match_expected(deg: float):
    # For a pure z rotation by theta: q = [0,0,sin(theta/2),cos(theta/2)]
    q = tm._quat_xyzw_from_R(_Rz(deg))
    th2 = math.radians(deg) / 2.0
    q_expected = np.array([0.0, 0.0, math.sin(th2), math.cos(th2)], dtype=np.float64)
    q_expected /= np.linalg.norm(q_expected)
    _quat_same_rotation(q, q_expected, atol=1e-10)


def test_quat_xyzw_from_R_accepts_nonfloat_input_and_normalizes():
    # Intentionally pass float32; implementation should coerce to float64 internally.
    R32 = _Rz(90.0).astype(np.float32)
    q = tm._quat_xyzw_from_R(cast(ArrF64, R32))
    assert q.dtype == np.float64
    _assert_unit_quat_xyzw(q)


def test_tag_corners_local_are_centered_and_ordered():
    corners = tm._tag_corners_local(tag_size=2.0)
    assert corners.shape == (4, 3)
    assert corners.dtype == np.float64
    # Expected order: (-,-), (+,-), (+,+), (-,+) at z=0
    expected = np.array(
        [[-1.0, -1.0, 0.0], [1.0, -1.0, 0.0], [1.0, 1.0, 0.0], [-1.0, 1.0, 0.0]],
        dtype=np.float64,
    )
    assert np.allclose(corners, expected)
    assert np.allclose(corners.mean(axis=0), np.array([0.0, 0.0, 0.0]))


def test_transform_points_applies_rotation_then_translation():
    pts = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=np.float64)
    R = _Rz(90.0)  # x->y, y->-x
    t = np.array([10.0, 20.0, 30.0], dtype=np.float64)
    T = _Rt_to_T(R, t)
    out = tm._transform_points(T, pts)
    expected = (pts @ R.T) + t.reshape(1, 3)
    assert out.shape == (2, 3)
    assert np.allclose(out, expected)


def test_componentize_transformation_matrix_extracts_R_and_t():
    R = _Rz(45.0)
    t = np.array([-1.0, 2.5, 3.0], dtype=np.float64)
    T = _Rt_to_T(R, t)
    R2, t2 = tm.componentize_transformation_matrix(T)
    assert R2.shape == (3, 3)
    assert t2.shape == (3,)
    assert np.allclose(R2, R)
    assert np.allclose(t2, t)


def test_make_colmap_camera_from_opencv_param_order_and_defaults(
    monkeypatch: pytest.MonkeyPatch,
):
    created: dict[str, object] = {}

    class FakeCamera:
        def __init__(self, model: str, width: int, height: int, params: list[float]):
            created["model"] = model
            created["width"] = width
            created["height"] = height
            created["params"] = list(params)

    monkeypatch.setattr(
        tm, "pycolmap", type("FakePycolmap", (), {"Camera": FakeCamera})
    )

    K = np.array([[100.0, 0.0, 320.0], [0.0, 200.0, 240.0], [0.0, 0.0, 1.0]])
    cam = tm.make_colmap_camera_from_opencv(
        width=640, height=480, K=K, dist=np.array([0.1])
    )
    assert cam is not None
    assert created["model"] == "OPENCV"
    assert created["width"] == 640
    assert created["height"] == 480
    # params: fx, fy, cx, cy, k1, k2, p1, p2
    assert created["params"] == pytest.approx(
        [100.0, 200.0, 320.0, 240.0, 0.1, 0.0, 0.0, 0.0]
    )


def test_rigid3d_from_R_t_passes_quaternion_xyzw_and_translation(
    monkeypatch: pytest.MonkeyPatch,
):
    @dataclass
    class FakeRotation3d:
        xyzw: ArrF64

    @dataclass
    class FakeRigid3d:
        rot: FakeRotation3d
        translation: ArrF64

    class FakePycolmap:
        Rotation3d: type[FakeRotation3d] = FakeRotation3d
        Rigid3d: type[FakeRigid3d] = FakeRigid3d

    monkeypatch.setattr(tm, "pycolmap", FakePycolmap)

    R = _Rz(90.0)
    t = np.array([1.0, 2.0, 3.0], dtype=np.float64)
    rigid = tm.rigid3d_from_R_t(R=R, t=t)

    assert isinstance(rigid, FakeRigid3d)
    assert rigid.translation.shape == (3, 1)
    assert np.allclose(rigid.translation.reshape(3), t)
    q_expected = np.array([0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0)])
    q_expected /= np.linalg.norm(q_expected)
    _quat_same_rotation(rigid.rot.xyzw, q_expected, atol=1e-10)


def test_solve_pnp_mulicam_raises_when_pose_estimation_returns_none(
    monkeypatch: pytest.MonkeyPatch,
):
    class FakeRANSACOptions:
        def __init__(self, max_error: float):
            self.max_error = max_error

    class FakeAbsolutePoseRefinementOptions:
        pass

    def fake_estimate_and_refine_generalized_absolute_pose(
        **kwargs: object,
    ) -> dict[str, object] | None:
        return None

    class FakePycolmap:
        RANSACOptions: type[FakeRANSACOptions] = FakeRANSACOptions
        AbsolutePoseRefinementOptions: type[FakeAbsolutePoseRefinementOptions] = (
            FakeAbsolutePoseRefinementOptions
        )
        estimate_and_refine_generalized_absolute_pose = staticmethod(
            fake_estimate_and_refine_generalized_absolute_pose
        )

        class Camera:
            def __init__(
                self, model: str, width: int, height: int, params: list[float]
            ):
                pass

        class Rotation3d:
            def __init__(self, xyzw: ArrF64):
                pass

        class Rigid3d:
            def __init__(self, rot: object, translation: ArrF64):
                pass

    monkeypatch.setattr(tm, "pycolmap", FakePycolmap)

    tag = tm.CornersAndWorld(
        tag_corners=np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]]),
        T_word_tag_loc=np.eye(4, dtype=np.float64),
        camera_position_in_robot=np.eye(4, dtype=np.float64),
        tag_size=0.2,
        camera_matrix=np.eye(3, dtype=np.float64),
        dist_coeff=np.zeros(5, dtype=np.float64),
        camera_width=640,
        camera_height=480,
    )

    with pytest.raises(RuntimeError, match="Pose estimation failed"):
        tm.solve_pnp_mulicam([tag])


def test_solve_pnp_mulicam_deduplicates_cameras_and_builds_point_arrays(
    monkeypatch: pytest.MonkeyPatch,
):
    calls: list[dict[str, object]] = []

    @dataclass
    class FakeRANSACOptions:
        max_error: float

    @dataclass
    class FakeAbsolutePoseRefinementOptions:
        pass

    @dataclass
    class FakeCamera:
        model: str
        width: int
        height: int
        params: list[float]

    @dataclass
    class FakeRotation3d:
        xyzw: ArrF64

    @dataclass
    class FakeRigid3d:
        rot: FakeRotation3d
        translation: ArrF64

    def fake_estimate_and_refine_generalized_absolute_pose(
        **kwargs: object,
    ) -> dict[str, object] | None:
        calls.append(kwargs)
        pts2d = cast(ArrF64, kwargs["points2D"])
        return {"ok": True, "num_pts": int(pts2d.shape[0])}

    class FakePycolmap:
        Camera: type[FakeCamera] = FakeCamera
        Rotation3d: type[FakeRotation3d] = FakeRotation3d
        Rigid3d: type[FakeRigid3d] = FakeRigid3d
        RANSACOptions: type[FakeRANSACOptions] = FakeRANSACOptions
        AbsolutePoseRefinementOptions: type[FakeAbsolutePoseRefinementOptions] = (
            FakeAbsolutePoseRefinementOptions
        )
        estimate_and_refine_generalized_absolute_pose = staticmethod(
            fake_estimate_and_refine_generalized_absolute_pose
        )

    monkeypatch.setattr(tm, "pycolmap", FakePycolmap)

    # Two tags, same camera intrinsics/extrinsics => should dedupe to 1 camera.
    K = np.array([[500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    dist = np.zeros(4, dtype=np.float64)
    T_cam_in_robot = _Rt_to_T(_Rz(0.0), np.array([0.1, 0.2, 0.3], dtype=np.float64))

    # Place tag1 at origin, tag2 translated in world by +1m x.
    T_w_t1 = np.eye(4, dtype=np.float64)
    T_w_t2 = _Rt_to_T(
        np.eye(3, dtype=np.float64), np.array([1.0, 0.0, 0.0], dtype=np.float64)
    )

    corners2d = np.array([[10.0, 20.0], [30.0, 20.0], [30.0, 40.0], [10.0, 40.0]])
    tag1 = tm.CornersAndWorld(
        tag_corners=corners2d,
        T_word_tag_loc=T_w_t1,
        camera_position_in_robot=T_cam_in_robot,
        tag_size=0.2,
        camera_matrix=K,
        dist_coeff=dist,
        camera_width=640,
        camera_height=480,
    )
    tag2 = tm.CornersAndWorld(
        tag_corners=corners2d + 1.0,  # different 2D observations
        T_word_tag_loc=T_w_t2,
        camera_position_in_robot=T_cam_in_robot,  # same camera => dedupe
        tag_size=0.2,
        camera_matrix=K,
        dist_coeff=dist,
        camera_width=640,
        camera_height=480,
    )

    out = tm.solve_pnp_mulicam([tag1, tag2])
    assert out["ok"] is True
    assert len(calls) == 1

    kwargs = calls[0]
    pts2d = cast(ArrF64, kwargs["points2D"])
    pts3d = cast(ArrF64, kwargs["points3D"])
    cam_idxs = cast(list[int], kwargs["camera_idxs"])
    cams_from_rig = cast(list[object], kwargs["cams_from_rig"])
    cameras = cast(list[object], kwargs["cameras"])

    assert pts2d.shape == (8, 2)
    assert pts3d.shape == (8, 3)
    assert len(cam_idxs) == 8

    # Deduped camera lists
    assert len(cameras) == 1
    assert len(cams_from_rig) == 1
    assert set(cam_idxs) == {0}

    # Options are constructed with expected types/values
    estimation_options = cast(FakeRANSACOptions, kwargs["estimation_options"])
    refinement_options = cast(FakeAbsolutePoseRefinementOptions, kwargs["refinement_options"])
    assert isinstance(estimation_options, FakeRANSACOptions)
    assert float(estimation_options.max_error) == pytest.approx(6.0)
    assert isinstance(refinement_options, FakeAbsolutePoseRefinementOptions)

    # Spot-check the 3D point construction for tag1 corners.
    local = tm._tag_corners_local(0.2)
    expected_t1 = tm._transform_points(T_w_t1, local)
    assert np.allclose(pts3d[:4], expected_t1)
    expected_t2 = tm._transform_points(T_w_t2, local)
    assert np.allclose(pts3d[4:], expected_t2)
