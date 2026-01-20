from dataclasses import dataclass
import math
from typing import Callable, Literal, cast
import numpy as np
from numpy.typing import NDArray
import pycolmap

QuatXYWZ = np.ndarray[tuple[Literal[4]], np.dtype[np.float64]]


def _make_ransac_options_fast() -> pycolmap.RANSACOptions:
    # With AprilTag corners, we generally do not expect outliers within a single tag.
    # The default COLMAP settings are *very* expensive (min_num_trials=1000, max_num_trials=100000).
    # Tuning these down is critical for real-time use.
    return pycolmap.RANSACOptions(
        max_error=4.0,
        confidence=0.9999,
        dyn_num_trials_multiplier=3.0,
        min_inlier_ratio=0.25,
        min_num_trials=1,
        max_num_trials=10000,
        random_seed=0,
    )


def _make_refinement_options_fast() -> pycolmap.AbsolutePoseRefinementOptions:
    # Keep refinement cheap; default is 100 iterations.
    return pycolmap.AbsolutePoseRefinementOptions(
        max_num_iterations=1000,
        # Larger tolerance terminates earlier; 1.0 is COLMAP default.
        gradient_tolerance=1.0,
        loss_function_scale=1.0,
        refine_focal_length=False,
        refine_extra_params=False,
        print_summary=False,
    )


# Cache COLMAP camera objects across calls, namespaced by the current `pycolmap`
# module object (tests monkeypatch it).
_CAMERA_CACHE_BY_PYCOLMAP: dict[
    int, dict[bytes, tuple[pycolmap.Camera, pycolmap.Rigid3d]]
] = {}


@dataclass
class CornersAndWorld:
    tag_corners: NDArray[np.float64]
    T_word_tag_loc: NDArray[np.float64]
    camera_position_in_robot: NDArray[np.float64]
    tag_size: float

    camera_matrix: NDArray[np.float64]
    dist_coeff: NDArray[np.float64]
    camera_width: int
    camera_height: int


def make_colmap_camera_from_opencv(
    width: int,
    height: int,
    K: NDArray[np.float64],
    dist: NDArray[np.float64],
):
    K33 = cast(
        np.ndarray[tuple[Literal[3], Literal[3]], np.dtype[np.float64]],
        np.asarray(K, dtype=np.float64).reshape((3, 3)),
    )
    fx, fy = float(cast(np.float64, K33[0, 0])), float(cast(np.float64, K33[1, 1]))
    cx, cy = float(cast(np.float64, K33[0, 2])), float(cast(np.float64, K33[1, 2]))

    dist_flat = np.asarray(dist, dtype=np.float64).reshape(-1)

    # Use COLMAP's OPENCV model (fx, fy, cx, cy, k1, k2, p1, p2)
    # Parameter ordering for OPENCV is fx, fy, cx, cy, k1, k2, p1, p2.
    # [oai_citation:2‡GitHub](https://github.com/colmap/colmap/blob/main/src/colmap/sensor/models.h)
    k1 = float(cast(np.float64, dist_flat[0])) if dist_flat.size > 0 else 0.0
    k2 = float(cast(np.float64, dist_flat[1])) if dist_flat.size > 1 else 0.0
    p1 = float(cast(np.float64, dist_flat[2])) if dist_flat.size > 2 else 0.0
    p2 = float(cast(np.float64, dist_flat[3])) if dist_flat.size > 3 else 0.0
    params = [fx, fy, cx, cy, k1, k2, p1, p2]

    cam = pycolmap.Camera(
        model="OPENCV",
        width=width,
        height=height,
        params=params,
    )
    return cam


def rigid3d_from_R_t(
    R: NDArray[np.float64], t: NDArray[np.float64]
) -> pycolmap.Rigid3d:
    rot = pycolmap.Rotation3d(xyzw=_quat_xyzw_from_R(R))
    translation = cast(
        np.ndarray[tuple[Literal[3], Literal[1]], np.dtype[np.float64]],
        t.reshape((3, 1)),
    )
    return pycolmap.Rigid3d(rot, translation)


def _quat_xyzw_from_R(R: NDArray[np.float64]) -> QuatXYWZ:
    """
    Convert a 3x3 rotation matrix to a quaternion in (x, y, z, w) order.

    This matches pycolmap's type stubs for `Rotation3d(xyzw=...)`.
    """
    R33 = cast(
        np.ndarray[tuple[Literal[3], Literal[3]], np.dtype[np.float64]],
        np.asarray(R, dtype=np.float64).reshape((3, 3)),
    )
    m00, m01, m02 = (
        float(cast(np.float64, R33[0, 0])),
        float(cast(np.float64, R33[0, 1])),
        float(cast(np.float64, R33[0, 2])),
    )
    m10, m11, m12 = (
        float(cast(np.float64, R33[1, 0])),
        float(cast(np.float64, R33[1, 1])),
        float(cast(np.float64, R33[1, 2])),
    )
    m20, m21, m22 = (
        float(cast(np.float64, R33[2, 0])),
        float(cast(np.float64, R33[2, 1])),
        float(cast(np.float64, R33[2, 2])),
    )

    tr = m00 + m11 + m22
    if tr > 0.0:
        s = math.sqrt(tr + 1.0) * 2.0
        w = 0.25 * s
        x = (m21 - m12) / s
        y = (m02 - m20) / s
        z = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    q = cast(QuatXYWZ, np.array([x, y, z, w], dtype=np.float64))
    n = float(np.linalg.norm(q))
    if n > 0.0:
        q /= n
    return q


def _tag_corners_local(tag_size: float) -> NDArray[np.float64]:
    half = float(tag_size) / 2.0
    return np.array(
        [
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ],
        dtype=np.float64,
    )


def _transform_points(
    T: NDArray[np.float64], pts: NDArray[np.float64]
) -> NDArray[np.float64]:
    R, t = T[:3, :3], T[:3, 3]
    return (pts @ R.T) + t.reshape(1, 3)


def componentize_transformation_matrix(
    T: NDArray[np.float64],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    R = T[:3, :3]
    t = T[:3, 3]
    return R, t


def solve_pnp_mulicam(tags: list[CornersAndWorld]) -> dict[str, object]:
    # Deduplicate cameras by their intrinsics+extrinsics so camera indices stay stable.
    cam_key_to_idx: dict[bytes, int] = {}
    cameras: list[pycolmap.Camera] = []
    cams_from_rig: list[pycolmap.Rigid3d] = []
    camera_cache = _CAMERA_CACHE_BY_PYCOLMAP.setdefault(id(pycolmap), {})

    # Pre-allocate for speed: 4 corners per tag.
    num_pts = 4 * len(tags)
    points2D_np = cast(
        np.ndarray[tuple[int, Literal[2]], np.dtype[np.float64]],
        np.empty((num_pts, 2), dtype=np.float64),
    )
    points3D_np = cast(
        np.ndarray[tuple[int, Literal[3]], np.dtype[np.float64]],
        np.empty((num_pts, 3), dtype=np.float64),
    )
    camera_idxs: list[int] = [0] * num_pts
    k = 0

    for tag in tags:
        key = (
            int(tag.camera_width).to_bytes(4, "little", signed=False)
            + int(tag.camera_height).to_bytes(4, "little", signed=False)
            + np.ascontiguousarray(tag.camera_matrix).tobytes()
            + np.ascontiguousarray(tag.dist_coeff).tobytes()
            + np.ascontiguousarray(tag.camera_position_in_robot).tobytes()
        )

        cam_idx = cam_key_to_idx.get(key)
        if cam_idx is None:
            cam_idx = len(cameras)
            cam_key_to_idx[key] = cam_idx

            cached = camera_cache.get(key)
            if cached is None:
                cam = make_colmap_camera_from_opencv(
                    tag.camera_width,
                    tag.camera_height,
                    tag.camera_matrix,
                    tag.dist_coeff,
                )
                Rcr, tcr = componentize_transformation_matrix(
                    tag.camera_position_in_robot
                )
                cam_from_rig = rigid3d_from_R_t(Rcr, tcr)
                cached = (cam, cam_from_rig)
                camera_cache[key] = cached

            cam, cam_from_rig = cached
            cameras.append(cam)
            cams_from_rig.append(cam_from_rig)

        corners2d = cast(
            np.ndarray[tuple[Literal[4], Literal[2]], np.dtype[np.float64]],
            np.asarray(tag.tag_corners, dtype=np.float64).reshape((4, 2)),
        )
        corners3d_local = _tag_corners_local(tag.tag_size)
        corners3d = cast(
            np.ndarray[tuple[Literal[4], Literal[3]], np.dtype[np.float64]],
            _transform_points(tag.T_word_tag_loc, corners3d_local).reshape((4, 3)),
        )

        for i in range(4):
            points2D_np[k, :] = corners2d[i]
            points3D_np[k, :] = corners3d[i]
            camera_idxs[k] = int(cam_idx)
            k += 1

    estimate = cast(
        Callable[..., dict[str, object] | None],
        pycolmap.estimate_and_refine_generalized_absolute_pose,
    )
    # Tests monkeypatch `pycolmap` with minimal fake option classes, so we fall back to
    # constructing the simplest compatible options when needed.
    try:
        estimation_options = _make_ransac_options_fast()
    except TypeError:
        estimation_options = pycolmap.RANSACOptions(max_error=6.0)

    try:
        refinement_options = _make_refinement_options_fast()
    except TypeError:
        refinement_options = pycolmap.AbsolutePoseRefinementOptions()

    result = estimate(
        points2D=points2D_np,
        points3D=points3D_np,
        camera_idxs=camera_idxs,
        cams_from_rig=cams_from_rig,
        cameras=cameras,
        estimation_options=estimation_options,
        refinement_options=refinement_options,
        return_covariance=False,
    )

    if result is None:
        raise RuntimeError("Pose estimation failed")

    return result
