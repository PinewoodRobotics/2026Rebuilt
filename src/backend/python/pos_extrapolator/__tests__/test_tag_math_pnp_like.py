from __future__ import annotations

import numpy as np
from numpy.typing import NDArray
import pytest
from typing import cast
import time

import cv2
import pycolmap

from backend.generated.proto.python.sensor.apriltags_pb2 import UnprocessedTag
from backend.generated.thrift.config.apriltag.ttypes import AprilDetectionConfig
from backend.python.april.src.__tests__.util import add_cur_dir, get_all_generated_tags
from backend.python.april.src.tag_detector import TagDetector
from backend.python.april.src.util import (
    post_process_detection,
    process_image,
    solve_pnp_tag_corners,
)
from backend.python.pos_extrapolator.preparers.util.tag_math import (
    CornersAndWorld,
    solve_pnp_mulicam,
)


def _get_detector() -> TagDetector:
    # Use April's wrapper so `process_image(...)` is strongly typed.
    cfg = AprilDetectionConfig(
        tag_size=1.0,
        family="tag36h11",
        nthreads=4,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
        searchpath=[],
        debug=False,
        post_tag_output_topic=None,
        send_stats=False,
        stats_topic="",
        pi_name_to_special_detector_config={},
    )
    return TagDetector.use_cpu(cfg)


def _cam_1_matrix() -> NDArray[np.float64]:
    # Copied from `backend.python.april.src.__tests__.test_util`.
    return np.array(
        [
            [450.0, 0.0, 400.0],
            [0.0, 450.0, 300.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def _cam_1_dist_coeff() -> NDArray[np.float64]:
    # Copied from `backend.python.april.src.__tests__.test_util`.
    return np.array(
        [
            0.02421893240091871,
            -0.019483745628435203,
            -0.0002353973963860865,
            0.0010526889774734997,
            -0.032565426732841275,
        ],
        dtype=np.float64,
    )


def _solve_with_tag_math(
    *,
    tag: UnprocessedTag,
    tag_size: float,
    camera_matrix: NDArray[np.float64],
    dist_coeff: NDArray[np.float64],
    width: int,
    height: int,
) -> NDArray[np.float64]:
    # Treat the tag-local frame as "world" (no global tag map needed).
    # Single camera, no multi-cam rig: identity camera-in-robot transform.
    corners = np.array([[c.x, c.y] for c in tag.corners], dtype=np.float64)
    result = solve_pnp_mulicam(
        [
            CornersAndWorld(
                tag_corners=corners,
                T_word_tag_loc=np.eye(4, dtype=np.float64),
                camera_position_in_robot=np.eye(4, dtype=np.float64),
                tag_size=tag_size,
                camera_matrix=camera_matrix,
                dist_coeff=dist_coeff,
                camera_width=width,
                camera_height=height,
            )
        ]
    )
    rig_from_world = cast(pycolmap.Rigid3d, result["rig_from_world"])
    # pycolmap.Rigid3d.translation is array-like length 3
    return cast(
        NDArray[np.float64],
        np.asarray(rig_from_world.translation, dtype=np.float64).reshape((3,)),
    )


@pytest.mark.parametrize(
    ("image_rel", "tag_size", "abs_tol"),
    [
        ("fixtures/images/cam_1_tag_6_37cm_d.png", 0.17, 0.02),
        ("fixtures/images/cam_1_tag_6_74cm_d.png", 0.17, 0.02),
        ("fixtures/images/cam_1_tag_6_90cm_d.png", 0.17, 0.01),
    ],
)
def test_solve_pnp_mulicam_matches_april_solve_pnp_tag_corners(
    image_rel: str, tag_size: float, abs_tol: float
) -> None:
    """
    Mirror of April's "regular PnP" tests, but using `solve_pnp_mulicam`.

    We intentionally follow the same detection + post-process pipeline as April's tests.
    """
    image_path = add_cur_dir(image_rel)
    image = cv2.imread(image_path)
    assert image is not None
    image_np = cast(NDArray[np.uint8], image)

    detector = _get_detector()
    K = _cam_1_matrix()
    dist = _cam_1_dist_coeff()

    detections = process_image(image_np, detector)
    tags = post_process_detection(detections, K, dist)
    assert len(tags) == 1

    # Reference: OpenCV solvePnP pipeline used by April.
    _R_cv, t_cv = solve_pnp_tag_corners(tags[0], tag_size, K, dist)
    t_cv = cast(NDArray[np.float64], np.asarray(t_cv, dtype=np.float64).reshape((3,)))

    # Under these conditions (identity rig), tag_math returns rig_from_world whose
    # translation matches solvePnP's tvec (world/tag -> camera).
    t_colmap = _solve_with_tag_math(
        tag=tags[0],
        tag_size=tag_size,
        camera_matrix=K,
        dist_coeff=dist,
        width=int(image_np.shape[1]),
        height=int(image_np.shape[0]),
    )

    np.testing.assert_allclose(t_colmap, t_cv, atol=abs_tol, rtol=0.0)


def test_solve_pnp_mulicam_generated_tags_matches_ground_truth_translation():
    """
    Mirror of April's `test_generated_tags`, but using `solve_pnp_mulicam`.

    The generated fixtures include a ground-truth pose position (translation).
    """
    all_generated_tags = get_all_generated_tags()
    K = np.array(
        [
            [691.5, 0.0, 691.0],
            [0.0, 691.5, 446.5],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    dist = np.zeros(5, dtype=np.float64)

    detector = _get_detector()
    for generated_tag in all_generated_tags:
        img_np = cast(NDArray[np.uint8], np.asarray(generated_tag.image))
        detections = process_image(img_np, detector)
        tags = post_process_detection(detections, K, dist)
        assert len(tags) == 1

        t_colmap = _solve_with_tag_math(
            tag=tags[0],
            tag_size=1.0,
            camera_matrix=K,
            dist_coeff=dist,
            width=int(img_np.shape[1]),
            height=int(img_np.shape[0]),
        )

        gt = cast(
            NDArray[np.float64],
            np.asarray(generated_tag.data.pose.position, dtype=np.float64).reshape(
                (3,)
            ),
        )
        np.testing.assert_allclose(t_colmap, gt, atol=0.05, rtol=0.0)


def test_solve_pnp_mulicam_timing_per_image(capsys: pytest.CaptureFixture[str]) -> None:
    """
    Timing smoke-test that prints per-stage and solver-only timings.

    Uses `capsys.disabled()` so output is shown during normal `pytest` runs.
    """
    detector = _get_detector()

    cases: list[
        tuple[str, NDArray[np.uint8], float, NDArray[np.float64], NDArray[np.float64]]
    ] = []

    # A few real fixture images (same ones used in the correctness tests).
    K1 = _cam_1_matrix()
    dist1 = _cam_1_dist_coeff()
    for image_rel in [
        "fixtures/images/cam_1_tag_6_37cm_d.png",
        "fixtures/images/cam_1_tag_6_74cm_d.png",
        "fixtures/images/cam_1_tag_6_90cm_d.png",
    ]:
        image_path = add_cur_dir(image_rel)
        image = cv2.imread(image_path)
        assert image is not None
        cases.append((image_rel, cast(NDArray[np.uint8], image), 0.17, K1, dist1))

    # Generated-tag fixtures (already include ground-truth pose). Keep output manageable.
    Kgen = np.array(
        [
            [691.5, 0.0, 691.0],
            [0.0, 691.5, 446.5],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    distgen = np.zeros(5, dtype=np.float64)
    for i, generated_tag in enumerate(get_all_generated_tags()[:10]):
        img_np = cast(NDArray[np.uint8], np.asarray(generated_tag.image))
        cases.append((f"generated[{i}]", img_np, 1.0, Kgen, distgen))

    with capsys.disabled():
        print("\n[timing] solve_pnp_mulicam timing breakdown")
        print(
            "[timing] note: detection dominates end-to-end; solver-only should be ~ms-level"
        )
        for name, img, tag_size, K, dist in cases:
            # 1) Time detection once
            t0 = time.perf_counter()
            detections = process_image(img, detector)
            t1 = time.perf_counter()

            # 2) Time post-processing (undistort) once
            tags = post_process_detection(detections, K, dist)
            t2 = time.perf_counter()
            assert len(tags) == 1

            # 3) Build `CornersAndWorld` once (so solver-only timing is fair)
            corners = np.array([[c.x, c.y] for c in tags[0].corners], dtype=np.float64)
            solve_input = [
                CornersAndWorld(
                    tag_corners=corners,
                    T_word_tag_loc=np.eye(4, dtype=np.float64),
                    camera_position_in_robot=np.eye(4, dtype=np.float64),
                    tag_size=tag_size,
                    camera_matrix=K,
                    dist_coeff=dist,
                    camera_width=int(img.shape[1]),
                    camera_height=int(img.shape[0]),
                )
            ]

            # 4) Warmup + estimate how many repeats to run (~100ms of solver time)
            _ = solve_pnp_mulicam(solve_input)
            s0 = time.perf_counter()
            _ = solve_pnp_mulicam(solve_input)
            s1 = time.perf_counter()
            single_solve_s = max(s1 - s0, 1e-6)
            repeats = int(0.10 / single_solve_s)
            repeats = max(5, min(50, repeats))

            # 5) Time solver-only
            elapsed_solve_s: list[float] = []
            for _ in range(repeats):
                a0 = time.perf_counter()
                result = solve_pnp_mulicam(solve_input)
                a1 = time.perf_counter()
                rig_from_world = cast(pycolmap.Rigid3d, result["rig_from_world"])
                t = np.asarray(rig_from_world.translation, dtype=np.float64).reshape(
                    (3,)
                )
                assert np.isfinite(t).all()
                elapsed_solve_s.append(a1 - a0)

            detect_ms = (t1 - t0) * 1000.0
            post_ms = (t2 - t1) * 1000.0
            avg_solve_ms = (sum(elapsed_solve_s) / len(elapsed_solve_s)) * 1000.0
            total_ms = detect_ms + post_ms + avg_solve_ms
            line = (
                f"[timing] {name}: detect={detect_ms:.2f}ms  post={post_ms:.2f}ms  "
                f"solve_pnp_mulicam={avg_solve_ms:.3f}ms/call (avg over {repeats})  "
                f"approx_total={total_ms:.2f}ms"
            )
            print(line)
