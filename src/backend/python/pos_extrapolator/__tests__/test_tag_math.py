import numpy as np
import pytest

from backend.python.pos_extrapolator.preparers.util.tag_math import solve_pnp_single_image


@pytest.mark.xfail(reason="solve_pnp_single_image() is currently a stub returning None")
def test_solve_pnp_single_image_returns_pose_for_minimal_valid_inputs():
    # Minimal-but-structurally-valid inputs; exact math is not asserted here.
    # Note: the production signature uses NDArray keys, which are not hashable in Python;
    # we pass a simple mapping here because the current implementation is a stub anyway.
    corners_to_T_world = {"corners0": np.eye(4, dtype=float)}
    out = solve_pnp_single_image(
        corners_to_T_world=corners_to_T_world,
        T_camera_in_world=np.eye(4, dtype=float),
        camera_matrix=np.eye(3, dtype=float),
        dist_coeff=np.zeros(5, dtype=float),
    )
    assert out is not None
