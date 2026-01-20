import numpy as np
from numpy.typing import NDArray


def solve_pnp_single_image(
    corners_to_T_world: dict[NDArray[np.float64], NDArray[np.float64]],
    T_camera_in_world: NDArray[np.float64],
    camera_matrix: NDArray[np.float64],
    dist_coeff: NDArray[np.float64],
) -> NDArray[np.float64] | None:
    return None
