import numpy as np
from numpy.typing import NDArray


def wrap_to_pi(angle_rad: float) -> float:
    return float(np.arctan2(np.sin(angle_rad), np.cos(angle_rad)))


def rotation_matrix_2d(theta_rad: float) -> NDArray[np.float64]:
    cos_theta = float(np.cos(theta_rad))
    sin_theta = float(np.sin(theta_rad))
    return np.array(
        [[cos_theta, -sin_theta], [sin_theta, cos_theta]],
        dtype=np.float64,
    )
