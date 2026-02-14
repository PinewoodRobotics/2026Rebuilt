import typing
import numpy as np
from numpy.typing import NDArray
from typing import Sequence


class GenericMatrix(object):
    """
    Attributes:
     - values
     - rows
     - cols

    """

    thrift_spec: typing.Any = None

    def __init__(
        self,
        values: list[list[float]] = None,
        rows: int = None,
        cols: int = None,
    ):
        self.values: list[list[float]] = values
        self.rows: int = rows
        self.cols: int = cols


def from_quaternion_no_roll_zyx(q: Sequence[float]) -> NDArray[np.float64]:
    """
    Converts a WPILib quaternion (WXYZ order, NWU coordinate system) to a rotation matrix.
    The quaternion represents the rotation of the tag's coordinate frame in the NWU world frame.
    Returns a 3x3 rotation matrix as a numpy array.
    """
    w, x, y, z = q
    n = (w**2 + x**2 + y**2 + z**2) ** 0.5 or 1
    w /= n
    x /= n
    y /= n
    z /= n

    # Standard quaternion (WXYZ) to rotation matrix conversion
    r11 = 1 - 2 * (y * y + z * z)
    r12 = 2 * (x * y - w * z)
    r13 = 2 * (x * z + w * y)
    r21 = 2 * (x * y + w * z)
    r22 = 1 - 2 * (x * x + z * z)
    r23 = 2 * (y * z - w * x)
    r31 = 2 * (x * z - w * y)
    r32 = 2 * (y * z + w * x)
    r33 = 1 - 2 * (x * x + y * y)

    return np.array(
        [
            [r11, r12, r13],
            [r21, r22, r23],
            [r31, r32, r33],
        ]
    )


def build_rotation_matrix_from_yaw(yaw_degrees: float) -> NDArray[np.float64]:
    yaw_radians = np.deg2rad(yaw_degrees)
    cos = np.cos(yaw_radians)
    sin = np.sin(yaw_radians)
    return np.array(
        [[cos, -sin, 0.0], [sin, cos, 0.0], [0.0, 0.0, 1.0]], dtype=np.float64
    )


def get_np_from_matrix(
    matrix: GenericMatrix,
) -> NDArray[np.float64]:
    return np.array(matrix.values)


if __name__ == "__main__":
    """
    intended output:
    [  1.0000000,  0.0000000,  0.0000000],
    [  0.0000000,  0.5892082, -0.8079813],
    [  0.0000000,  0.8079813,  0.5892082 ]
    """
    q = [0.0, 0.0, 0.0, 1.0]

    print(
        get_np_from_matrix(
            GenericMatrix(
                values=[
                    [1.0, 0.0, 0.0],
                    [0.0, 0.5892082, -0.8079813],
                    [0.0, 0.8079813, 0.5892082],
                ],
                rows=3,
                cols=3,
            )
        )
    )
