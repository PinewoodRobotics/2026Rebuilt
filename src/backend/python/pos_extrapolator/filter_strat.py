import numpy as np
from numpy.typing import NDArray
from typing import TYPE_CHECKING

from backend.generated.proto.python.util.vector_pb2 import Vector2

if TYPE_CHECKING:
    from backend.python.pos_extrapolator.data_prep import KalmanFilterInput


class GenericFilterStrategy:
    """
    Abstract base class for filter strategies.
    """

    kPosXIdx = 0
    kPosYIdx = 1
    kVelXIdx = 2
    kVelYIdx = 3
    kAngleRadIdx = 4
    kAngleVelRadSIdx = 5

    kNumStates = 6
    kNumOutputs = 6

    kStandardDeviationsAwayThreshold = 5.0

    def __init__(self, x: NDArray[np.float64]):
        self.x = x

    def insert_data(self, data: "KalmanFilterInput") -> None:
        raise NotImplementedError("insert_data not implemented")

    def get_state(self, future_s: float | None = None) -> NDArray[np.float64]:
        raise NotImplementedError("get_state not implemented")

    def get_confidence(self) -> float:
        raise NotImplementedError("get_confidence not implemented")

    def get_P(self) -> NDArray[np.float64]:
        # only for kalman filters and usually for debugging purposes
        return np.array([])

    def _debug_set_state(self, x: NDArray[np.float64]) -> None:
        pass

    def pose_2d(self) -> Vector2:
        return Vector2(x=self.x[self.kPosXIdx], y=self.x[self.kPosYIdx])

    def velocity_2d(self) -> Vector2:
        return Vector2(x=self.x[self.kVelXIdx], y=self.x[self.kVelYIdx])

    def angle_vector(self) -> Vector2:
        return Vector2(
            x=np.cos(self.x[self.kAngleRadIdx]), y=np.sin(self.x[self.kAngleRadIdx])
        )

    def angle_rad(self) -> float:
        return self.x[self.kAngleRadIdx]

    def angle_matrix(self) -> NDArray[np.float64]:
        cos = np.cos(self.x[self.kAngleRadIdx])
        sin = np.sin(self.x[self.kAngleRadIdx])
        return np.array([[cos, -sin], [sin, cos]])

    def angle(self) -> NDArray[np.float64]:
        return np.array(
            [np.cos(self.x[self.kAngleRadIdx]), np.sin(self.x[self.kAngleRadIdx])]
        )

    def direction_vector(self) -> Vector2:
        return Vector2(
            x=np.cos(self.x[self.kAngleRadIdx]), y=np.sin(self.x[self.kAngleRadIdx])
        )

    def angular_velocity_rad(self) -> float:
        return self.x[self.kAngleVelRadSIdx]
