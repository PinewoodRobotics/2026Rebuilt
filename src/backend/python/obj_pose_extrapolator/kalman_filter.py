import time
from filterpy.kalman import KalmanFilter
import numpy as np

from backend.generated.thrift.config.obj_pose_extrapolator.ttypes import (
    KalmanFilterConfig,
)


class ObjPersonalKalmanFilter(KalmanFilter):
    def __init__(self, config: KalmanFilterConfig, initial_state: list[float]):
        super().__init__(dim_x=4, dim_z=4)  # x, y, vx, vy. no other data can be used.
        self.x = np.array(initial_state)
        self.P = np.array(
            config.uncertainty_matrix, dtype=np.float64
        )  # Covariance matrix
        self.Q = np.array(
            config.process_noise_matrix, dtype=np.float64
        )  # Process noise
        self.H = np.eye(
            4, dtype=np.float64
        )  # Measurement matrix (state directly observed)
        self.R = np.array(
            config.measurement_noise_matrix, dtype=np.float64
        )  # Measurement noise
        self.config = config

        self.last_update_time = time.time()

    def predict_with_time_replacement(self):
        dt = time.time() - self.last_update_time
        if dt < 0:
            dt = 0.05

        super().predict(
            F=np.array(
                [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
                dtype=np.float64,
            )
        )

    def insert_sensor_data(self, data: list[float]):
        self.predict_with_time_replacement()
        self.update(np.array(data))
        self.last_update_time = time.time()

    def get_state(self) -> list[float]:
        return self.x.tolist()
