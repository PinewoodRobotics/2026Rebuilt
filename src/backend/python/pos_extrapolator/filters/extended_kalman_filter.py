import time
from typing import Any, Callable
from filterpy.kalman import ExtendedKalmanFilter
import numpy as np
import warnings

from numpy.typing import NDArray
from scipy.linalg import cho_factor, cho_solve
from scipy.stats import chi2

from backend.python.common.util.math import get_np_from_matrix, get_np_from_vector
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterConfig,
    KalmanFilterSensorType,
)
from backend.python.pos_extrapolator.data_prep import (
    ExtrapolationContext,
    KalmanFilterInput,
)
from backend.python.pos_extrapolator.filter_strat import GenericFilterStrategy


class ExtendedKalmanFilterStrategy(  # pyright: ignore[reportUnsafeMultipleInheritance]
    ExtendedKalmanFilter, GenericFilterStrategy
):  # pyright: ignore[reportUnsafeMultipleInheritance]
    def __init__(
        self,
        config: KalmanFilterConfig,
        fake_dt: float | None = None,
    ):
        super().__init__(dim_x=config.dim_x_z[0], dim_z=config.dim_x_z[1])
        self.hw = config.dim_x_z[0]
        self.x = get_np_from_vector(config.state_vector)
        self.P = get_np_from_matrix(config.uncertainty_matrix)
        self.Q = get_np_from_matrix(config.process_noise_matrix)
        self.F = get_np_from_matrix(config.state_transition_matrix)
        self.config = config
        self.R_sensors = self.get_R_sensors(config)
        self.last_update_time = time.time()
        self.fake_dt = fake_dt

    def get_R_sensors(
        self, config: KalmanFilterConfig
    ) -> dict[KalmanFilterSensorType, dict[str, NDArray[np.float64]]]:
        output: dict[KalmanFilterSensorType, dict[str, NDArray[np.float64]]] = {}
        for sensor_type in config.sensors:
            for sensor_id in config.sensors[sensor_type]:
                numpy_arr: NDArray[np.float64] = get_np_from_matrix(
                    config.sensors[sensor_type][sensor_id].measurement_noise_matrix
                )

                output[sensor_type] = output.get(sensor_type, {})
                output[sensor_type][sensor_id] = numpy_arr

        return output

    def jacobian_h(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        return np.eye(7)

    def hx(self, x: NDArray[np.float64]) -> NDArray[np.float64]:
        return x

    def prediction_step(self):
        if self.fake_dt is not None:
            dt = self.fake_dt
        else:
            dt = time.time() - self.last_update_time

        if dt > 5 or dt < 0:
            dt = 0.05

        self._update_transformation_delta_t_with_size(dt)
        self.predict()
        self.last_update_time = time.time()

    def insert_data(self, data: KalmanFilterInput) -> None:
        self.prediction_step()

        if data.sensor_type not in self.R_sensors:
            warnings.warn(
                f"Sensor type {data.sensor_type} not found in R_sensors, skipping update"
            )
            return

        if data.sensor_id not in self.R_sensors[data.sensor_type]:
            warnings.warn(
                f"Sensor id {data.sensor_id} not found in R_sensors, skipping update"
            )
            return

        R_sensor = self.R_sensors[data.sensor_type][data.sensor_id]

        for datapoint in data.get_input_list():
            R = R_sensor.copy() * datapoint.R_multipl
            add_to_diagonal(R, datapoint.R_add)

            self.update(
                datapoint.data,
                data.jacobian_h if data.jacobian_h is not None else self.jacobian_h,
                data.hx if data.hx is not None else self.hx,
                R=R,
            )

    def predict_x_no_update(self, dt: float) -> NDArray[np.float64]:
        self._update_transformation_delta_t_with_size(dt)
        return np.dot(self.F, self.x) + np.dot(self.B, 0)

    def get_state(self, future_s: float | None = None) -> NDArray[np.float64]:
        predicted_x: NDArray[np.float64]
        if future_s is not None and future_s > 0:
            predicted_x = self.predict_x_no_update(future_s)
            self.prediction_step()
        else:
            self.prediction_step()
            predicted_x = self.x

        return predicted_x

    def get_position_confidence(self) -> float:
        P_position = self.P[:2, :2]
        if np.any(np.isnan(P_position)) or np.any(np.isinf(P_position)):
            return 0.0
        try:
            eigenvalues: NDArray[np.float64] = np.real(
                np.linalg.eigvals(P_position)
            )  # pyright: ignore[reportAssignmentType]
            max_eigen = np.max(eigenvalues)
            if np.isnan(max_eigen) or np.isinf(max_eigen) or max_eigen < 0:
                return 0.0
            return 1.0 / (1.0 + np.sqrt(max_eigen))
        except np.linalg.LinAlgError:
            return 0.0

    def get_velocity_confidence(self) -> float:
        P_velocity = self.P[2:4, 2:4]
        if np.any(np.isnan(P_velocity)) or np.any(np.isinf(P_velocity)):
            return 0.0
        try:
            eigenvalues = np.linalg.eigvals(P_velocity)
            max_eigen = np.max(eigenvalues)
            if np.isnan(max_eigen) or np.isinf(max_eigen) or max_eigen < 0:
                return 0.0
            return 1.0 / (1.0 + np.sqrt(max_eigen))
        except np.linalg.LinAlgError:
            return 0.0

    def get_rotation_confidence(self) -> float:
        angle_var = self.P[4, 4] + self.P[5, 5]
        if np.isnan(angle_var) or np.isinf(angle_var) or angle_var < 0:
            return 0.0
        angle_uncertainty = np.sqrt(angle_var)
        return 1.0 / (1.0 + angle_uncertainty)

    def get_confidence(self) -> float:
        pos_conf = self.get_position_confidence()
        vel_conf = self.get_velocity_confidence()
        rot_conf = self.get_rotation_confidence()
        if pos_conf == 0.0 or vel_conf == 0.0 or rot_conf == 0.0:
            return 0.0
        return (pos_conf * vel_conf * rot_conf) ** (1 / 3)

    def _update_transformation_delta_t_with_size(self, new_delta_t: float):
        try:
            vel_idx_x = 2  # vx is at index 2 in [x, y, vx, vy, cos, sin, angular_velocity_rad_s]
            vel_idx_y = 3  # vy is at index 3 in [x, y, vx, vy, cos, sin, angular_velocity_rad_s]
            cos_idx = 4  # cos is at index 4
            sin_idx = 5  # sin is at index 5
            angular_vel_idx = 6  # angular velocity is at index 6 in [x, y, vx, vy, cos, sin, angular_velocity_rad_s]

            # Update position based on velocity
            self.F[0][vel_idx_x] = new_delta_t
            self.F[1][vel_idx_y] = new_delta_t

            # Update rotation (cos/sin) based on angular velocity
            # d(cos)/dt = -sin * omega, d(sin)/dt = cos * omega
            self.F[cos_idx][angular_vel_idx] = -self.x[sin_idx] * new_delta_t
            self.F[sin_idx][angular_vel_idx] = self.x[cos_idx] * new_delta_t
        except IndexError as e:
            warnings.warn(f"Error updating F matrix: {e}")

    def _debug_set_state(self, x: NDArray[np.float64]) -> None:
        self.x = x

    def update(
        self,
        z: NDArray[np.float64],
        HJacobian: Callable[[NDArray[np.float64]], NDArray[np.float64]],
        Hx: Callable[[NDArray[np.float64]], NDArray[np.float64]],
        R: NDArray[np.float64] | None = None,
        args: Any = (),  # pyright: ignore[reportExplicitAny, reportAny]
        hx_args: Any = (),  # pyright: ignore[reportExplicitAny, reportAny]
        residual=np.subtract,  # pyright: ignore[reportUnknownParameterType, reportMissingParameterType]
    ):
        super().update(z, HJacobian, Hx, R, args, hx_args, residual)


def add_to_diagonal(mat: NDArray[np.float64], num: float):
    pass
