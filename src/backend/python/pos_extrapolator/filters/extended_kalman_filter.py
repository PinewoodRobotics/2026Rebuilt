import time
from typing import Any, Callable
from enum import Enum
from filterpy.kalman import ExtendedKalmanFilter
import numpy as np
import warnings

from numpy.typing import NDArray

from backend.python.common.debug.logger import warning
from backend.python.common.util.math import (
    get_np_from_matrix,
    get_np_from_vector,
    transform_matrix_to_size,
    transform_vector_to_size,
)
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterConfig,
    KalmanFilterSensorType,
)
from backend.python.pos_extrapolator.data_prep import (
    KalmanFilterInput,
)
from backend.python.pos_extrapolator.filter_strat import GenericFilterStrategy


def _wrap_to_pi(angle_rad: float) -> float:
    return float(np.arctan2(np.sin(angle_rad), np.cos(angle_rad)))


def _add_to_diagonal(mat: NDArray[np.float64], num: float):
    for i in range(min(mat.shape[0], mat.shape[1])):
        mat[i, i] += num


def _residual_with_angle_wrap(
    z: NDArray[np.float64],
    h_x: NDArray[np.float64],
    angle_measurement_idx: int | None,
) -> NDArray[np.float64]:
    """
    Residual with angle wrap. This is used to wrap the angle residual to the range [-π, π].
    Residual definition:
      A function that returns the difference between the measurement and the prediction.
      Essentially a vector subtraction function specific to the filter.

    Args:
        z: Measurement vector.
        h_x: Prediction vector.

    Returns:
        Residual vector with angle wrapped to the range [-π, π].

    Reason needed:
      The angle residual is not automatically wrapped to the range [-π, π] by the filter so it will bug out when
      the angle goes outside of this range or changes from -pi to pi (0 angle difference read as huge change).
    """

    residual = np.subtract(z, h_x)
    if angle_measurement_idx is None:
        return residual

    if 0 <= angle_measurement_idx < len(residual):
        residual[angle_measurement_idx] = _wrap_to_pi(
            float(residual[angle_measurement_idx])
        )

    return residual


class FilterStateType(Enum):
    POS_X = GenericFilterStrategy.kPosXIdx
    POS_Y = GenericFilterStrategy.kPosYIdx
    VEL_X = GenericFilterStrategy.kVelXIdx
    VEL_Y = GenericFilterStrategy.kVelYIdx
    ANGLE_RAD = GenericFilterStrategy.kAngleRadIdx
    ANGLE_VEL_RAD_S = GenericFilterStrategy.kAngleVelRadSIdx


class ExtendedKalmanFilterStrategy(ExtendedKalmanFilter, GenericFilterStrategy):
    def __init__(
        self,
        config: KalmanFilterConfig,
        fake_dt: float | None = None,
    ):
        ExtendedKalmanFilter.__init__(
            self,
            dim_x=GenericFilterStrategy.kNumStates,
            dim_z=GenericFilterStrategy.kNumOutputs,
        )
        GenericFilterStrategy.__init__(self, x=self.x)

        self.hw = GenericFilterStrategy.kNumStates
        self.x = get_np_from_vector(config.initial_state_vector)
        self.P = get_np_from_matrix(config.uncertainty_matrix)
        self.Q = get_np_from_matrix(config.process_noise_matrix)
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

    @staticmethod
    def generic_jacobian_h(
        used_indices: list[bool],
    ) -> Callable[[NDArray[np.float64]], NDArray[np.float64]]:
        """
        Returns a function that returns the Jacobian of the measurement function. This is generalized for common preparation steps.
        """
        return lambda _: transform_matrix_to_size(
            np.eye(len(used_indices)), used_indices
        )

    @staticmethod
    def generic_hx(
        used_indices: list[bool],
    ) -> Callable[[NDArray[np.float64]], NDArray[np.float64]]:
        """
        Returns a function that returns the measurement function. This is generalized for common preparation steps.
        """
        return lambda x: transform_vector_to_size(x, used_indices)

    def get_R(self) -> NDArray[np.float64]:
        return self.R

    def _infer_measurement_idx(
        self,
        jacobian_h: Callable[[NDArray[np.float64]], NDArray[np.float64]],
        state_type: FilterStateType,
    ) -> int | None:
        """
        Infer which measurement index maps to a given state component from H Jacobian.
        Returns None if this measurement does not include the requested state.
        """
        state_idx = state_type.value
        H = jacobian_h(self.x)
        if H.ndim != 2 or H.shape[1] <= state_idx:
            return None

        state_col = np.abs(H[:, state_idx])
        if state_col.size == 0:
            return None

        idx = int(np.argmax(state_col))
        if float(state_col[idx]) <= 1e-12:
            return None
        return idx

    def prediction_step(self):
        if self.fake_dt is not None:
            dt = self.fake_dt
        else:
            dt = time.time() - self.last_update_time

        if dt > 5 or dt < 0:
            dt = 0.05

        self.set_delta_t(dt)
        self.predict()

        self.last_update_time = time.time()

    def insert_data(self, data: KalmanFilterInput) -> None:
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

        if (
            self.get_standard_deviations_away(
                data.get_input(), [FilterStateType.POS_X, FilterStateType.POS_Y]
            )
            > self.kStandardDeviationsAwayThreshold
        ):
            warning(f"Position is too far away from expected position, skipping update")
            return

        self.prediction_step()

        R_sensor = self.R_sensors[data.sensor_type][data.sensor_id]

        jacobian_h = (
            data.jacobian_h
            if data.jacobian_h is not None
            else self.generic_jacobian_h(GenericFilterStrategy.kNumStates * [True])
        )
        hx = (
            data.hx
            if data.hx is not None
            else self.generic_hx(GenericFilterStrategy.kNumStates * [True])
        )
        angle_measurement_idx = self._infer_measurement_idx(
            jacobian_h, FilterStateType.ANGLE_RAD
        )

        R = R_sensor.copy() * data.R_mult
        _add_to_diagonal(R, data.R_add)

        residual_fn = lambda z, h_x: _residual_with_angle_wrap(
            z, h_x, angle_measurement_idx
        )
        self.update(
            data.get_input(),
            jacobian_h,
            hx,
            R=R,
            residual=residual_fn,
        )

    def get_standard_deviations_away(
        self,
        state: NDArray[np.float64],
        state_types: list[FilterStateType] | FilterStateType,
    ) -> float:
        """Mahalanobis distance (number of std devs) of state from current estimate."""
        types = (
            [state_types]
            if isinstance(state_types, FilterStateType)
            else list(state_types)
        )
        idx = np.array([t.value for t in types])
        r = np.asarray(state, dtype=np.float64).flatten()[idx] - self.x[idx]
        P_sub = self.P[np.ix_(idx, idx)]
        if len(idx) == 1:
            var = float(P_sub[0, 0])
            return float("inf") if var <= 0 else float(np.abs(r[0]) / np.sqrt(var))
        try:
            d_sq = float(r @ np.linalg.solve(P_sub, r))
        except np.linalg.LinAlgError:
            return float("inf")
        return float(np.sqrt(max(0.0, d_sq)))

    def get_P(self) -> NDArray[np.float64]:
        return self.P

    def predict_x_no_update(self, dt: float) -> NDArray[np.float64]:
        self.set_delta_t(dt)
        return np.dot(self.F, self.x) + np.dot(self.B, 0)

    def get_state(self, future_s: float | None = None) -> NDArray[np.float64]:
        predicted_x: NDArray[np.float64] = self.x
        if future_s is not None and future_s > 0:
            predicted_x = self.predict_x_no_update(future_s)

        self.prediction_step()

        return predicted_x

    def get_confidence(self) -> float:
        return 1.0

    def set_delta_t(self, delta_t: float):
        """
        Sets the delta_t in the F matrix (state transition matrix which is multiplied by the state to get the next state).
        This is used because the delta_t is not constant for the filter.
        """

        try:
            self.F[GenericFilterStrategy.kPosXIdx][
                GenericFilterStrategy.kVelXIdx
            ] = delta_t  # vx innovation
            self.F[GenericFilterStrategy.kPosYIdx][
                GenericFilterStrategy.kVelYIdx
            ] = delta_t  # vy innovation

            self.F[GenericFilterStrategy.kAngleRadIdx][
                GenericFilterStrategy.kAngleVelRadSIdx
            ] = delta_t  # angular velocity innovation
        except IndexError as e:
            warnings.warn(f"Error setting delta_t in F matrix: {e}")

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


T_EKF = ExtendedKalmanFilterStrategy  # alias for the class
