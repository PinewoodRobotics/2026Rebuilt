# Based on https://www.chiefdelphi.com/t/kalman-filter-tuning-and-gps

from __future__ import annotations

import time
from typing import Any, Callable
from filterpy.kalman import ExtendedKalmanFilter
import numpy as np
from numpy.typing import NDArray

from backend.generated.proto.python.sensor.apriltags_pb2 import AprilTagData
from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.generated.proto.python.util.position_pb2 import RobotPosition
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterSensorType,
)
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    DataSources,
    PosExtrapolator,
)
from backend.generated.thrift.config.ttypes import Config
from backend.python.pos_extrapolator.processor_registry import (
    AllowedSensors,
    get_processor,
)
from backend.python.pos_extrapolator.util.solver_models import (
    CachedFilterState,
    MotionInput,
    SensorEvent,
    SensorPayload,
)
from backend.python.pos_extrapolator.util.time_conversion import SensorsTimeConverter
from backend.python.pos_extrapolator.util.conversion import (
    get_R_sensors,
    load_matrix,
    load_vector,
)
from backend.python.pos_extrapolator.util.extrapolator_math import (
    wrap_to_pi,
    rotation_matrix_2d,
)
from backend.python.common.debug.logger import error

SENSOR_TYPE_TO_SOURCE: dict[AllowedSensors, DataSources] = {
    KalmanFilterSensorType.APRIL_TAG: DataSources.APRIL_TAG,
    KalmanFilterSensorType.ODOMETRY: DataSources.ODOMETRY,
    KalmanFilterSensorType.IMU: DataSources.IMU,
}


def residual_general(
    measurement: NDArray[np.float64],
    estimate: NDArray[np.float64],
    theta_idx: int | None = None,
) -> NDArray[np.float64]:
    delta = measurement - estimate
    if theta_idx is not None:
        delta[theta_idx] = wrap_to_pi(float(delta[theta_idx]))

    return delta


def residual(
    measurement: NDArray[np.float64], estimate: NDArray[np.float64]
) -> NDArray[np.float64]:
    return residual_general(measurement, estimate, PositionSolver2d.kThetaIdx)


class CachedHistory:
    def __init__(self, max_history_length: int):
        self.max_history_length = max_history_length
        self.history_sensor_event = []
        self.history_cached_filter_state = []

    def add_sensor_event(self, sensor_event: SensorEvent) -> None:
        self.add(self.history_sensor_event, sensor_event)

    def add_cached_filter_state(self, cached_filter_state: CachedFilterState) -> None:
        self.add(self.history_cached_filter_state, cached_filter_state)

    def add(self, list, item) -> None:
        list.append(item)
        if len(list) > self.max_history_length:
            list.pop(0)

    def closest_state(self, timestamp_s: float) -> CachedFilterState:
        closest_state = min(
            self.history_cached_filter_state,
            key=lambda x: abs(x.timestamp_s - timestamp_s),
        )

        return closest_state


class PositionSolver2d(ExtendedKalmanFilter):
    kNumStates = 3
    kPosXIdx = 0
    kPosYIdx = 1
    kThetaIdx = 2

    CAMERA_OUTPUT_TO_ROBOT_ROTATION = np.array(
        [
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0],
        ],
        dtype=np.float64,
    )

    def __init__(self, config: PosExtrapolator, general_config: Config):
        super().__init__(dim_x=self.kNumStates, dim_z=self.kNumStates, dim_u=3)
        self.config = config
        self.general_config = general_config
        self.enabled_sources = set(config.enabled_data_sources)

        self.x = load_vector(config.kalman_filter_config.initial_state_vector, 3)
        self.P = load_matrix(config.kalman_filter_config.uncertainty_matrix, 3)
        self.Q = load_matrix(config.kalman_filter_config.process_noise_matrix, 3)

        self.R_sensors = get_R_sensors(config.kalman_filter_config)

        self.current_control = MotionInput(vx_robot=0.0, vy_robot=0.0, omega=0.0)

        self._time_converter = SensorsTimeConverter()

        self.last_action_time_s = None
        self.current_time = time.time()

    def insert_sensor_data(
        self,
        data: SensorPayload,
        sensor_id: str,
        timestamp_ms: int,
        *,
        sensor_type: AllowedSensors | None = None,
        received_at_s: float | None = None,
    ) -> None:
        if received_at_s is None:
            received_at_s = time.time()
        if sensor_type is None:
            sensor_type = self._sensor_type_for_data(data)

        if SENSOR_TYPE_TO_SOURCE[sensor_type] not in self.enabled_sources:
            return

        timestamp_s = self._time_converter.get_local_time_s(
            sensor_type,
            sensor_id,
            timestamp_ms,
            local_reference_s=received_at_s,
        )

        event = SensorEvent(
            timestamp_s=timestamp_s,
            sensor_type=sensor_type,
            sensor_id=sensor_id,
            data=data,
        )
        self._apply_event(event)

    def _apply_event(self, event: SensorEvent) -> None:
        processor = get_processor(event.sensor_type)
        if processor is None:
            raise ValueError(
                f"No processor registered for sensor type {event.sensor_type}"
            )
        processor(self, event)

    def get_dt_s(self, timestamp_s: float | None = None) -> float:
        self.current_time = time.time() if timestamp_s is None else float(timestamp_s)

        if self.last_action_time_s is None:
            self.last_action_time_s = self.current_time
            return 0.0

        delta_t = float(self.current_time - self.last_action_time_s)
        self.last_action_time_s = self.current_time
        return max(0.0, delta_t)

    def nonlinear_predict_next(self, timestamp_s: float | None = None):
        return self.nonlinear_predict(
            delta_t=self.get_dt_s(timestamp_s), motion_input=self.current_control
        )

    def nonlinear_predict(
        self,
        delta_t: float,
        motion_input: "MotionInput",
        innovation_function: Callable[..., NDArray[np.float64]] | None = None,
        innovation_args: tuple = (),
    ) -> None:
        """
        Predict the state of the system using the motion input and the delta time.

        USE THIS INSTEAD OF predict_x/predict methods!
        """

        if innovation_function is None:
            innovation_function = self._predict_no_change

        control_vector = motion_input.as_vector()
        self.F = self._motion_jacobian(self.x, control_vector, delta_t)

        self.x = innovation_function(self.x, control_vector, delta_t, *innovation_args)
        self.P = np.dot(self.F, self.P).dot(self.F.T) + self.Q

        # save prior
        self.x_prior = np.copy(self.x)
        self.P_prior = np.copy(self.P)

    def _predict_no_change(
        self,
        state: NDArray[np.float64],
        control: NDArray[np.float64],
        dt_s: float,
    ) -> NDArray[np.float64]:
        if dt_s <= 0.0:
            return state.copy()

        theta = float(state[self.kThetaIdx])
        omega = float(control[2])
        theta_mid = theta + 0.5 * omega * dt_s
        sin_theta = float(np.sin(theta_mid))
        cos_theta = float(np.cos(theta_mid))

        next_state = state.copy()
        next_state[self.kPosXIdx] += dt_s * (
            cos_theta * float(control[0]) - sin_theta * float(control[1])
        )
        next_state[self.kPosYIdx] += dt_s * (
            sin_theta * float(control[0]) + cos_theta * float(control[1])
        )
        next_state[self.kThetaIdx] = wrap_to_pi(theta + omega * dt_s)
        return next_state

    def _motion_jacobian(
        self,
        state: NDArray[np.float64],
        control: NDArray[np.float64],
        dt_s: float,
    ) -> NDArray[np.float64]:
        F = np.eye(self.kNumStates, dtype=np.float64)
        if dt_s <= 0.0:
            return F

        theta = float(state[self.kThetaIdx])
        omega = float(control[2])
        theta_mid = theta + 0.5 * omega * dt_s
        sin_theta = float(np.sin(theta_mid))
        cos_theta = float(np.cos(theta_mid))

        F[self.kPosXIdx, self.kThetaIdx] = dt_s * (
            -sin_theta * float(control[0]) - cos_theta * float(control[1])
        )
        F[self.kPosYIdx, self.kThetaIdx] = dt_s * (
            cos_theta * float(control[0]) - sin_theta * float(control[1])
        )
        return F

    def update(
        self,
        z: NDArray[np.float64] | float | None,
        HJacobian: Callable[..., NDArray[np.float64]],
        Hx: Callable[..., NDArray[np.float64]],
        R: NDArray[np.float64] | float | None = None,
        args: tuple = (),
        hx_args: tuple = (),
        residual: Callable[[Any, Any], Any] = residual,
    ) -> None:
        """
        NOTE: the residual assumes the state is in the order of [x, y, theta]
        """

        super().update(z, HJacobian, Hx, R, args, hx_args, residual)
        self.x[self.kThetaIdx] = wrap_to_pi(float(self.x[self.kThetaIdx]))

    def _sensor_noise(
        self,
        sensor_type: KalmanFilterSensorType,
        sensor_id: str,
        sensor_indices: list[int],
    ) -> NDArray[np.float64]:
        matrix = self.R_sensors.get(sensor_type, {}).get(sensor_id)
        if matrix is None:
            error(
                f"No sensor noise matrix found for sensor type {sensor_type} and sensor id {sensor_id}"
            )
            return np.eye(len(sensor_indices), dtype=np.float64)

        valid_indices = [idx for idx in sensor_indices if idx < matrix.shape[0]]
        if not valid_indices:
            return np.eye(len(sensor_indices), dtype=np.float64)

        submatrix = matrix[np.ix_(valid_indices, valid_indices)].astype(np.float64)
        if len(valid_indices) == len(sensor_indices):
            return submatrix

        padded = np.eye(len(sensor_indices), dtype=np.float64)
        for output_idx, matrix_idx in enumerate(valid_indices):
            padded[output_idx, output_idx] = float(matrix[matrix_idx, matrix_idx])
        return padded

    def _world_velocity_from_pose(self, theta_rad: float) -> NDArray[np.float64]:
        return rotation_matrix_2d(theta_rad) @ np.array(
            [self.current_control.vx_robot, self.current_control.vy_robot],
            dtype=np.float64,
        )

    def get_state(self, future_s: float | None = None) -> NDArray[np.float64]:
        if future_s is None or future_s <= 0.0:
            return self.x.copy()
        return self._predict_no_change(
            self.x, self.current_control.as_vector(), future_s
        )

    def get_robot_state_estimate(
        self, future_s: float | None = None
    ) -> NDArray[np.float64]:
        pose = self.get_state(future_s=future_s)
        velocity = self._world_velocity_from_pose(float(pose[self.kThetaIdx]))
        return np.array(
            [
                pose[self.kPosXIdx],
                pose[self.kPosYIdx],
                velocity[0],
                velocity[1],
                pose[self.kThetaIdx],
                self.current_control.omega,
            ],
            dtype=np.float64,
        )

    def get_robot_position_estimate(self, future_s: float | None = None) -> list[float]:
        return self.get_robot_state_estimate(future_s=future_s).flatten().tolist()

    def get_robot_position(self) -> RobotPosition:
        filtered_position = self.get_robot_position_estimate(
            self.config.future_position_prediction_margin_s
        )
        proto_position = RobotPosition()
        proto_position.timestamp = time.time() * 1000
        proto_position.confidence = self.get_confidence()
        proto_position.position_2d.position.x = filtered_position[0]
        proto_position.position_2d.position.y = filtered_position[1]
        proto_position.position_2d.velocity.x = filtered_position[2]
        proto_position.position_2d.velocity.y = filtered_position[3]
        proto_position.position_2d.direction.x = float(np.cos(filtered_position[4]))
        proto_position.position_2d.direction.y = float(np.sin(filtered_position[4]))
        proto_position.position_2d.rotation_speed_rad_s = filtered_position[5]
        proto_position.P.extend(self.get_position_covariance())
        return proto_position

    def get_P(self) -> NDArray[np.float64]:
        return self.P.copy()

    def get_position_covariance(self) -> list[float]:
        return self.get_P().flatten().tolist()

    def get_confidence(self) -> float:
        return 1.0

    @staticmethod
    def _sensor_type_for_data(data: SensorPayload) -> AllowedSensors:
        if isinstance(data, OdometryData):
            return KalmanFilterSensorType.ODOMETRY
        if isinstance(data, AprilTagData):
            return KalmanFilterSensorType.APRIL_TAG
        if isinstance(data, ImuData):
            return KalmanFilterSensorType.IMU
        raise TypeError(f"Unsupported sensor payload type: {type(data).__name__}")
