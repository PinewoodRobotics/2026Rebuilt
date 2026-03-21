# Based on https://www.chiefdelphi.com/t/kalman-filter-tuning-and-gps

from __future__ import annotations

from copy import deepcopy
import time

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
from backend.python.pos_extrapolator.util.position_solver_history import (
    PositionSolverHistory,
)
from backend.python.pos_extrapolator.processor_registry import (
    AllowedSensors,
    get_processor,
)
from backend.python.pos_extrapolator.util.solver_models import (
    MotionInput,
    SensorEvent,
    SensorPayload,
    SolverSnapshot,
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

SENSOR_TYPE_TO_SOURCE: dict[AllowedSensors, DataSources] = {
    KalmanFilterSensorType.APRIL_TAG: DataSources.APRIL_TAG,
    KalmanFilterSensorType.ODOMETRY: DataSources.ODOMETRY,
    KalmanFilterSensorType.IMU: DataSources.IMU,
}


class PositionSolver2d(ExtendedKalmanFilter):
    kNumStates = 3
    kPosXIdx = 0
    kPosYIdx = 1
    kThetaIdx = 2

    kHistoryWindowS = 0.25

    CAMERA_OUTPUT_TO_ROBOT_ROTATION = np.array(
        [
            [0, 0, 1],
            [-1, 0, 0],
            [0, -1, 0],
        ],
        dtype=np.float64,
    )

    def __init__(self, config: PosExtrapolator):
        super().__init__(dim_x=self.kNumStates, dim_z=self.kNumStates, dim_u=3)
        self.config = config
        self.general_config = config
        self.enabled_sources = set(config.enabled_data_sources)

        self.x = load_vector(config.kalman_filter_config.initial_state_vector, 3)
        self.P = load_matrix(config.kalman_filter_config.uncertainty_matrix, 3)
        self._base_Q = load_matrix(config.kalman_filter_config.process_noise_matrix, 3)
        self.Q = np.zeros((self.kNumStates, self.kNumStates), dtype=np.float64)
        self.F = np.eye(self.kNumStates, dtype=np.float64)
        self.B = np.zeros((self.kNumStates, 3), dtype=np.float64)
        self.R_sensors = get_R_sensors(config.kalman_filter_config)

        self.current_control = MotionInput(vx_robot=0.0, vy_robot=0.0, omega=0.0)
        self.current_time_s: float = 0.0
        self._time_converter = SensorsTimeConverter()
        self._history = PositionSolverHistory(self.kHistoryWindowS)

        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def _make_snapshot(self) -> SolverSnapshot:
        return SolverSnapshot(
            timestamp_s=self.current_time_s,
            x=self.x.copy(),
            P=self.P.copy(),
            control=MotionInput(
                vx_robot=self.current_control.vx_robot,
                vy_robot=self.current_control.vy_robot,
                omega=self.current_control.omega,
            ),
        )

    def _restore_snapshot(self, snapshot: SolverSnapshot) -> None:
        self.current_time_s = snapshot.timestamp_s
        self.x = snapshot.x.copy()
        self.P = snapshot.P.copy()
        self.current_control = MotionInput(
            vx_robot=snapshot.control.vx_robot,
            vy_robot=snapshot.control.vy_robot,
            omega=snapshot.control.omega,
        )

        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def _replay_history_from(self, start_event_idx: int) -> None:
        history = self._history
        if history.is_empty():
            raise ValueError("History must be initialized before replay")

        if not history.has_seed_snapshot():
            raise ValueError("History seed snapshot must exist before replay")
        self._restore_snapshot(history.rollback_snapshot(start_event_idx))
        history.discard_snapshots_from(start_event_idx)
        for event in history.events_from(start_event_idx):
            self._apply_event(event)
            history.append_snapshot(self._make_snapshot())

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

        latest_history_time_s = self._history.latest_timestamp_s()
        prune_reference_s = (
            timestamp_s
            if latest_history_time_s is None
            else max(timestamp_s, latest_history_time_s)
        )
        self._history.prune(prune_reference_s)

        if (
            latest_history_time_s is not None
            and timestamp_s < latest_history_time_s - self.kHistoryWindowS
        ):
            return

        history_start_s = self._history.start_timestamp_s()
        if history_start_s is not None and timestamp_s < history_start_s:
            return

        event = SensorEvent(
            timestamp_s=timestamp_s,
            sensor_type=sensor_type,
            sensor_id=sensor_id,
            data=deepcopy(data),
        )

        if not self._history.has_seed_snapshot():
            seed_snapshot = self._make_snapshot()
            if self.current_time_s == 0.0:
                seed_snapshot.timestamp_s = timestamp_s
            self._history.ensure_seed_snapshot(seed_snapshot)

        replay_start_idx = self._history.insert_event(event)
        self._replay_history_from(replay_start_idx)

        current_snapshot = self._history.current_snapshot()
        if current_snapshot is not None:
            self._restore_snapshot(current_snapshot)

    def _apply_event(self, event: SensorEvent) -> None:
        processor = get_processor(event.sensor_type)
        if processor is None:
            raise ValueError(
                f"No processor registered for sensor type {event.sensor_type}"
            )
        processor(self, event)

    def predict_to_timestamp(self, timestamp_s: float, control: MotionInput) -> None:
        dt_s = float(timestamp_s - self.current_time_s)
        if dt_s <= 0.0:
            self.current_time_s = timestamp_s
            return

        self._set_prediction_model(control, dt_s)
        control_vector = control.as_vector()
        self.x = self._propagate_state(self.x, control_vector, dt_s)
        self.P = self.F @ self.P @ self.F.T + self.Q
        self.current_time_s = timestamp_s
        self.x[self.kThetaIdx] = wrap_to_pi(float(self.x[self.kThetaIdx]))
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def _set_prediction_model(self, control: MotionInput, dt_s: float) -> None:
        theta_mid = float(self.x[self.kThetaIdx]) + 0.5 * control.omega * dt_s
        sin_theta = float(np.sin(theta_mid))
        cos_theta = float(np.cos(theta_mid))

        self.F = np.eye(self.kNumStates, dtype=np.float64)
        self.F[self.kPosXIdx, self.kThetaIdx] = dt_s * (
            -sin_theta * control.vx_robot - cos_theta * control.vy_robot
        )
        self.F[self.kPosYIdx, self.kThetaIdx] = dt_s * (
            cos_theta * control.vx_robot - sin_theta * control.vy_robot
        )
        self.Q = self._base_Q * dt_s

    def _propagate_state(
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

    def _sensor_noise(
        self,
        sensor_type: KalmanFilterSensorType,
        sensor_id: str,
        sensor_indices: list[int],
    ) -> NDArray[np.float64]:
        matrix = self.R_sensors.get(sensor_type, {}).get(sensor_id)
        if matrix is None:
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

    def _correct(
        self,
        z: NDArray[np.float64],
        state_indices: list[int],
        R: NDArray[np.float64],
    ) -> None:
        H = np.zeros((len(state_indices), self.kNumStates), dtype=np.float64)
        for row, state_idx in enumerate(state_indices):
            H[row, state_idx] = 1.0

        angle_measurement_idx = None
        if self.kThetaIdx in state_indices:
            angle_measurement_idx = state_indices.index(self.kThetaIdx)

        def residual(
            measurement: NDArray[np.float64], estimate: NDArray[np.float64]
        ) -> NDArray[np.float64]:
            delta = measurement - estimate
            if angle_measurement_idx is not None:
                delta[angle_measurement_idx] = wrap_to_pi(
                    float(delta[angle_measurement_idx])
                )
            return delta

        self.update(
            np.asarray(z, dtype=np.float64).reshape(-1),
            HJacobian=lambda _x: H,
            Hx=lambda x: x[state_indices],
            R=R,
            residual=residual,
        )
        self.x[self.kThetaIdx] = wrap_to_pi(float(self.x[self.kThetaIdx]))
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def _world_velocity_from_pose(self, theta_rad: float) -> NDArray[np.float64]:
        return rotation_matrix_2d(theta_rad) @ np.array(
            [self.current_control.vx_robot, self.current_control.vy_robot],
            dtype=np.float64,
        )

    def get_state(self, future_s: float | None = None) -> NDArray[np.float64]:
        if future_s is None or future_s <= 0.0:
            return self.x.copy()
        return self._propagate_state(self.x, self.current_control.as_vector(), future_s)

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
