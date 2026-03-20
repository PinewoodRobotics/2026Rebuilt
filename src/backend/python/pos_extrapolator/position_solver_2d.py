from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import bisect
import time

from filterpy.kalman import ExtendedKalmanFilter
import numpy as np
from numpy.typing import NDArray

from backend.generated.proto.python.sensor.apriltags_pb2 import AprilTagData
from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.generated.proto.python.util.position_pb2 import RobotPosition
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterConfig,
    KalmanFilterSensorType,
)
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    DataSources,
    PosExtrapolator,
)
from backend.python.common.util.math import get_np_from_matrix, get_np_from_vector
from backend.python.pos_extrapolator.processor_registry import (
    AllowedSensors,
    get_processor,
)
from backend.python.pos_extrapolator.processors import (  # noqa: F401
    apriltag_processor,
    imu_processor,
    odometry_processor,
)
from backend.python.pos_extrapolator.util.mahalanobis import mahalanobis_distance
from backend.python.pos_extrapolator.util.time_conversion import SensorsTimeConverter

ArrayF64 = NDArray[np.float64]
SensorPayload = AprilTagData | ImuData | OdometryData

SENSOR_TYPE_TO_SOURCE: dict[AllowedSensors, DataSources] = {
    KalmanFilterSensorType.APRIL_TAG: DataSources.APRIL_TAG,
    KalmanFilterSensorType.ODOMETRY: DataSources.ODOMETRY,
    KalmanFilterSensorType.IMU: DataSources.IMU,
}


def _wrap_to_pi(angle_rad: float) -> float:
    return float(np.arctan2(np.sin(angle_rad), np.cos(angle_rad)))


def _rotation_matrix_2d(theta_rad: float) -> ArrayF64:
    cos_theta = float(np.cos(theta_rad))
    sin_theta = float(np.sin(theta_rad))
    return np.array(
        [[cos_theta, -sin_theta], [sin_theta, cos_theta]],
        dtype=np.float64,
    )


def _load_vector(vector: object, expected_size: int) -> ArrayF64:
    result = np.asarray(get_np_from_vector(vector), dtype=np.float64).reshape(-1)
    if result.shape != (expected_size,):
        raise ValueError(
            f"Expected vector with shape {(expected_size,)}, got {result.shape}"
        )
    return result


def _load_matrix(matrix: object, size: int) -> ArrayF64:
    result = np.asarray(get_np_from_matrix(matrix), dtype=np.float64)
    if result.shape != (size, size):
        raise ValueError(
            f"Expected matrix with shape {(size, size)}, got {result.shape}"
        )
    return result


@dataclass
class MotionInput:
    vx_robot: float = 0.0
    vy_robot: float = 0.0
    omega: float = 0.0

    def as_vector(self) -> ArrayF64:
        return np.array(
            [self.vx_robot, self.vy_robot, self.omega],
            dtype=np.float64,
        )


@dataclass(order=True)
class SensorEvent:
    timestamp_s: float
    sequence: int
    sensor_type: AllowedSensors = field(compare=False)
    sensor_id: str = field(compare=False)
    data: SensorPayload = field(compare=False)


@dataclass
class SolverSnapshot:
    timestamp_s: float
    x: ArrayF64
    P: ArrayF64
    control: MotionInput
    has_gotten_rotation: bool


class PositionSolverHistory:
    def __init__(self, window_s: float) -> None:
        self.window_s = window_s
        self.baseline_snapshot: SolverSnapshot | None = None
        self.events: list[SensorEvent] = []
        self.post_snapshots: list[SolverSnapshot] = []

    def is_initialized(self) -> bool:
        return self.baseline_snapshot is not None

    def initialize(self, snapshot: SolverSnapshot) -> None:
        self.baseline_snapshot = snapshot
        self.events.clear()
        self.post_snapshots.clear()

    def insert_event(self, event: SensorEvent) -> None:
        bisect.insort_right(self.events, event)

    def replay(self, solver: PositionSolver2d) -> None:
        if self.baseline_snapshot is None:
            raise ValueError("History must be initialized before replay")

        solver._restore_snapshot(self.baseline_snapshot)
        self.post_snapshots = []
        for event in self.events:
            solver._apply_event(event)
            self.post_snapshots.append(solver._make_snapshot())

    def prune(self, latest_time_s: float) -> None:
        cutoff_s = latest_time_s - self.window_s
        while self.events and self.events[0].timestamp_s < cutoff_s:
            if not self.post_snapshots:
                break
            self.baseline_snapshot = self.post_snapshots.pop(0)
            self.events.pop(0)

    def current_snapshot(self) -> SolverSnapshot | None:
        if self.post_snapshots:
            return self.post_snapshots[-1]
        return self.baseline_snapshot

    def latest_timestamp_s(self) -> float | None:
        if self.events:
            return self.events[-1].timestamp_s
        if self.baseline_snapshot is not None:
            return self.baseline_snapshot.timestamp_s
        return None


class PositionSolver2d(ExtendedKalmanFilter):
    kNumStates = 3
    kPosXIdx = 0
    kPosYIdx = 1
    kThetaIdx = 2

    kNumRobotStateOutputs = 6
    kVelXIdx = 2
    kVelYIdx = 3
    kAngleVelIdx = 5

    kHistoryWindowS = 0.25
    kAprilTagMahalanobisThreshold = 5.0

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

        self.x = _load_vector(config.kalman_filter_config.initial_state_vector, 3)
        self.P = _load_matrix(config.kalman_filter_config.uncertainty_matrix, 3)
        self._base_Q = _load_matrix(config.kalman_filter_config.process_noise_matrix, 3)
        self.Q = np.zeros((self.kNumStates, self.kNumStates), dtype=np.float64)
        self.F = np.eye(self.kNumStates, dtype=np.float64)
        self.B = np.zeros((self.kNumStates, 3), dtype=np.float64)
        self.R_sensors = self._get_R_sensors(config.kalman_filter_config)

        self.current_control = MotionInput()
        self.current_time_s: float | None = None
        self.has_gotten_rotation = False
        self._prediction_dt_s = 0.0
        self._time_converter = SensorsTimeConverter()
        self._history = PositionSolverHistory(self.kHistoryWindowS)
        self._next_sequence = 0

        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def _get_R_sensors(
        self, config: KalmanFilterConfig
    ) -> dict[KalmanFilterSensorType, dict[str, ArrayF64]]:
        output: dict[KalmanFilterSensorType, dict[str, ArrayF64]] = {}
        for sensor_type, sensors in config.sensors.items():
            output[sensor_type] = {}
            for sensor_id, sensor_config in sensors.items():
                output[sensor_type][sensor_id] = np.asarray(
                    get_np_from_matrix(sensor_config.measurement_noise_matrix),
                    dtype=np.float64,
                )
        return output

    def _make_snapshot(self) -> SolverSnapshot:
        return SolverSnapshot(
            timestamp_s=0.0 if self.current_time_s is None else self.current_time_s,
            x=self.x.copy(),
            P=self.P.copy(),
            control=MotionInput(
                vx_robot=self.current_control.vx_robot,
                vy_robot=self.current_control.vy_robot,
                omega=self.current_control.omega,
            ),
            has_gotten_rotation=self.has_gotten_rotation,
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
        self.has_gotten_rotation = snapshot.has_gotten_rotation
        self.x_prior = self.x.copy()
        self.P_prior = self.P.copy()
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def insert_sensor_data(
        self,
        data: SensorPayload,
        sensor_id: str,
        timestamp_ms: float | int | None = None,
        *,
        sensor_type: AllowedSensors | None = None,
        received_at_s: float | None = None,
    ) -> None:
        if sensor_type is None:
            sensor_type = self._sensor_type_for_data(data)

        if SENSOR_TYPE_TO_SOURCE[sensor_type] not in self.enabled_sources:
            return

        if received_at_s is None:
            received_at_s = time.time()
        if timestamp_ms is None:
            timestamp_ms = received_at_s * 1000.0

        timestamp_s = self._time_converter.get_local_time_s(
            sensor_type,
            sensor_id,
            float(timestamp_ms),
            local_reference_s=received_at_s,
        )

        if not self._history.is_initialized():
            self.current_time_s = timestamp_s
            self._history.initialize(self._make_snapshot())

        latest_history_time_s = self._history.latest_timestamp_s()
        if (
            latest_history_time_s is not None
            and timestamp_s < latest_history_time_s - self.kHistoryWindowS
        ):
            return

        baseline = self._history.baseline_snapshot
        if baseline is None:
            raise ValueError("History baseline should be initialized before insertion")
        if timestamp_s < baseline.timestamp_s:
            return

        event = SensorEvent(
            timestamp_s=timestamp_s,
            sequence=self._next_sequence,
            sensor_type=sensor_type,
            sensor_id=sensor_id,
            data=deepcopy(data),
        )
        self._next_sequence += 1

        self._history.insert_event(event)
        self._history.replay(self)

        latest_time_s = self._history.latest_timestamp_s()
        if latest_time_s is not None:
            self._history.prune(latest_time_s)

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
        if self.current_time_s is None:
            self.current_time_s = timestamp_s
            return

        dt_s = float(timestamp_s - self.current_time_s)
        if dt_s <= 0.0:
            self.current_time_s = timestamp_s
            return

        self._set_prediction_model(control, dt_s)
        self.predict(control.as_vector())  # pyright: ignore[reportArgumentType]
        self.current_time_s = timestamp_s
        self.x[self.kThetaIdx] = _wrap_to_pi(float(self.x[self.kThetaIdx]))
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def _set_prediction_model(self, control: MotionInput, dt_s: float) -> None:
        theta_mid = float(self.x[self.kThetaIdx]) + 0.5 * control.omega * dt_s
        sin_theta = float(np.sin(theta_mid))
        cos_theta = float(np.cos(theta_mid))

        self._prediction_dt_s = dt_s
        self.F = np.eye(self.kNumStates, dtype=np.float64)
        self.F[self.kPosXIdx, self.kThetaIdx] = dt_s * (
            -sin_theta * control.vx_robot - cos_theta * control.vy_robot
        )
        self.F[self.kPosYIdx, self.kThetaIdx] = dt_s * (
            cos_theta * control.vx_robot - sin_theta * control.vy_robot
        )
        self.Q = self._base_Q * dt_s

    def predict_x(self, u: ArrayF64 | float | int = 0) -> None:
        if np.isscalar(u):
            control = np.zeros(3, dtype=np.float64)
        else:
            control = np.asarray(u, dtype=np.float64).reshape(-1)
        self.x = self._propagate_state(self.x, control, self._prediction_dt_s)

    def _propagate_state(
        self,
        state: ArrayF64,
        control: ArrayF64,
        dt_s: float,
    ) -> ArrayF64:
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
        next_state[self.kThetaIdx] = _wrap_to_pi(theta + omega * dt_s)
        return next_state

    def _sensor_noise(
        self,
        sensor_type: KalmanFilterSensorType,
        sensor_id: str,
        sensor_indices: list[int],
    ) -> ArrayF64:
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
        z: ArrayF64,
        state_indices: list[int],
        R: ArrayF64,
    ) -> None:
        H = np.zeros((len(state_indices), self.kNumStates), dtype=np.float64)
        for row, state_idx in enumerate(state_indices):
            H[row, state_idx] = 1.0

        angle_measurement_idx = None
        if self.kThetaIdx in state_indices:
            angle_measurement_idx = state_indices.index(self.kThetaIdx)

        def residual(measurement: ArrayF64, estimate: ArrayF64) -> ArrayF64:
            delta = measurement - estimate
            if angle_measurement_idx is not None:
                delta[angle_measurement_idx] = _wrap_to_pi(
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
        self.x[self.kThetaIdx] = _wrap_to_pi(float(self.x[self.kThetaIdx]))
        self.x_post = self.x.copy()
        self.P_post = self.P.copy()

    def should_accept_apriltag_measurement(
        self,
        measurement_xy: ArrayF64,
        measurement_covariance: ArrayF64,
    ) -> bool:
        state_indices = [self.kPosXIdx, self.kPosYIdx]
        innovation_covariance = self.P[np.ix_(state_indices, state_indices)] + (
            measurement_covariance[np.ix_([0, 1], [0, 1])]
        )
        distance = mahalanobis_distance(
            measurement_xy,
            self.x[state_indices],
            innovation_covariance,
        )
        return distance <= self.kAprilTagMahalanobisThreshold

    def _world_velocity_from_pose(self, theta_rad: float) -> ArrayF64:
        return _rotation_matrix_2d(theta_rad) @ np.array(
            [self.current_control.vx_robot, self.current_control.vy_robot],
            dtype=np.float64,
        )

    def get_state(self, future_s: float | None = None) -> ArrayF64:
        if future_s is None or future_s <= 0.0:
            return self.x.copy()
        return self._propagate_state(self.x, self.current_control.as_vector(), future_s)

    def get_robot_state_estimate(self, future_s: float | None = None) -> ArrayF64:
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

    def get_P(self) -> ArrayF64:
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
