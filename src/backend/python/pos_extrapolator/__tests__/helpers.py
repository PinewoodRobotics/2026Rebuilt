from __future__ import annotations

from types import SimpleNamespace
from typing import Any, cast

import numpy as np

from backend.generated.proto.python.sensor.apriltags_pb2 import (
    AprilTagData,
    ProcessedTag,
    WorldTags,
)
from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.generated.thrift.config.common.ttypes import (
    GenericMatrix,
    GenericVector,
    Point3,
)
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterConfig,
    KalmanFilterSensorConfig,
    KalmanFilterSensorType,
)
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    AprilTagConfig,
    DataSources,
    ImuConfig,
    OdomConfig,
    PosExtrapolator,
    PosExtrapolatorMessageConfig,
    TagNoiseAdjustConfig,
)
from backend.python.common.util.math import from_theta_to_3x3_mat
from backend.python.pos_extrapolator.position_solver_2d import PositionSolver2d
from backend.python.pos_extrapolator.processor_registry import AllowedSensors

BASE_TIMESTAMP_MS = 1_700_000_000_000
BASE_RECEIVED_AT_S = 1_000.0


def diag_matrix(*values: float) -> GenericMatrix:
    size = len(values)
    rows = [[0.0] * size for _ in range(size)]
    for i, value in enumerate(values):
        rows[i][i] = float(value)
    return GenericMatrix(values=rows, rows=size, cols=size)


def point3(position: np.ndarray, rotation: np.ndarray) -> Point3:
    return Point3(
        position=GenericVector(values=[float(v) for v in position], size=3),
        rotation=GenericMatrix(
            values=[[float(v) for v in row] for row in rotation],
            rows=3,
            cols=3,
        ),
    )


def make_config(
    *,
    initial_state: list[float] | None = None,
    imu_use_velocity: bool = False,
    insert_predicted_global_rotation: bool = True,
) -> PosExtrapolator:
    if initial_state is None:
        initial_state = [0.0, 0.0, 0.0]

    message_config = PosExtrapolatorMessageConfig(
        post_tag_input_topic="test/tag",
        post_odometry_input_topic="test/odom",
        post_imu_input_topic="test/imu",
        post_robot_position_output_topic="test/output",
    )

    tags_in_world = {
        0: point3(np.array([0.0, 0.0, 0.0]), from_theta_to_3x3_mat(0)),
        1: point3(np.array([1.0, 0.0, 0.0]), from_theta_to_3x3_mat(90)),
    }
    cameras_in_robot = {
        "cam0": point3(np.array([0.0, 0.0, 0.0]), from_theta_to_3x3_mat(0))
    }

    april_tag_config = AprilTagConfig(
        tag_position_config=tags_in_world,
        camera_position_config=cameras_in_robot,
        noise_change_modes=[],
        tag_noise_adjust_config=TagNoiseAdjustConfig(
            weight_per_m_from_distance_from_tag=0.0,
            weight_per_degree_from_angle_error_tag=0.0,
            weight_per_confidence_tag=0.0,
            min_distance_from_tag_to_use_noise_adjustment=0.0,
        ),
        insert_predicted_global_rotation=insert_predicted_global_rotation,
    )

    kalman_config = KalmanFilterConfig(
        initial_state_vector=GenericVector(
            values=initial_state, size=len(initial_state)
        ),
        uncertainty_matrix=diag_matrix(1.0, 1.0, 1.0),
        process_noise_matrix=diag_matrix(0.1, 0.1, 0.1),
        sensors={
            KalmanFilterSensorType.APRIL_TAG: {
                "cam0": KalmanFilterSensorConfig(
                    measurement_noise_matrix=diag_matrix(0.1, 0.1, 0.1)
                )
            }
        },
    )

    return PosExtrapolator(
        message_config=message_config,
        enabled_data_sources=[
            DataSources.APRIL_TAG,
            DataSources.ODOMETRY,
            DataSources.IMU,
        ],
        april_tag_config=april_tag_config,
        odom_config=OdomConfig(),
        imu_config={"imu0": ImuConfig(use_velocity=imu_use_velocity)},
        kalman_filter_config=kalman_config,
        time_s_between_position_sends=0.02,
        future_position_prediction_margin_s=0.0,
    )


def make_solver(**kwargs: Any) -> PositionSolver2d:
    config = make_config(**kwargs)
    return PositionSolver2d(
        config,
        cast(Any, SimpleNamespace(april_tag_config=config.april_tag_config)),
    )


def make_extrapolator(**kwargs: Any) -> PositionSolver2d:
    config = make_config(**kwargs)
    return PositionSolver2d(
        config,
        cast(Any, SimpleNamespace(april_tag_config=config.april_tag_config)),
    )


def insert_sensor(
    solver: PositionSolver2d,
    data: OdometryData | AprilTagData | ImuData,
    sensor_id: str,
    timestamp_ms: float | int,
    *,
    received_at_s: float | None = None,
) -> None:
    sensor_type = _sensor_type_for_data(data)
    solver.insert_sensor_data(
        data=data,
        sensor_type=cast(AllowedSensors, sensor_type),
        sensor_id=sensor_id,
        timestamp_ms=int(timestamp_ms),
        received_at_s=received_at_s,
    )


def _sensor_type_for_data(
    data: OdometryData | AprilTagData | ImuData,
) -> KalmanFilterSensorType:
    if isinstance(data, OdometryData):
        return KalmanFilterSensorType.ODOMETRY
    if isinstance(data, AprilTagData):
        return KalmanFilterSensorType.APRIL_TAG
    if isinstance(data, ImuData):
        return KalmanFilterSensorType.IMU
    raise TypeError(f"Unsupported sensor payload type: {type(data).__name__}")


def make_odom(
    *,
    vx: float = 0.0,
    vy: float = 0.0,
    dx: float | None = None,
    dy: float | None = None,
    dt_s: float = 0.02,
    omega: float = 0.0,
    x: float = 0.0,
    y: float = 0.0,
) -> OdometryData:
    if dx is None:
        dx = vx * dt_s
    if dy is None:
        dy = vy * dt_s

    odom = OdometryData()
    odom.velocity.x = vx
    odom.velocity.y = vy
    odom.position_change.x = dx
    odom.position_change.y = dy
    odom.time_change_s = dt_s
    odom.omega = omega
    odom.position.position.x = x
    odom.position.position.y = y
    odom.position.direction.x = 1.0
    odom.position.direction.y = 0.0
    return odom


def make_imu(
    *,
    theta_rad: float = 0.0,
    omega: float = 0.0,
    vx: float = 0.0,
    vy: float = 0.0,
) -> ImuData:
    imu = ImuData()
    imu.position.direction.x = float(np.cos(theta_rad))
    imu.position.direction.y = float(np.sin(theta_rad))
    imu.angularVelocityXYZ.z = omega
    imu.velocity.x = vx
    imu.velocity.y = vy
    return imu


def make_processed_tag(
    *,
    tag_id: int = 0,
    pose_R: np.ndarray,
    pose_t: np.ndarray,
    confidence: float = 0.0,
) -> AprilTagData:
    return AprilTagData(
        world_tags=WorldTags(
            tags=[
                ProcessedTag(
                    id=tag_id,
                    pose_R=pose_R.reshape(-1).tolist(),
                    pose_t=pose_t.reshape(-1).tolist(),
                    confidence=confidence,
                )
            ]
        )
    )
