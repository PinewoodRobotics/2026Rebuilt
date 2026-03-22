from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING, cast

import numpy as np
from numpy.typing import NDArray

from backend.generated.proto.python.sensor.apriltags_pb2 import (
    AprilTagData,
    ProcessedTag,
)
from backend.generated.thrift.config.common.ttypes import Point3
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterSensorType,
)
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    AprilTagConfig,
    PosExtrapolator,
    TagNoiseAdjustConfig,
    TagNoiseAdjustMode,
)
from backend.python.common.util.math import (
    create_transformation_matrix,
    extract_2d_from_3d_transformation,
    from_float_list,
    get_np_from_matrix,
    get_np_from_vector,
    get_robot_in_world,
    get_translation_rotation_components,
    make_transformation_matrix_p_d,
    world_robot_to_tag_camera,
)
from backend.python.pos_extrapolator.processor_registry import processor_for_data
from backend.python.pos_extrapolator.util.extrapolator_math import rotation_matrix_2d
from backend.python.pos_extrapolator.util.mahalanobis import mahalanobis_distance

if TYPE_CHECKING:
    from backend.python.pos_extrapolator.position_solver_2d import PositionSolver2d
    from backend.python.pos_extrapolator.util.solver_models import SensorEvent


def yaw_from_T(T: np.ndarray) -> float:
    return np.arctan2(T[1, 0], T[0, 0])


def wrap_angle(a: float) -> float:
    return (a + np.pi) % (2 * np.pi) - np.pi


def AprilTagHx2d(
    x_hat: NDArray[np.float64],
    *,
    T_tag_in_world: NDArray[np.float64],
    T_camera_in_robot: NDArray[np.float64],
) -> NDArray[np.float64]:
    x, y, theta = x_hat
    T_robot_in_world = create_transformation_matrix(
        rotation_matrix=rotation_matrix_2d(theta),
        translation_vector=np.array([x, y], dtype=np.float64),
    )

    T_tag_in_camera = world_robot_to_tag_camera(
        T_robot_in_world=T_robot_in_world,
        T_camera_in_robot=T_camera_in_robot,
        T_tag_in_world=T_tag_in_world,
    )

    return T_tag_in_camera


def AprilTagHJacobean2d(
    x_hat: NDArray[np.float64],
    *,
    T_tag_in_world_2d: NDArray[np.float64],
    T_camera_in_robot_2d: NDArray[np.float64],
) -> NDArray[np.float64]:
    x, y, theta = x_hat

    phi = yaw_from_T(T_camera_in_robot_2d)
    tx = T_tag_in_world_2d[0, 3]
    ty = T_tag_in_world_2d[1, 3]

    beta = theta + phi
    dx = tx - x
    dy = ty - y

    c_beta = np.cos(beta)
    s_beta = np.sin(beta)

    H = np.array(
        [
            [-c_beta, -s_beta, -s_beta * dx + c_beta * dy],
            [s_beta, -c_beta, -c_beta * dx - s_beta * dy],
            [0.0, 0.0, -1.0],
        ],
        dtype=float,
    )

    return H


@processor_for_data(KalmanFilterSensorType.APRIL_TAG)
def process_apriltags(solver: "PositionSolver2d", event: "SensorEvent") -> None:
    data = cast(AprilTagData, event.data)
    sensor_id = event.sensor_id

    if data.WhichOneof("data") == "raw_tags":
        raise ValueError(
            "Tried to insert AprilTagData with raw tags, but tags are not in processed format"
        )

    solver.nonlinear_predict_next()
    R: NDArray[np.float64] = solver._sensor_noise(
        KalmanFilterSensorType.APRIL_TAG,
        sensor_id,
        [solver.kPosXIdx, solver.kPosYIdx, solver.kThetaIdx],
    )

    for measurement in data.world_tags.tags:
        R_local: NDArray[np.float64] = np.copy(R)

        R_tag_in_camera = (
            solver.CAMERA_OUTPUT_TO_ROBOT_ROTATION
            @ from_float_list(list(measurement.pose_R), 3, 3)
            @ solver.CAMERA_OUTPUT_TO_ROBOT_ROTATION.T
        )
        tag_in_camera_pose = solver.CAMERA_OUTPUT_TO_ROBOT_ROTATION @ np.array(
            measurement.pose_t,
            dtype=np.float64,
        )

        theta_tag_rad = _theta_from_rotation(R_tag_in_camera)

        add, mult = april_tag_noise_adjustment(
            solver.x,
            np.array(measurement.pose_t, dtype=np.float64),
            measurement,
            solver.config.april_tag_config.tag_noise_adjust_config,
            solver.config.april_tag_config,
        )
        R_local = (R_local * mult) + (add * np.eye(R_local.shape[0], dtype=np.float64))

        data = np.array(
            [tag_in_camera_pose[0], tag_in_camera_pose[1], theta_tag_rad],
            dtype=np.float64,
        )

        T_tag_in_world_2d, T_camera_in_robot_2d = get_tag_information(
            measurement.id, sensor_id, solver
        )

        solver.update(
            data,
            AprilTagHJacobean2d,
            AprilTagHx2d,
            R_local,
            args=(
                T_tag_in_world_2d,
                T_camera_in_robot_2d,
            ),
            hx_args=(
                T_tag_in_world_2d,
                T_camera_in_robot_2d,
            ),
        )


def get_tag_information(
    tag_id: int, sensor_id: str, solver: "PositionSolver2d"
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    tag_info = solver.config.april_tag_config.tag_position_config.get(tag_id)
    camera_info = solver.config.april_tag_config.camera_position_config.get(sensor_id)

    if tag_info is None or camera_info is None:
        raise ValueError(f"Tag {tag_id} or camera {sensor_id} not found in config")

    T_tag_in_world_2d = extract_2d_from_3d_transformation(
        create_transformation_matrix(
            rotation_matrix=get_np_from_matrix(tag_info.rotation),
            translation_vector=get_np_from_vector(tag_info.position),
        )
    )

    T_camera_in_robot_2d = extract_2d_from_3d_transformation(
        create_transformation_matrix(
            rotation_matrix=get_np_from_matrix(camera_info.rotation),
            translation_vector=get_np_from_vector(camera_info.position),
        )
    )

    return T_tag_in_world_2d, T_camera_in_robot_2d


def april_tag_noise_adjustment(
    x: NDArray[np.float64],
    tag_pose_in_camera: NDArray[np.float64],
    tag: ProcessedTag,
    config_noise: TagNoiseAdjustConfig,
    config_tag: AprilTagConfig,
) -> tuple[float, float]:
    total_add = 0.0
    total_mult = 1.0

    estimate_xy = x[[PositionSolver2d.kPosXIdx, PositionSolver2d.kPosYIdx]]
    tag_xy = tag_pose_in_camera[:2]

    distance_from_estimate = float(np.linalg.norm(tag_xy - estimate_xy))

    min_distance = float(config_noise.min_distance_from_tag_to_use_noise_adjustment)
    if distance_from_estimate < min_distance:
        return total_add, total_mult

    noise_modes = set(config_tag.noise_change_modes)
    if TagNoiseAdjustMode.ADD_WEIGHT_PER_M_DISTANCE_TAG in noise_modes:
        total_add += float(config_noise.weight_per_m_from_distance_from_tag) * (
            distance_from_estimate
        )
    if TagNoiseAdjustMode.ADD_WEIGHT_PER_TAG_CONFIDENCE in noise_modes:
        total_add += float(config_noise.weight_per_confidence_tag) * float(
            tag.confidence
        )

    return total_add, total_mult


def _theta_from_rotation(rotation_world: NDArray[np.float64]) -> float:
    direction_vector = rotation_world[0:3, 0]
    return float(np.arctan2(direction_vector[1], direction_vector[0]))
