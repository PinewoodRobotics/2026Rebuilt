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
from backend.generated.thrift.config.pos_extrapolator.ttypes import TagNoiseAdjustMode
from backend.python.common.util.math import (
    create_transformation_matrix,
    from_float_list,
    get_np_from_matrix,
    get_np_from_vector,
    get_robot_in_world,
    get_translation_rotation_components,
    make_transformation_matrix_p_d,
)
from backend.python.pos_extrapolator.processor_registry import processor_for_data

if TYPE_CHECKING:
    from backend.python.pos_extrapolator.position_solver_2d import (
        PositionSolver2d,
        SensorEvent,
    )


@dataclass
class AprilTagMeasurement:
    values: NDArray[np.float64]
    state_indices: list[int]
    add: float
    mult: float


@processor_for_data(KalmanFilterSensorType.APRIL_TAG)
def process_apriltags(solver: "PositionSolver2d", event: "SensorEvent") -> None:
    data = cast(AprilTagData, event.data)
    sensor_id = event.sensor_id

    if data.WhichOneof("data") == "raw_tags":
        raise ValueError(
            "Tried to insert AprilTagData with raw tags, but tags are not in processed format"
        )

    solver.predict_to_timestamp(event.timestamp_s, solver.current_control)

    for measurement in build_apriltag_measurements(solver, data, sensor_id):
        sensor_indices = [
            solver.kPosXIdx if state_idx == solver.kPosXIdx else state_idx
            for state_idx in measurement.state_indices
        ]
        R = solver._sensor_noise(
            KalmanFilterSensorType.APRIL_TAG,
            sensor_id,
            sensor_indices,
        )
        R = R * measurement.mult
        if measurement.add != 0.0:
            R = R.copy()
            for idx in range(min(R.shape[0], R.shape[1])):
                R[idx, idx] += measurement.add

        if not solver.should_accept_apriltag_measurement(
            measurement.values[:2],
            R if R.shape[0] >= 2 else np.eye(2, dtype=np.float64),
        ):
            continue

        solver._correct(
            z=measurement.values,
            state_indices=measurement.state_indices,
            R=R,
        )
        if solver.kThetaIdx in measurement.state_indices:
            solver.has_gotten_rotation = True


def build_apriltag_measurements(
    solver: "PositionSolver2d",
    data: AprilTagData,
    sensor_id: str,
) -> list[AprilTagMeasurement]:
    output: list[AprilTagMeasurement] = []

    for tag in data.world_tags.tags:
        if tag.id not in solver.general_config.april_tag_config.tag_position_config:
            continue
        if (
            sensor_id
            not in solver.general_config.april_tag_config.camera_position_config
        ):
            continue

        measurement_values, state_indices = _solve_world_measurement_from_tag(
            solver, tag, sensor_id
        )
        add, mult = april_tag_noise_adjustment(solver, measurement_values, tag)
        output.append(
            AprilTagMeasurement(
                values=measurement_values,
                state_indices=state_indices,
                add=add,
                mult=mult,
            )
        )

    return output


def _solve_world_measurement_from_tag(
    solver: "PositionSolver2d",
    tag: ProcessedTag,
    sensor_id: str,
) -> tuple[NDArray[np.float64], list[int]]:
    camera_pose: Point3 = solver.general_config.april_tag_config.camera_position_config[
        sensor_id
    ]
    tag_pose_world: Point3 = solver.general_config.april_tag_config.tag_position_config[
        tag.id
    ]

    T_camera_in_robot = create_transformation_matrix(
        rotation_matrix=get_np_from_matrix(camera_pose.rotation),
        translation_vector=get_np_from_vector(camera_pose.position),
    )
    T_tag_in_world = create_transformation_matrix(
        rotation_matrix=get_np_from_matrix(tag_pose_world.rotation),
        translation_vector=get_np_from_vector(tag_pose_world.position),
    )
    tag_in_camera_rotation = (
        solver.CAMERA_OUTPUT_TO_ROBOT_ROTATION
        @ from_float_list(list(tag.pose_R), 3, 3)
        @ solver.CAMERA_OUTPUT_TO_ROBOT_ROTATION.T
    )
    tag_in_camera_pose = solver.CAMERA_OUTPUT_TO_ROBOT_ROTATION @ np.array(
        tag.pose_t,
        dtype=np.float64,
    )
    T_tag_in_camera = create_transformation_matrix(
        rotation_matrix=tag_in_camera_rotation,
        translation_vector=tag_in_camera_pose,
    )

    predicted_direction = np.array(
        [
            float(np.cos(solver.x[solver.kThetaIdx])),
            float(np.sin(solver.x[solver.kThetaIdx])),
            0.0,
        ],
        dtype=np.float64,
    )
    predicted_robot_rotation_world = make_transformation_matrix_p_d(
        direction_vector=predicted_direction
    )[:3, :3]

    robot_in_world_with_predicted_theta = get_robot_in_world(
        T_tag_in_camera=T_tag_in_camera,
        T_camera_in_robot=T_camera_in_robot,
        T_tag_in_world=T_tag_in_world,
        R_robot_rotation_world=predicted_robot_rotation_world,
    )
    position_world, constrained_rotation_world = get_translation_rotation_components(
        robot_in_world_with_predicted_theta
    )

    if not solver.general_config.april_tag_config.insert_predicted_global_rotation:
        return (
            np.array([position_world[0], position_world[1]], dtype=np.float64),
            [solver.kPosXIdx, solver.kPosYIdx],
        )

    robot_in_world_unconstrained = get_robot_in_world(
        T_tag_in_camera=T_tag_in_camera,
        T_camera_in_robot=T_camera_in_robot,
        T_tag_in_world=T_tag_in_world,
    )
    _, unconstrained_rotation_world = get_translation_rotation_components(
        robot_in_world_unconstrained
    )
    theta_rad = _theta_from_rotation(unconstrained_rotation_world)
    return (
        np.array([position_world[0], position_world[1], theta_rad], dtype=np.float64),
        [solver.kPosXIdx, solver.kPosYIdx, solver.kThetaIdx],
    )


def _theta_from_rotation(rotation_world: NDArray[np.float64]) -> float:
    direction_vector = rotation_world[0:3, 0]
    return float(np.arctan2(direction_vector[1], direction_vector[0]))


def april_tag_noise_adjustment(
    solver: "PositionSolver2d",
    measurement: NDArray[np.float64],
    tag: ProcessedTag,
) -> tuple[float, float]:
    config = solver.general_config.april_tag_config.tag_noise_adjust_config
    total_add = 0.0
    total_mult = 1.0

    distance_from_estimate = float(
        np.linalg.norm(measurement[:2] - solver.x[[solver.kPosXIdx, solver.kPosYIdx]])
    )
    min_distance = float(config.min_distance_from_tag_to_use_noise_adjustment)
    if distance_from_estimate < min_distance:
        return total_add, total_mult

    noise_modes = set(solver.general_config.april_tag_config.noise_change_modes)
    if TagNoiseAdjustMode.ADD_WEIGHT_PER_M_DISTANCE_TAG in noise_modes:
        total_add += float(config.weight_per_m_from_distance_from_tag) * (
            distance_from_estimate
        )
    if TagNoiseAdjustMode.ADD_WEIGHT_PER_TAG_CONFIDENCE in noise_modes:
        total_add += float(config.weight_per_confidence_tag) * float(tag.confidence)

    return total_add, total_mult
