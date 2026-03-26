import numpy as np
import pytest

from backend.generated.thrift.config.common.ttypes import GenericVector
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    TagNoiseAdjustMode,
    TagRejectMode,
)
from backend.python.common.util.math import from_theta_to_3x3_mat
from backend.python.pos_extrapolator.__tests__.helpers import (
    make_processed_tag,
    make_solver,
)
from backend.python.pos_extrapolator.position_solver_2d import PositionSolver2d
from backend.python.pos_extrapolator.processors.apriltag_processor import (
    AprilTagHJacobean2d,
    AprilTagHx2d,
    april_tag_noise_adjustment,
    april_tag_should_reject,
    get_tag_information,
    wrap_angle,
)
from backend.python.pos_extrapolator.util.extrapolator_math import rotation_matrix_2d


def _robot_to_camera_translation(vector: np.ndarray) -> np.ndarray:
    return PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION.T @ vector


def _robot_to_camera_rotation(rotation: np.ndarray) -> np.ndarray:
    return (
        PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION.T
        @ rotation
        @ PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION
    )


def _transform_2d(x: float, y: float, theta: float) -> np.ndarray:
    transform = np.eye(3, dtype=np.float64)
    transform[:2, :2] = rotation_matrix_2d(theta)
    transform[:2, 2] = np.array([x, y], dtype=np.float64)
    return transform


def test_get_tag_information_returns_configured_2d_transforms():
    solver = make_solver()

    T_tag_in_world, T_camera_in_robot = get_tag_information(0, "cam0", solver)

    assert T_tag_in_world.shape == (3, 3)
    assert T_camera_in_robot.shape == (3, 3)
    assert T_tag_in_world == pytest.approx(_transform_2d(0.0, 0.0, 0.0), abs=1e-6)
    assert T_camera_in_robot == pytest.approx(_transform_2d(0.0, 0.0, 0.0), abs=1e-6)


def test_apriltag_hx2d_returns_planar_tag_pose_in_camera_frame():
    state = np.array([0.0, 0.0, 0.0], dtype=np.float64)
    T_camera_in_robot_2d = _transform_2d(0.0, 0.0, 0.0)
    T_tag_in_world_2d = _transform_2d(2.0, -1.0, np.deg2rad(45.0))

    measurement = AprilTagHx2d(
        state,
        T_tag_in_world=T_tag_in_world_2d,
        T_camera_in_robot=T_camera_in_robot_2d,
    )

    assert measurement.shape == (3,)
    assert measurement == pytest.approx(
        np.array([2.0, -1.0, np.deg2rad(45)], dtype=np.float64),
        abs=1e-6,
    )


def test_apriltag_hjacobian2d_matches_finite_difference():
    state = np.array([1.2, -0.7, 0.35], dtype=np.float64)
    T_camera_in_robot_2d = _transform_2d(0.4, -0.2, np.deg2rad(15.0))
    T_tag_in_world_2d = _transform_2d(3.0, 1.1, np.deg2rad(-25.0))

    analytic = AprilTagHJacobean2d(
        state,
        T_tag_in_world_2d=T_tag_in_world_2d,
        T_camera_in_robot_2d=T_camera_in_robot_2d,
    )

    numeric = np.zeros((3, 3), dtype=np.float64)
    epsilon = 1e-6
    for column in range(3):
        plus = state.copy()
        minus = state.copy()
        plus[column] += epsilon
        minus[column] -= epsilon
        delta = AprilTagHx2d(
            plus,
            T_tag_in_world=T_tag_in_world_2d,
            T_camera_in_robot=T_camera_in_robot_2d,
        ) - AprilTagHx2d(
            minus,
            T_tag_in_world=T_tag_in_world_2d,
            T_camera_in_robot=T_camera_in_robot_2d,
        )
        delta[2] = wrap_angle(float(delta[2]))
        numeric[:, column] = delta / (2.0 * epsilon)

    assert analytic == pytest.approx(numeric, abs=1e-6)


def test_apriltag_hx2d_reflects_robot_heading_in_camera_frame():
    state = np.array([0.0, 0.0, np.pi / 2], dtype=np.float64)
    measurement = AprilTagHx2d(
        state,
        T_tag_in_world=_transform_2d(1.0, 0.0, 0.0),
        T_camera_in_robot=_transform_2d(0.0, 0.0, 0.0),
    )

    assert measurement == pytest.approx(
        np.array([0.0, -1.0, -np.pi / 2], dtype=np.float64),
        abs=1e-6,
    )


def test_apriltag_distance_noise_adjustment_applies_additive_weight():
    solver = make_solver()
    solver.general_config.april_tag_config.noise_change_modes = [
        TagNoiseAdjustMode.ADD_WEIGHT_PER_M_DISTANCE_TAG
    ]
    solver.general_config.april_tag_config.tag_noise_adjust_config.weight_per_m_from_distance_from_tag = (
        2.0
    )
    measurement = np.array([3.0, 4.0, 0.0], dtype=np.float64)
    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([1.0, 0.0, 0.0]))
    data = make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t)

    add, mult = april_tag_noise_adjustment(
        solver.x,
        measurement,
        data.world_tags.tags[0],
        solver.config.april_tag_config.tag_noise_adjust_config,
        solver.config.april_tag_config,
    )

    assert add == pytest.approx(np.array([10.0, 10.0, 10.0], dtype=np.float64))
    assert mult == pytest.approx(1.0)


def test_apriltag_confidence_noise_adjustment_applies_additive_weight():
    solver = make_solver()
    solver.general_config.april_tag_config.noise_change_modes = [
        TagNoiseAdjustMode.ADD_WEIGHT_PER_TAG_CONFIDENCE
    ]
    solver.general_config.april_tag_config.tag_noise_adjust_config.weight_per_confidence_tag = (
        4.0
    )
    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([1.0, 0.0, 0.0]))
    data = make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t, confidence=0.25)

    add, mult = april_tag_noise_adjustment(
        solver.x,
        np.array([0.0, 0.0, 0.0]),
        data.world_tags.tags[0],
        solver.config.april_tag_config.tag_noise_adjust_config,
        solver.config.april_tag_config,
    )

    assert add == pytest.approx(np.array([1.0, 1.0, 1.0], dtype=np.float64))
    assert mult == pytest.approx(1.0)


def test_apriltag_tag_specific_noise_adjustment_applies_per_axis_noise():
    solver = make_solver()
    solver.general_config.april_tag_config.noise_change_modes = [
        TagNoiseAdjustMode.ADD_ADDITIVE_NOISE_BY_TAG_ID
    ]
    solver.general_config.april_tag_config.tag_noise_adjust_config.additive_noise_by_tag_id = {
        0: GenericVector(values=[0.5, 1.5, 2.5], size=3)
    }
    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([1.0, 0.0, 0.0]))
    data = make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t)

    add, mult = april_tag_noise_adjustment(
        solver.x,
        np.array([0.0, 0.0, 0.0]),
        data.world_tags.tags[0],
        solver.config.april_tag_config.tag_noise_adjust_config,
        solver.config.april_tag_config,
    )

    assert add == pytest.approx(np.array([0.5, 1.5, 2.5], dtype=np.float64))
    assert mult == pytest.approx(1.0)


def test_apriltag_tag_specific_noise_adjustment_ignores_unconfigured_tags():
    solver = make_solver()
    solver.general_config.april_tag_config.noise_change_modes = [
        TagNoiseAdjustMode.ADD_ADDITIVE_NOISE_BY_TAG_ID
    ]
    solver.general_config.april_tag_config.tag_noise_adjust_config.additive_noise_by_tag_id = {
        1: GenericVector(values=[0.5, 1.5, 2.5], size=3)
    }
    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([1.0, 0.0, 0.0]))
    data = make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t)

    add, mult = april_tag_noise_adjustment(
        solver.x,
        np.array([0.0, 0.0, 0.0]),
        data.world_tags.tags[0],
        solver.config.april_tag_config.tag_noise_adjust_config,
        solver.config.april_tag_config,
    )

    assert add == pytest.approx(np.zeros(3, dtype=np.float64))
    assert mult == pytest.approx(1.0)


def test_apriltag_rejects_measurement_over_max_distance():
    solver = make_solver()
    solver.general_config.april_tag_config.reject_modes = [
        TagRejectMode.REJECT_OVER_MAX_DISTANCE_FROM_TAG
    ]
    solver.general_config.april_tag_config.tag_reject_config.max_distance_from_tag = 4.0

    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([3.0, 4.0, 0.0]))
    data = make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t)

    assert april_tag_should_reject(
        PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION
        @ np.array(data.world_tags.tags[0].pose_t, dtype=np.float64),
        data.world_tags.tags[0],
        solver.config.april_tag_config.tag_reject_config,
        solver.config.april_tag_config,
    )


def test_apriltag_rejects_measurement_under_min_confidence():
    solver = make_solver()
    solver.general_config.april_tag_config.reject_modes = [
        TagRejectMode.REJECT_UNDER_MIN_TAG_CONFIDENCE
    ]
    solver.general_config.april_tag_config.tag_reject_config.min_tag_confidence = 0.5

    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([1.0, 0.0, 0.0]))
    data = make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t, confidence=0.25)

    assert april_tag_should_reject(
        PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION
        @ np.array(data.world_tags.tags[0].pose_t, dtype=np.float64),
        data.world_tags.tags[0],
        solver.config.april_tag_config.tag_reject_config,
        solver.config.april_tag_config,
    )


def test_apriltag_keeps_measurement_when_reject_thresholds_pass():
    solver = make_solver()
    solver.general_config.april_tag_config.reject_modes = [
        TagRejectMode.REJECT_OVER_MAX_DISTANCE_FROM_TAG,
        TagRejectMode.REJECT_UNDER_MIN_TAG_CONFIDENCE,
    ]
    solver.general_config.april_tag_config.tag_reject_config.max_distance_from_tag = 5.0
    solver.general_config.april_tag_config.tag_reject_config.min_tag_confidence = 0.5

    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([3.0, 4.0, 0.0]))
    data = make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t, confidence=0.5)

    assert not april_tag_should_reject(
        PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION
        @ np.array(data.world_tags.tags[0].pose_t, dtype=np.float64),
        data.world_tags.tags[0],
        solver.config.april_tag_config.tag_reject_config,
        solver.config.april_tag_config,
    )
