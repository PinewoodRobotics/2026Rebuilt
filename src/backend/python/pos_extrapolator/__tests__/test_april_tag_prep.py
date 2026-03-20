import numpy as np
import pytest

from backend.generated.thrift.config.pos_extrapolator.ttypes import TagNoiseAdjustMode
from backend.python.common.util.math import from_theta_to_3x3_mat
from backend.python.pos_extrapolator.__tests__.helpers import (
    make_processed_tag,
    make_solver,
)
from backend.python.pos_extrapolator.position_solver_2d import PositionSolver2d
from backend.python.pos_extrapolator.processors.apriltag_processor import (
    april_tag_noise_adjustment,
    build_apriltag_measurements,
)


def _robot_to_camera_translation(vector: np.ndarray) -> np.ndarray:
    return PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION.T @ vector


def _robot_to_camera_rotation(rotation: np.ndarray) -> np.ndarray:
    return (
        PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION.T
        @ rotation
        @ PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION
    )


def test_apriltag_measurement_converts_into_world_pose_using_predicted_heading():
    solver = make_solver(insert_predicted_global_rotation=False)
    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([1.0, 0.0, 0.0]))
    data = make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t)

    output = build_apriltag_measurements(solver, data, "cam0")

    assert len(output) == 1
    measurement = output[0]
    assert measurement.values.shape == (2,)
    assert measurement.state_indices == [solver.kPosXIdx, solver.kPosYIdx]
    assert float(measurement.values[0]) == pytest.approx(-1.0, abs=1e-6)
    assert float(measurement.values[1]) == pytest.approx(0.0, abs=1e-6)


def test_apriltag_measurement_uses_unconstrained_theta_when_enabled():
    solver = make_solver(insert_predicted_global_rotation=True)
    solver.x[solver.kThetaIdx] = np.deg2rad(30.0)
    noisy_rotation = _robot_to_camera_rotation(from_theta_to_3x3_mat(10))
    tag_t = _robot_to_camera_translation(np.array([1.0, 0.0, 0.0]))
    data = make_processed_tag(tag_id=0, pose_R=noisy_rotation, pose_t=tag_t)

    output = build_apriltag_measurements(solver, data, "cam0")

    assert len(output) == 1
    measurement = output[0]
    assert measurement.values.shape == (3,)
    assert measurement.state_indices == [
        solver.kPosXIdx,
        solver.kPosYIdx,
        solver.kThetaIdx,
    ]
    assert float(measurement.values[2]) != pytest.approx(float(solver.x[solver.kThetaIdx]))


def test_apriltag_distance_noise_adjustment_applies_additive_weight():
    solver = make_solver()
    solver.general_config.april_tag_config.noise_change_modes = [
        TagNoiseAdjustMode.ADD_WEIGHT_PER_M_DISTANCE_TAG
    ]
    solver.general_config.april_tag_config.tag_noise_adjust_config.weight_per_m_from_distance_from_tag = (
        2.0
    )
    measurement = np.array([3.0, 4.0, 0.0])
    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([1.0, 0.0, 0.0]))
    data = make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t)

    add, mult = april_tag_noise_adjustment(solver, measurement, data.world_tags.tags[0])

    assert add == pytest.approx(10.0)
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
        solver,
        np.array([0.0, 0.0, 0.0]),
        data.world_tags.tags[0],
    )

    assert add == pytest.approx(1.0)
    assert mult == pytest.approx(1.0)
