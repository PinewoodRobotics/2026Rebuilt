import numpy as np
import pytest

from backend.python.common.util.math import from_theta_to_3x3_mat
from backend.python.pos_extrapolator.__tests__.helpers import (
    BASE_RECEIVED_AT_S,
    BASE_TIMESTAMP_MS,
    insert_sensor,
    make_imu,
    make_odom,
    make_processed_tag,
    make_solver,
)
from backend.python.pos_extrapolator.position_solver_2d import PositionSolver2d


def _robot_to_camera_translation(vector: np.ndarray) -> np.ndarray:
    return PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION.T @ vector


def _robot_to_camera_rotation(rotation: np.ndarray) -> np.ndarray:
    return (
        PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION.T
        @ rotation
        @ PositionSolver2d.CAMERA_OUTPUT_TO_ROBOT_ROTATION
    )


def test_late_apriltag_replays_history_from_the_past():
    solver = make_solver()
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=0.0, dt_s=0.1),
        "odom",
        BASE_TIMESTAMP_MS,
        received_at_s=BASE_RECEIVED_AT_S,
    )
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=0.0, dt_s=0.1),
        "odom",
        BASE_TIMESTAMP_MS + 100,
        received_at_s=BASE_RECEIVED_AT_S + 0.1,
    )
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=0.0, dt_s=0.1),
        "odom",
        BASE_TIMESTAMP_MS + 200,
        received_at_s=BASE_RECEIVED_AT_S + 0.2,
    )
    before = solver.get_state()

    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([0.9, 0.0, 0.0]))
    late_tag = make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t)
    insert_sensor(
        solver,
        late_tag,
        "cam0",
        BASE_TIMESTAMP_MS + 50,
        received_at_s=BASE_RECEIVED_AT_S + 0.05,
    )
    after = solver.get_state()

    assert float(before[solver.kPosXIdx]) == pytest.approx(0.2, abs=1e-6)
    assert float(after[solver.kPosXIdx]) < float(before[solver.kPosXIdx])


def test_predict_jacobian_tracks_heading_sensitivity():
    solver = make_solver(initial_state=[0.0, 0.0, np.pi / 2])
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=0.0, dt_s=0.1),
        "odom",
        BASE_TIMESTAMP_MS,
        received_at_s=BASE_RECEIVED_AT_S,
    )
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=0.0, dt_s=0.1),
        "odom",
        BASE_TIMESTAMP_MS + 100,
        received_at_s=BASE_RECEIVED_AT_S + 0.1,
    )

    assert float(solver.F[solver.kPosXIdx, solver.kThetaIdx]) == pytest.approx(-0.1)


def test_get_confidence_returns_constant_one():
    solver = make_solver()
    solver.P = np.eye(3)
    assert solver.get_confidence() == 1.0


def test_future_projection_rotates_heading_with_angular_velocity():
    solver = make_solver()
    insert_sensor(
        solver,
        make_imu(theta_rad=0.0, omega=1.0),
        "imu0",
        BASE_TIMESTAMP_MS,
        received_at_s=BASE_RECEIVED_AT_S,
    )

    projected = solver.get_robot_state_estimate(future_s=np.pi / 2)

    assert float(projected[4]) == pytest.approx(np.pi / 2, abs=1e-6)
    assert float(projected[5]) == pytest.approx(1.0, abs=1e-6)


def test_imu_velocity_rotates_world_velocity_output():
    solver = make_solver(imu_use_velocity=True, initial_state=[0.0, 0.0, np.pi / 2])
    insert_sensor(
        solver,
        make_imu(theta_rad=np.pi / 2, omega=0.0, vx=1.0, vy=0.0),
        "imu0",
        BASE_TIMESTAMP_MS,
        received_at_s=BASE_RECEIVED_AT_S,
    )

    estimate = solver.get_robot_state_estimate()
    assert float(estimate[2]) == pytest.approx(0.0, abs=1e-6)
    assert float(estimate[3]) == pytest.approx(1.0, abs=1e-6)


def test_odometry_prediction_stays_smooth_after_tag_correction():
    solver = make_solver(insert_predicted_global_rotation=False)
    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([1.0, 0.0, 0.0]))
    insert_sensor(
        solver,
        make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t),
        "cam0",
        BASE_TIMESTAMP_MS,
        received_at_s=BASE_RECEIVED_AT_S,
    )

    positions: list[float] = []
    for step in range(1, 4):
        insert_sensor(
            solver,
            make_odom(vx=1.0, vy=0.0, dt_s=0.1),
            "odom",
            BASE_TIMESTAMP_MS + (step * 100),
            received_at_s=BASE_RECEIVED_AT_S + (step * 0.1),
        )
        positions.append(float(solver.get_state()[solver.kPosXIdx]))

    deltas = np.diff(positions)
    assert positions == pytest.approx(sorted(positions), abs=1e-6)
    assert deltas == pytest.approx([0.1, 0.1], abs=1e-6)


def test_stale_late_tag_is_ignored_once_history_seed_has_advanced():
    solver = make_solver(insert_predicted_global_rotation=False)
    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    initial_tag_t = _robot_to_camera_translation(np.array([1.0, 0.0, 0.0]))
    insert_sensor(
        solver,
        make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=initial_tag_t),
        "cam0",
        BASE_TIMESTAMP_MS,
        received_at_s=BASE_RECEIVED_AT_S,
    )
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=0.0, dt_s=0.1),
        "odom",
        BASE_TIMESTAMP_MS + 100,
        received_at_s=BASE_RECEIVED_AT_S + 0.1,
    )
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=0.0, dt_s=0.9),
        "odom",
        BASE_TIMESTAMP_MS + 1000,
        received_at_s=BASE_RECEIVED_AT_S + 1.0,
    )
    before = solver.get_state().copy()

    tag_t = _robot_to_camera_translation(np.array([0.9, 0.0, 0.0]))
    insert_sensor(
        solver,
        make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t),
        "cam0",
        BASE_TIMESTAMP_MS + 50,
        received_at_s=BASE_RECEIVED_AT_S + 1.05,
    )
    after = solver.get_state()

    assert float(before[solver.kPosXIdx]) == pytest.approx(
        0.09090909090909094,
        abs=1e-6,
    )
    assert np.allclose(after, before)
