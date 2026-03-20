import numpy as np
import pytest

from backend.generated.proto.python.sensor.apriltags_pb2 import AprilTagData
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
from backend.python.pos_extrapolator.processors.apriltag_processor import (
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


def test_imu_missing_sensor_id_raises_keyerror():
    solver = make_solver()
    with pytest.raises(KeyError):
        insert_sensor(
            solver,
            make_imu(theta_rad=0.0),
            "missing",
            BASE_TIMESTAMP_MS,
            received_at_s=BASE_RECEIVED_AT_S,
        )


def test_apriltag_raw_tags_raise_value_error():
    solver = make_solver()
    data = AprilTagData()
    data.raw_tags.corners.extend([])

    with pytest.raises(ValueError):
        insert_sensor(
            solver,
            data,
            "cam0",
            BASE_TIMESTAMP_MS,
            received_at_s=BASE_RECEIVED_AT_S,
        )


def test_unknown_apriltag_ids_are_skipped():
    solver = make_solver()
    data = AprilTagData()
    data.world_tags.tags.add(
        id=999,
        pose_R=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        pose_t=[1.0, 0.0, 0.0],
    )

    measurements = build_apriltag_measurements(solver, data, "cam0")
    assert measurements == []


def test_mahalanobis_gate_rejects_far_apriltag_outlier():
    solver = make_solver()
    solver.P = np.eye(3) * 1e-4

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
    before = solver.get_state()

    tag_R = _robot_to_camera_rotation(from_theta_to_3x3_mat(0))
    tag_t = _robot_to_camera_translation(np.array([100.0, 0.0, 0.0]))
    outlier = make_processed_tag(tag_id=0, pose_R=tag_R, pose_t=tag_t)
    insert_sensor(
        solver,
        outlier,
        "cam0",
        BASE_TIMESTAMP_MS + 100,
        received_at_s=BASE_RECEIVED_AT_S + 0.1,
    )
    after = solver.get_state()

    assert np.allclose(after, before)


def test_late_packet_older_than_history_window_is_ignored():
    solver = make_solver()
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=0.0),
        "odom",
        BASE_TIMESTAMP_MS,
        received_at_s=BASE_RECEIVED_AT_S,
    )
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=0.0),
        "odom",
        BASE_TIMESTAMP_MS + 400,
        received_at_s=BASE_RECEIVED_AT_S + 0.4,
    )
    before = solver.get_state()

    insert_sensor(
        solver,
        make_odom(vx=0.0, vy=0.0),
        "odom",
        BASE_TIMESTAMP_MS + 50,
        received_at_s=BASE_RECEIVED_AT_S + 0.45,
    )
    after = solver.get_state()

    assert np.allclose(after, before)
