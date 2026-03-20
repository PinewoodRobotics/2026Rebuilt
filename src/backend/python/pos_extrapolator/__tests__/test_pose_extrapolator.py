import numpy as np
import pytest

from backend.generated.proto.python.util.position_pb2 import RobotPosition
from backend.python.pos_extrapolator.__tests__.helpers import (
    BASE_TIMESTAMP_MS,
    make_extrapolator,
    make_imu,
    make_odom,
)


def test_position_extrapolator_dispatches_raw_sensor_data_into_solver():
    extrapolator = make_extrapolator()

    extrapolator.insert_sensor_data(make_odom(vx=1.0, vy=0.0), "odom", BASE_TIMESTAMP_MS)
    extrapolator.insert_sensor_data(
        make_imu(theta_rad=0.0, omega=0.25),
        "imu0",
        BASE_TIMESTAMP_MS + 100,
    )

    estimate = extrapolator.get_robot_position_estimate()
    assert len(estimate) == 6
    assert estimate[0] >= 0.0
    assert float(estimate[5]) == pytest.approx(0.25, abs=1e-6)


def test_get_robot_position_estimate_returns_six_value_state():
    extrapolator = make_extrapolator()
    extrapolator.insert_sensor_data(make_odom(vx=1.0, vy=0.0), "odom", BASE_TIMESTAMP_MS)
    extrapolator.insert_sensor_data(
        make_odom(vx=1.0, vy=0.0),
        "odom",
        BASE_TIMESTAMP_MS + 100,
    )

    estimate = extrapolator.get_robot_position_estimate()

    assert len(estimate) == 6
    assert estimate[0] == pytest.approx(0.1, abs=1e-6)
    assert estimate[2] == pytest.approx(1.0, abs=1e-6)
    assert estimate[5] == pytest.approx(0.0, abs=1e-6)


def test_get_robot_position_maps_solver_state_to_proto_fields():
    extrapolator = make_extrapolator()
    extrapolator.insert_sensor_data(
        make_odom(vx=1.1, vy=-2.2, omega=0.05),
        "odom",
        BASE_TIMESTAMP_MS,
    )
    extrapolator.insert_sensor_data(
        make_imu(theta_rad=0.6435, omega=0.05),
        "imu0",
        BASE_TIMESTAMP_MS + 20,
    )
    proto: RobotPosition = extrapolator.get_robot_position()
    estimate = extrapolator.get_robot_position_estimate()

    assert float(proto.position_2d.velocity.x) == pytest.approx(float(estimate[2]))
    assert float(proto.position_2d.velocity.y) == pytest.approx(float(estimate[3]))
    assert float(proto.position_2d.direction.x) == pytest.approx(np.cos(estimate[4]))
    assert float(proto.position_2d.direction.y) == pytest.approx(np.sin(estimate[4]))
    assert float(proto.position_2d.rotation_speed_rad_s) == pytest.approx(float(estimate[5]))
    assert list(proto.P) == pytest.approx(extrapolator.get_position_covariance())


def test_get_position_covariance_flattens_three_state_matrix():
    extrapolator = make_extrapolator()

    flat = extrapolator.get_position_covariance()

    assert len(flat) == 9


def test_get_confidence_is_forwarded():
    extrapolator = make_extrapolator()
    assert extrapolator.get_confidence() == 1.0
