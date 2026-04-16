import numpy as np
import pytest

from backend.python.pos_extrapolator.__tests__.helpers import (
    BASE_RECEIVED_AT_S,
    BASE_TIMESTAMP_MS,
    insert_sensor,
    make_imu,
    make_odom,
    make_solver,
)
from backend.python.pos_extrapolator.position_solver_2d import residual_general


def test_predict_only_motion_uses_control_updates_between_timestamps():
    solver = make_solver()
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=1.0, dt_s=0.1),
        "odom",
        BASE_TIMESTAMP_MS,
        received_at_s=BASE_RECEIVED_AT_S,
    )
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=1.0, dt_s=0.1),
        "odom",
        BASE_TIMESTAMP_MS + 1000,
        received_at_s=BASE_RECEIVED_AT_S + 1.0,
    )

    state = solver.get_robot_state_estimate()

    assert len(state) == 6
    assert float(state[0]) == pytest.approx(0.1, abs=1e-6)
    assert float(state[1]) == pytest.approx(0.1, abs=1e-6)
    assert float(state[2]) == pytest.approx(1.0, abs=1e-6)
    assert float(state[3]) == pytest.approx(1.0, abs=1e-6)


def test_rotation_measurement_wraps_angle_residual():
    solver = make_solver(initial_state=[0.0, 0.0, 3.10])
    solver.update(
        z=np.array([-3.10], dtype=np.float64),
        HJacobian=lambda _: np.array([[0.0, 0.0, 1.0]], dtype=np.float64),
        Hx=lambda x: np.array([x[solver.kThetaIdx]], dtype=np.float64),
        R=np.array([[0.1]], dtype=np.float64),
        residual=lambda measurement, estimate: residual_general(
            measurement, estimate, 0
        ),
    )

    state = solver.get_state()
    assert abs(float(state[solver.kThetaIdx])) > 3.0


def test_future_projection_uses_latest_control_without_mutating_state():
    solver = make_solver()
    insert_sensor(
        solver,
        make_odom(vx=2.0, vy=0.0, dt_s=0.1),
        "odom",
        BASE_TIMESTAMP_MS,
        received_at_s=BASE_RECEIVED_AT_S,
    )
    insert_sensor(
        solver,
        make_odom(vx=2.0, vy=0.0, dt_s=0.1),
        "odom",
        BASE_TIMESTAMP_MS + 100,
        received_at_s=BASE_RECEIVED_AT_S + 0.1,
    )

    current = solver.get_state()
    projected = solver.get_robot_state_estimate(future_s=0.5)

    assert float(current[solver.kPosXIdx]) == pytest.approx(0.2, abs=1e-6)
    assert float(projected[0]) == pytest.approx(1.2, abs=1e-6)
    assert np.allclose(current, solver.get_state())


def test_imu_updates_angular_control_without_direct_measurement_update():
    solver = make_solver()
    insert_sensor(
        solver,
        make_imu(theta_rad=0.0, omega=1.5),
        "imu0",
        BASE_TIMESTAMP_MS,
        received_at_s=BASE_RECEIVED_AT_S,
    )
    insert_sensor(
        solver,
        make_imu(theta_rad=0.0, omega=1.5),
        "imu0",
        BASE_TIMESTAMP_MS + 200,
        received_at_s=BASE_RECEIVED_AT_S + 0.2,
    )

    state = solver.get_state()
    assert float(state[solver.kThetaIdx]) == pytest.approx(0.3, abs=1e-6)
