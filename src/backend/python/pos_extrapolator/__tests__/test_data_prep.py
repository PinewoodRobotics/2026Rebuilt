import pytest

from backend.python.pos_extrapolator.__tests__.helpers import (
    BASE_RECEIVED_AT_S,
    BASE_TIMESTAMP_MS,
    insert_sensor,
    make_imu,
    make_odom,
    make_solver,
)


def test_solver_uses_message_timestamps_for_predict_step():
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
        received_at_s=BASE_RECEIVED_AT_S + 3.5,
    )

    state = solver.get_state()
    assert float(state[solver.kPosXIdx]) == pytest.approx(0.2, abs=1e-6)


def test_different_sensor_clock_offsets_normalize_into_shared_timeline():
    solver = make_solver()
    insert_sensor(
        solver,
        make_odom(vx=1.0, vy=0.0),
        "odom",
        10_000.0,
        received_at_s=1_000.0,
    )
    insert_sensor(
        solver,
        make_imu(theta_rad=0.0, omega=0.0),
        "imu0",
        20_500.0,
        received_at_s=1_000.5,
    )
    insert_sensor(
        solver,
        make_imu(theta_rad=0.0, omega=0.0),
        "imu0",
        20_700.0,
        received_at_s=1_000.7,
    )

    estimate = solver.get_robot_state_estimate()
    assert float(estimate[0]) == pytest.approx(0.7, abs=1e-6)
    assert float(estimate[4]) == pytest.approx(0.0, abs=1e-6)
