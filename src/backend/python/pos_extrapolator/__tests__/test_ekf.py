import time

from numpy.typing import NDArray
import pytest
from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterConfig,
    KalmanFilterSensorConfig,
)
from backend.generated.thrift.config.common.ttypes import GenericMatrix, GenericVector
from backend.python.pos_extrapolator.data_prep import KalmanFilterInput
from backend.python.pos_extrapolator.filters.extended_kalman_filter import (
    ExtendedKalmanFilterStrategy,
)
import numpy as np


def _eye(n: int) -> list[list[float]]:
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def make_test_kalman_filter_config() -> KalmanFilterConfig:
    # 6D state: [x, y, vx, vy, angle_rad, angular_velocity_rad_s]
    dim_x = 6
    dim_z = 4  # [vx, vy, angle, omega]

    state_vector = GenericVector(values=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], size=dim_x)
    P = GenericMatrix(values=_eye(dim_x), rows=dim_x, cols=dim_x)
    Q = GenericMatrix(
        values=[[0.01 if i == j else 0.0 for j in range(dim_x)] for i in range(dim_x)],
        rows=dim_x,
        cols=dim_x,
    )

    R = GenericMatrix(values=_eye(dim_z), rows=dim_z, cols=dim_z)
    sensors = {
        KalmanFilterSensorType.IMU: {
            "0": KalmanFilterSensorConfig(measurement_noise_matrix=R)
        }
    }

    return KalmanFilterConfig(
        initial_state_vector=state_vector,
        uncertainty_matrix=P,
        process_noise_matrix=Q,
        sensors=sensors,
    )


def sample_jacobian_h(_x: NDArray[np.float64]) -> NDArray[np.float64]:
    # 6D state: [x, y, vx, vy, angle, omega]
    # Measurement: [vx, vy, angle, omega]
    H = np.zeros((4, 6))
    H[0, 2] = 1  # vx
    H[1, 3] = 1  # vy
    H[2, 4] = 1  # angle
    H[3, 5] = 1  # omega
    return H


def sample_hx(x: NDArray[np.float64]) -> NDArray[np.float64]:
    return x[[2, 3, 4, 5]]  # vx, vy, angle, omega


def ekf_dataset_imu_input():
    return [
        KalmanFilterInput(
            input=np.array([1.0, 1.0, 0.0, 0.0]),
            sensor_id="0",
            sensor_type=KalmanFilterSensorType.IMU,
            jacobian_h=sample_jacobian_h,
            hx=sample_hx,
        ),
        KalmanFilterInput(
            input=np.array([1.0, 1.0, 0.0, 0.0]),
            sensor_id="0",
            sensor_type=KalmanFilterSensorType.IMU,
            jacobian_h=sample_jacobian_h,
            hx=sample_hx,
        ),
        KalmanFilterInput(
            input=np.array([1.0, 1.0, 0.0, 0.0]),
            sensor_id="0",
            sensor_type=KalmanFilterSensorType.IMU,
            jacobian_h=sample_jacobian_h,
            hx=sample_hx,
        ),
    ]


def test_ekf():
    ekf = ExtendedKalmanFilterStrategy(make_test_kalman_filter_config(), fake_dt=1)
    for input in ekf_dataset_imu_input():
        ekf.insert_data(input)

    # Normalize to plain python floats for stable assertions/type checking.
    state = [float(v) for v in ekf.get_state().flatten().tolist()]
    print(state)

    # Behavior-level checks for the current EKF tuning:
    # - symmetric x/y motion from symmetric measurements
    # - positive position and velocity from repeated +1 velocity measurements
    # - bounded velocity due to Kalman blending
    assert len(state) == 6
    assert state[0] == pytest.approx(state[1], abs=1e-6)
    assert state[2] == pytest.approx(state[3], abs=1e-6)
    assert 1.5 < state[0] < 3.0
    assert 0.4 < state[2] < 1.1
    assert state[4] == pytest.approx(0.0, abs=1e-6)
    assert state[5] == pytest.approx(0.0, abs=1e-6)


def test_ekf_timing():
    ekf = ExtendedKalmanFilterStrategy(make_test_kalman_filter_config(), fake_dt=1)
    avg_time = 0
    for input in ekf_dataset_imu_input():
        start_time = time.time()
        ekf.insert_data(input)
        end_time = time.time()
        avg_time += end_time - start_time

    avg_time /= len(ekf_dataset_imu_input())
    print(f"Average time per insert: {avg_time} seconds")

    assert avg_time < 0.01
