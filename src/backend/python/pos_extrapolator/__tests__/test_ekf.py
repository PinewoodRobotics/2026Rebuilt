import time

from numpy.typing import NDArray
from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterConfig,
    KalmanFilterSensorConfig,
)
from backend.generated.thrift.config.common.ttypes import GenericMatrix, GenericVector
from backend.python.pos_extrapolator.data_prep import KalmanFilterInput, ProcessedData
from backend.python.pos_extrapolator.filters.extended_kalman_filter import (
    ExtendedKalmanFilterStrategy,
)
import numpy as np


def _eye(n: int) -> list[list[float]]:
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def make_test_kalman_filter_config() -> KalmanFilterConfig:
    # 7D state: [x, y, vx, vy, cos, sin, angular_velocity_rad_s]
    dim_x = 7
    dim_z = 5  # [vx, vy, cos, sin, omega]

    state_vector = GenericVector(values=[0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0], size=dim_x)
    F = GenericMatrix(values=_eye(dim_x), rows=dim_x, cols=dim_x)
    P = GenericMatrix(values=_eye(dim_x), rows=dim_x, cols=dim_x)
    Q = GenericMatrix(
        values=[[0.01 if i == j else 0.0 for j in range(dim_x)] for i in range(dim_x)],
        rows=dim_x,
        cols=dim_x,
    )

    R = GenericMatrix(values=_eye(dim_z), rows=dim_z, cols=dim_z)
    H = GenericMatrix(values=_eye(dim_z), rows=dim_z, cols=dim_z)
    sensors = {
        KalmanFilterSensorType.IMU: {
            "0": KalmanFilterSensorConfig(
                measurement_noise_matrix=R,
                measurement_conversion_matrix=H,
            )
        }
    }

    return KalmanFilterConfig(
        state_vector=state_vector,
        state_transition_matrix=F,
        uncertainty_matrix=P,
        process_noise_matrix=Q,
        time_step_initial=0.05,
        sensors=sensors,
        dim_x_z=[dim_x, dim_z],
    )


def sample_jacobian_h(_x: NDArray[np.float64]) -> NDArray[np.float64]:
    # 7D state: [x, y, vx, vy, cos, sin, angular_velocity_rad_s]
    # Measurement: [vx, vy, cos, sin, omega]
    H = np.zeros((5, 7))
    H[0, 2] = 1  # vx
    H[1, 3] = 1  # vy
    H[2, 4] = 1  # cos
    H[3, 5] = 1  # sin
    H[4, 6] = 1  # omega
    return H


def sample_hx(x: NDArray[np.float64]) -> NDArray[np.float64]:
    return x[[2, 3, 4, 5, 6]]  # vx, vy, cos, sin, omega


def ekf_dataset_imu_input():
    return [
        KalmanFilterInput(
            input=ProcessedData(data=np.array([1.0, 1.0, 1.0, 0.0, 0.0])),
            sensor_id="0",
            sensor_type=KalmanFilterSensorType.IMU,
            jacobian_h=sample_jacobian_h,
            hx=sample_hx,
        ),
        KalmanFilterInput(
            input=ProcessedData(data=np.array([1.0, 1.0, 1.0, 0.0, 0.0])),
            sensor_id="0",
            sensor_type=KalmanFilterSensorType.IMU,
            jacobian_h=sample_jacobian_h,
            hx=sample_hx,
        ),
        KalmanFilterInput(
            input=ProcessedData(data=np.array([1.0, 1.0, 1.0, 0.0, 0.0])),
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

    # Check that the state is close to expected values (accounting for noise)
    # Logic: Start near [0,0,0,0,1,0,0], measure vx=1,vy=1,cos=1,sin=0,omega=0 and predict 1s each step.
    expected = [3, 3, 1, 1, 1, 0, 0]
    assert len(state) == len(expected)
    for i, (actual, exp) in enumerate(zip(state, expected)):
        assert abs(actual - exp) < 0.25, f"State[{i}]: expected {exp}, got {actual}"


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
