import warnings

import numpy as np
import pytest

from backend.generated.thrift.config.common.ttypes import GenericMatrix, GenericVector
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterConfig,
    KalmanFilterSensorConfig,
    KalmanFilterSensorType,
)
from backend.python.pos_extrapolator.data_prep import KalmanFilterInput, ProcessedData
from backend.python.pos_extrapolator.filters.extended_kalman_filter import (
    ExtendedKalmanFilterStrategy,
    _add_to_diagonal,
)


def _eye(n: int) -> list[list[float]]:
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def make_cfg(*, include_sensors: bool = True) -> KalmanFilterConfig:
    # 6D state: [x, y, vx, vy, angle, omega]
    dim_x = 6
    dim_z = 4

    state_vector = GenericVector(values=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0], size=dim_x)
    P = GenericMatrix(values=_eye(dim_x), rows=dim_x, cols=dim_x)
    Q = GenericMatrix(values=_eye(dim_x), rows=dim_x, cols=dim_x)
    R = GenericMatrix(values=_eye(dim_z), rows=dim_z, cols=dim_z)

    sensors = {}
    if include_sensors:
        sensors = {
            KalmanFilterSensorType.IMU: {
                "imu0": KalmanFilterSensorConfig(measurement_noise_matrix=R)
            }
        }

    return KalmanFilterConfig(
        state_vector=state_vector,
        uncertainty_matrix=P,
        process_noise_matrix=Q,
        sensors=sensors,
        time_step_initial=0.05,
    )


def make_kfi(
    *, sensor_type: KalmanFilterSensorType, sensor_id: str
) -> KalmanFilterInput:
    return KalmanFilterInput(
        input=ProcessedData(data=np.array([0.0, 0.0, 0.0, 0.0])),
        sensor_id=sensor_id,
        sensor_type=sensor_type,
    )


def test_get_R_sensors_mapping_contains_sensor_and_id():
    ekf = ExtendedKalmanFilterStrategy(make_cfg(), fake_dt=0.1)
    assert KalmanFilterSensorType.IMU in ekf.R_sensors
    assert "imu0" in ekf.R_sensors[KalmanFilterSensorType.IMU]
    assert ekf.R_sensors[KalmanFilterSensorType.IMU]["imu0"].shape == (4, 4)


def test_insert_data_warns_and_skips_unknown_sensor_type():
    ekf = ExtendedKalmanFilterStrategy(make_cfg(), fake_dt=0.1)
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        ekf.insert_data(
            make_kfi(sensor_type=KalmanFilterSensorType.ODOMETRY, sensor_id="odom")
        )
        assert any("Sensor type" in str(x.message) for x in w)


def test_insert_data_warns_and_skips_unknown_sensor_id():
    ekf = ExtendedKalmanFilterStrategy(make_cfg(), fake_dt=0.1)
    with warnings.catch_warnings(record=True) as w:
        warnings.simplefilter("always")
        ekf.insert_data(
            make_kfi(sensor_type=KalmanFilterSensorType.IMU, sensor_id="missing")
        )
        assert any("Sensor id" in str(x.message) for x in w)


def test_prediction_step_clamps_dt_when_negative_or_too_large(monkeypatch):
    ekf = ExtendedKalmanFilterStrategy(make_cfg(), fake_dt=None)

    # Too large dt
    ekf.last_update_time = 0.0
    monkeypatch.setattr("time.time", lambda: 10_000.0)
    ekf.prediction_step()
    assert ekf.F[0, 2] == pytest.approx(0.05)
    assert ekf.F[1, 3] == pytest.approx(0.05)

    # Negative dt
    ekf.last_update_time = 10_000.0
    monkeypatch.setattr("time.time", lambda: 9_999.0)
    ekf.prediction_step()
    assert ekf.F[0, 2] == pytest.approx(0.05)
    assert ekf.F[1, 3] == pytest.approx(0.05)


def test_set_delta_t_sets_velocity_and_rotation_entries():
    ekf = ExtendedKalmanFilterStrategy(make_cfg(), fake_dt=0.1)
    ekf._debug_set_state(np.array([0.0, 0.0, 0.0, 0.0, 0.6, 0.2]))
    ekf.set_delta_t(0.2)
    assert ekf.F[0, 2] == pytest.approx(0.2)
    assert ekf.F[1, 3] == pytest.approx(0.2)
    assert ekf.F[4, 5] == pytest.approx(0.2)


def test_get_confidence_returns_zero_for_nan_or_inf_covariance():
    ekf = ExtendedKalmanFilterStrategy(make_cfg(), fake_dt=0.1)
    ekf.P = np.eye(6)
    ekf.P[0, 0] = np.nan
    assert ekf.get_confidence() == 0.0

    ekf.P = np.eye(6)
    ekf.P[2, 2] = np.inf
    assert ekf.get_confidence() == 0.0


@pytest.mark.xfail(reason="add_to_diagonal is currently unimplemented (pass)")
def test_add_to_diagonal_adds_value_to_diagonal_entries():
    m = np.zeros((3, 3), dtype=float)
    _add_to_diagonal(m, 2.5)
    assert np.allclose(np.diag(m), np.array([2.5, 2.5, 2.5]))


def test_get_state_future_predicts_from_current_filter_time():
    ekf = ExtendedKalmanFilterStrategy(make_cfg(), fake_dt=1.0)
    ekf._debug_set_state(np.array([0.0, 0.0, 2.0, 0.0, 0.0, 0.0]))

    projected = ekf.get_state(future_s=2.0)

    # get_state() projects +2s from current state (x=4), then advances filter by fake_dt (x=2)
    assert projected[0] == pytest.approx(4.0)
    assert ekf.x[0] == pytest.approx(2.0)


def test_get_state_future_rotates_direction_with_angular_velocity():
    ekf = ExtendedKalmanFilterStrategy(make_cfg(), fake_dt=0.0)
    ekf._debug_set_state(np.array([0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))

    projected = ekf.get_state(future_s=np.pi / 2)

    assert projected[4] == pytest.approx(np.pi / 2, abs=1e-6)
    assert projected[5] == pytest.approx(1.0, abs=1e-6)
