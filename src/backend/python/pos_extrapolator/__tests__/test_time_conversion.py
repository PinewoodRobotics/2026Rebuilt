import pytest

from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType
from backend.python.pos_extrapolator.util.time_conversion import SensorsTimeConverter


def test_time_conversion_handles_sensor_clock_ahead_of_local_clock():
    converter = SensorsTimeConverter()

    t0 = converter.get_local_time_s(
        KalmanFilterSensorType.IMU,
        "imu0",
        20_500.0,
        local_reference_s=1_000.0,
    )
    t1 = converter.get_local_time_s(
        KalmanFilterSensorType.IMU,
        "imu0",
        20_650.0,
        local_reference_s=1_000.4,
    )

    assert t0 == pytest.approx(1_000.0)
    assert t1 == pytest.approx(1_000.15)


def test_time_conversion_handles_sensor_clock_behind_local_clock():
    converter = SensorsTimeConverter()

    t0 = converter.get_local_time_s(
        KalmanFilterSensorType.ODOMETRY,
        "odom",
        9_500.0,
        local_reference_s=1_000.0,
    )
    t1 = converter.get_local_time_s(
        KalmanFilterSensorType.ODOMETRY,
        "odom",
        9_700.0,
        local_reference_s=1_000.4,
    )

    assert t0 == pytest.approx(1_000.0)
    assert t1 == pytest.approx(1_000.2)
