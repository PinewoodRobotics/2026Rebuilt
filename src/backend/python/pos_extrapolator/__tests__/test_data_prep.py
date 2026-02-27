import numpy as np
from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    ImuConfig,
    OdomConfig,
    OdometryPositionSource,
)
from backend.python.pos_extrapolator.data_prep import DataPreparerManager, ExtrapolationContext
from backend.python.pos_extrapolator.preparers.ImuDataPreparer import ImuDataPreparerConfig
from backend.python.pos_extrapolator.preparers.OdomDataPreparer import OdomDataPreparerConfig


class _FakeFilter:
    def __init__(self, angle_rad: float = 0.0):
        self.x = np.array([0.0, 0.0, 0.0, 0.0, angle_rad, 0.0], dtype=np.float64)

    def angle_matrix(self) -> np.ndarray:
        c = float(np.cos(self.x[4]))
        s = float(np.sin(self.x[4]))
        return np.array([[c, -s], [s, c]], dtype=np.float64)

    def get_state(self) -> np.ndarray:
        return self.x.copy()


def sample_imu_data():
    imu_data = ImuData()
    imu_data.position.position.x = 1
    imu_data.position.position.y = 2
    imu_data.position.position.z = 3
    imu_data.position.direction.x = 0.5
    imu_data.position.direction.y = 0.5
    imu_data.position.direction.z = 4
    imu_data.acceleration.x = 7
    imu_data.acceleration.y = 8
    imu_data.acceleration.z = 9
    imu_data.velocity.x = 10
    imu_data.velocity.y = 11
    imu_data.velocity.z = 12
    return imu_data


def sample_odometry_data():
    odometry_data = OdometryData()
    odometry_data.position.position.x = 13
    odometry_data.position.position.y = 14
    odometry_data.position.direction.x = 0.7
    odometry_data.position.direction.y = 0.7
    odometry_data.velocity.x = 15
    odometry_data.velocity.y = 16
    odometry_data.position_change.x = 13
    odometry_data.position_change.y = 14
    return odometry_data


def test_data_prep():
    DataPreparerManager.set_config(
        ImuData,
        ImuDataPreparerConfig(
            {
                "0": ImuConfig(
                    use_rotation=True,
                    use_position=False,
                    use_velocity=True,
                )
            }
        ),
    )
    DataPreparerManager.set_config(
        OdometryData,
        OdomDataPreparerConfig(
            OdomConfig(
                position_source=OdometryPositionSource.ABSOLUTE, use_rotation=True
            )
        ),
    )

    data_preparer_manager = DataPreparerManager()
    imu_data = sample_imu_data()
    odometry_data = sample_odometry_data()

    context = ExtrapolationContext(filter=_FakeFilter(), has_gotten_rotation=False)

    imu_input = data_preparer_manager.prepare_data(imu_data, "0", context)
    odometry_input = data_preparer_manager.prepare_data(odometry_data, "odom", context)

    assert imu_input is not None and odometry_input is not None
    assert len(imu_input) == 1
    assert len(odometry_input) == 1

    imu_vals = imu_input[0].get_input()
    assert np.allclose(
        imu_vals,
        np.array(
            [
                imu_data.velocity.x,
                imu_data.velocity.y,
                np.arctan2(imu_data.position.direction.y, imu_data.position.direction.x),
                imu_data.angularVelocityXYZ.z,
            ]
        ),
    )
    assert imu_input[0].sensor_id == "0"
    assert imu_input[0].sensor_type == KalmanFilterSensorType.IMU

    odom_vals = odometry_input[0].get_input()
    assert np.allclose(
        odom_vals,
        np.array(
            [
                odometry_data.position.position.x,
                odometry_data.position.position.y,
                odometry_data.velocity.x,
                odometry_data.velocity.y,
                np.arctan2(
                    odometry_data.position.direction.y,
                    odometry_data.position.direction.x,
                ),
            ]
        ),
    )
    assert odometry_input[0].sensor_id == "odom"
    assert odometry_input[0].sensor_type == KalmanFilterSensorType.ODOMETRY


def test_get_config():
    preparer_manager = DataPreparerManager()
    DataPreparerManager.set_config(
        ImuData,
        ImuDataPreparerConfig(
            {
                "0": ImuConfig(
                    use_rotation=True,
                    use_position=False,
                    use_velocity=True,
                )
            }
        ),
    )

    imu = sample_imu_data()
    ctx = ExtrapolationContext(filter=_FakeFilter(), has_gotten_rotation=False)
    imu_input = preparer_manager.prepare_data(imu, "0", ctx)
    assert imu_input is not None

    DataPreparerManager.set_config(
        OdometryData,
        OdomDataPreparerConfig(
            OdomConfig(
                position_source=OdometryPositionSource.ABSOLUTE, use_rotation=True
            )
        ),
    )
    odom = sample_odometry_data()
    odom_input = preparer_manager.prepare_data(odom, "odom", ctx)
    assert odom_input is not None
