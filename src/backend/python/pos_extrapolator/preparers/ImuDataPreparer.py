# from typing import final, overload, override
import numpy as np
from numpy.typing import NDArray
from backend.python.common.util.math import (
    _transform_matrix_to_size,
    transform_vector_to_size,
)
from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType
from backend.generated.thrift.config.pos_extrapolator.ttypes import ImuConfig
from backend.python.pos_extrapolator.data_prep import (
    ConfigProvider,
    DataPreparer,
    DataPreparerManager,
    ExtrapolationContext,
    KalmanFilterInput,
)
from backend.python.pos_extrapolator.filters.extended_kalman_filter import T_EKF


class ImuDataPreparerConfig(ConfigProvider[dict[str, ImuConfig]]):
    pass


@DataPreparerManager.register(proto_type=ImuData)
class ImuDataPreparer(DataPreparer[ImuData, ImuDataPreparerConfig]):
    def __init__(self, config: ImuDataPreparerConfig):
        super().__init__(config)
        self.config: ImuDataPreparerConfig = config

    # @override
    def get_data_type(self) -> type[ImuData]:
        return ImuData

    # @override
    def _used_indices(self, sensor_id: str) -> list[bool]:
        used_indices: list[bool] = []
        used_indices.extend([self.config.config[sensor_id].use_position] * 2)
        used_indices.extend([self.config.config[sensor_id].use_velocity] * 2)
        used_indices.extend([self.config.config[sensor_id].use_rotation] * 2)
        return used_indices

    # @override
    def _prepare(
        self, data: ImuData, sensor_id: str, context: ExtrapolationContext | None = None
    ) -> list[KalmanFilterInput] | KalmanFilterInput | None:
        assert context is not None
        config = self.config.config[sensor_id]
        values: list[float] = []

        if config.use_position:
            values.append(data.position.position.x)
            values.append(data.position.position.y)
        if config.use_velocity:
            velocity = context.filter.angle_matrix() @ np.array(
                [data.velocity.x, data.velocity.y]
            )
            values.append(velocity[0])
            values.append(velocity[1])
        if config.use_rotation:
            values.append(
                np.atan2(data.position.direction.y, data.position.direction.x)
            )
            values.append(data.angularVelocityXYZ.z)

        return KalmanFilterInput(
            input=np.array(values),
            sensor_id=sensor_id,
            sensor_type=KalmanFilterSensorType.IMU,
            jacobian_h=T_EKF.generic_jacobian_h(self.get_used_indices(sensor_id)),
            hx=T_EKF.generic_hx(self.get_used_indices(sensor_id)),
        )
