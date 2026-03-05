import math
from typing import TYPE_CHECKING
import numpy as np
from numpy.typing import NDArray
from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType
from backend.generated.thrift.config.pos_extrapolator.ttypes import (
    OdomConfig,
    OdometryPositionSource,
)
from backend.python.pos_extrapolator.data_prep import (
    ConfigProvider,
    DataPreparer,
    DataPreparerManager,
    ExtrapolationContext,
    KalmanFilterInput,
)
from backend.python.pos_extrapolator.filters.extended_kalman_filter import T_EKF


# from typing import override


class OdomDataPreparerConfig(ConfigProvider[OdomConfig]):
    pass


@DataPreparerManager.register(proto_type=OdometryData)
class OdomDataPreparer(DataPreparer[OdometryData, OdomDataPreparerConfig]):
    def __init__(self, config: OdomDataPreparerConfig):
        super().__init__(config)
        self.config = config.get_config()
        self.use_position = (
            self.config.position_source != OdometryPositionSource.DONT_USE
        )

    def calc_next_absolute_position(
        self,
        x: NDArray[np.float64],
        x_change: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        next_pos = x.copy()
        next_pos[0] += x_change[0]
        next_pos[1] += x_change[1]
        return next_pos

    # @override
    def get_data_type(self) -> type[OdometryData]:
        return OdometryData

    # @override
    def _used_indices(self) -> list[bool]:
        used_indices: list[bool] = []

        used_indices.extend([self.use_position] * 2)
        used_indices.extend([True] * 2)
        used_indices.extend([self.config.use_rotation])
        used_indices.extend([False])

        return used_indices

    # @override
    def _prepare(
        self,
        data: OdometryData,
        sensor_id: str,
        context: ExtrapolationContext | None = None,
    ) -> list[KalmanFilterInput] | KalmanFilterInput | None:
        assert context is not None

        values: list[float] = []

        if self.config.position_source == OdometryPositionSource.ABSOLUTE:
            values.append(data.position.position.x)
            values.append(data.position.position.y)
        elif self.config.position_source == OdometryPositionSource.ABS_CHANGE:
            next_position = self.calc_next_absolute_position(
                context.filter.get_state(),
                np.array([data.position_change.x, data.position_change.y]),
            )
            values.append(next_position[0])
            values.append(next_position[1])

        vel = context.filter.angle_matrix() @ np.array(
            [data.velocity.x, data.velocity.y]
        )

        values.append(vel[0])
        values.append(vel[1])

        if self.config.use_rotation:
            values.append(
                np.atan2(data.position.direction.y, data.position.direction.x)
            )

        if values[0] > 1000000 or values[1] > 1000000:
            return None

        return KalmanFilterInput(
            input=np.array(values),
            sensor_id=sensor_id,
            sensor_type=KalmanFilterSensorType.ODOMETRY,
            jacobian_h=T_EKF.generic_jacobian_h(self._used_indices()),
            hx=T_EKF.generic_hx(self._used_indices()),
        )
