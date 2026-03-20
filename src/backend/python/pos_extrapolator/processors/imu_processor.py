from __future__ import annotations

from typing import TYPE_CHECKING, cast

from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterSensorType,
)
from backend.python.pos_extrapolator.processor_registry import processor_for_data

if TYPE_CHECKING:
    from backend.python.pos_extrapolator.position_solver_2d import (
        PositionSolver2d,
        SensorEvent,
    )


@processor_for_data(KalmanFilterSensorType.IMU)
def process_imu(solver: "PositionSolver2d", event: "SensorEvent") -> None:
    data = cast(ImuData, event.data)
    imu_config = solver.general_config.imu_config[event.sensor_id]

    if imu_config.use_velocity:
        solver.current_control.vx_robot = float(data.velocity.x)
        solver.current_control.vy_robot = float(data.velocity.y)

    solver.current_control.omega = float(data.angularVelocityXYZ.z)
    solver.predict_to_timestamp(event.timestamp_s, solver.current_control)
