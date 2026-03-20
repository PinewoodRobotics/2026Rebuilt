from __future__ import annotations

from typing import TYPE_CHECKING, cast

import numpy as np

from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterSensorType,
)
from backend.python.pos_extrapolator.processor_registry import processor_for_data

if TYPE_CHECKING:
    from backend.python.pos_extrapolator.position_solver_2d import (
        PositionSolver2d,
        SensorEvent,
    )


@processor_for_data(KalmanFilterSensorType.ODOMETRY)
def process_odometry(solver: "PositionSolver2d", event: "SensorEvent") -> None:
    data = cast(OdometryData, event.data)

    dt_s = float(data.time_change_s) if float(data.time_change_s) > 0.0 else None
    vx_robot = float(data.velocity.x)
    vy_robot = float(data.velocity.y)
    if dt_s is not None and abs(vx_robot) < 1e-9 and abs(vy_robot) < 1e-9:
        vx_robot = float(data.position_change.x) / dt_s
        vy_robot = float(data.position_change.y) / dt_s

    solver.current_control.vx_robot = vx_robot
    solver.current_control.vy_robot = vy_robot
    if np.isfinite(float(data.omega)):
        solver.current_control.omega = float(data.omega)

    solver.predict_to_timestamp(event.timestamp_s, solver.current_control)
