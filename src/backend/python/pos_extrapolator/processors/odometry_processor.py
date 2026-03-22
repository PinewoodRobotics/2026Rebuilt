from __future__ import annotations

from typing import TYPE_CHECKING, cast

import numpy as np
from numpy.typing import NDArray

from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterSensorType,
)
from backend.python.pos_extrapolator.processor_registry import processor_for_data
from backend.python.pos_extrapolator.util.measurement_rate_log import (
    log_odometry_measurement_hz,
)
from backend.python.pos_extrapolator.util.extrapolator_math import wrap_to_pi

if TYPE_CHECKING:
    from backend.python.pos_extrapolator.position_solver_2d import PositionSolver2d
    from backend.python.pos_extrapolator.util.solver_models import SensorEvent


def _predict_odometry(
    state: NDArray[np.float64],
    control: NDArray[np.float64],
    dt_s: float,
    robot_translation: NDArray[np.float64],
) -> NDArray[np.float64]:
    if dt_s <= 0.0:
        return state.copy()

    theta = float(state[2])
    omega = float(control[2])
    theta_mid = theta + 0.5 * omega * dt_s
    sin_theta = float(np.sin(theta_mid))
    cos_theta = float(np.cos(theta_mid))

    next_state = state.copy()
    next_state[0] += cos_theta * float(robot_translation[0]) - sin_theta * float(
        robot_translation[1]
    )
    next_state[1] += sin_theta * float(robot_translation[0]) + cos_theta * float(
        robot_translation[1]
    )

    next_state[2] = wrap_to_pi(theta + omega * dt_s)
    return next_state


@processor_for_data(KalmanFilterSensorType.ODOMETRY)
def process_odometry(solver: "PositionSolver2d", event: "SensorEvent") -> None:
    data = cast(OdometryData, event.data)

    log_odometry_measurement_hz()

    solver.current_control.vx_robot = float(data.velocity.x)
    solver.current_control.vy_robot = float(data.velocity.y)
    solver.current_control.omega = float(data.omega)

    event_dt_s = solver.get_dt_s(event.timestamp_s)
    odometry_dt_s = float(data.time_change_s)

    solver.nonlinear_predict(
        odometry_dt_s if odometry_dt_s > 0.0 else event_dt_s,
        solver.current_control,
        innovation_function=_predict_odometry,
        innovation_args=(
            np.array(
                [data.position_change.x, data.position_change.y], dtype=np.float64
            ),
        ),
    )
