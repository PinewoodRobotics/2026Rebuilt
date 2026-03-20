from __future__ import annotations

from typing import TYPE_CHECKING, Callable, Literal, TypeAlias

from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType

AllowedSensors: TypeAlias = Literal[
    KalmanFilterSensorType.APRIL_TAG,
    KalmanFilterSensorType.ODOMETRY,
    KalmanFilterSensorType.IMU,
]

if TYPE_CHECKING:
    from backend.python.pos_extrapolator.position_solver_2d import (
        PositionSolver2d,
        SensorEvent,
    )

ProcessorFunc: TypeAlias = Callable[["PositionSolver2d", "SensorEvent"], None]

_PROCESSORS: dict[AllowedSensors, ProcessorFunc] = {}


def processor_for_data(
    sensor_type: AllowedSensors,
) -> Callable[[ProcessorFunc], ProcessorFunc]:
    def decorator(func: ProcessorFunc) -> ProcessorFunc:
        if sensor_type in _PROCESSORS:
            raise ValueError(f"Duplicate processor for sensor type {sensor_type}")
        _PROCESSORS[sensor_type] = func
        return func

    return decorator


def get_processor(sensor_type: AllowedSensors) -> ProcessorFunc | None:
    return _PROCESSORS.get(sensor_type)
