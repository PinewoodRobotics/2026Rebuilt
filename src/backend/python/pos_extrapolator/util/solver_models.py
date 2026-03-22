from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray

from backend.generated.proto.python.sensor.apriltags_pb2 import AprilTagData
from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.python.pos_extrapolator.processor_registry import AllowedSensors

SensorPayload = AprilTagData | ImuData | OdometryData


@dataclass
class MotionInput:
    vx_robot: float
    vy_robot: float
    omega: float

    def as_vector(self) -> NDArray[np.float64]:
        return np.array(
            [self.vx_robot, self.vy_robot, self.omega],
            dtype=np.float64,
        )

    def __str__(self) -> str:
        return f"MotionInput(vx_robot={self.vx_robot}, vy_robot={self.vy_robot}, omega={self.omega})"


@dataclass(order=True)
class SensorEvent:
    timestamp_s: float
    sensor_type: AllowedSensors = field(compare=False)
    sensor_id: str = field(compare=False)
    data: SensorPayload = field(compare=False)
