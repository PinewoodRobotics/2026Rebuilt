from __future__ import annotations

from dataclasses import dataclass
import time

from backend.generated.thrift.config.kalman_filter.ttypes import KalmanFilterSensorType


@dataclass
class SensorTime:
    sensor_reference_ms: float
    local_reference_s: float

    def convert_time_to_local_s(self, sensor_time_ms: float) -> float:
        return self.local_reference_s + (
            float(sensor_time_ms) - self.sensor_reference_ms
        ) / 1000.0


class SensorsTimeConverter:
    def __init__(self) -> None:
        self._sensors: dict[KalmanFilterSensorType, dict[str, SensorTime]] = {}

    def add_sensor(
        self,
        sensor_type: KalmanFilterSensorType,
        sensor_id: str,
        sensor_time_ms: float,
        *,
        local_reference_s: float | None = None,
    ) -> SensorTime:
        if local_reference_s is None:
            local_reference_s = time.time()

        sensor = SensorTime(
            sensor_reference_ms=float(sensor_time_ms),
            local_reference_s=float(local_reference_s),
        )
        self._sensors.setdefault(sensor_type, {})[sensor_id] = sensor
        return sensor

    def contains_sensor(
        self, sensor_type: KalmanFilterSensorType, sensor_id: str
    ) -> bool:
        return sensor_id in self._sensors.get(sensor_type, {})

    def get_sensor(
        self, sensor_type: KalmanFilterSensorType, sensor_id: str
    ) -> SensorTime:
        return self._sensors[sensor_type][sensor_id]

    def get_local_time_s(
        self,
        sensor_type: KalmanFilterSensorType,
        sensor_id: str,
        sensor_time_ms: float,
        *,
        local_reference_s: float | None = None,
    ) -> float:
        if not self.contains_sensor(sensor_type, sensor_id):
            self.add_sensor(
                sensor_type,
                sensor_id,
                sensor_time_ms,
                local_reference_s=local_reference_s,
            )

        return self.get_sensor(sensor_type, sensor_id).convert_time_to_local_s(
            sensor_time_ms
        )
