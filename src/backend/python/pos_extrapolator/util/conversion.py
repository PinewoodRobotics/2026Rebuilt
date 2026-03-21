import numpy as np
from numpy.typing import NDArray
from typing import cast

from backend.generated.thrift.config.common.ttypes import GenericVector, GenericMatrix
from backend.python.common.util.math import get_np_from_vector, get_np_from_matrix
from backend.generated.thrift.config.kalman_filter.ttypes import (
    KalmanFilterConfig,
    KalmanFilterSensorType,
)


def load_vector(vector: GenericVector, expected_size: int) -> NDArray[np.float64]:
    result = np.asarray(get_np_from_vector(vector), dtype=np.float64).reshape(-1)
    if result.shape != (expected_size,):
        raise ValueError(
            f"Expected vector with shape {(expected_size,)}, got {result.shape}"
        )
    return result


def load_matrix(matrix: GenericMatrix, size: int) -> NDArray[np.float64]:
    result = np.asarray(get_np_from_matrix(matrix), dtype=np.float64)
    if result.shape != (size, size):
        raise ValueError(
            f"Expected matrix with shape {(size, size)}, got {result.shape}"
        )
    return result


def get_R_sensors(
    config: KalmanFilterConfig,
) -> dict[KalmanFilterSensorType, dict[str, NDArray[np.float64]]]:
    output: dict[KalmanFilterSensorType, dict[str, NDArray[np.float64]]] = {}
    for sensor_type, sensors in config.sensors.items():
        output[sensor_type] = {}
        for sensor_id, sensor_config in sensors.items():
            output[sensor_type][sensor_id] = np.asarray(
                get_np_from_matrix(sensor_config.measurement_noise_matrix),
                dtype=np.float64,
            )
    return output
