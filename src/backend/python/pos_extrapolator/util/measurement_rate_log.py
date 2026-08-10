from __future__ import annotations

import time

from backend.python.common.debug.logger import info

_DEFAULT_INTERVAL_S = 2
_state: dict[str, tuple[float | None, int]] = {}


def _log_measurement_hz(stream_id: str, label: str, interval_s: float) -> None:
    now_s = time.monotonic()
    window_start_s, count = _state.get(stream_id, (None, 0))
    if window_start_s is None:
        window_start_s = now_s
    count += 1
    elapsed_s = now_s - window_start_s
    if elapsed_s >= interval_s:
        hz = count / elapsed_s if elapsed_s > 0.0 else 0.0
        info(f"{label}: {hz:.1f} Hz (over {elapsed_s * 1000:.0f} ms)")
        _state[stream_id] = (now_s, 0)
    else:
        _state[stream_id] = (window_start_s, count)


def log_odometry_measurement_hz(interval_s: float = _DEFAULT_INTERVAL_S) -> None:
    _log_measurement_hz("odometry", "Odometry measurement rate", interval_s)


def log_apriltags_measurement_hz(interval_s: float = _DEFAULT_INTERVAL_S) -> None:
    _log_measurement_hz("apriltags", "Apriltags measurement rate", interval_s)


def log_imu_measurement_hz(
    sensor_id: str, interval_s: float = _DEFAULT_INTERVAL_S
) -> None:
    _log_measurement_hz(
        f"imu.{sensor_id}",
        f"IMU ({sensor_id}) measurement rate",
        interval_s,
    )
