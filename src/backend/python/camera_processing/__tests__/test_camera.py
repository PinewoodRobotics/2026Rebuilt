from numpy.typing import NDArray
from backend.python.common.camera.type_camera.OV2311_camera import OV2311Camera
from backend.python.common.util.system import SystemStatus, get_system_status
import numpy as np


def test_camera_open():
    if get_system_status() != SystemStatus.DEVELOPMENT:
        return  # hardware-only

    video_capture = OV2311Camera(
        camera_port="/dev/video0",
        width=800,
        height=600,
        max_fps=30,
        camera_name="test_camera",
        camera_matrix=np.eye(3),
        dist_coeff=np.zeros(5),
    )
    frames: list[NDArray[np.uint8]] = []
    for _ in range(10):
        ret, frame = video_capture.get_frame()
        assert ret and frame is not None
        frames.append(frame)

    assert len(frames) == 10

    video_capture.release()


def calculate_avg_pixel_value(frame: NDArray[np.uint8]) -> float:
    return float(np.mean(frame))


def test_camera_exposure_time():
    if get_system_status() != SystemStatus.DEVELOPMENT:
        return  # hardware-only

    video_capture = OV2311Camera(
        camera_port="/dev/video0",
        width=800,
        height=600,
        max_fps=30,
        camera_name="test_camera",
        camera_matrix=np.eye(3),
        dist_coeff=np.zeros(5),
        exposure_time=20,
    )
    frames: list[NDArray[np.uint8]] = []
    for _ in range(10):
        ret, frame = video_capture.get_frame()
        assert ret and frame is not None
        frames.append(frame)

    assert len(frames) == 10
    avg_pixel_values = np.array(
        [calculate_avg_pixel_value(frame) for frame in frames]
    ).mean()
    avg_pixel_values = float(avg_pixel_values)

    assert avg_pixel_values > 200

    video_capture.release()
