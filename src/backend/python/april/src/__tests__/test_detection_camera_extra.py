import numpy as np
import pytest

from backend.generated.proto.python.sensor.apriltags_pb2 import ProcessedTag
from backend.generated.proto.python.sensor.general_sensor_data_pb2 import (
    GeneralSensorData,
    SensorName,
)
from backend.python.april.src import detection_camera as detection_camera_module
from backend.python.april.src.detection_camera import DetectionCamera
from backend.python.april.src.tag_detector import TagDetection, TagDetector


class _DummyCapture:
    def get_matrix(self):
        return np.eye(3, dtype=np.float64)

    def get_dist_coeff(self):
        return np.zeros(5, dtype=np.float64)

    def release(self):
        return None


class _DummyDetector(TagDetector):
    def __init__(self):
        super().__init__(detector=None, detector_type="dummy", processing_function=lambda _d, _f: [])


def test_overlay_tag_on_frame_handles_gray_and_bgr_inputs():
    dc = DetectionCamera(
        name="cam0",
        video_capture=_DummyCapture(),
        tag_size=0.17,
        detector=_DummyDetector(),
        publication_lambda=None,
        publication_image_lambda=None,
        overlay_tags=True,
    )

    det = TagDetection(
        corners=np.array([[0, 0], [10, 0], [10, 10], [0, 10]], dtype=np.int32),
        tag_id=1,
        hamming=0,
        decision_margin=1.0,
        homography=np.eye(3),
        center=np.array([5.0, 5.0]),
    )

    gray = np.zeros((20, 20), dtype=np.uint8)
    out_gray = dc._overlay_tag_on_frame(gray, [det])
    assert out_gray.shape == (20, 20, 3)

    gray_1ch = np.zeros((20, 20, 1), dtype=np.uint8)
    out_gray_1ch = dc._overlay_tag_on_frame(gray_1ch, [det])
    assert out_gray_1ch.shape == (20, 20, 3)

    bgr = np.zeros((20, 20, 3), dtype=np.uint8)
    out_bgr = dc._overlay_tag_on_frame(bgr, [det])
    assert out_bgr.shape == (20, 20, 3)


def test_publish_emits_tag_data_only_when_tags_present():
    published: list[bytes] = []

    def pub(b: bytes) -> None:
        published.append(b)

    dc = DetectionCamera(
        name="cam0",
        video_capture=_DummyCapture(),
        tag_size=0.17,
        detector=_DummyDetector(),
        publication_lambda=pub,
        publication_image_lambda=None,
        overlay_tags=False,
    )

    frame = np.zeros((10, 10), dtype=np.uint8)
    dc._publish(frame, found_tags=[], processing_time=0.001)
    assert published == []

    dc._publish(frame, found_tags=[ProcessedTag(id=6)], processing_time=0.001)
    assert len(published) == 1
    msg = GeneralSensorData()
    msg.ParseFromString(published[0])
    assert msg.sensor_name == SensorName.APRIL_TAGS
    assert msg.sensor_id == "cam0"
    assert len(msg.apriltags.world_tags.tags) == 1
    assert msg.apriltags.world_tags.tags[0].id == 6


def test_publish_emits_image_data_when_image_publisher_present():
    published_images: list[bytes] = []

    def pub_img(b: bytes) -> None:
        published_images.append(b)

    dc = DetectionCamera(
        name="cam0",
        video_capture=_DummyCapture(),
        tag_size=0.17,
        detector=_DummyDetector(),
        publication_lambda=None,
        publication_image_lambda=pub_img,
        overlay_tags=False,
    )

    frame = np.zeros((10, 10), dtype=np.uint8)
    dc._publish(frame, found_tags=[], processing_time=0.001, do_compression=False)
    assert len(published_images) == 1
    msg = GeneralSensorData()
    msg.ParseFromString(published_images[0])
    assert msg.sensor_name == SensorName.CAMERA
    assert msg.sensor_id == "cam0"
    assert msg.image.width == 10
    assert msg.image.height == 10


def test_process_tags_filters_out_detections_near_frame_edge(monkeypatch):
    inside_detection = TagDetection(
        corners=np.array([[15, 15], [30, 15], [30, 30], [15, 30]], dtype=np.int32),
        tag_id=1,
        hamming=0,
        decision_margin=1.0,
        homography=np.eye(3),
        center=np.array([22.5, 22.5]),
    )
    edge_detection = TagDetection(
        corners=np.array([[5, 15], [20, 15], [20, 30], [5, 30]], dtype=np.int32),
        tag_id=2,
        hamming=0,
        decision_margin=1.0,
        homography=np.eye(3),
        center=np.array([12.5, 22.5]),
    )
    seen_ids: list[int] = []

    def fake_post_process_detection(detections, *_args):
        seen_ids.extend(det.tag_id for det in detections)
        return []

    monkeypatch.setattr(
        detection_camera_module,
        "process_image",
        lambda _frame, _detector: [inside_detection, edge_detection],
    )
    monkeypatch.setattr(
        detection_camera_module,
        "post_process_detection",
        fake_post_process_detection,
    )

    dc = DetectionCamera(
        name="cam0",
        video_capture=_DummyCapture(),
        tag_size=0.17,
        detector=_DummyDetector(),
        publication_lambda=None,
        publication_image_lambda=None,
        overlay_tags=False,
        image_edge_reject_margin_percent=10.0,
    )

    tags, filtered_detections = dc._process_tags(np.zeros((100, 100, 3), dtype=np.uint8))
    assert tags == []
    assert [det.tag_id for det in filtered_detections] == [1]
    assert seen_ids == [1]


def test_detection_camera_rejects_invalid_image_edge_margin():
    with pytest.raises(ValueError):
        DetectionCamera(
            name="cam0",
            video_capture=_DummyCapture(),
            tag_size=0.17,
            detector=_DummyDetector(),
            publication_lambda=None,
            publication_image_lambda=None,
            image_edge_reject_margin_percent=50.0,
        )
