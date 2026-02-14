import numpy as np

from backend.generated.thrift.config.apriltag.ttypes import AprilDetectionConfig
from backend.python.april.src.tag_detector import TagDetector


class _FakeDetection:
    def __init__(self):
        self.corners = np.array([[0, 0], [1, 0], [1, 1], [0, 1]], dtype=np.float64)
        self.tag_id = 6
        self.hamming = 0
        self.decision_margin = 42.0
        self.homography = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]
        self.center = [0.5, 0.5]


class _FakeDetector:
    def __init__(self, *args, **kwargs):
        self._args = args
        self._kwargs = kwargs

    def detect(self, _frame):
        return [_FakeDetection()]


def test_use_cpu_wraps_pyapriltags_detections(monkeypatch):
    import pyapriltags

    monkeypatch.setattr(pyapriltags, "Detector", _FakeDetector)

    cfg = AprilDetectionConfig(
        family="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=True,
        decode_sharpening=0.25,
        pi_name_to_special_detector_config={},
    )
    td = TagDetector.use_cpu(cfg)

    out = td.detect(np.zeros((10, 10), dtype=np.uint8))
    assert len(out) == 1
    det = out[0]
    assert det.tag_id == 6
    assert det.corners.shape == (4, 2)
    assert det.center.shape == (2,)
