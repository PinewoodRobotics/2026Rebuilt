from dataclasses import dataclass
from typing import Any, Callable
from typing_extensions import cast

from cv2.typing import MatLike
from backend.generated.thrift.config.apriltag.ttypes import (
    AprilDetectionConfig,
    SpecialDetectorConfig,
)
import pyapriltags
from numpy.typing import NDArray
import numpy as np
from backend.python.common.util.system import setup_shared_library_python_extension


@dataclass
class TagDetection:
    corners: NDArray[np.int32]
    tag_id: int
    hamming: int
    decision_margin: float
    homography: NDArray[np.float64]
    center: NDArray[np.float64]


class TagDetector:
    def __init__(
        self,
        detector: Any,
        detector_type: str,
        processing_function: Callable[
            [Any, NDArray[np.uint8] | MatLike], list[TagDetection]
        ],
    ):
        self.detector = detector
        self.detector_type = detector_type
        self.processing_function: Callable[
            [Any, NDArray[np.uint8] | MatLike], list[TagDetection]
        ] = processing_function

    @classmethod
    def use_cpu(
        cls,
        config: AprilDetectionConfig,
    ) -> "TagDetector":
        def processing_function(
            detector: pyapriltags.Detector, frame: NDArray[np.uint8] | MatLike
        ) -> list[TagDetection]:
            detections = detector.detect(frame)

            return [
                TagDetection(
                    corners=detection.corners.copy(),
                    tag_id=detection.tag_id,
                    hamming=detection.hamming,
                    decision_margin=detection.decision_margin,
                    homography=np.array(detection.homography).copy(),
                    center=np.array(detection.center).copy(),
                )
                for detection in detections
            ]

        return cls(
            pyapriltags.Detector(
                families=str(config.family),
                nthreads=config.nthreads,
                quad_decimate=config.quad_decimate,
                quad_sigma=config.quad_sigma,
                refine_edges=config.refine_edges,
                decode_sharpening=config.decode_sharpening,
            ),
            "cpu_generic",
            processing_function,
        )

    @classmethod
    def use_cuda_tags(
        cls,
        config: AprilDetectionConfig,
        special_detector_config: SpecialDetectorConfig,
        width: int,
        height: int,
        dist_coeffs: NDArray[np.float64],
        camera_matrix: NDArray[np.float64],
    ) -> "TagDetector":
        cuda_tags = setup_shared_library_python_extension(
            module_name="cuda_tags",
            py_lib_searchpath=special_detector_config.py_lib_searchpath,
            module_basename="cuda_tags",
        )

        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]

        camera_matrix = cuda_tags.CameraMatrix(fx, cx, fy, cy)
        dist_coeffs = cuda_tags.DistCoeffs(
            dist_coeffs[0],
            dist_coeffs[1],
            dist_coeffs[2],
            dist_coeffs[3],
            dist_coeffs[4],
        )

        tags_wrapper = cuda_tags.CudaTagsWrapper(
            cuda_tags.TagType.tag36h11,
            camera_matrix,
            dist_coeffs,
            config.nthreads,
            width,
            height,
        )

        def processing_function(
            detector: Any, frame: NDArray[np.uint8] | MatLike
        ) -> list[TagDetection]:
            detections = detector.process(frame)
            return [
                TagDetection(
                    corners=np.array(detection.corners, dtype=np.int32),
                    tag_id=detection.id,
                    hamming=detection.hamming,
                    decision_margin=detection.decision_margin,
                    homography=np.array(detection.homography),
                    center=np.array(detection.center),
                )
                for detection in detections
            ]

        return cls(tags_wrapper, "cuda_tags", processing_function)

    def detect(self, frame: NDArray[np.uint8] | MatLike) -> list[TagDetection]:
        return self.processing_function(self.detector, frame)
