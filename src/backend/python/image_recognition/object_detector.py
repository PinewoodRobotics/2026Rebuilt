from threading import Thread
import time
from typing import Callable

from ultralytics.engine.results import Results

from backend.generated.thrift.config.image_recognition.ttypes import (
    ObjectRecognitionConfig,
)
from backend.generated.proto.python.Inference_pb2 import InferenceList, RawInference
from backend.python.common.camera.abstract_camera import AbstractCaptureDevice
from ultralytics import YOLO

from backend.python.image_recognition.util import post_process_results, to_raw_inference


class ObjectDetector(Thread):
    def __init__(
        self,
        config: ObjectRecognitionConfig,
        capture_device: AbstractCaptureDevice,
        on_detection: Callable[[bytes], None],
    ):
        super().__init__()
        self.config: ObjectRecognitionConfig = config
        self.capture_device: AbstractCaptureDevice = capture_device
        self.on_detection: Callable[[bytes], None] = on_detection
        self.model: YOLO = YOLO(self.config.model_path).to(self.config.device)

    def run(self):
        while True:
            ret, frame = self.capture_device.get_frame()
            if not ret or frame is None:
                continue

            start_time = time.time()
            results: list[Results] = self.model(
                frame, conf=self.config.conf_threshold, iou=self.config.iou_threshold
            )
            processing_time = time.time() - start_time
            inference_list: InferenceList = self._process_results(
                results, processing_time
            )
            self.on_detection(inference_list.SerializeToString())

    def _process_results(
        self, results: list[Results], processing_time: float
    ) -> InferenceList:
        raw_inferences: list[RawInference] = [
            inference for result in results for inference in to_raw_inference(result)
        ]

        processed_inferences = post_process_results(
            raw_inferences,
            self.capture_device.get_matrix(),
            self.capture_device.get_dist_coeff(),
            self.config.objects_to_detect[0].x_size_meters,
            self.config.objects_to_detect[0].y_size_meters,
        )

        return InferenceList(
            camera_name=self.capture_device.get_name(),
            raw_inferences=raw_inferences,
            processed_inferences=processed_inferences,
            processing_time=float(processing_time),
        )
