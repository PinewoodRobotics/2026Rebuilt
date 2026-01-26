from typing import Callable
from numpy.typing import NDArray
import numpy as np
from backend.generated.thrift.camera_processor.ttypes import CameraPipelineType
from backend.generated.thrift.config.ttypes import Config
from backend.python.camera_processing.detection_camera import CameraPipeline
from backend.python.camera_processing.detection_camera import CameraProcessor


@CameraProcessor.register(CameraPipelineType.APRIL_TAGS)
class AprilTagsPipeline(CameraPipeline):
    def process(
        self,
        frame: NDArray[np.uint8],
        send_data_lambda: Callable[[bytes], None],
        config: Config,
    ):
        pass
