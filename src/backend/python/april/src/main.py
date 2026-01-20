import argparse
import asyncio
import random
import time

import pyapriltags

from backend.python.april.src.detection_camera import DetectionCamera
from backend.python.april.src.util import build_detector
from backend.python.common.camera.abstract_camera import get_camera_capture_device
from backend.python.common.debug.logger import LogLevel, init_logging, success
from backend.generated.thrift.config.apriltag.ttypes import AprilDetectionConfig
from backend.generated.thrift.config.camera.ttypes import CameraParameters, CameraType
from autobahn_client.client import Autobahn
from autobahn_client.util import Address
from backend.python.common.config import from_uncertainty_config
from backend.python.common.util.math import get_np_from_matrix, get_np_from_vector
from backend.python.common.util.system import (
    get_config_parser,
    get_system_name,
    load_basic_system_config,
    load_configs,
)


async def main():
    basic_system_config, config = load_configs()
    autobahn_server = Autobahn(
        Address(
            basic_system_config.autobahn.host,
            basic_system_config.autobahn.port,
        )
    )
    await autobahn_server.begin()

    camera_detector_list: list[DetectionCamera] = []
    init_logging(
        "APRIL_SERVER",
        LogLevel(basic_system_config.logging.global_logging_level),
        system_pub_topic=basic_system_config.logging.global_log_pub_topic,
        autobahn=autobahn_server,
        system_name=get_system_name(),
    )

    loop = asyncio.get_running_loop()

    def publish_nowait(topic: str, data: bytes):
        _ = asyncio.run_coroutine_threadsafe(autobahn_server.publish(topic, data), loop)

    success("Starting APRIL server")
    for camera in config.cameras:
        if camera.pi_to_run_on != get_system_name():
            continue

        success(f"Starting camera: {camera.name}")

        send_feed = camera.video_options.send_feed
        publication_topic = camera.video_options.publication_topic
        post_tag_output_topic = config.april_detection.post_tag_output_topic
        video_capture = get_camera_capture_device(camera)
        detector = build_detector(config.april_detection, video_capture)

        detector_cam = DetectionCamera(
            name=camera.name,
            video_capture=video_capture,
            tag_size=config.april_detection.tag_size,
            detector=detector,
            publication_lambda=lambda tags, post_tag_output_topic=post_tag_output_topic: (
                publish_nowait(
                    post_tag_output_topic,
                    tags,
                )
                if post_tag_output_topic
                else None
            ),
            publication_image_lambda=lambda message, send_feed=send_feed, publication_topic=publication_topic: (
                publish_nowait(
                    publication_topic,
                    message,
                )
                if send_feed == True and publication_topic != None
                else None
            ),
            do_compression=camera.video_options.do_compression or True,
            compression_quality=camera.video_options.compression_quality or 90,
            overlay_tags=camera.video_options.overlay_tags,
        )

        camera_detector_list.append(detector_cam)

        detector_cam.start()

    _ = await asyncio.Event().wait()


def cli_main():
    asyncio.run(main())


if __name__ == "__main__":
    asyncio.run(main())
