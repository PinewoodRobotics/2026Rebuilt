import asyncio

from backend.python.common.camera.abstract_camera import get_camera_capture_device
from backend.python.common.debug.logger import LogLevel, init_logging, success
from autobahn_client.client import Autobahn
from autobahn_client.util import Address
from backend.python.common.util.system import (
    get_system_name,
    load_configs,
)
from backend.python.image_recognition.object_detector import ObjectDetector


async def main():
    basic_system_config, config = load_configs()
    autobahn_server = Autobahn(
        Address(
            basic_system_config.autobahn.host,
            basic_system_config.autobahn.port,
        )
    )
    await autobahn_server.begin()

    init_logging(
        "IMAGE_RECOGNITION_SERVER",
        LogLevel(basic_system_config.logging.global_logging_level),
        system_pub_topic=basic_system_config.logging.global_log_pub_topic,
        autobahn=autobahn_server,
        system_name=get_system_name(),
    )

    loop = asyncio.get_running_loop()

    def publish_nowait(topic: str, data: bytes):
        _ = asyncio.run_coroutine_threadsafe(autobahn_server.publish(topic, data), loop)

    success("Starting Image Recognition server")
    for camera in config.cameras:
        if (
            camera.pi_to_run_on != get_system_name()
            or camera.name not in config.object_recognition.cameras_to_use
        ):
            continue

        success(f"Starting camera: {camera.name}")
        object_detector = ObjectDetector(
            config.object_recognition,
            get_camera_capture_device(camera),
            lambda inference_list: publish_nowait(
                config.object_recognition.output_topic, inference_list
            ),
        )

        object_detector.start()

    _ = await asyncio.Event().wait()


def cli_main():
    asyncio.run(main())


if __name__ == "__main__":
    asyncio.run(main())
