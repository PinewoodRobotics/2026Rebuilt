import asyncio
import ntcore

from autobahn_client import Address, Autobahn

from backend.python.common.util.extension import subscribe_to_multiple_topics
from backend.python.common.util.system import load_configs

from backend.generated.proto.python.sensor.camera_sensor_pb2 import ImageData
from backend.generated.proto.python.sensor.general_sensor_data_pb2 import (
    GeneralSensorData,
)


async def main():
    basic_system_config, config = load_configs()
    autobahn_server = Autobahn(
        Address(
            basic_system_config.autobahn.host,
            basic_system_config.autobahn.port,
        )
    )

    image_subscribe_topics: list[str] = [
        camera.video_options.publication_topic
        for camera in config.cameras
        if camera.video_options.send_feed
        and camera.video_options.publication_topic is not None
    ]

    inst = ntcore.NetworkTableInstance.getDefault()
    table = inst.getTable("retranslator")

    async def process_data(message: bytes):
        data = GeneralSensorData.FromString(message)
        one_of_name = data.WhichOneof("data")
        table_topic = table.getTopic(
            str(data.sensor_name) + "/" + str(data.sensor_id)
        ).genericPublish(str(data.__getattribute__(one_of_name).DESCRIPTOR.name))
        table_topic.set(data.__getattribute__(one_of_name).SerializeToString())

    await subscribe_to_multiple_topics(
        autobahn_server,
        image_subscribe_topics,
        process_data,
    )

    await autobahn_server.begin()


if __name__ == "__main__":
    asyncio.run(main())
