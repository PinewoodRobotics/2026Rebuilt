import asyncio

from backend.python.common.debug.logger import LogLevel, error, info, init_logging
from autobahn_client import Address, Autobahn

from backend.python.common.util.extension import subscribe_to_multiple_topics
from backend.python.common.util.system import (
    BasicSystemConfig,
    get_system_name,
    load_configs,
)

from backend.generated.proto.python.util.position_pb2 import RobotPosition
from backend.python.obj_pose_extrapolator.obj_processor import (
    ExtrapolationStrategy,
    ObjPositionProcessor,
)
from backend.generated.proto.python.sensor.general_sensor_data_pb2 import (
    GeneralSensorData,
)

current_robot_position: RobotPosition | None = None


def init_utilities(basic_system_config: BasicSystemConfig, autobahn_server: Autobahn):
    init_logging(
        "OBJ_POSE_EXTRAPOLATOR",
        LogLevel(basic_system_config.logging.global_logging_level),
        system_pub_topic=basic_system_config.logging.global_log_pub_topic,
        autobahn=autobahn_server,
        system_name=get_system_name(),
    )


async def main():
    system_config, config = load_configs()
    if config.obj_pose_extrapolator is None:
        error("ObjPoseExtrapolatorConfig is not set in the config")
        return

    autobahn_server = Autobahn(
        Address(system_config.autobahn.host, system_config.autobahn.port)
    )
    await autobahn_server.begin()

    init_utilities(system_config, autobahn_server)
    info(f"Starting Position Extrapolator...")

    async def process_pose_data(message: bytes):
        global current_robot_position
        current_robot_position = RobotPosition.FromString(message)

    await autobahn_server.subscribe(
        config.obj_pose_extrapolator.message_config.robot_global_position_topic,
        process_pose_data,
    )

    obj_processor = ObjPositionProcessor(
        config.obj_pose_extrapolator,
        cameras={camera.name: camera for camera in config.cameras},
    )

    async def process_data(message: bytes):
        data = GeneralSensorData.FromString(message)
        one_of_name = data.WhichOneof("data")
        if one_of_name != "inference":
            return

        try:
            obj_processor.insert_data(
                data.__getattribute__("inference"),
                data.sensor_id,
                extrapolation_strategy=ExtrapolationStrategy.AVG_SIZE,
                depth_estimate=None,
            )
        except:
            error(
                "Something went wrong when inserting data into Obj Position Processor"
            )

    await subscribe_to_multiple_topics(
        autobahn_server,
        config.obj_pose_extrapolator.message_config.objects_out_topic,
        process_data,
    )

    while True:
        await asyncio.sleep(config.obj_pose_extrapolator.update_rate_sec)
        obj_processor.tick()


if __name__ == "__main__":
    asyncio.run(main())
