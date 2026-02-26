from autobahn_client.client import Autobahn
from autobahn_client.util import Address
from backend.python.common.debug.logger import LogLevel, init_logging
from backend.python.common.util.system import (
    BasicSystemConfig,
    get_system_name,
    load_configs,
)
from backend.python.pos_extrapolator.data_prep import DataPreparerManager
from backend.generated.proto.python.sensor.imu_pb2 import ImuData
from backend.generated.proto.python.sensor.odometry_pb2 import OdometryData
from backend.generated.proto.python.sensor.apriltags_pb2 import AprilTagData
from backend.generated.thrift.config.ttypes import Config
from backend.python.pos_extrapolator.preparers.ImuDataPreparer import (
    ImuDataPreparerConfig,
)
from backend.python.pos_extrapolator.preparers.OdomDataPreparer import (
    OdomDataPreparerConfig,
)
from backend.python.pos_extrapolator.preparers.AprilTagPreparer import (
    AprilTagDataPreparerConfig,
    AprilTagPreparerConfig,
)


def _init_utilities(basic_system_config: BasicSystemConfig) -> Autobahn:
    server = Autobahn(
        Address(basic_system_config.autobahn.host, basic_system_config.autobahn.port)
    )

    init_logging(
        "POSE_EXTRAPOLATOR",
        LogLevel(basic_system_config.logging.global_logging_level),
        system_pub_topic=basic_system_config.logging.global_log_pub_topic,
        autobahn=server,
        system_name=get_system_name(),
    )

    return server


def _init_data_preparer_manager(config: Config):
    if config.pos_extrapolator.enable_imu:
        DataPreparerManager.set_config(
            ImuData, ImuDataPreparerConfig(config.pos_extrapolator.imu_config)
        )

    if config.pos_extrapolator.enable_odom:
        DataPreparerManager.set_config(
            OdometryData, OdomDataPreparerConfig(config.pos_extrapolator.odom_config)
        )

    if config.pos_extrapolator.enable_tags:
        DataPreparerManager.set_config(
            AprilTagData,
            AprilTagDataPreparerConfig(
                AprilTagPreparerConfig(
                    tags_in_world=config.pos_extrapolator.april_tag_config.tag_position_config,
                    cameras_in_robot=config.pos_extrapolator.april_tag_config.camera_position_config,
                    use_imu_rotation=config.pos_extrapolator.april_tag_config.tag_use_imu_rotation,
                    april_tag_config=config.pos_extrapolator.april_tag_config,
                ),
            ),
        )


def _get_subscribe_topics(config: Config):
    subscribe_topics: list[str] = []
    if config.pos_extrapolator.enable_imu:
        subscribe_topics.append(
            config.pos_extrapolator.message_config.post_imu_input_topic
        )
    if config.pos_extrapolator.enable_odom:
        subscribe_topics.append(
            config.pos_extrapolator.message_config.post_odometry_input_topic
        )
    if config.pos_extrapolator.enable_tags:
        subscribe_topics.append(
            config.pos_extrapolator.message_config.post_tag_input_topic
        )
    if config.pos_extrapolator.composite_publish_topic:
        subscribe_topics.append(config.pos_extrapolator.composite_publish_topic)

    return subscribe_topics


def main_init_phase() -> tuple[BasicSystemConfig, Config, Autobahn, list[str]]:
    system_config, config = load_configs()

    autobahn_server = _init_utilities(system_config)
    _init_data_preparer_manager(config)
    subscribe_topics = _get_subscribe_topics(config)

    return system_config, config, autobahn_server, subscribe_topics
