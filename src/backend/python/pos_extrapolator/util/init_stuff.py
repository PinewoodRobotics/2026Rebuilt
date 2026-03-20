from autobahn_client.client import Autobahn
from autobahn_client.util import Address
from backend.generated.thrift.config.pos_extrapolator.ttypes import DataSources
from backend.python.common.debug.logger import LogLevel, init_logging
from backend.python.common.util.system import (
    BasicSystemConfig,
    get_system_name,
    load_configs,
)
from backend.generated.thrift.config.ttypes import Config


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


def _get_enabled_sensor_topics(config: Config) -> list[str]:
    enabled = config.pos_extrapolator.enabled_data_sources
    msg_config = config.pos_extrapolator.message_config
    topics: list[str] = []

    if DataSources.IMU in enabled:
        topics.append(msg_config.post_imu_input_topic)

    if DataSources.ODOMETRY in enabled:
        topics.append(msg_config.post_odometry_input_topic)

    if DataSources.APRIL_TAG in enabled:
        topics.append(msg_config.post_tag_input_topic)

    return topics


def main_init_phase() -> tuple[BasicSystemConfig, Config, Autobahn, list[str]]:
    system_config, config = load_configs()

    autobahn_server = _init_utilities(system_config)
    subscribe_topics = _get_enabled_sensor_topics(config)

    return system_config, config, autobahn_server, subscribe_topics
