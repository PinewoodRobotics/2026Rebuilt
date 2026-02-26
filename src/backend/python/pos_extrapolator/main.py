# TODO: need to add a better way to handle the non-used indices in sensors (config method).

import asyncio

from backend.generated.proto.python.sensor.general_sensor_data_pb2 import (
    GeneralSensorData,
)
from backend.python.common.debug.logger import (
    error,
    info,
)
from backend.python.common.util.extension import subscribe_to_multiple_topics
from backend.python.pos_extrapolator.data_prep import DataPreparerManager
from backend.python.pos_extrapolator.filters.extended_kalman_filter import (
    ExtendedKalmanFilterStrategy,
)
from backend.python.pos_extrapolator.position_extrapolator import PositionExtrapolator
from backend.python.pos_extrapolator.util.init_stuff import main_init_phase


async def main():
    _, config, autobahn_server, subscribe_topics = main_init_phase()

    info(f"Starting Position Extrapolator...")
    await autobahn_server.begin()

    position_extrapolator = PositionExtrapolator(
        config.pos_extrapolator,
        ExtendedKalmanFilterStrategy(config.pos_extrapolator.kalman_filter_config),
        DataPreparerManager(),
    )

    async def process_data(message: bytes):
        data = GeneralSensorData.FromString(message)
        one_of_name = data.WhichOneof("data")

        try:
            position_extrapolator.insert_sensor_data(
                data.__getattribute__(one_of_name), data.sensor_id
            )
        except Exception as e:
            error(
                f"Something went wrong when inserting data into Position Extrapolator: {e}"
            )

    await subscribe_to_multiple_topics(
        autobahn_server,
        subscribe_topics,
        process_data,
    )

    info(f"Subscribed to topics: {subscribe_topics}. Starting position extrapolation.")

    while True:
        proto_position = position_extrapolator.get_robot_position()

        await autobahn_server.publish(
            config.pos_extrapolator.message_config.post_robot_position_output_topic,
            proto_position.SerializeToString(),
        )

        await asyncio.sleep(
            config.pos_extrapolator.time_s_between_position_sends
            if config.pos_extrapolator.time_s_between_position_sends
            else 0.025
        )


if __name__ == "__main__":
    asyncio.run(main())
