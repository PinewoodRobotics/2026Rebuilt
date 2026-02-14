import asyncio
import board
import neopixel
from backend.python.common.util.system import load_configs
from autobahn_client.client import Autobahn
from autobahn_client.util import Address
from backend.generated.proto.python.status.PiStatus_pb2 import PiStatus
from collections import deque


MAX_MEASUREMENT_NUM = 10
LED_COUNT = 8
PIN = board.D18


def show_pie(
    pixels: neopixel.NeoPixel, percent: float, color=(0, 255, 0), bg=(0, 0, 0)
):
    """
    percent: 0-100%
    color:   RGB tuple for filled portion
    bg:      RGB tuple for unfilled portion
    """

    lit_leds = round(percent * LED_COUNT)

    for i in range(LED_COUNT):
        if i < lit_leds:
            pixels[i] = color
        else:
            pixels[i] = bg

    pixels.show()


async def main():
    basic_system_config, config = load_configs()
    pixels = neopixel.NeoPixel(
        PIN, LED_COUNT, brightness=1, auto_write=False, pixel_order=neopixel.GRB
    )

    autobahn_server = Autobahn(
        Address(
            basic_system_config.autobahn.host,
            basic_system_config.autobahn.port,
        )
    )
    await autobahn_server.begin()

    cpu_loads = deque(maxlen=MAX_MEASUREMENT_NUM)
    current_cpu_load = 0.0

    async def process_data(message: bytes):
        nonlocal current_cpu_load

        data = PiStatus.FromString(message)
        cpu_loads.append(data.cpu_usage_total)
        if cpu_loads:
            current_cpu_load = sum(cpu_loads) / len(cpu_loads)
        else:
            current_cpu_load = 0.0

    await autobahn_server.subscribe(
        basic_system_config.logging.global_log_pub_topic, process_data
    )

    while True:
        show_pie(pixels, current_cpu_load, color=(255, 0, 0), bg=(0, 255, 0))
        await asyncio.sleep(0.3)


if __name__ == "__main__":
    asyncio.run(main())
