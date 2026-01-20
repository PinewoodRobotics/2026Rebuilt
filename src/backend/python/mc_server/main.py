import asyncio

from autobahn_client.client import Autobahn
from autobahn_client.util import Address
from backend.python.common.debug.logger import LogLevel, error, init_logging, success
from backend.python.common.util.system import get_system_name, load_configs

SERVER_JAR_PATH = "/home/ubuntu/mcserver"
SERVER_JAR_NAME = "paper.jar"

ALLOWED_GB_MEMORY = 8
ALLOWED_CORES = 4

# Aikar's flags optimized for multi-core Minecraft servers
# These flags improve garbage collection and enable better parallel processing
JAVA_CMD = (
    f"java "
    f"-Xms{ALLOWED_GB_MEMORY}G -Xmx{ALLOWED_GB_MEMORY}G "
    f"-XX:+UseG1GC "
    f"-XX:+ParallelRefProcEnabled "
    f"-XX:MaxGCPauseMillis=200 "
    f"-XX:+UnlockExperimentalVMOptions "
    f"-XX:+DisableExplicitGC "
    f"-XX:+AlwaysPreTouch "
    f"-XX:G1HeapWastePercent=5 "
    f"-XX:G1MixedGCCountTarget=4 "
    f"-XX:InitiatingHeapOccupancyPercent=15 "
    f"-XX:G1MixedGCLiveThresholdPercent=90 "
    f"-XX:G1RSetUpdatingPauseTimePercent=5 "
    f"-XX:SurvivorRatio=32 "
    f"-XX:+PerfDisableSharedMem "
    f"-XX:MaxTenuringThreshold=1 "
    f"-XX:G1NewSizePercent=30 "
    f"-XX:G1MaxNewSizePercent=40 "
    f"-XX:G1HeapRegionSize=8M "
    f"-XX:G1ReservePercent=20 "
    f"-XX:ActiveProcessorCount={ALLOWED_CORES} "
    f"-Dusing.aikars.flags=https://mcflags.emc.gs "
    f"-Daikars.new.flags=true "
    f"-jar {SERVER_JAR_NAME} nogui"
)


async def main():
    basic_system_config, _ = load_configs()
    autobahn_server = Autobahn(
        Address(
            basic_system_config.autobahn.host,
            basic_system_config.autobahn.port,
        )
    )

    await autobahn_server.begin()

    init_logging(
        "MC_SERVER",
        LogLevel(basic_system_config.logging.global_logging_level),
        system_pub_topic=basic_system_config.logging.global_log_pub_topic,
        autobahn=autobahn_server,
        system_name=get_system_name(),
    )

    success("Starting MC server")

    process = await asyncio.create_subprocess_shell(
        f"cd {SERVER_JAR_PATH} && {JAVA_CMD}",
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )

    if not process.stderr or not process.stdout:
        error("Failed to start MC server")
        return

    while True:
        line = await process.stdout.readline()
        if not line:
            break
        success(line.decode().rstrip())

    stderr = await process.stderr.read()
    if stderr:
        error(f"{stderr.decode().rstrip()}")

    await process.wait()


if __name__ == "__main__":
    asyncio.run(main())
