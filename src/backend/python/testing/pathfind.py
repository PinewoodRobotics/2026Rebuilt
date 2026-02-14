# Add src/ directory to Python path so backend imports work
from pathlib import Path
import sys


script_dir = Path(__file__).parent
src_dir = script_dir.parent.parent  # src/backend/python -> src/
if str(src_dir) not in sys.path:
    sys.path.insert(0, str(src_dir))

import asyncio
from autobahn_client.util import Address
from backend.generated.proto.python.util.vector_pb2 import Vector2
from backend.generated.proto.python.pathing.pathfind_pb2 import (
    Algorithm,
    PathfindRequest,
    PathfindResult,
)
from autobahn_client.client import Autobahn


@Autobahn.rpc_callable()
async def pathfind(request: PathfindRequest) -> PathfindResult:
    return PathfindResult()


async def main():

    client = Autobahn(Address("localhost", 8090))
    await client.begin()

    result = await pathfind(
        client,
        PathfindRequest(
            start=Vector2(x=2, y=1),
            goal=Vector2(x=4, y=1),
            algorithm=Algorithm.ASTAR,
            optimize_path=False,
        ),
    )
    print(result)


if __name__ == "__main__":
    asyncio.run(main())
