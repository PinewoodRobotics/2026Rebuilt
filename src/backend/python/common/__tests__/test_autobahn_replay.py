import asyncio
from backend.generated.proto.python.replay.replay_pb2 import Replay
from backend.python.common.debug import pubsub_replay
from backend.python.common.debug.pubsub_replay import autolog
from backend.python.common.debug.replay_recorder import (
    close,
    get_next_replay,
    init_replay_recorder,
)


def test_autolog():
    init_replay_recorder("test.db", mode="w")

    @autolog("test")
    async def test_func(data: bytes):
        return data

    async def run_async():
        await test_func(b"test")
        await test_func(b"test2")

    asyncio.run(run_async())

    close()

    init_replay_recorder("test.db", mode="r")

    first = get_next_replay()
    second = get_next_replay()
    assert first is not None
    assert first.key == "test"
    assert first.data == b"test"
    assert second is not None
    assert second.key == "test"
    assert second.data == b"test2"

    assert get_next_replay() is None

    close()


def test_autolog_records_each_topic(monkeypatch):
    calls: list[tuple[str, bytes]] = []

    def record_output_spy(topic: str, data: bytes):
        calls.append((topic, data))

    monkeypatch.setattr(pubsub_replay, "record_output", record_output_spy)

    @pubsub_replay.autolog("a", "b")
    async def f(data: bytes):
        return data + b"!"

    async def run():
        out = await f(b"hi")
        assert out == b"hi!"

    asyncio.run(run())
    assert calls == [("a", b"hi"), ("b", b"hi")]


def test_autolog_can_disable_recording(monkeypatch):
    calls: list[tuple[str, bytes]] = []

    def record_output_spy(topic: str, data: bytes):
        calls.append((topic, data))

    monkeypatch.setattr(pubsub_replay, "record_output", record_output_spy)

    @pubsub_replay.autolog("a", do_record=False)
    async def f(data: bytes):
        return data

    asyncio.run(f(b"x"))  # pyright: ignore[reportArgumentType]
    assert calls == []


def test_replay_autobahn_replays_and_sleeps(monkeypatch):
    # Provide deterministic replays without touching the global recorder.
    replays = iter(
        [
            Replay(key="t", data_type="bytes", time=1.0, data=b"p1"),
            Replay(key="t", data_type="bytes", time=1.5, data=b"p2"),
            Replay(key="t", data_type="bytes", time=20.0, data=b"p3"),
            None,
        ]
    )

    monkeypatch.setattr(pubsub_replay, "get_next_replay", lambda: next(replays))

    sleep_calls: list[float] = []

    def fake_sleep(dt: float):
        sleep_calls.append(dt)

    monkeypatch.setattr(pubsub_replay.time, "sleep", fake_sleep)

    seen: list[bytes] = []

    async def runner():
        event = asyncio.Event()
        ab = pubsub_replay.ReplayAutobahn()

        async def cb(payload: bytes):
            seen.append(payload)
            if len(seen) == 3:
                event.set()

        await ab.subscribe("t", cb)
        await asyncio.wait_for(event.wait(), timeout=2.0)
        ab.close()

    asyncio.run(runner())

    assert seen == [b"p1", b"p2", b"p3"]
    # Between 1.0->1.5 should be exact, between 1.5->20.0 should clamp.
    assert sleep_calls == [0.5, 0.025]


def test_replay_autobahn_drives_callbacks_with_infinite_stream(monkeypatch):
    """
    If the replay source never ends (i.e. get_next_replay never returns None),
    the ReplayAutobahn background loop should still drive subscribed callbacks.
    """

    i = 0

    def infinite_replay():
        nonlocal i
        i += 1
        # Force the loop to "sleep" each iteration via the clamped branch.
        return Replay(key="t", data_type="bytes", time=float(i * 1000), data=f"p{i}".encode())

    monkeypatch.setattr(pubsub_replay, "get_next_replay", infinite_replay)

    sleep_calls: list[float] = []
    orig_sleep = pubsub_replay.time.sleep

    def fake_sleep(dt: float):
        sleep_calls.append(dt)
        # Yield to the event loop / other threads without slowing the test down.
        orig_sleep(0)

    monkeypatch.setattr(pubsub_replay.time, "sleep", fake_sleep)

    seen: list[bytes] = []

    async def runner():
        event = asyncio.Event()
        ab = pubsub_replay.ReplayAutobahn()

        async def cb(payload: bytes):
            seen.append(payload)
            if len(seen) >= 25:
                event.set()

        await ab.subscribe("t", cb)
        await asyncio.wait_for(event.wait(), timeout=2.0)
        ab.close()

    asyncio.run(runner())

    assert len(seen) >= 25
    assert seen[0].startswith(b"p")
    # We should have slept at least once after the first replay.
    assert len(sleep_calls) >= 1
