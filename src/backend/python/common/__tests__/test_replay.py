import os
import time
from pathlib import Path

from _pytest.capture import capsys
from backend.python.common.debug.pubsub_replay import autolog
from backend.python.common.debug.replay_recorder import (
    GLOBAL_INSTANCE,
    Recorder,
    Player,
    close,
    get_next_key_replay,
    get_next_replay,
    init_replay_recorder,
    record_output,
)
from backend.generated.proto.python.replay.replay_pb2 import Replay
from backend.python.common.debug import replay_recorder
import pytest


def read_file_bytes(path: str) -> bytes:
    with open(path, "rb") as f:
        return f.read()


def write_file_bytes(path: str, data: bytes):
    with open(path, "wb") as f:
        f.write(data)


def read_first_line_file(path: str) -> bytes:
    with open(path, "rb") as f:
        return f.readline()


def add_sample_replay_data(additional: dict[str, str] = {}):
    recorder = Recorder("test")
    recorder.record_output("test", "test")
    for key, value in additional.items():
        recorder.record_output(key, value)
    recorder.close()


def test_replay_recorder():
    add_sample_replay_data()


def test_global_replay_player():
    add_sample_replay_data()
    player = Player("test")
    most_recent_replay = player.get_next_replay()
    assert most_recent_replay is not None
    assert most_recent_replay.key == "test"
    assert most_recent_replay.data_type == "str"
    assert most_recent_replay.data == b"test"
    assert most_recent_replay.time is not None
    assert player.get_next_replay() is None


def test_key_replay_player():
    add_sample_replay_data({"test2": "test2"})
    player = Player("test")
    most_recent_replay = player.get_next_key_replay("test")
    assert most_recent_replay is not None
    assert most_recent_replay.key == "test"
    assert most_recent_replay.data_type == "str"
    assert most_recent_replay.data == b"test"
    assert player.get_next_key_replay("test2") is not None
    assert player.get_next_key_replay("test2") is None
    assert player.get_next_key_replay("test") is None


def test_key_global_replay_player():
    add_sample_replay_data({"test2": "test2"})
    player = Player("test")
    most_recent_replay = player.get_next_replay()

    assert most_recent_replay is not None
    assert most_recent_replay.key == "test"
    assert most_recent_replay.data_type == "str"
    assert most_recent_replay.data == b"test"

    next_replay = player.get_next_replay()
    assert next_replay is not None
    assert next_replay.key == "test2"
    assert next_replay.data_type == "str"
    assert next_replay.data == b"test2"

    assert player.get_next_key_replay("test") is not None

    assert player.get_next_replay() is None


def test_replay_api(tmp_path):
    replay_dir = tmp_path
    replay_path = os.path.join(replay_dir, "replay-test.db")

    path = init_replay_recorder("replay_api_test", replay_path=replay_path, mode="w")
    assert os.path.exists(path)

    record_output("foo", "bar")
    record_output("baz", "qux")
    record_output("num", 123)
    record_output("flt", 3.14)

    close()
    init_replay_recorder("replay_api_test", replay_path=replay_path, mode="r")
    assert os.path.exists(replay_path)

    replay1 = get_next_replay()
    assert isinstance(replay1, Replay)
    assert replay1.key in {"foo", "baz", "num", "flt"}
    assert replay1.data is not None

    replay2 = get_next_key_replay("foo")
    assert isinstance(replay2, Replay)
    assert replay2.key == "foo"
    assert replay2.data == b"bar"

    assert get_next_key_replay("foo") is None

    while get_next_replay() is not None:
        pass
    assert get_next_replay() is None

    # Clean up
    close()


def test_replay_latest():
    init_replay_recorder("replay_latest_test", mode="w")
    record_output("test", "test")
    close()

    init_replay_recorder("replay_latest_test", replay_path="latest", mode="r")
    current_replay = get_next_replay()
    assert current_replay is not None
    assert current_replay.key == "test"
    assert current_replay.data == b"test"
    close()


def test_api_raises_if_not_initialized():
    replay_recorder.close()
    with pytest.raises(RuntimeError):
        replay_recorder.get_next_replay()
    with pytest.raises(RuntimeError):
        replay_recorder.record_output("k", b"v")


def test_record_output_uses_current_time_each_call(tmp_path, monkeypatch):
    db_path = tmp_path / "t.db"
    recorder = replay_recorder.Recorder(str(db_path))

    times = iter([100.0, 101.25])
    monkeypatch.setattr(replay_recorder.time_module, "time", lambda: next(times))

    recorder.record_output("a", b"x")
    recorder.record_output("b", b"y")

    rows = list(replay_recorder.ReplayDB.select().order_by(replay_recorder.ReplayDB.id))
    assert [row.timestamp for row in rows] == [100.0, 101.25]

    recorder.close()


def test_record_output_encodes_supported_types(tmp_path):
    db_path = tmp_path / "types.db"
    replay_recorder.init_replay_recorder(
        "record_output_encodes_supported_types", replay_path=str(db_path), mode="w"
    )

    replay_recorder.record_output("s", "hello")
    replay_recorder.record_output("i", 123)
    replay_recorder.record_output("f", 3.14)
    replay_recorder.record_output("b", b"\x00\x01")

    replay_recorder.close()
    replay_recorder.init_replay_recorder(
        "record_output_encodes_supported_types", replay_path=str(db_path), mode="r"
    )

    replays = []
    while True:
        r = replay_recorder.get_next_replay()
        if r is None:
            break
        replays.append(r)

    by_key = {r.key: r for r in replays}
    assert by_key["s"].data_type == "str"
    assert by_key["s"].data == b"hello"

    assert by_key["i"].data_type == "int"
    assert int.from_bytes(by_key["i"].data, byteorder="little", signed=True) == 123

    assert by_key["f"].data_type == "float"
    assert by_key["f"].data == (3.14).hex().encode("utf-8")

    assert by_key["b"].data_type == "bytes"
    assert by_key["b"].data == b"\x00\x01"

    replay_recorder.close()


def test_player_follow_global_filters_older_rows(tmp_path):
    db_path = tmp_path / "fg.db"
    recorder = replay_recorder.Recorder(str(db_path))

    recorder.record_output("a", "a1")  # id 1
    recorder.record_output("b", "b1")  # id 2
    recorder.record_output("a", "a2")  # id 3
    recorder.close()

    player = replay_recorder.Player(str(db_path))
    first = player.get_next_replay()
    assert first is not None
    assert first.key == "a"

    b = player.get_next_key_replay("b", follow_global=True)
    assert b is not None
    assert b.key == "b"
    assert b.data == b"b1"

    # because we already consumed global id 1, follow_global should skip it and allow id 3
    a2 = player.get_next_key_replay("a", follow_global=True)
    assert a2 is not None
    assert a2.key == "a"
    assert a2.data == b"a2"

    assert player.get_next_key_replay("a", follow_global=True) is None
    player.close()


def test_find_latest_replay_picks_highest_filename(tmp_path):
    d = tmp_path / "replays"
    d.mkdir()

    # should be ignored
    (d / "not-a-replay.db").write_bytes(b"")
    (d / "replay-aaaa.txt").write_text("")

    (d / "replay-2026-01-01_00-00-00.db").write_bytes(b"")
    latest = d / "replay-2026-01-02_00-00-00.db"
    latest.write_bytes(b"")

    assert replay_recorder.find_latest_replay(str(d)) == str(latest)


def test_init_replay_recorder_latest_uses_folder(tmp_path, monkeypatch):
    monkeypatch.chdir(tmp_path)
    replays_dir = tmp_path / "replays"
    replays_dir.mkdir()

    latest = replays_dir / "replay-2026-01-03_00-00-00.db"
    latest.write_bytes(b"")

    path = replay_recorder.init_replay_recorder(
        "init_replay_recorder_latest_uses_folder",
        replay_path="latest",
        mode="r",
        folder_path="replays",
    )
    assert Path(path) == latest
    assert isinstance(replay_recorder.GLOBAL_INSTANCE, replay_recorder.Player)

    replay_recorder.close()


def test_write_duration_per_call(tmp_path, capsys: pytest.CaptureFixture[str]):
    """Measure how long each Recorder.write() takes and assert timings are sane."""
    with capsys.disabled():
        db_path = tmp_path / "write_timing.db"
        recorder = replay_recorder.Recorder(str(db_path))

        # Payloads of different sizes to see if duration scales with size
        payloads = [
            (b"tiny", "tiny"),
            (b"x" * 100, "100B"),
            (b"x" * 10_000, "10KB"),
            (b"x" * 100_000, "100KB"),
        ]

        durations_seconds: list[tuple[str, float]] = []

        for data, label in payloads:
            t0 = time.perf_counter()
            recorder.write(f"key_{label}", "bytes", data)
            elapsed = time.perf_counter() - t0
            durations_seconds.append((label, elapsed))
            print(f"Write for {label}: {elapsed:.6f} seconds.")

        recorder.close()

        # Each write must complete in finite time
        for label, duration in durations_seconds:
            print(f"Asserting duration for {label}: {duration:.6f} seconds.")
            assert duration >= 0, f"write ({label}) reported negative duration"
            assert (
                duration < 5.0
            ), f"write ({label}) took {duration:.3f}s (sanity cap 5s)"

        # Optional: ensure we actually measured something (at least one write took > 0 or we have 4 timings)
        print(
            f"Total measured write durations: {len(durations_seconds)} (expected {len(payloads)})"
        )
        assert len(durations_seconds) == len(payloads)
