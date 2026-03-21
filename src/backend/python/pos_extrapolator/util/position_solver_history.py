from __future__ import annotations

import bisect

from backend.python.pos_extrapolator.util.solver_models import (
    SensorEvent,
    SolverSnapshot,
)


class PositionSolverHistory:
    def __init__(self, window_s: float) -> None:
        self.window_s = window_s
        self.seed_snapshot: SolverSnapshot | None = None
        self.events: list[SensorEvent] = []
        self.post_snapshots: list[SolverSnapshot] = []

    def is_empty(self) -> bool:
        return not self.events

    def ensure_seed_snapshot(self, snapshot: SolverSnapshot) -> None:
        if self.seed_snapshot is None:
            self.seed_snapshot = snapshot

    def has_seed_snapshot(self) -> bool:
        return self.seed_snapshot is not None

    def insert_event(self, event: SensorEvent) -> int:
        insert_idx = bisect.bisect_right(self.events, event)
        self.events.insert(insert_idx, event)
        return insert_idx

    def prune(self, latest_time_s: float) -> SolverSnapshot | None:
        cutoff_s = latest_time_s - self.window_s
        latest_pruned_snapshot: SolverSnapshot | None = None
        while self.events and self.events[0].timestamp_s < cutoff_s:
            self.events.pop(0)
            if self.post_snapshots:
                latest_pruned_snapshot = self.post_snapshots.pop(0)
        if latest_pruned_snapshot is not None:
            self.seed_snapshot = latest_pruned_snapshot
        return latest_pruned_snapshot

    def rollback_snapshot(self, event_idx: int) -> SolverSnapshot:
        if self.seed_snapshot is None:
            raise ValueError("Seed snapshot must exist before rollback")
        if event_idx == 0:
            return self.seed_snapshot
        return self.post_snapshots[event_idx - 1]

    def discard_snapshots_from(self, event_idx: int) -> None:
        del self.post_snapshots[event_idx:]

    def append_snapshot(self, snapshot: SolverSnapshot) -> None:
        self.post_snapshots.append(snapshot)

    def current_snapshot(self) -> SolverSnapshot | None:
        if self.post_snapshots:
            return self.post_snapshots[-1]
        return None

    def latest_timestamp_s(self) -> float | None:
        if self.events:
            return self.events[-1].timestamp_s
        if self.seed_snapshot is not None:
            return self.seed_snapshot.timestamp_s
        return None

    def start_timestamp_s(self) -> float | None:
        if self.events:
            return self.events[0].timestamp_s
        if self.seed_snapshot is not None:
            return self.seed_snapshot.timestamp_s
        return None

    def events_from(self, start_event_idx: int) -> list[SensorEvent]:
        return self.events[start_event_idx:]
