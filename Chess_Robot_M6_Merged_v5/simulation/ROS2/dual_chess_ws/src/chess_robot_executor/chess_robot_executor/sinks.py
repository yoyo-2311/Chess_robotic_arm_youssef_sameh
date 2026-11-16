"""Command sinks: where joint commands go. A sink receives {'shoulder','elbow','z','gripper'} per arm."""
from __future__ import annotations


class RecordingSink:
    """Keeps every command (for tests/offline simulation)."""

    def __init__(self):
        self.log: list = []          # (arm, cmd dict)
        self.stops: list = []

    def send(self, arm: str, cmd: dict) -> None:
        self.log.append((arm, dict(cmd)))

    def stop(self, arm: str) -> None:
        self.stops.append(arm)


class LogSink:
    """Dry-run sink: reports commands through `logger` (a callable) but moves nothing."""

    def __init__(self, logger, every: int = 25):
        self.logger, self.every, self.n = logger, max(1, every), 0

    def send(self, arm: str, cmd: dict) -> None:
        self.n += 1
        if self.n % self.every == 1:
            self.logger(f'[dry-run] {arm} cmd shoulder={cmd["shoulder"]:.3f} elbow={cmd["elbow"]:.3f} '
                        f'z={cmd["z"]:.3f} gripper={cmd["gripper"]:.3f}')

    def stop(self, arm: str) -> None:
        self.logger(f'[dry-run] stop {arm}')
