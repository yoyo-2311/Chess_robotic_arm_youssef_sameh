"""Board and robot-mount geometry (pure Python)."""
from __future__ import annotations
import math
from dataclasses import dataclass

FILES = 'abcdefgh'


def parse_square(square: str) -> tuple[int, int]:
    """'e4' -> (file=4, rank=3), both 0-based. Raises ValueError for anything else."""
    if not isinstance(square, str) or len(square) != 2 or square[0].lower() not in FILES or square[1] not in '12345678':
        raise ValueError(f'invalid square {square!r}')
    return FILES.index(square[0].lower()), int(square[1]) - 1


@dataclass(frozen=True)
class BoardGeometry:
    """Playing area only (the rim is not part of the grid). Defaults: supplied board CAD, 35 cm playing area."""
    play_size: float = 0.35        # metres, edge of the 8x8 grid
    surface_z: float = 0.010       # height of the playing surface above the ground (provisional, from the board CAD)

    @property
    def square(self) -> float:
        return self.play_size / 8.0

    def square_center(self, square: str) -> tuple[float, float, float]:
        f, r = parse_square(square)
        h = self.play_size / 2.0
        return (-h + self.square * (f + 0.5), -h + self.square * (r + 0.5), self.surface_z)

    def bounds(self, margin: float = 0.0) -> tuple[float, float, float, float]:
        h = self.play_size / 2.0 + margin
        return (-h, -h, h, h)

    def contains_xy(self, x: float, y: float, margin: float = 0.0) -> bool:
        x0, y0, x1, y1 = self.bounds(margin)
        return x0 <= x <= x1 and y0 <= y <= y1


@dataclass(frozen=True)
class RobotMount:
    """Pose of an arm's vertical base axis in the board frame. yaw = direction of the robot +x axis in B."""
    name: str
    x: float
    y: float
    yaw: float
    base_z: float = 0.0            # height of the robot-frame zero above the ground

    def to_robot(self, xb: float, yb: float, zb: float = 0.0) -> tuple[float, float, float]:
        dx, dy = xb - self.x, yb - self.y
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (c * dx + s * dy, -s * dx + c * dy, zb - self.base_z)

    def to_board(self, xr: float, yr: float, zr: float = 0.0) -> tuple[float, float, float]:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return (self.x + c * xr - s * yr, self.y + s * xr + c * yr, zr + self.base_z)
