"""Executor configuration. EVERY value below is a PLACEHOLDER unless its comment says otherwise.

Sources: board = supplied board CAD (40 cm frame, 35 cm playing area) [board decision pending];
mount layout = current simulation layout (robot1 at -Y facing +Y, robot2 at +Y facing -Y);
everything else (arm geometry, limits, speeds, piece heights, bins, park pose) = placeholder until the Design team,
M2 (pieces), M4/M9 (limits, motors) and M5 (control) provide real data. config/executor_params.yaml mirrors DEFAULT_CONFIG.
"""
from __future__ import annotations
import copy
import math
from dataclasses import dataclass

from .board_geometry import BoardGeometry, RobotMount
from .scara import ScaraParams

DEFAULT_CONFIG = {
    'board': {'play_size': 0.35, 'surface_z': 0.010},
    'mounts': {
        'robot1': {'x': 0.0, 'y': -0.30, 'yaw': math.pi / 2, 'base_z': 0.0},
        'robot2': {'x': 0.0, 'y': 0.30, 'yaw': -math.pi / 2, 'base_z': 0.0},
    },
    'arm_for_side': {'white': 'robot1', 'black': 'robot2'},
    'scara': {                                   # PLACEHOLDER geometry (same for both arms)
        'l1': 0.30, 'l2': 0.24,
        'q1_limits': [-2.8, 2.8], 'q2_limits': [-2.8, 2.8], 'z_limits': [0.0, 0.15],
        'reach_margin': 0.003, 'elbow_preference': 'left',
    },
    'limits': {                                  # PLACEHOLDER, per axis [shoulder rad, elbow rad, z m]
        'vmax': [1.0, 1.5, 0.10], 'amax': [3.0, 4.0, 0.50], 'speed_scale': 0.5,
    },
    'park': {'q1': 1.5708, 'q2': 2.3},           # PLACEHOLDER joint pose of an idle arm (outside the board)
    'heights': {                                 # PLACEHOLDER until M2 provides piece data (metres)
        'piece_height': {'p': 0.035, 'n': 0.045, 'b': 0.050, 'r': 0.040, 'q': 0.060, 'k': 0.070},
        'grasp_fraction': 0.6, 'travel_clearance': 0.030, 'place_clearance': 0.003,
    },
    'gripper': {'open_width': 0.045, 'grasp_width': 0.012, 'time': 0.6},   # PLACEHOLDER (metres, seconds)
    'capture_bins': {                            # PLACEHOLDER storage grid (board frame): up to 15 captured pieces + 8 promoted pawns per arm
        'robot1': {'x': 0.26, 'y': -0.15, 'dx': 0.035, 'dy': 0.035, 'columns': 4, 'slots': 24},
        'robot2': {'x': -0.26, 'y': 0.15, 'dx': -0.035, 'dy': -0.035, 'columns': 4, 'slots': 24},
    },
    'timing': {'operator_timeout': 120.0, 'settle_time': 0.10, 'min_segment': 0.05},
}


def _merge(base: dict, over: dict) -> dict:
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            _merge(base[k], v)
        else:
            base[k] = v
    return base


@dataclass(frozen=True)
class ExecutorConfig:
    raw: dict
    board: BoardGeometry
    mounts: dict
    scara: ScaraParams
    elbow_preference: str

    @property
    def arms(self):
        return tuple(self.mounts)

    def arm_for_side(self, side: str) -> str:
        try:
            return self.raw['arm_for_side'][side]
        except KeyError:
            raise ValueError(f'unknown side {side!r}') from None

    @property
    def travel_z(self) -> float:
        h = self.raw['heights']
        return self.board.surface_z + max(h['piece_height'].values()) + h['travel_clearance']

    def piece_height(self, piece: str | None) -> float:
        ph = self.raw['heights']['piece_height']
        return ph.get((piece or 'p').lower(), ph['p'])

    def grasp_z(self, piece: str | None) -> float:
        return self.board.surface_z + self.raw['heights']['grasp_fraction'] * self.piece_height(piece)

    @property
    def place_z(self) -> float:
        return self.board.surface_z + self.raw['heights']['place_clearance']

    def vmax(self) -> list:
        s = self.raw['limits']['speed_scale']
        return [v * s for v in self.raw['limits']['vmax']]

    def amax(self) -> list:
        s = self.raw['limits']['speed_scale']
        return [a * s for a in self.raw['limits']['amax']]

    def bin_slot(self, arm: str, index: int) -> tuple[float, float]:
        b = self.raw['capture_bins'][arm]
        if not 0 <= index < b['slots']:
            raise ValueError('bin_full')
        return (b['x'] + (index % b['columns']) * b['dx'], b['y'] + (index // b['columns']) * b['dy'])


def build_config(overrides: dict | None = None) -> ExecutorConfig:
    raw = copy.deepcopy(DEFAULT_CONFIG)
    if overrides:
        _merge(raw, overrides)
    sc = raw['scara']
    scara = ScaraParams(sc['l1'], sc['l2'], sc['q1_limits'][0], sc['q1_limits'][1], sc['q2_limits'][0],
                        sc['q2_limits'][1], sc['z_limits'][0], sc['z_limits'][1], sc['reach_margin'])
    mounts = {n: RobotMount(n, m['x'], m['y'], m['yaw'], m.get('base_z', 0.0)) for n, m in raw['mounts'].items()}
    return ExecutorConfig(raw, BoardGeometry(**raw['board']), mounts, scara, sc.get('elbow_preference', 'left'))


def load_config_file(path: str) -> ExecutorConfig:
    import yaml
    with open(path, encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    return build_config(data.get('executor', data))
