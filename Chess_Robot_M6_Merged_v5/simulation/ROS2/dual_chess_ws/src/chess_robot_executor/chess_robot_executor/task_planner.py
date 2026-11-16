"""Move plan (JSON dict from the chess brain) -> ordered Cartesian pick/place steps for one arm (pure Python).

Handles the brain's action kinds: remove (captures, en passant), move (normal, castling = two moves), promote
(pawn to the bin + operator places the new piece; the robot does not own a reserve of pieces yet).
Does not compute joints (see controller) and does not do collision checking against the other arm.
"""
from __future__ import annotations
from dataclasses import dataclass, field

from .board_geometry import parse_square
from .config import ExecutorConfig

INITIAL_FEN_PREFIX = 'rnbqkbnr/pppppppp/8/8/8/8/PPPPPPPP/RNBQKBNR'


class PlanError(ValueError):
    """The plan is malformed or cannot be executed; `.reason` is a short machine-readable string."""

    def __init__(self, reason: str, detail: str = ''):
        super().__init__(f'{reason}: {detail}' if detail else reason)
        self.reason = reason


@dataclass(frozen=True)
class Step:
    kind: str                 # 'travel' (xy at current z) | 'z' | 'grip' | 'operator' | 'park'
    x: float = 0.0            # board frame (travel)
    y: float = 0.0
    z: float = 0.0            # board-frame height (z)
    width: float = 0.0        # gripper opening in metres (grip)
    note: str = ''


@dataclass
class TaskPlan:
    plan_id: str
    arm: str
    steps: list = field(default_factory=list)
    bin_count: int = 0        # captured-piece count of this arm's bin AFTER this plan
    summary: str = ''


def _pick(cfg: ExecutorConfig, xy, piece, label) -> list:
    g = cfg.raw['gripper']
    return [Step('z', z=cfg.travel_z, note='raise to travel height'),
            Step('travel', x=xy[0], y=xy[1], note=f'above {label}'),
            Step('grip', width=g['open_width'], note='open'),
            Step('z', z=cfg.grasp_z(piece), note=f'descend to grasp {label}'),
            Step('grip', width=g['grasp_width'], note=f'grasp {label}'),
            Step('z', z=cfg.travel_z, note='lift')]


def _place(cfg: ExecutorConfig, xy, label) -> list:
    g = cfg.raw['gripper']
    return [Step('travel', x=xy[0], y=xy[1], note=f'above {label}'),
            Step('z', z=cfg.place_z, note=f'descend to place at {label}'),
            Step('grip', width=g['open_width'], note=f'release at {label}'),
            Step('z', z=cfg.travel_z, note='retreat')]


def plan_tasks(plan: dict, cfg: ExecutorConfig, bin_counts: dict | None = None) -> TaskPlan:
    if not isinstance(plan, dict):
        raise PlanError('malformed_plan', 'plan must be a JSON object')
    plan_id = plan.get('plan_id')
    if not isinstance(plan_id, str) or not plan_id:
        raise PlanError('malformed_plan', 'missing plan_id')
    side = plan.get('side')
    if side not in ('white', 'black'):
        raise PlanError('malformed_plan', f'bad side {side!r}')
    actions = plan.get('actions')
    if not isinstance(actions, list) or not actions:
        raise PlanError('malformed_plan', 'no actions')
    arm = cfg.arm_for_side(side)
    counts = dict(bin_counts or {})
    if str(plan.get('before_fen', '')).startswith(INITIAL_FEN_PREFIX):
        counts = {}                                   # a new game starts: bins are assumed emptied by the operator
    n_bin = counts.get(arm, 0)
    steps: list = []
    summary = []
    seen_moves = 0
    for i, act in enumerate(actions):
        kind = act.get('kind') if isinstance(act, dict) else None
        try:
            if kind == 'remove':
                src = act['source']; parse_square(src)
                piece = plan.get('captured')
                slot = cfg.bin_slot(arm, n_bin); n_bin += 1
                steps += _pick(cfg, cfg.board.square_center(src)[:2], piece, f'captured piece on {src}')
                steps += _place(cfg, slot, f'capture bin slot {n_bin}')
                summary.append(f'remove {src}')
            elif kind == 'move':
                src, dst = act['source'], act['target']; parse_square(src); parse_square(dst)
                if seen_moves == 0:
                    piece = plan.get('piece')
                else:                                  # second move of a castling: the rook of the moving side
                    piece = 'R' if side == 'white' else 'r'
                seen_moves += 1
                steps += _pick(cfg, cfg.board.square_center(src)[:2], piece, f'{src}')
                steps += _place(cfg, cfg.board.square_center(dst)[:2], f'{dst}')
                summary.append(f'move {src}-{dst}')
            elif kind == 'promote':
                sq = act['target']; parse_square(sq)
                pawn = 'P' if side == 'white' else 'p'
                slot = cfg.bin_slot(arm, n_bin); n_bin += 1
                steps += _pick(cfg, cfg.board.square_center(sq)[:2], pawn, f'pawn on {sq}')
                steps += _place(cfg, slot, f'capture bin slot {n_bin}')
                name = {'q': 'queen', 'r': 'rook', 'b': 'bishop', 'n': 'knight'}.get(str(act.get('piece', 'q')).lower(), 'queen')
                steps.append(Step('operator', note=f'place a {side} {name} on {sq}'))
                summary.append(f'promote {sq}')
            else:
                raise PlanError('unknown_action', f'action {i}: {kind!r}')
        except KeyError as exc:
            raise PlanError('malformed_plan', f'action {i} misses {exc}') from None
        except ValueError as exc:
            if isinstance(exc, PlanError):
                raise
            if str(exc) == 'bin_full':
                raise PlanError('bin_full', f'capture bin of {arm} has no free slot') from None
            raise PlanError('invalid_action', f'action {i}: {exc}') from None
    steps += [Step('z', z=cfg.travel_z, note='raise before parking'), Step('park', note='park')]
    return TaskPlan(plan_id, arm, steps, n_bin, ', '.join(summary))
