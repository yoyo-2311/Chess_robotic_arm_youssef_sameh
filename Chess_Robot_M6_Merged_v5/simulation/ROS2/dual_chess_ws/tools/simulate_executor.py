#!/usr/bin/env python3
"""Offline demonstration of the control chain: chess brain plan -> task planner -> IK -> trajectories -> commands.
No ROS needed. Plays a seeded random game with python-chess, runs every move through the executor and reports timing.

  python3 tools/simulate_executor.py [--plies 60] [--seed 1] [--plot docs/figures/executor_tool_paths.png] [--plot-moves 3]

Needs python-chess (vendored in chess_brain_handover) and, for --plot, matplotlib. All geometry is PLACEHOLDER."""
import argparse
import os
import random
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
for p in (os.path.join(SRC, 'chess_robot_executor'), os.path.join(SRC, 'chess_brain_handover'),
          os.path.join(SRC, 'chess_brain_handover', 'chess_brain_handover', '_vendor')):
    sys.path.insert(0, p)
import chess                                                   # noqa: E402
from chess_brain_handover import core                          # noqa: E402
from chess_robot_executor.config import build_config           # noqa: E402
from chess_robot_executor.controller import ExecutorController, State   # noqa: E402
from chess_robot_executor.sinks import RecordingSink            # noqa: E402

DT = 0.02


def run_move(ex, confirm=True):
    t = 0.0
    while t < 2000:
        ex.tick(DT); t += DT
        if ex.state is State.IDLE:
            return t
        if ex.state is State.WAIT_OPERATOR and confirm:
            ex.on_operator_confirm()
    raise RuntimeError('move did not finish')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--plies', type=int, default=60)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--plot', default=None)
    ap.add_argument('--plot-moves', type=int, default=3, help='how many moves (starting at the first capture) to draw')
    ap.add_argument('--speed-scale', type=float, default=None)
    a = ap.parse_args()
    over = {'limits': {'speed_scale': a.speed_scale}} if a.speed_scale else None
    cfg = build_config(over)
    sink, acks = RecordingSink(), []
    ex = ExecutorController(cfg, sink, acks.append)
    rng, board = random.Random(a.seed), chess.Board()
    rows, paths = [], []
    print(f'{"ply":>3} {"move":<6} {"san":<8} {"arm":<7} {"time[s]":>8} {"cmds":>6}  note')
    for ply in range(1, a.plies + 1):
        moves = list(board.legal_moves)
        if not moves:
            break
        mv = rng.choice(moves)
        note = ' '.join(k for k, c in (('capture', board.is_capture(mv)), ('castle', board.is_castling(mv)),
                                       ('en-passant', board.is_en_passant(mv)), ('promotion', bool(mv.promotion))) if c)
        plan = core.describe_move(board, mv).to_dict()
        sink.log.clear()
        ex.on_plan(plan)
        t = run_move(ex)
        ack = acks[-1]
        assert ack['success'], ack
        rows.append((ply, mv.uci(), plan['san'], ack['arm'], t, len(sink.log), note))
        paths.append((ack['arm'], [dict(c) for _, c in sink.log], note))
        print(f'{ply:>3} {mv.uci():<6} {plan["san"]:<8} {ack["arm"]:<7} {t:8.1f} {len(sink.log):6d}  {note}')
        board.push(mv)
    total = sum(r[4] for r in rows)
    print(f'\n{len(rows)} moves, all acknowledged as success. Total {total:.0f} s ({total/len(rows):.1f} s per move, '
          f'speed_scale={cfg.raw["limits"]["speed_scale"]}); longest {max(r[4] for r in rows):.1f} s. Captured-piece bins: {ex.bin_counts}.')
    print('Timings are PLACEHOLDER-based (placeholder speeds/accelerations/heights); the real arm will differ.')
    if a.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
        fig, ax = plt.subplots(figsize=(7.5, 8))
        h, sq = cfg.board.play_size / 2, cfg.board.square
        for f in range(8):
            for r in range(8):
                ax.add_patch(Rectangle((-h + f * sq, -h + r * sq), sq, sq, color='#d9c9a3' if (f + r) % 2 else '#8a6d46', alpha=0.8))
        for i, c in enumerate('abcdefgh'):
            ax.text(-h + (i + .5) * sq, -h - 0.015, c, ha='center', va='top', fontsize=8)
            ax.text(-h - 0.01, -h + (i + .5) * sq, str(i + 1), ha='right', va='center', fontsize=8)
        colors = {'robot1': 'tab:blue', 'robot2': 'tab:orange'}
        for arm, m in cfg.mounts.items():
            ax.plot(m.x, m.y, 's', color=colors[arm], ms=10)
            ax.text(m.x + 0.012, m.y, arm + ' base', color=colors[arm], fontsize=8, va='center')
            b = cfg.raw['capture_bins'][arm]
            xs, ys = zip(*[cfg.bin_slot(arm, i) for i in range(b['slots'])])
            ax.plot(xs, ys, '.', color=colors[arm], alpha=0.35)
            ax.text(xs[0], ys[0] - 0.02, 'capture bin (placeholder)', fontsize=7, color=colors[arm])
        first = next((i for i, p in enumerate(paths) if 'capture' in p[2]), 0)
        for arm, cmds, note in paths[first:first + a.plot_moves]:
            m = cfg.mounts[arm]
            pts = [m.to_board(*cfg.scara.fk(c['shoulder'], c['elbow'], c['z'])[:2])[:2] for c in cmds]
            ax.plot([p[0] for p in pts], [p[1] for p in pts], '-', lw=1.4, color=colors[arm])
            ax.plot(pts[0][0], pts[0][1], 'o', color='k', ms=3)
        ax.set_aspect('equal'); ax.set_xlim(-0.45, 0.45); ax.set_ylim(-0.55, 0.55)
        ax.set_title(f'Tool-centre paths of {a.plot_moves} executed moves (from the first capture)\nPLACEHOLDER SCARA geometry, top view, board frame')
        ax.set_xlabel('x [m] (files a -> h)'); ax.set_ylabel('y [m] (ranks 1 -> 8)')
        os.makedirs(os.path.dirname(os.path.abspath(a.plot)), exist_ok=True)
        fig.savefig(a.plot, dpi=110, bbox_inches='tight')
        print('plot written to', a.plot)


if __name__ == '__main__':
    main()
