#!/usr/bin/env python3
"""Reach analysis for a SCARA arm on the chessboard (stdlib only; uses the executor package for the geometry).

Answers the question the Design team needs answered: for a given base distance from the board centre, how long must the
links be and how much joint range is needed to reach all 64 squares (+ the capture-bin slots)?

  python3 tools/scara_reach_report.py [--play 0.35] [--l1 0.30 --l2 0.24] [--bins]

All results are GEOMETRIC minimums for the tool centre point at the working heights (positions only): no gripper
orientation, no collision with the base column or the other arm, no payload/torque. Bin positions are placeholders.
"""
import argparse
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src', 'chess_robot_executor'))
from chess_robot_executor.board_geometry import BoardGeometry, RobotMount   # noqa: E402
from chess_robot_executor.config import build_config                        # noqa: E402
from chess_robot_executor.scara import ScaraParams                          # noqa: E402

SQUARES = [f + r for f in 'abcdefgh' for r in '12345678']


def targets(board, mount, include_bins, cfg):
    pts = [board.square_center(s)[:2] for s in SQUARES]
    if include_bins:
        pts += [cfg.bin_slot('robot1', i) for i in range(cfg.raw['capture_bins']['robot1']['slots'])]
    return [mount.to_robot(x, y)[:2] for x, y in pts]


def distances(board, d):
    m = RobotMount('r', 0.0, -d, math.pi / 2)
    r = [math.hypot(*m.to_robot(*board.square_center(s)[:2])[:2]) for s in SQUARES]
    return min(r), max(r)


def needed_ranges(l1, l2, pts):
    """Per elbow branch: q1 range, max |q2|. None if any point is unreachable."""
    wide = ScaraParams(l1, l2, -math.pi, math.pi, -math.pi, math.pi, -1, 1, 0.0)
    out = {}
    for branch in ('left', 'right'):
        q1s, q2s = [], []
        for x, y in pts:
            sol = [s for s in wide.ik(x, y, 0.0) if s.elbow == branch]
            if not sol:
                out[branch] = None
                break
            q1s.append(sol[0].q1); q2s.append(sol[0].q2)
        else:
            out[branch] = (min(q1s), max(q1s), max(abs(q) for q in q2s))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--play', type=float, default=0.35)
    ap.add_argument('--l1', type=float, default=0.30)
    ap.add_argument('--l2', type=float, default=0.24)
    ap.add_argument('--bins', action='store_true', help='include the placeholder capture-bin slots')
    a = ap.parse_args()
    board, cfg = BoardGeometry(play_size=a.play), build_config()
    print(f'Board playing area {a.play*100:.1f} cm, squares {a.play/8*100:.3f} cm. Targets: 64 square centres'
          + (' + capture-bin slots (placeholders)' if a.bins else '') + '.\n')
    print('1) Distance from the base axis to the nearest / farthest square centre, by base distance d from the board centre')
    print('   (base on the centre line, d measured from board centre; arm must span [nearest, farthest])')
    print('   d [cm]   nearest [cm]   farthest [cm]   needed l1+l2 with 10 % margin [cm]   needed |l1-l2| <= [cm]')
    for d in (0.20, 0.25, 0.30, 0.35, 0.40):
        lo, hi = distances(board, d)
        print(f'   {d*100:5.0f}    {lo*100:9.1f}      {hi*100:9.1f}        {hi*1.10*100:14.1f}                 {lo*100:14.1f}')
    print(f'\n2) Joint ranges needed for l1={a.l1*100:.0f} cm, l2={a.l2*100:.0f} cm (both arms identical, rotated copies)')
    print('   d [cm]   reachable   left elbow: q1 range [deg], max|q2| [deg]      right elbow: q1 range [deg], max|q2| [deg]')
    for d in (0.20, 0.25, 0.30, 0.35, 0.40):
        m = RobotMount('r', 0.0, -d, math.pi / 2)
        res = needed_ranges(a.l1, a.l2, targets(board, m, a.bins, cfg))
        f = lambda r: 'unreachable' if r is None else f'[{math.degrees(r[0]):6.1f}, {math.degrees(r[1]):6.1f}]  {math.degrees(r[2]):6.1f}'
        print(f'   {d*100:5.0f}    {"yes" if any(res.values()) else "NO ":>9}   {f(res["left"]):<44}{f(res["right"])}')
    print('\n3) Sweep (d = 30 cm): which link pairs reach everything, and the elbow range they need (left elbow)')
    print('   l1 [cm]  l2 [cm]  l1+l2    reach all?   q1 range [deg]        max|q2| [deg]')
    m = RobotMount('r', 0.0, -0.30, math.pi / 2)
    pts = targets(board, m, a.bins, cfg)
    for l1, l2 in ((0.25, 0.25), (0.28, 0.24), (0.30, 0.24), (0.30, 0.30), (0.32, 0.22), (0.35, 0.20), (0.35, 0.25), (0.20, 0.20)):
        r = needed_ranges(l1, l2, pts)['left']
        print(f'   {l1*100:6.0f}  {l2*100:6.0f}  {(l1+l2)*100:6.0f}    {"yes" if r else "NO ":>9}    '
              + ('' if r is None else f'[{math.degrees(r[0]):6.1f}, {math.degrees(r[1]):6.1f}]     {math.degrees(r[2]):6.1f}'))
    print('\nReading: the FARTHEST square sets the minimum l1+l2; the NEAREST squares force the deepest elbow fold (large |q2|) and'
          ' need |l1-l2| below their distance. Moving the base away from the board reduces the fold but raises the reach needed.'
          ' Final values depend on the real arm and layout.')


if __name__ == '__main__':
    main()
