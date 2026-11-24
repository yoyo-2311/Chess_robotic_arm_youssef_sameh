#!/usr/bin/env python3
"""Replay a selfplay CSV and require an actual terminal game, not a ply cutoff."""
import argparse
import csv
import json
from pathlib import Path

from chess_brain_handover.core import chess
import chess.pgn


def audit(directory):
    summary = json.loads((directory / 'summary.json').read_text())
    assert summary['stop_reason'] == 'game_over', 'Game did not finish'
    assert not summary['errors'], summary['errors']
    with (directory / 'moves.csv').open(newline='') as stream:
        records = list(csv.DictReader(stream))
    assert len(records) == summary['plies_committed'] and records
    board = chess.Board()
    plan_ids, capture_ids = set(), set()
    captures = castlings = promotions = en_passants = 0
    for ply, row in enumerate(records, 1):
        assert board.outcome(claim_draw=True) is None, f'Move after game over at ply {ply}'
        assert row['before_fen'] == board.fen(), f'Broken FEN chain at ply {ply}'
        assert row['side'] == ('white' if board.turn else 'black')
        assert row['plan_id'] not in plan_ids and row['capture_id'] not in capture_ids
        plan_ids.add(row['plan_id']); capture_ids.add(row['capture_id'])
        move = chess.Move.from_uci(row['uci'])
        assert move in board.legal_moves, f'Illegal move at ply {ply}'
        assert row['san'] == board.san(move), f'Incorrect SAN at ply {ply}'
        captures += board.is_capture(move)
        castlings += board.is_castling(move)
        en_passants += board.is_en_passant(move)
        promotions += bool(move.promotion)
        board.push(move)
        assert row['after_fen'] == board.fen(), f'Incorrect committed FEN at ply {ply}'
    outcome = board.outcome(claim_draw=True)
    assert outcome is not None and outcome.result() == summary['result']
    game = chess.pgn.Game.from_board(board)
    game.headers.update(Event='Chess Brain logic-only Stockfish self-play',
                        White='Stockfish ' + summary['white_preset'],
                        Black='Stockfish ' + summary['black_preset'], Result=outcome.result())
    (directory / 'game.pgn').write_text(str(game) + '\n')
    report = {'passed': True, 'plies': len(records), 'result': outcome.result(),
              'termination': outcome.termination.name, 'captures': captures,
              'castlings': castlings, 'en_passant': en_passants, 'promotions': promotions,
              'all_moves_legal': True, 'all_san_correct': True, 'all_fen_transitions_match': True,
              'unique_plan_ids': True, 'unique_vision_capture_ids': True,
              'no_moves_after_terminal_state': True, 'final_fen': board.fen(),
              'scope': 'Logic only; execution and camera observations are synthetic.'}
    (directory / 'audit.json').write_text(json.dumps(report, indent=2) + '\n')
    return report


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory', type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.directory), indent=2))
