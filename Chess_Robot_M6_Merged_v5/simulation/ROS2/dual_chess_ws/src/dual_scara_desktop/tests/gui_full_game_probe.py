"""Opt-in complete-game Qt/ROS probe; never fabricates robot acknowledgements.

scripted: a deterministic four-ply game using the existing full mock mode.
stockfish: an external easy engine drives only the human controls; the desktop
remains the sole owner of automatic opponent progression.
"""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
from pathlib import Path
import time

from PySide6.QtCore import Qt, QTimer
from PySide6.QtTest import QTest
from PySide6.QtWidgets import QApplication
from ament_index_python.packages import get_package_share_directory
from chess_brain_handover.core import chess, StockfishPlayer
from dual_scara_desktop.controller import DesktopController
from dual_scara_desktop.modes import MODES
from dual_scara_desktop.window import MainWindow


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--workspace', required=True)
    parser.add_argument('--domain', type=int, default=219)
    parser.add_argument('--mode', choices=['scripted', 'stockfish'], default='scripted')
    parser.add_argument('--max-plies', type=int, default=400)
    args = parser.parse_args()
    output = Path(args.workspace) / 'docs/full_game_validation'
    output.mkdir(parents=True, exist_ok=True)
    app = QApplication([])
    app.setQuitOnLastWindowClosed(False)
    profile = Path(get_package_share_directory('dual_chess_description')) / 'config/chess_geometry.yaml'
    w = MainWindow(profile, args.domain)
    c = DesktopController(w, args.workspace, args.domain)
    w.rviz.setChecked(False)
    key = 'mock' if args.mode == 'scripted' else 'stockfish'
    w.mode.setCurrentIndex(next(i for i, m in enumerate(MODES) if m.key == key))
    w.show()
    pulses = []
    timer = QTimer()
    timer.setInterval(20)
    timer.timeout.connect(lambda: pulses.append(time.monotonic()))
    timer.start()
    events, acks, statuses, transitions = [], {}, [], []
    original = c.consume
    def consume(kind, data, stamp):
        if kind == 'event': events.append(data)
        if kind == 'ack': acks[data.get('plan_id')] = data
        if kind == 'motion': statuses.append(data)
        original(kind, data, stamp)
    c.consume = consume
    report = {'mode': args.mode, 'complete': False, 'result': None}
    pool = ThreadPoolExecutor(max_workers=1)
    engine = None
    reference = chess.Board()
    started = time.monotonic()
    def until(predicate, timeout=180):
        deadline = time.monotonic() + timeout
        last_log = 0
        while not predicate():
            app.processEvents()
            if c.policy.paused:
                raise AssertionError(c.policy.paused)
            if c.lifecycle in ['failed', 'stopped']:
                raise AssertionError(c.last_error or 'Backend stopped')
            if time.monotonic() > deadline:
                raise AssertionError('Timeout: ' + w.status.text())
            if time.monotonic() - last_log > 10:
                print(w.status.text(), '|', w.pending.text(), flush=True)
                last_log = time.monotonic()
            time.sleep(.015)
    def observe_commits():
        history = c.policy.history
        while len(transitions) < len(history):
            item = history[len(transitions)]
            move = chess.Move.from_uci(item['uci'])
            assert move in reference.legal_moves, item
            assert item['arm'] == ('robot1' if reference.turn else 'robot2'), item
            assert item['san'] == reference.san(move), item
            reference.push(move)
            transitions.append(dict(item))
        assert reference.fen() == c.data['state']['fen'] == w.board.fen
        assert len(transitions) == c.data['state']['committed_plies']
    try:
        QTest.mouseClick(w.start, Qt.LeftButton)
        until(lambda: c.policy.can_human(c.ready))
        if args.mode == 'stockfish':
            future = pool.submit(StockfishPlayer, 'easy', '/usr/games/stockfish')
            until(future.done)
            engine = future.result()
        script = ['f2f3', 'e7e5', 'g2g4', 'd8h4']
        while not c.data['state'].get('result'):
            observe_commits()
            assert len(transitions) < args.max_plies, 'Ply cutoff reached; this is not a completed game'
            until(lambda: c.policy.can_human(c.ready) or c.data['state'].get('result'))
            if c.data['state'].get('result'): break
            before = c.data['state']['committed_plies']
            if engine:
                snapshot = reference.copy(stack=True)
                future = pool.submit(engine.choose_move, snapshot)
                until(future.done)
                uci = future.result()[0].uci()
            else:
                assert before < len(script), 'Script exhausted without a terminal result'
                uci = script[before]
            assert uci in c.data['state']['legal_moves']
            print('Qt submits', uci, 'at confirmed ply', before, flush=True)
            w.uci.setText(uci)
            QTest.mouseClick(w.submit, Qt.LeftButton)
            until(lambda: c.data['state']['committed_plies'] > before or c.data['state'].get('result'))
            observe_commits()
            if engine:
                until(lambda: c.policy.can_human(c.ready) or c.data['state'].get('result'))
                observe_commits()
        observe_commits()
        until(lambda: c.ready, 20)
        # Drain cross-topic events and prove game-over does not trigger another request.
        requests = set(c.policy.requested)
        end = time.monotonic() + 2
        while time.monotonic() < end: app.processEvents(); time.sleep(.02)
        assert requests == c.policy.requested
        assert not w.submit.isEnabled() and not w.board.interactive
        assert reference.outcome(claim_draw=True).result() == c.data['state']['result']
        successful = {pid for pid, ack in acks.items() if ack.get('success')}
        assert len(successful) == len(transitions)
        confirmations = [e for e in events if e.get('event') == 'vision_confirmation' and e.get('status') == 'committed']
        assert len(confirmations) == len(transitions)
        report.update(complete=True, result=c.data['state']['result'],
                      termination=reference.outcome(claim_draw=True).termination.name,
                      successful_robot_acks=len(successful), fresh_vision_commits=len(confirmations),
                      game_over_disables_input=True, no_extra_engine_request_after_game_over=True)
    except Exception as exc:
        report['error'] = str(exc)
        report['recent_motion_status'] = statuses[-12:]
        report['recent_events'] = events[-12:]
        # A physical failure must not advance the independently replayed confirmed board.
        observe_commits()
        report['confirmed_board_unchanged_by_failed_pending_move'] = bool(c.data.get('state', {}).get('pending_plan_id'))
        print('GAME TEST BLOCKED:', exc, flush=True)
    finally:
        report.update(confirmed_plies=len(transitions), history=transitions,
                      final_state=c.data.get('state'), automatic_reply_ply_keys=sorted(c.policy.requested),
                      elapsed_seconds=round(time.monotonic()-started, 2), launch_log=c.log_path,
                      maximum_qt_timer_gap_seconds=max((b-a for a,b in zip(pulses,pulses[1:])), default=0))
        w.grab().save(str(output / (args.mode + '.png')))
        c.close()
        deadline = time.monotonic() + 25
        while (c.supervisor.alive() or c.bridge.alive()) and time.monotonic() < deadline:
            app.processEvents(); time.sleep(.03)
        report['clean_shutdown'] = not c.supervisor.alive() and not c.bridge.alive()
        if engine: engine.close()
        pool.shutdown(wait=True)
        (output / (args.mode + '.json')).write_text(json.dumps(report, indent=2))
        print(json.dumps(report, indent=2), flush=True)
    return 0 if report['complete'] and report['clean_shutdown'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
