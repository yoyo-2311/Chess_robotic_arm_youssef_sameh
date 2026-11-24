from dual_scara_desktop.modes import MODES,launch_argv,SCENARIOS
from dual_scara_desktop.environment import local_environment


def test_paths_are_arguments_not_shell_commands_and_turn_owners_are_exclusive():
    for mode in MODES:
        args=launch_argv(mode,'/tmp/a profile;literal.yaml',False,'/usr/games/stockfish','Standard game')
        assert args[:2]==['ros2','launch']
        if mode.key!='logic':assert 'geometry_file:=/tmp/a profile;literal.yaml' in args
        if mode.key=='logic':assert 'auto_coordinator_enabled:=false' in args
        if mode.key=='mock':assert 'initial_fen:='+SCENARIOS['Standard game'] in args


def test_communication_is_local_only():
    env=local_environment(211,{'ROS_STATIC_PEERS':'remote.example'})
    assert env['ROS_AUTOMATIC_DISCOVERY_RANGE']=='LOCALHOST' and env['ROS_LOCALHOST_ONLY']=='1'
    assert 'ROS_STATIC_PEERS' not in env and env['GZ_IP']=='127.0.0.1'


def test_special_scenarios_reuse_brain_multi_operation_plans():
    from chess_brain_handover.core import chess,describe_move
    for name,fen in SCENARIOS.items():
        if name=='Standard game':continue
        move=chess.Move.from_uci(name.split()[-1]);board=chess.Board(fen)
        assert move in board.legal_moves
        actions=describe_move(board,move).to_dict()['actions']
        assert len(actions)==(1 if name.startswith('After c4 e5') else 2)
        if name.startswith('After c4 e5'):assert board.piece_at(chess.H2).symbol()=='P'
        if name.startswith(('Capture','En passant')):assert actions[0]['kind']=='remove'
        if name.startswith('Castling'):assert [a['kind'] for a in actions]==['move','move']
        if name.startswith('Promotion'):assert actions[-1]['kind']=='promote'
