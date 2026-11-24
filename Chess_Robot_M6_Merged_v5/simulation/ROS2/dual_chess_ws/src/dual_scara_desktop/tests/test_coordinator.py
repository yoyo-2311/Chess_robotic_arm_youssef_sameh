import pytest
from dual_scara_desktop.coordinator import MatchCoordinator


def state(ply=0,turn='white',fen='initial',pending=None,ack=False,blocked=False):
    return {'committed_plies':ply,'turn':turn,'fen':fen,'pending_plan_id':pending,'robot_ack_received':ack,
            'synchronized':True,'blocked':blocked,'result':None,'legal_moves':['e2e4','e7e5']}

def commit(c,ply,turn,fen,uci,side):
    c.observe_plan({'plan_id':'p'+str(ply),'expected_fen':fen,'uci':uci,'side':side})
    c.observe_state(state(ply,turn,fen))


def test_white_human_reply_only_after_confirmed_transition_and_once():
    c=MatchCoordinator();c.reset('white',True);c.observe_state(state())
    assert c.next_reply(True) is None
    command=c.human_move('e2e4',True)
    with pytest.raises(ValueError):c.human_move('e2e4',True)
    c.event({'event':'command_response','request_id':command['request_id'],'ok':True})
    c.observe_state(state(pending='p1',ack=True))
    assert c.next_reply(True) is None and c.history==[]
    commit(c,1,'black','after-human','e2e4','white')
    assert c.next_reply(False) is None
    reply=c.next_reply(True);assert reply['action']=='next'
    assert c.next_reply(True) is None
    c.observe_state(state(1,'black','after-human'))
    assert c.next_reply(True) is None
    commit(c,2,'white','after-engine','e7e5','black')
    assert c.can_human(True) and c.next_reply(True) is None
    assert [h['arm'] for h in c.history]==['robot1','robot2']


def test_black_human_gets_exactly_one_opening_white_move():
    c=MatchCoordinator();c.reset('black',True);c.observe_state(state())
    assert c.next_reply(True)['action']=='next'
    for _ in range(10):c.observe_state(state());assert c.next_reply(True) is None
    commit(c,1,'black','opened','e2e4','white');assert c.can_human(True)
    c.human_move('e7e5',True);commit(c,2,'white','human-black','e7e5','black')
    assert c.next_reply(True)['action']=='next'
    assert c.requested=={0,2}


def test_failed_execution_or_stop_never_requests_reply():
    c=MatchCoordinator();c.reset('white',True);c.observe_state(state());c.human_move('e2e4',True)
    c.motion({'status':'failed','reason':'gripper_operation_failed'})
    assert not c.can_human(True) and c.next_reply(True) is None and c.history==[]
    assert c.state['fen']=='initial'


def test_delayed_vision_cannot_advance_history_or_turn():
    c=MatchCoordinator();c.reset('white',True);c.observe_state(state());c.human_move('e2e4',True)
    c.observe_plan({'plan_id':'p','expected_fen':'after','uci':'e2e4','side':'white'})
    for _ in range(5):c.observe_state(state(pending='p',ack=True));assert c.next_reply(True) is None
    assert c.last_fen=='initial' and not c.history
    c.observe_state(state(1,'black','after'))
    assert c.next_reply(True) is not None


def test_reconnect_and_rejected_command_are_not_retried_automatically():
    c=MatchCoordinator();c.reset('black',True);c.observe_state(state());request=c.next_reply(True)
    c.connection_lost();c.observe_state(state());assert c.next_reply(True) is None
    c.event({'event':'command_response','request_id':request['request_id'],'ok':False,'status':'waiting_for_safety_ready'})
    assert c.next_reply(True) is None and c.requested=={0}
    fresh=MatchCoordinator();fresh.reset('white',True);fresh.observe_state(state(5,'black','existing'))
    assert fresh.paused and fresh.next_reply(True) is None


def test_game_over_and_foreign_confirmations_pause_progression():
    c=MatchCoordinator();c.reset('black',True);s=state();s['result']='1-0';c.observe_state(s);assert c.next_reply(True) is None
    c=MatchCoordinator();c.reset('white',True);c.observe_state(state());c.observe_state(state(1,'black','foreign'))
    assert c.paused and c.next_reply(True) is None


def test_rejection_cause_survives_generic_blocked_state_and_busy_heartbeat():
    c=MatchCoordinator();c.reset('white',True);c.observe_state(state());c.human_move('e2e4',True)
    c.observe_plan({'plan_id':'p','expected_fen':'after','uci':'g1f3','side':'white'})
    c.motion({'status':'stage','plan_id':'p','stage':'pickup_preflight'})
    c.motion({'status':'rejected','plan_id':'p','reason':'no_collision_free_descent: chess_piece_h2'})
    c.observe_state(state(pending='p',blocked=True))
    c.motion({'status':'busy','plan_id':'p'})
    assert 'robot1 g1f3' in c.paused and 'chess_piece_h2' in c.paused and 'pickup preflight' in c.paused
    assert not c.can_human(True) and c.next_reply(True) is None


def test_missing_acceptance_queries_state_once_without_retry_or_unlocking():
    c=MatchCoordinator();c.reset('white',True);c.observe_state(state());command=c.human_move('e2e4',True)
    deadline=c.inflight['sent']+11
    assert c.watch(deadline)=={'action':'state'}
    assert 'timeout' in c.paused and c.watch(deadline+10) is None
    c.observe_state(state())  # An idle snapshot alone cannot prove a command was never accepted.
    assert c.inflight['request_id']==command['request_id'] and not c.can_human(True)
    assert c.next_reply(True) is None


def test_busy_heartbeats_do_not_extend_missing_controller_result_deadline():
    c=MatchCoordinator();c.reset('white',True);c.observe_state(state());c.human_move('e2e4',True)
    c.observe_plan({'plan_id':'p','expected_fen':'after','uci':'e2e4','side':'white'})
    c.observe_state(state(pending='p'))
    c.motion({'status':'operation','plan_id':'p','phase':'waiting','operation':'controller result','operation_id':'o','timeout_seconds':20})
    c.stage='transfer'
    assert 'transfer • controller result' in c.progress_text()
    deadline=c.operation['deadline']
    for _ in range(20):c.motion({'status':'busy','plan_id':'p'})
    assert c.operation['deadline']==deadline
    assert c.watch(deadline+1)=={'action':'state'}
    assert 'controller result' in c.paused and not c.can_human(True)


def test_late_vision_reconciles_known_plan_without_repeating_dispatch():
    c=MatchCoordinator();c.reset('white',True);c.observe_state(state());c.human_move('e2e4',True)
    c.observe_plan({'plan_id':'p','expected_fen':'after','uci':'e2e4','side':'white'})
    c.observe_state(state(pending='p',ack=True))
    assert c.watch(c.vision_since+31)=={'action':'state'}
    assert 'Vision confirmation delayed' in c.wait_notice and not c.paused
    assert not c.can_human(True) and c.next_reply(True) is None
    c.observe_state(state(1,'black','after'))
    assert not c.wait_notice and c.next_reply(True)['action']=='next'
    assert c.next_reply(True) is None


def test_stockfish_failure_context_identifies_black_arm_after_white_commit():
    c=MatchCoordinator();c.reset('white',True);c.observe_state(state())
    commit(c,1,'black','after-white','g1f3','white');c.next_reply(True)
    c.observe_plan({'plan_id':'black','expected_fen':'reply','uci':'g8f6','side':'black'})
    c.motion({'status':'stage','plan_id':'black','stage':'transfer'})
    c.motion({'status':'failed','plan_id':'black','reason':'controller_execution_failed'})
    assert 'Black / robot2 g8f6' in c.paused and 'transfer' in c.paused
    assert c.last_fen=='after-white' and len(c.history)==1


def test_safe_rejection_ack_never_unlocks_without_authoritative_pending_clear():
    c=MatchCoordinator();c.reset('white',True);c.observe_state(state())
    c.human_move('e2e4',True)
    c.observe_plan({'plan_id':'p','before_fen':'initial','expected_fen':'after','uci':'e2e4','side':'white'})
    c.motion({'status':'rejected','plan_id':'p','reason':'no_collision_free_descent','safe_rejection':True,'reconciliation_required':False})
    assert not c.can_human(True)
    c.observe_state(state(pending='p'))
    assert not c.can_human(True)
    s=state();s['last_rejection']={'status':'rejected_before_activity','plan_id':'p','before_fen':'initial','reason':'blocked by pawn'}
    c.observe_state(s)
    assert c.can_human(True) and not c.inflight and c.rejection_cleared_at
    assert c.next_reply(True) is None and not c.history
    c.motion({'status':'rejected','plan_id':'p','reason':'late duplicate'})
    assert not c.paused and c.can_human(True)
    request=c.human_move('e2e4',True)
    c.observe_state(s)  # An old rejection snapshot must not discard a new request.
    assert c.inflight['request_id']==request['request_id'] and not c.can_human(True)


def test_safe_rejection_does_not_clear_unrelated_uncertainty_pause():
    c=MatchCoordinator();c.reset('white',True);c.observe_state(state());c.human_move('e2e4',True)
    c.observe_plan({'plan_id':'p','before_fen':'initial','expected_fen':'after','uci':'e2e4','side':'white'})
    c.motion({'status':'rejected','plan_id':'p','reason':'collision','safe_rejection':True})
    c.connection_lost();c.paused='Software stop remains latched'
    s=state();s['last_rejection']={'status':'rejected_before_activity','plan_id':'p','before_fen':'initial'}
    c.observe_state(s)
    assert c.paused=='Software stop remains latched' and not c.can_human(True)


def test_safe_engine_rejection_keeps_exact_cause_and_never_auto_retries():
    c=MatchCoordinator();c.reset('black',True);c.observe_state(state());c.next_reply(True)
    c.observe_plan({'plan_id':'p','before_fen':'initial','expected_fen':'after','uci':'g1f3','side':'white'})
    c.motion({'status':'rejected','plan_id':'p','reason':'pawn contact','safe_rejection':True})
    s=state();s['last_rejection']={'status':'rejected_before_activity','plan_id':'p','before_fen':'initial','reason':'pawn contact'}
    c.observe_state(s)
    assert 'White / robot1 g1f3' in c.paused and 'pawn contact' in c.paused
    assert c.requested=={0} and c.next_reply(True) is None
