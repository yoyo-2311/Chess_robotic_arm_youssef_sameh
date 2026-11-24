"""Single turn owner. No Qt/ROS dependencies and never a speculative board update."""
import time
import uuid


class MatchCoordinator:
    def __init__(self):
        self.reset()

    def reset(self, human='white', enabled=False):
        self.human=human
        self.enabled=enabled
        self.state=None
        self.inflight=None
        self.requested=set()
        self.paused=''
        self.history=[]
        self.plans={}
        self.last_ply=None
        self.last_fen=None
        self.session_seen=False
        self.active_plan=None
        self.completed_plan_ids=set()
        self.stage=''
        self.operation=None
        self.progress_at=0.
        self.vision_since=None
        self.wait_notice=''
        self.timeout_queried=False
        self.rejected_plan_ids=set()
        self.recoverable_pause=None
        self.rejection_cleared_at=0.
        self.last_rejection_notice=''

    def observe_plan(self, plan):
        if plan['plan_id'] in self.rejected_plan_ids:return
        self.plans[plan['plan_id']]=plan
        if plan['plan_id'] not in self.completed_plan_ids:
            self.active_plan=plan
            self.last_rejection_notice=''
            self.progress_at=time.monotonic()
            self.stage='Motion plan queued'
            if self.inflight:self.inflight['plan_id']=plan['plan_id']

    def observe_state(self, state):
        ply=state.get('committed_plies',0)
        if self.last_ply is None:
            if ply != 0:
                self.paused='Existing game detected. Restart the simulated mode; no automatic replay on reconnect.'
            self.session_seen=True
        elif ply < self.last_ply or (ply==self.last_ply and self.last_fen and state.get('fen')!=self.last_fen):
            self.paused='Confirmed state reset or changed unexpectedly. Restart the mode.'
        elif ply > self.last_ply:
            plan=next((p for p in self.plans.values() if p.get('expected_fen')==state.get('fen')),None)
            if ply != self.last_ply+1 or not plan:
                self.paused='Unrecognized confirmed transition. Automatic progression paused.'
            else:
                self.history.append({'ply':ply,'uci':plan['uci'],'san':plan.get('san',plan['uci']),
                                     'side':plan['side'],'arm':'robot1' if plan['side']=='white' else 'robot2'})
                self.inflight=None
                self.completed_plan_ids.add(plan['plan_id'])
                if self.active_plan and self.active_plan['plan_id']==plan['plan_id']:
                    self.active_plan=None;self.operation=None;self.stage=''
                self.vision_since=None;self.wait_notice='';self.timeout_queried=False
        self.last_ply,self.last_fen=ply,state.get('fen')
        self.state=state
        rejection=state.get('last_rejection') or {}
        pid=rejection.get('plan_id');plan=self.plans.get(pid)
        if (pid and pid not in self.rejected_plan_ids and plan
                and rejection.get('status')=='rejected_before_activity'
                and rejection.get('before_fen')==plan.get('before_fen')==state.get('fen')
                and state.get('pending_plan_id') is None and not state.get('robot_ack_received')
                and state.get('synchronized') and not state.get('blocked')
                and (not self.active_plan or self.active_plan['plan_id']==pid)
                and (not self.inflight or self.inflight.get('plan_id')==pid)):
            self.rejected_plan_ids.add(pid)
            self.rejection_cleared_at=time.monotonic()
            self.inflight=None;self.active_plan=None;self.operation=None;self.stage=''
            description=f"{plan.get('side','').title()} / {'robot1' if plan.get('side')=='white' else 'robot2'} {plan.get('uci','')}"
            self.last_rejection_notice=description+' • Rejected before any arm/gripper activity: '+rejection.get('reason','')+('. Choose another legal move.' if plan.get('side')==self.human else '. Stockfish automatic retry disabled; use explicit recovery.')
            if not self.paused or self.paused==self.recoverable_pause:
                self.paused='' if plan.get('side')==self.human else self.last_rejection_notice
            self.recoverable_pause=None
            self.wait_notice='';self.timeout_queried=False
        if state.get('robot_ack_received') and self.vision_since is None:self.vision_since=time.monotonic()
        if state.get('blocked') and not self.paused:
            self.paused=state.get('block_reason') or 'Chess Brain blocked; reconcile and restart the simulated mode.'

    def gate(self, ready):
        s=self.state
        return bool(ready and self.enabled and not self.paused and not self.inflight and s and
                    s.get('synchronized') and not s.get('blocked') and not s.get('result') and
                    s.get('pending_plan_id') is None and not s.get('robot_ack_received'))

    def can_human(self, ready):
        return self.gate(ready) and self.state.get('turn')==self.human

    def human_move(self, uci, ready):
        if not self.can_human(ready):
            raise ValueError(self.paused or 'Wait for confirmed human turn and backend readiness.')
        # Legality remains authoritative in the brain. Use its advertised legal moves for UX.
        if uci not in self.state.get('legal_moves',[]):
            raise ValueError('Not a legal move in the confirmed position.')
        return self._request('uci',uci)

    def next_reply(self, ready):
        if not self.gate(ready) or self.state.get('turn')==self.human:
            return None
        ply=self.state['committed_plies']
        if ply in self.requested:
            return None
        self.requested.add(ply)  # Before publishing: duplicate callbacks/reconnects cannot retry it.
        return self._request('next')

    def _request(self, action, uci=None):
        command={'action':action,'request_id':uuid.uuid4().hex}
        if uci:command['uci']=uci
        self.inflight={'request_id':command['request_id'],'ply':self.state['committed_plies'],'sent':time.monotonic(),
                       'side':self.state.get('turn'),'uci':uci,'action':action,'accepted':False}
        self.wait_notice='';self.timeout_queried=False;self.last_rejection_notice=''
        return command

    def context(self):
        plan=self.active_plan
        if plan:
            side=plan.get('side','');uci=plan.get('uci','')
        elif self.inflight:
            side=self.inflight.get('side','');uci=self.inflight.get('uci') or 'Stockfish reply'
        else:return ''
        arm='robot1' if side=='white' else 'robot2'
        return f'{side.title()} / {arm} {uci}'

    def progress_text(self):
        context=self.context()
        operation=self.operation.get('operation','') if self.operation else ''
        detail=' • '.join(s for s in [self.stage,operation] if s)
        if self.state and self.state.get('robot_ack_received'):
            detail='Robot finished; waiting for fresh vision confirmation (board unchanged)'
        elif self.inflight and not self.active_plan:
            detail='Waiting for Chess Brain acceptance / move plan'
        return ' • '.join(s for s in [context,detail] if s)

    def event(self, event):
        if event.get('event')=='command_error':
            self.paused=(self.context()+' • ' if self.context() else '')+event.get('error','Chess command error')
        if self.inflight and event.get('request_id')==self.inflight['request_id']:
            if event.get('ok') is False:
                self.paused=self.context()+' • Command rejected: '+event.get('status','unknown reason')
            elif event.get('event')=='command_response':
                self.inflight['accepted']=True
                self.inflight['plan_id']=event.get('plan_id')
        # Acceptance and robot acknowledgements NEVER clear the in-flight turn.

    def motion(self, status):
        now=time.monotonic();kind=status.get('status')
        pid=status.get('plan_id') or status.get('request_id')
        if pid in self.completed_plan_ids or pid in self.rejected_plan_ids:return
        if pid in self.plans:self.active_plan=self.plans[pid]
        if kind in ['accepted','preflight','stage','descent_checked','operation','operator_required']:
            self.progress_at=now
        if kind=='stage':self.stage=status.get('stage','').replace('_',' ')
        elif kind=='preflight':self.stage='Checking reachability and collision scene'
        elif kind=='operator_required':
            self.stage='Operator confirmation required'
            self.operation={'operation':self.stage,'deadline':now+float(status.get('timeout_seconds',120.))+3}
        elif kind=='operation':
            if status.get('phase')=='waiting':
                self.operation={**status,'deadline':now+float(status.get('timeout_seconds',30.))+3.}
            elif self.operation and status.get('operation_id')==self.operation.get('operation_id'):
                self.operation=None
        elif kind in ['succeeded','execution_complete','awaiting_vision']:
            self.operation=None;self.stage='Waiting for robot acknowledgement / fresh vision'
        if kind in ['failed','estop','initialization_failed','geometry_rollback_failed'] or kind=='rejected' and pid in self.plans:
            context=self.context()
            stage=(status.get('stage') or self.stage).replace('_',' ')
            reason=status.get('reason',kind).replace('no_collision_free_descent:', 'No collision-free descent:',1)
            if status.get('reconciliation_required') is False:
                reason='Rejected before arm movement • '+reason
            elif status.get('reconciliation_required') is True:
                reason='Execution stopped; reconciliation required • '+reason
            self.paused=' • '.join(str(s) for s in [context,stage,reason] if s)
            self.operation=None
            if kind=='rejected' and status.get('safe_rejection') is True:self.recoverable_pause=self.paused

    def watch(self, now=None):
        """Report missing results, query authoritative state once, NEVER resend.

        Heartbeats do not reset progress deadlines. A delayed vision commit can
        reconcile a known plan normally; an ambiguous command/operation timeout
        stays paused even if a snapshot says idle, until explicit recovery.
        """
        now=time.monotonic() if now is None else now
        if self.paused or not self.enabled:return None
        reason=''
        if self.inflight and not self.active_plan and now-self.inflight['sent']>10.:
            reason='Command / move-plan acknowledgement timeout; outcome unknown. Do not resend; reconcile state and restart simulated mode.'
        elif self.state and self.state.get('pending_plan_id'):
            if self.state.get('robot_ack_received'):
                if self.vision_since is not None and now-self.vision_since>30.:
                    self.wait_notice=self.context()+' • Vision confirmation delayed; confirmed board unchanged. No retry while pending.'
            elif self.operation and now>self.operation['deadline']:
                reason=self.operation.get('operation','Motion operation')+' result timeout; outcome unknown. Reconcile state and restart simulated mode.'
            elif not self.operation and self.progress_at and now-self.progress_at>90.:
                reason='Motion progress timeout; outcome unknown. Reconcile state and restart simulated mode.'
        if reason:self.paused=self.context()+' • '+reason
        if (reason or self.wait_notice) and not self.timeout_queried:
            self.timeout_queried=True
            return {'action':'state'}
        return None

    def connection_lost(self):
        if not self.paused:
            self.paused='Backend heartbeat lost. Automatic progression remains paused; restart the simulated mode.'
