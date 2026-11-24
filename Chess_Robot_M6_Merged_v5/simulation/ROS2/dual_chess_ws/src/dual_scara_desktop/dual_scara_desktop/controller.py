"""Application orchestration outside widgets: mode lifecycle, readiness and turn ownership."""
import json
import os
from pathlib import Path
import time
import uuid
from PySide6.QtCore import QObject,QTimer,Signal
from PySide6.QtWidgets import QMessageBox
from .coordinator import MatchCoordinator
from .modes import MODES,launch_argv,START_FEN
from .ros_bridge import RosBridge
from .supervisor import ProcessSupervisor
from .environment import configure_process


class DesktopController(QObject):
    closed=Signal()
    def __init__(self,window,workspace,domain,headless_gazebo=False):
        super().__init__();self.w=window;self.workspace=Path(workspace);self.domain=domain;self.headless_gazebo=headless_gazebo
        configure_process(domain)
        self.bridge=RosBridge(domain);self.supervisor=ProcessSupervisor(self.workspace/'.desktop-state',domain)
        self.policy=MatchCoordinator();self.lifecycle='stopped';self.mode=None;self.generation=0;self.pending_start=None
        self.data={};self.arrival={};self.graph={};self.selected=None;self.operator=None;self.square_id=None;self.stage_info=None
        self.initialized=False;self.ready=False;self.idle_since=None;self.software_stopped=False;self.closing=False;self.commit_arrival=0;self.last_position_key=None
        self.last_operator_token=None;self.owned_pid=None;self.parameter_pending=set();self.bridge_failed=False;self.component_failed=False;self.configuration=None;self.last_error='';self.log_path=''
        self.timer=QTimer(self);self.timer.setInterval(50);self.timer.timeout.connect(self.tick);self.timer.start()
        window.start_requested.connect(self.start_selected);window.stop_requested.connect(self.stop)
        window.close_requested.connect(self.close);window.move_requested.connect(self.human_move)
        window.board.square_clicked.connect(self.square_clicked)
        window.mode.currentIndexChanged.connect(self.selection_changed);window.arm.currentTextChanged.connect(self.refresh_joints)
        window.square_requested.connect(self.square_command);window.joint_requested.connect(self.joint_changed)
        window.apply_joints_requested.connect(self.gazebo_apply);window.stop_motion_requested.connect(self.motion_stop)
        window.restart_requested.connect(self.restart);window.failure_requested.connect(self.fail_gripper)
        window.vision_hold_requested.connect(self.hold_vision);window.promotion_requested.connect(self.confirm_promotion)
        window.sync_requested.connect(lambda:self.publish('/chess/command',{'action':'sync'}))
        window.state_requested.connect(lambda:self.publish('/chess/command',{'action':'state'}))
        self.update_controls()

    def log(self,text):self.w.log.appendPlainText(str(text))
    def error(self,text):
        self.last_error=str(text);self.w.status.setText('Error • '+str(text));self.log('ERROR: '+str(text))
    def selection_changed(self):
        mode=MODES[self.w.mode.currentIndex()];self.w.description.setText(mode.explanation);self.update_controls()

    def capture_configuration(self):
        from chess_motion_interface.geometry import Geometry
        mode=MODES[self.w.mode.currentIndex()]
        profile=Path(self.w.profile.text()).expanduser().resolve()
        geometry=Geometry.load(profile) if mode.key!='logic' else None
        initial={f'{arm}_joint{i}':v for arm in ['robot1','robot2'] for i,v in enumerate(geometry.home(arm),1)} if geometry else {}
        engine=Path(self.w.engine_path.text()).expanduser().resolve()
        if mode.key in ['stockfish','logic'] and not os.access(engine,os.X_OK):raise ValueError('Stockfish executable not found: '+str(engine))
        return {'mode':mode,'profile':str(profile),'engine':str(engine),'rviz':self.w.rviz.isChecked(),
                'human':self.w.human.currentData(),'scenario':self.w.scenario.currentText(),'initial_joints':initial}

    def start_selected(self):
        try:config=self.capture_configuration()
        except Exception as exc:self.error('Cannot start: '+str(exc));return
        if self.lifecycle not in ['stopped','failed']:
            if self.lifecycle=='stopping':self.pending_start=config;return
            if self.mode and self.mode.key==config['mode'].key:return
            self.pending_start=config;self.stop();return
        self.begin_start(config)

    def begin_start(self,config):
        self.configuration=config;self.mode=config['mode'];self.lifecycle='checking';self.started_at=time.monotonic()
        self.clear_since=None
        self.generation+=1;self.policy.reset(config['human'],enabled=self.mode.key in ['stockfish','mock'])
        self.data={};self.arrival={};self.ready=False;self.initialized=False;self.idle_since=None;self.software_stopped=False;self.selected=None;self.operator=None;self.square_id=None;self.stage_info=None
        self.last_position_key=None;self.last_error='';self.component_failed=False;self.last_operator_token=None;self.parameter_pending.clear();self.w.history.clear()
        self.w.hold_vision.blockSignals(True);self.w.hold_vision.setChecked(False);self.w.hold_vision.blockSignals(False)
        self.w.board.set_position(START_FEN,flipped=config['human']=='black');self.w.pending.setText('Waiting for the selected backend…')
        self.w.status.setText('Checking ROS domain ownership…');self.update_controls()

    def stop(self):
        if self.lifecycle=='stopped':return
        self.policy.enabled=False;self.ready=False;self.lifecycle='stopping'
        self.bridge.submit('deactivate');self.supervisor.stop();self.w.status.setText('Stopping app-managed mode…');self.update_controls()

    def close(self):
        if self.closing:return
        self.closing=True;self.pending_start=None;self.policy.enabled=False;self.ready=False
        self.bridge.submit('deactivate');self.supervisor.shutdown();self.lifecycle='closing';self.w.status.setText('Closing • waiting for owned ROS processes to stop…');self.update_controls()

    def publish(self,topic,payload):
        if self.lifecycle not in ['starting','running']:return
        self.bridge.submit('publish',{'generation':self.generation,'topic':topic,'payload':payload})

    def tick(self):
        for _ in range(300):
            try:generation,kind,data,stamp=self.bridge.events.get_nowait()
            except __import__('queue').Empty:break
            if kind=='graph':self.graph=data
            if kind=='fatal':
                self.error(data);self.policy.connection_lost();self.ready=False;self.bridge_failed=True
                if self.lifecycle in ['starting','running','checking']:self.stop()
            if generation!=self.generation or self.lifecycle not in ['starting','running']:continue
            self.consume(kind,data,stamp)
        for _ in range(50):
            try:kind,value=self.supervisor.events.get_nowait()
            except __import__('queue').Empty:break
            if kind=='started':self.owned_pid=value['pid'];self.log_path=value['log'];self.log('Launch log: '+self.log_path)
            elif kind=='error':self.error(value);self.lifecycle='failed';self.bridge.submit('deactivate')
            elif kind=='stopping':self.w.status.setText(value)
            elif kind=='stopped':
                expected=self.lifecycle in ['stopping','closing']
                self.ready=False;self.initialized=False;self.bridge.submit('deactivate')
                if not expected:self.error(f'Backend exited (code {value.get("code")}). Inspect Logs / errors.');self.policy.connection_lost()
                if not self.closing:
                    self.lifecycle='stopped'
                    if self.pending_start:
                        config,self.pending_start=self.pending_start,None;self.begin_start(config)
                    elif expected:self.w.status.setText('Stopped • only app-managed processes were closed')
        for _ in range(80):
            try:log_generation,line=self.supervisor.logs.popleft()
            except IndexError:break
            # Full log is on disk; suppress repetitive controller loop noise in the visible pane.
            self.log(line)
            if log_generation==self.generation and self.lifecycle in ['starting','running'] and '[ERROR]' in line and 'process has died' in line:
                self.component_failed=True;self.policy.paused='A launched component exited. See Logs / errors and restart this mode.'
                self.error('Launched component failed: '+line)
        if self.lifecycle=='checking':
            conflicts=self.foreign_backends()
            if conflicts:
                self.clear_since=None
                self.w.status.setText('Waiting for ROS discovery to clear: '+', '.join(conflicts))
            elif self.graph and self.clear_since is None:self.clear_since=time.monotonic()
            if self.graph and not conflicts and self.clear_since is not None and time.monotonic()-self.clear_since>=1.:
                self.bridge.submit('activate',{'generation':self.generation,'mode':self.mode.key,'initial_joints':self.configuration.get('initial_joints',{})})
                argv=launch_argv(self.mode,self.configuration['profile'],self.configuration['rviz'],self.configuration['engine'],self.configuration['scenario'],self.headless_gazebo)
                self.log('Starting '+self.mode.title);self.lifecycle='starting';self.supervisor.start(argv,self.generation)
            elif time.monotonic()-self.started_at>45:
                self.error('ROS domain is occupied by '+', '.join(conflicts or ['unavailable ROS discovery'])+'. Stop that backend or use another --ros-domain-id.');self.lifecycle='failed'
        if self.lifecycle in ['starting','running']:
            self.ready,self.readiness_text=self.compute_readiness()
            if self.ready:
                self.lifecycle='running';self.initialized=True
            if self.lifecycle=='starting' and time.monotonic()-self.started_at>100:
                self.error('Backend readiness timed out. See launch log: '+self.log_path);self.stop()
            if self.mode.key in ['stockfish','mock']:
                if self.initialized and time.monotonic()-self.arrival.get('state',0)>6:self.policy.connection_lost()
                competitors=self.competitors()
                if competitors:self.policy.paused='Competing coordinator/command owner detected: '+', '.join(competitors)
                snapshot=self.policy.watch()
                if snapshot:
                    self.log('Waiting result deadline reached; requesting authoritative state without resending the move')
                    self.publish('/chess/command',snapshot)
                if self.mode.key=='stockfish':
                    command=self.policy.next_reply(self.ready and not self.square_id)
                    if command:self.log('Requesting exactly one Stockfish move for confirmed ply '+str(self.policy.state['committed_plies']));self.publish('/chess/command',command)
        self.update_controls()
        if self.closing and not self.supervisor.alive():
            if self.bridge.alive():self.bridge.shutdown()
            else:
                self.timer.stop();self.w.allow_close=True;self.w.close();self.closed.emit()

    def foreign_backends(self):
        critical={'robot_state_publisher','chess_brain','controller_manager','chess_motion_interface','joint_state_publisher','chess_auto_coordinator','move_group'}
        return sorted(set(self.graph.get('nodes',[])) & critical)

    def competitors(self):
        nodes=self.graph.get('nodes',[])
        result=[name for name in nodes if name=='chess_auto_coordinator']
        for key in ['chess_brain','controller_manager','chess_motion_interface','dual_scara_desktop_bridge']:
            if nodes.count(key)>1:result.append('duplicate '+key)
        result+=sorted({p for p in self.graph.get('command_publishers',[]) if p and not p.startswith('_NODE_NAME_UNKNOWN')}-{'dual_scara_desktop_bridge','mock_chess_vision'})
        return result

    def consume(self,kind,data,stamp):
        previous=self.data.get(kind);self.data[kind]=data;self.arrival[kind]=stamp
        if kind=='state':
            key=(data.get('committed_plies'),data.get('fen'),data.get('synchronized'))
            if key!=self.last_position_key:self.commit_arrival=stamp;self.last_position_key=key
            self.policy.observe_state(data)
            if self.policy.last_rejection_notice and not self.policy.inflight:
                self.w.execution_result.setText(self.policy.last_rejection_notice)
            if self.mode.key=='mock':self.policy.human=data.get('turn','white')
            self.w.history.clear()
            for h in self.policy.history:self.w.history.addItem(f"{h['ply']}. {h['san']}  ({h['uci']})  • {h['arm']}")
            last=self.policy.history[-1]['uci'] if self.policy.history else ''
            self.w.board.set_position(data.get('fen',''),self.selected,self.configuration['human']=='black',last)
        elif kind=='plan':self.policy.observe_plan(data);self.log('Move plan '+data.get('uci','')+' • '+data.get('side',''))
        elif kind=='event':
            self.policy.event(data)
            if self.mode.key=='logic' and data.get('action')=='sync':
                self.w.execution_result.setText('Chess sync: '+str(data.get('status',data.get('error',''))).replace('_',' '))
            if data.get('event') in ['command_error','robot_ack_error'] or data.get('ok') is False and data.get('action')!='sync':self.log(json.dumps(data))
            if data.get('event')=='vision_confirmation' and data.get('status')=='committed':self.w.execution_result.setText('Execution / confirmation: fresh vision committed the move')
        elif kind=='motion':
            self.policy.motion(data)
            status=data.get('status')
            if status=='ready':self.idle_since=None
            if status=='stage' or status=='operation' and data.get('phase')=='waiting':self.stage_info=data
            elif status in ['idle','succeeded','failed','estop','awaiting_vision','board_confirmed']:self.stage_info=None
            if status=='idle':
                if self.idle_since is None:self.idle_since=stamp
            else:self.idle_since=None
            if status=='operator_required':self.operator=data
            if status in ['execution_complete','succeeded','failed','estop','rejected'] and data.get('plan_id'):self.operator=None
            if self.square_id and data.get('request_id')==self.square_id and status in ['succeeded','failed','rejected']:
                self.square_id=None
                if status=='succeeded':
                    self.last_error=''
                    if data.get('executed'):self.w.execution_result.setText('Execution: completed on mock controllers with feedback')
                    else:self.w.plan_result.setText('Planning: succeeded • no movement')
                else:self.error(data.get('reason',status))
            if status not in ['busy','idle','initializing','awaiting_vision','blocked'] and data!=previous:self.log(json.dumps(data))
            if self.policy.paused and status in ['failed','rejected','estop']:
                self.w.execution_result.setText('Execution: '+self.policy.paused)
        elif kind=='ack':
            if data.get('plan_id') in self.policy.completed_plan_ids or data.get('plan_id') in self.policy.rejected_plan_ids:return
            self.w.execution_result.setText('Robot ack: success; awaiting fresh vision' if data.get('success') else 'Motion result: '+(self.policy.paused or data.get('reason','robot not safe')))
            if data.get('safe_rejection') is True:return  # Only authoritative state may unlock input.
            if data.get('success') is False and not self.policy.paused:self.policy.paused=data.get('reason','Robot execution failed')
        elif kind=='geometry':
            cfg=data.get('configuration',{});b=cfg.get('board',{})
            self.w.effective.setPlainText(f"Profile revision {data.get('revision')} • PLACEHOLDERS\n"+json.dumps({'board':b,'mounts':cfg.get('mounts')},indent=2))
        elif kind=='limits':self.refresh_joints()
        elif kind=='joints':
            for name,(spin,slider,label) in self.w.joint_controls.items():
                if name in data:
                    lim=self.data.get('limits',{}).get(name,{})
                    label.setText(f"{name.split('_',1)[1]} ({lim.get('unit','')}) • feedback {data[name]:.5f}")
        elif kind=='parameter_result':
            self.parameter_pending.discard(data.get('request_id'))
            self.log(('Applied parameter: '+str(data.get('name'))) if data.get('ok') else 'Parameter error: '+data.get('reason',''))
            if not data.get('ok'):
                self.error(data.get('reason','Parameter update failed'));self.policy.paused='Backend parameter result uncertain. Verify state and restart this simulated mode.'
        elif kind in ['log','conflict']:self.log(data)

    def compute_readiness(self):
        now=time.monotonic();s=self.data.get('state',{});m=self.data.get('motion',{})
        if self.component_failed:return False,'Launched component failed • inspect logs and restart'
        if self.software_stopped:return False,'Software motion stop latched; reconcile and restart this mode'
        if self.parameter_pending:return False,'Waiting for backend parameter acknowledgement'
        duplicates=[name for name in ['robot_state_publisher','chess_brain','controller_manager','chess_motion_interface'] if self.graph.get('nodes',[]).count(name)>1]
        if duplicates or len(self.graph.get('joint_publishers',[]))>1:return False,'Conflicting backends / joint-state publishers: '+', '.join(duplicates or self.graph['joint_publishers'])
        if self.mode.key in ['stockfish','mock','moveit']:
            if m.get('status') in ['failed','blocked','estop','initialization_failed']:return False,m.get('reason',m.get('status'))
            if now-self.arrival.get('motion',0)>3:return False,'Waiting for motion backend heartbeat'
            if any(p.startswith('_NODE_NAME_UNKNOWN') for p in self.graph.get('command_publishers',[])):return False,'Waiting for ROS publisher discovery metadata'
            if self.square_id:return False,'Square command in progress'
            if m.get('status')!='idle':
                policy=getattr(self,'policy',None)
                if policy and self.mode.key in ['stockfish','mock'] and policy.context():
                    return False,policy.progress_text()
                if self.stage_info:
                    stage=self.stage_info
                    detail=' → '.join(str(stage[k]) for k in ['source','target'] if stage.get(k))
                    return False,'Executing '+str(stage.get('operation') or stage.get('stage','')).replace('_',' ')+(' • '+detail if detail else '')
                return False,{'preflight':'Checking reachability / collision endpoints','awaiting_vision':'Waiting for fresh vision confirmation','busy':'Command running • waiting for backend progress'}.get(m.get('status'),m.get('status','Starting'))
            if self.idle_since is None or now-self.idle_since<.6:return False,'Waiting for stable idle heartbeat'
            if not self.data.get('safety') or now-self.arrival.get('safety',0)>2:return False,'Waiting for unlocked motion readiness'
            policy=getattr(self,'policy',None)
            cleared=getattr(policy,'rejection_cleared_at',0.)
            if cleared and (self.arrival.get('safety',0)<cleared or self.arrival.get('motion',0)<cleared):
                return False,'Waiting for fresh ROS readiness after safe preflight rejection'
            if self.mode.key=='moveit':return True,'MoveIt ready • mock controllers'
            if now-self.arrival.get('state',0)>5:return False,'Waiting for Chess Brain state'
            if not s.get('synchronized'):return False,'Waiting for initial board vision sync'
            if s.get('blocked'):return False,s.get('block_reason','Chess Brain blocked')
            if self.operator:return False,'Operator confirmation required for simulated promotion replacement'
            if s.get('pending_plan_id'):return False,'Pending '+str(s.get('pending_move'))+(' • waiting for fresh vision' if s.get('robot_ack_received') else ' • executing')
            if not self.data.get('safety') or self.arrival.get('safety',0)<self.commit_arrival:return False,'Waiting for a fresh idle readiness assertion'
            return True,'Confirmed board ready • robot1 White / robot2 Black'
        if self.mode.key=='manual':
            pubs=self.graph.get('joint_publishers',[])
            return (bool(self.data.get('limits') and pubs==['dual_scara_desktop_bridge'] and now-self.arrival.get('joints',0)<2), 'Visualization • desktop is the sole joint-state publisher' if pubs==['dual_scara_desktop_bridge'] else 'Waiting for exclusive manual publisher / model')
        if self.mode.key=='gazebo':
            ok=now-self.arrival.get('clock',0)<3 and now-self.arrival.get('joints',0)<3 and bool(self.data.get('limits'))
            return ok,'Gazebo clock and joint feedback active' if ok else 'Waiting for Gazebo spawn, clock and joint feedback'
        if self.mode.key=='logic':return bool(s),'Brain available • external vision/safety required; no robot execution'
        return False,'Stopped'

    def update_controls(self):
        active=self.lifecycle in ['starting','running'];mode=self.mode.key if self.mode else ''
        stopped=self.lifecycle in ['stopped','failed'];match=mode in ['stockfish','mock'];human_ok=active and match and self.policy.can_human(self.ready)
        selected_mode=MODES[self.w.mode.currentIndex()].key
        self.w.start.setEnabled(not self.closing and not self.bridge_failed and (stopped or active and selected_mode!=mode));self.w.start.setText('Switch mode' if active else 'Start mode')
        self.w.stop.setEnabled(not stopped and not self.closing);self.w.mode.setEnabled(not self.closing)
        for widget in [self.w.profile,self.w.browse,self.w.rviz,self.w.engine_path]:widget.setEnabled(stopped and not self.closing)
        self.w.human.setEnabled(stopped and selected_mode=='stockfish');self.w.scenario.setEnabled(stopped and selected_mode=='mock')
        self.w.submit.setEnabled(bool(human_ok));self.w.uci.setEnabled(bool(human_ok));self.w.promotion.setEnabled(bool(human_ok));self.w.board.interactive=bool(human_ok)
        self.w.arm.setVisible(selected_mode in ['manual','moveit','gazebo']);self.w.arm_label.setVisible(selected_mode in ['manual','moveit','gazebo'])
        self.w.arm.setEnabled(active and mode in ['manual','moveit','gazebo'])
        self.w.joint_box.setVisible(selected_mode in ['manual','gazebo'])
        self.w.apply_joints.setVisible(selected_mode=='gazebo')
        self.w.operator.setVisible(selected_mode in ['mock','stockfish'])
        self.w.fail_gripper.setVisible(selected_mode in ['mock','stockfish'])
        self.w.hold_vision.setVisible(selected_mode in ['mock','stockfish'])
        self.w.square.setVisible(selected_mode=='moveit')
        for button in self.w.square_buttons.values():button.setVisible(selected_mode=='moveit')
        for button in self.w.square_buttons.values():button.setEnabled(active and mode=='moveit' and self.ready)
        self.w.square.setEnabled(active and mode=='moveit' and self.ready)
        for spin,slider,label in self.w.joint_controls.values():spin.setEnabled(active and self.ready and mode in ['manual','gazebo']);slider.setEnabled(spin.isEnabled())
        self.w.apply_joints.setEnabled(active and self.ready and mode=='gazebo')
        self.w.operator.setEnabled(active and self.operator is not None and self.last_operator_token!=self.operator.get('token'))
        self.w.fail_gripper.setEnabled(active and self.initialized and match and self.ready)
        self.w.hold_vision.setEnabled(active and self.initialized and match and not self.parameter_pending)
        self.w.motion_stop.setEnabled(active and self.initialized and mode in ['stockfish','mock','moveit'])
        self.w.restart.setEnabled(active or stopped and self.configuration is not None)
        self.w.sync.setEnabled(active and mode=='logic' and bool(self.data.get('state')));self.w.state_button.setEnabled(active and mode in ['stockfish','mock','logic'])
        if active:
            text=getattr(self,'readiness_text','Starting backend…');self.w.backend.setText('Backend: '+text)
            reason=self.policy.paused or self.last_error
            if self.policy.wait_notice:text=self.policy.wait_notice
            self.w.status.setText(('Paused • '+reason) if reason else self.mode.title+' • '+text)
            s=self.data.get('state',{});turn=s.get('turn');result=s.get('result')
            self.w.turn.setText('Game over • '+str(result) if result else (f"{turn.title()} to move • {'robot1' if turn=='white' else 'robot2'}" if turn else self.mode.title))
            pending=(self.policy.progress_text() if s.get('pending_move') or self.policy.inflight else None)
            self.w.pending.setText(('Pending: '+str(pending)) if pending else ('Human turn' if human_ok else 'No pending move'))
        elif stopped:self.w.backend.setText('Backend: stopped')

    def human_move(self,uci):
        if not uci:return
        try:
            command=self.policy.human_move(uci,self.ready and self.lifecycle=='running')
            self.last_error='';self.w.execution_result.setText('Command sent • '+self.policy.progress_text());self.publish('/chess/command',command);self.selected=None;self.w.uci.clear();self.update_controls()
        except ValueError as exc:self.error(str(exc))

    def square_clicked(self,square):
        if not self.policy.can_human(self.ready):return
        legal=self.policy.state.get('legal_moves',[])
        if self.selected is None:
            if not any(move.startswith(square) for move in legal):self.error('Select a piece with a legal move for the active human side.');return
            self.selected=square
        elif square==self.selected:self.selected=None
        elif any(move.startswith(square) for move in legal):self.selected=square
        else:
            move=self.selected+square
            if any(v.startswith(move) and len(v)==5 for v in legal):move+=self.w.promotion.currentData()
            self.human_move(move)
        s=self.policy.state;last=self.policy.history[-1]['uci'] if self.policy.history else ''
        self.w.board.set_position(s['fen'],self.selected,self.configuration['human']=='black',last)

    # Mode-specific actions are implemented below; all communications stay here.
    def refresh_joints(self,*args):
        positions=dict(getattr(self,'configuration',{}).get('initial_joints',{}))
        positions.update(self.data.get('joints',{}))
        self.w.show_joints(self.data.get('limits',{}),self.w.arm.currentText(),positions,self.ready)
    def square_command(self,kind):
        if not (self.lifecycle=='running' and self.mode.key=='moveit' and self.ready and not self.square_id):return
        if kind=='probe' and QMessageBox.question(self.w,'Empty-square probe','Confirm this square is empty. The mock arm will approach, descend to the configured pawn grasp height, then retreat. Continue?')!=QMessageBox.Yes:return
        self.square_id=uuid.uuid4().hex
        self.publish('/chess/motion/square_command',{'request_id':self.square_id,'arm':self.w.arm.currentText(),
                     'square':self.w.square.currentText(),'execute':kind!='plan','probe':kind=='probe','home':kind=='home'})
        if kind=='plan':self.w.plan_result.setText('Planning: checking reachability and collision path…')
        else:self.w.execution_result.setText('Execution: request sent to mock controllers; waiting for result and feedback…')
        self.ready=False;self.update_controls()
    def joint_changed(self,name,value):
        if self.lifecycle=='running' and self.mode.key=='manual' and self.ready:
            self.bridge.submit('joint',{'name':name,'value':value})
    def gazebo_apply(self):
        if not (self.lifecycle=='running' and self.mode.key=='gazebo' and self.ready):return
        for name,(spin,slider,label) in self.w.joint_controls.items():
            arm,joint=name.split('_',1)
            self.publish('/'+arm+'/'+joint+'/cmd_pos',spin.value())
        self.w.execution_result.setText('Gazebo: direct position commands sent; inspect live joint feedback. No MoveIt collision planning.')
    def motion_stop(self):
        self.software_stopped=True
        self.publish('/chess/motion/estop',True);self.policy.paused='Software motion stop requested. Reconcile, then restart the simulated mode.';self.ready=False
    def restart(self):
        if QMessageBox.question(self.w,'Reset simulated mode?','Stop this mode and reset its simulated board to the selected starting scenario? This is not physical homing.')!=QMessageBox.Yes:return
        config=self.configuration
        if config:self.pending_start=dict(config);self.stop()
    def fail_gripper(self):
        if self.mode and self.mode.key in ['stockfish','mock'] and self.ready:
            self.parameter('/mock_gripper','fail_next',True)
    def hold_vision(self,hold):
        if self.mode and self.mode.key in ['stockfish','mock'] and self.initialized:
            self.parameter('/mock_chess_vision','confirm_after_ack',not hold)
    def confirm_promotion(self):
        if not self.operator or self.last_operator_token==self.operator.get('token'):return
        operator=dict(self.operator)
        if QMessageBox.question(self.w,'Simulated promotion replacement',
                'The pawn was removed and the arm parked. Confirm replacement with '+str(operator.get('piece'))+' on '+str(operator.get('square'))+'?')!=QMessageBox.Yes:return
        if not self.operator or self.operator.get('token')!=operator['token'] or self.lifecycle!='running':
            self.error('Promotion confirmation expired; inspect backend state.');return
        self.last_operator_token=operator['token']
        self.publish('/chess/motion/operator_confirm',{'plan_id':operator['plan_id'],'token':operator['token'],'confirmed':True})
        self.update_controls()

    def parameter(self,node,name,value):
        request_id=uuid.uuid4().hex;self.parameter_pending.add(request_id);self.ready=False
        self.bridge.submit('parameter',{'generation':self.generation,'node':node,'name':name,'value':value,'request_id':request_id})
        self.update_controls()
