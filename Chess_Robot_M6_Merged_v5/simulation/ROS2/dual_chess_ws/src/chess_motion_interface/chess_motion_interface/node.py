"""Single-owner chess motion adapter; rules stay in chess_brain_handover.

Geometry changes and commands share one lock. ROS callbacks remain responsive
while a worker waits for MoveIt/controller/gripper results. A failed operation
blocks further movement; it never synthesizes a successful robot/vision ack.
"""
import copy
import json
import threading
import time
import uuid
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from rcl_interfaces.msg import SetParametersResult, ParameterDescriptor
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
from ament_index_python.packages import get_package_share_directory
import xacro
from chess_brain_handover.core import describe_move, chess
from .geometry import Geometry
from .model import Model
from .backend import Backend, MotionError, requires_reconciliation
from .scene import Scene


def send(pub, data):
    pub.publish(String(data=json.dumps(data,separators=(',',':'))))


class MotionNode(Node):
    def __init__(self):
        super().__init__('chess_motion_interface')
        self.io_group = ReentrantCallbackGroup()
        default = str(Path(get_package_share_directory('dual_chess_description'))/'config/chess_geometry.yaml')
        for name, value in [('geometry_file',default),('initial_fen',''),('accept_move_plans',False),('publish_mock_readiness',False)]:
            self.declare_parameter(name,value,ParameterDescriptor(read_only=True))
        self.geometry = Geometry.load(self.get_parameter('geometry_file').value)
        self.dynamic_names = {}
        for section in ['board','support','pieces','motion']:
            def declare(mapping, prefix):
                for key, value in mapping.items():
                    name = prefix+'.'+key
                    if isinstance(value,dict):
                        declare(value,name)
                    else:
                        self.dynamic_names[name] = name.split('.')
                        self.declare_parameter(name,value)
            declare(self.geometry.raw[section],section)
        raw = copy.deepcopy(self.geometry.raw)
        for name, keys in self.dynamic_names.items():
            target = raw
            for key in keys[:-1]:
                target = target[key]
            target[keys[-1]] = self.get_parameter(name).value
        self.geometry = Geometry(raw)
        urdf = Path(get_package_share_directory('dual_chess_description'))/'urdf/dual_chess.urdf.xacro'
        self.model = Model(ET.fromstring(xacro.process_file(str(urdf),mappings={'geometry_file':self.get_parameter('geometry_file').value,'include_board':'false'}).toxml()))
        self.lock = threading.Lock()
        self.ready, self.blocked, self.busy = False, False, False
        self.active_id, self.revision = '', 0
        self.awaiting_fen = None
        self.seen = set()
        self.bin_counts = {'robot1':0,'robot2':0}
        self.confirmed_fen = self.get_parameter('initial_fen').value
        self.status_pub = self.create_publisher(String,'/chess/motion/status',20)
        self.ack_pub = self.create_publisher(String,'/chess/robot_ack',10)
        self.safety_pub = self.create_publisher(Bool,'/chess/safety_ready',10)
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.geometry_pub = self.create_publisher(String,'/chess/geometry',qos)
        self.gripper_pub = self.create_publisher(String,'/chess/gripper/command',10)
        self.gripper_pending = {}
        self.operator_event = threading.Event()
        self.operator_token = None
        self.tf = TransformBroadcaster(self)
        self.backend = Backend(self,self.model,self.geometry,self.event)
        self.scene = Scene(self.backend,self.geometry)
        self.create_subscription(String,'/chess/move_plan',self.on_plan,10)
        self.create_subscription(String,'/chess/motion/square_command',self.on_square,10)
        self.create_subscription(String,'/chess/state',self.on_state,10)
        self.create_subscription(String,'/chess/gripper/status',self.on_gripper,10)
        self.create_subscription(String,'/chess/motion/operator_confirm',self.on_operator,10)
        self.create_subscription(Bool,'/chess/motion/estop',self.on_stop,10)
        self.create_subscription(PoseStamped,'/chess/vision/board_pose',self.on_board_pose,10)
        self.create_subscription(String,'/chess/vision/pieces',self.on_piece_observation,10)
        self.syncing_derived_parameters = False
        self.add_on_set_parameters_callback(self.on_parameters)
        self.add_post_set_parameters_callback(self.sync_derived_parameters)
        self.create_timer(.1,self.publish_frames)
        self.create_timer(.5,self.readiness)
        self.worker = threading.Thread(target=self.bootstrap,daemon=True)
        self.worker.start()

    def event(self, status, **data):
        payload = {'schema_version':1,'status':status,'plan_id':self.active_id,'geometry_revision':self.revision,'simulation_grasp':True,**data}
        send(self.status_pub,payload)
        if status not in ['idle','busy','initializing','blocked','awaiting_vision']:
            self.get_logger().info(json.dumps(payload,separators=(',',':')))

    def bootstrap(self):
        with self.lock:
            try:
                deadline = time.monotonic()+40
                while self.backend.latest is None and time.monotonic()<deadline:
                    time.sleep(.1)
                self.scene.initialize(self.confirmed_fen or None)
                ok, contacts = self.backend.valid()
                if not ok:
                    raise MotionError('initial_scene_collision: '+str(contacts))
                self.ready = True
                self.publish_geometry()
                self.event('ready',detail='Scene and controllers available; gripper handshake required for chess moves')
            except Exception as exc:
                self.blocked = True
                self.event('initialization_failed',reason=str(exc))

    def readiness(self):
        if self.get_parameter('publish_mock_readiness').value:
            self.safety_pub.publish(Bool(data=self.ready and not self.busy and not self.blocked and not getattr(self,'awaiting_fen',None) and not self.lock.locked() and not self.backend.stop.is_set() and time.monotonic()-self.backend.received_at<2.))
        self.event('blocked' if self.blocked else 'busy' if self.busy else 'awaiting_vision' if getattr(self,'awaiting_fen',None) else 'idle' if self.ready else 'initializing')

    def publish_geometry(self):
        send(self.geometry_pub,{'revision':self.revision,'frame_id':'world','configuration':self.geometry.raw,'placeholders':True})

    def publish_frames(self):
        g = self.geometry
        transforms = []
        def add(parent, child, xyz, q):
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id, t.child_frame_id = parent, child
            t.transform.translation.x,t.transform.translation.y,t.transform.translation.z = map(float,xyz)
            t.transform.rotation.x,t.transform.rotation.y,t.transform.rotation.z,t.transform.rotation.w = map(float,q)
            transforms.append(t)
        add('world','chess_board',g.world([0,0,g.raw['board']['surface_height']]),Rotation.from_euler('xyz',g.raw['board']['rpy']).as_quat())
        for f in 'abcdefgh':
            for r in '12345678':
                add('chess_board','chess_square_'+f+r,g.local_square(f+r)-[0,0,g.raw['board']['surface_height']],[0.,0.,0.,1.])
        self.tf.sendTransform(transforms)

    def sync_derived_parameters(self, parameters):
        if self.syncing_derived_parameters:
            return
        updates = []
        for key in ['square_size_x','square_size_y']:
            name = 'board.'+key
            value = float(self.geometry.raw['board'][key])
            if self.get_parameter(name).value != value:
                updates.append(rclpy.parameter.Parameter(name,value=value))
        if updates:
            self.syncing_derived_parameters = True
            try:
                self.set_parameters_atomically(updates)
            finally:
                self.syncing_derived_parameters = False

    def on_parameters(self, parameters):
        if getattr(self,'syncing_derived_parameters',False):
            return SetParametersResult(successful=True)
        relevant = [p for p in parameters if p.name in self.dynamic_names]
        if not relevant:
            return SetParametersResult(successful=True)
        if not self.ready or self.blocked or self.awaiting_fen or not self.lock.acquire(blocking=False):
            return SetParametersResult(successful=False,reason='geometry_locked: wait for ready and idle; failed/partial moves need reconciliation')
        previous_geometry, previous_scene, previous_revision = self.geometry, self.scene, self.revision
        scene_changed = False
        try:
            if self.scene.attached:
                raise ValueError('cannot change geometry with an attached piece')
            raw = copy.deepcopy(self.geometry.raw)
            for p in relevant:
                target = raw
                keys = self.dynamic_names[p.name]
                for key in keys[:-1]:
                    target = target[key]
                target[keys[-1]] = p.value
            if raw['board'].get('grid_mode')=='fit_board' and any(p.name in ['board.square_size_x','board.square_size_y'] for p in relevant):
                raise ValueError('set board.grid_mode to square_size before editing independent square pitches')
            candidate = Geometry(raw)
            scene = Scene(self.backend,candidate)
            scene.ids = set(self.scene.ids)
            scene.pieces = copy.deepcopy(self.scene.pieces)
            # Dimensions/placement cannot be silently changed after a partially executed move.
            scene_changed = True
            scene.initialize(self.confirmed_fen or None)
            ok, contacts = self.backend.valid()
            if not ok:
                raise ValueError('new_geometry_collides_with_robot: '+str(contacts))
            self.geometry = candidate
            self.backend.geometry, self.scene = candidate, scene
            self.revision += 1
            self.publish_geometry()
            self.event('geometry_updated',reachability_requires_recheck=True)
            return SetParametersResult(successful=True)
        except Exception as exc:
            if scene_changed:
                self.geometry, self.scene, self.revision = previous_geometry, previous_scene, previous_revision
                self.backend.geometry = previous_geometry
                try:
                    previous_scene.initialize(self.confirmed_fen or None)
                    self.publish_geometry()
                except Exception as rollback:
                    self.blocked = True
                    self.event('geometry_rollback_failed',reason=str(rollback))
            return SetParametersResult(successful=False,reason=str(exc))
        finally:
            self.lock.release()

    def on_board_pose(self, msg):
        try:
            age = (self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
            if not 0 <= age <= 2.:
                raise ValueError('stale_or_future_board_pose')
            q = np.array([msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w])
            if abs(np.linalg.norm(q)-1.) > .001:
                raise ValueError('invalid_board_quaternion')
            t = np.eye(4)
            t[:3,:3] = Rotation.from_quat(q).as_matrix()
            t[:3,3] = [msg.pose.position.x,msg.pose.position.y,msg.pose.position.z]
            if msg.header.frame_id != 'world':
                if msg.header.frame_id not in ['robot1_base_link','robot2_base_link']:
                    raise ValueError('unsupported_pose_frame: use world or a CAD base_link')
                t = self.model.matrix(msg.header.frame_id,self.backend.joints()) @ t
            # CV board pose denotes the playing-surface centre; YAML position denotes underside.
            t[:3,3] -= t[:3,:3] @ np.array([0.,0.,self.geometry.raw['board']['surface_height']])
            params = [rclpy.parameter.Parameter('board.position',value=t[:3,3].tolist()),rclpy.parameter.Parameter('board.rpy',value=Rotation.from_matrix(t[:3,:3]).as_euler('xyz').tolist())]
            result = self.set_parameters_atomically(params)
            if not result.successful:
                raise ValueError(result.reason)
        except Exception as exc:
            self.event('board_pose_rejected',reason=str(exc))

    def on_piece_observation(self, msg):
        if not self.ready or self.blocked or not self.lock.acquire(blocking=False):
            self.event('piece_observation_rejected',reason='scene_busy_or_blocked')
            return
        try:
            data = json.loads(msg.data)
            if data.get('schema_version')!=1 or not 0 <= time.time()-float(data['captured_at']) <= 2.:
                raise ValueError('invalid_or_stale_observation')
            if data.get('frame_id') not in ['world','chess_board']:
                raise ValueError('piece positions must be in world or chess_board frame')
            entries = data['pieces']
            if not isinstance(entries,list) or len(entries)>32:
                raise ValueError('pieces must be a complete list with at most 32 entries')
            candidate = copy.deepcopy(self.scene.pieces)
            new_at, objects = {}, []
            for entry in entries:
                sq, symbol = entry['square'], entry['piece']
                centre = self.geometry.local_square(sq,self.geometry.piece_height(symbol)/2)
                actual = np.asarray(entry['position'],dtype=float)
                if actual.shape!=(3,) or not np.all(np.isfinite(actual)) or sq in new_at:
                    raise ValueError('invalid_or_duplicate_piece')
                if data['frame_id']=='world':
                    actual = (np.linalg.inv(self.geometry.T) @ np.array([*actual,1.]))[:3]
                else:
                    actual[2] += self.geometry.raw['board']['surface_height']
                b = self.geometry.raw['board']
                if np.any(np.abs(actual[:2]-centre[:2]) > np.array([b['square_size_x'],b['square_size_y']])*.45) or abs(actual[2]-centre[2])>.01:
                    raise ValueError('piece_detection_inconsistent_with_board_mapping')
                name = 'chess_piece_'+sq
                new_at[sq] = name
                candidate[name] = {'symbol':symbol,'local':actual.tolist()}
                objects.append(self.scene.piece_object(name,symbol,actual))
            from moveit_msgs.msg import PlanningScene, PlanningSceneWorld, CollisionObject
            offboard = {k:v for k,v in self.scene.pieces.items() if k.startswith('chess_bin_')}
            removed = set(self.scene.pieces)-set(new_at.values())-set(offboard)
            objects = [CollisionObject(id=n,operation=CollisionObject.REMOVE) for n in removed]+objects
            self.backend.apply(PlanningScene(is_diff=True,world=PlanningSceneWorld(collision_objects=objects)))
            self.scene.pieces = {**offboard,**{name:candidate[name] for name in new_at.values()}}
            self.scene.at = new_at
            self.scene.ids.difference_update(removed)
            self.scene.ids.update(new_at.values())
            self.event('piece_observation_applied',confirmed_board_unchanged=True)
        except Exception as exc:
            self.event('piece_observation_rejected',reason=str(exc))
        finally:
            self.lock.release()

    def on_state(self, msg):
        try:
            state = json.loads(msg.data)
            # Confirmation authority remains the chess brain, not the motion worker.
            if state.get('fen') and state.get('pending_plan_id') is None:
                self.confirmed_fen = state['fen']
                if self.awaiting_fen == state['fen']:
                    self.awaiting_fen = None
                    self.event('board_confirmed',fen=state['fen'])
        except (ValueError,TypeError):
            pass

    def on_stop(self, msg):
        if msg.data:
            self.backend.stop.set()
            self.blocked = True
            if self.backend.active_goal:
                self.backend.active_goal.cancel_goal_async()
            self.event('estop',reason='Reset/reconcile physical and scene state before restarting; false does not auto-resume')

    def on_gripper(self, msg):
        try:
            data = json.loads(msg.data)
            entry = self.gripper_pending.get(data.get('request_id'))
            if entry:
                entry['success'] = data.get('success') is True
                entry['event'].set()
        except (ValueError,TypeError):
            pass

    def grip(self, operation, arm):
        key = uuid.uuid4().hex
        entry = {'event':threading.Event(),'success':False}
        self.gripper_pending[key] = entry
        send(self.gripper_pub,{'schema_version':1,'request_id':key,'plan_id':self.active_id,'arm':arm,'operation':operation})
        try:
            deadline = time.monotonic()+5.
            while not entry['event'].wait(.05):
                if self.backend.stop.is_set() or time.monotonic()>deadline:
                    raise MotionError('gripper_feedback_timeout_or_stop')
            if not entry['success']:
                raise MotionError('gripper_operation_failed')
        finally:
            self.gripper_pending.pop(key,None)

    def on_operator(self, msg):
        try:
            d = json.loads(msg.data)
            if d.get('plan_id')==self.active_id and d.get('token')==self.operator_token and d.get('confirmed') is True:
                self.operator_event.set()
        except (ValueError,TypeError):
            pass

    def submit(self, data, worker, chess_plan=False):
        id = data.get('plan_id') or data.get('request_id')
        if not isinstance(id,str) or not id or len(id)>128:
            self.event('rejected',request_id=id,reason='missing_or_invalid_request_id')
            return
        if id in self.seen:
            self.event('duplicate_ignored',request_id=id)
            return
        if not self.ready or self.blocked or self.awaiting_fen or not self.lock.acquire(blocking=False):
            self.event('rejected',request_id=id,reason='not_ready_busy_or_blocked')
            if chess_plan:
                send(self.ack_pub,{'plan_id':id,'success':False,'arm_parked':False,'gripper_released':False,'reason':'not_ready_busy_or_blocked'})
            return
        self.busy, self.active_id = True, id
        self.seen.add(id)
        def run():
            epoch = self.backend.execution_epoch
            try:
                self.event('accepted',request_id=id)
                worker(data)
                self.event('succeeded',request_id=id,awaiting_vision=chess_plan,executed=chess_plan or data.get('execute') is True,mode='controller_execution' if chess_plan or data.get('execute') is True else 'plan_only')
                if chess_plan:
                    self.awaiting_fen = data['expected_fen']
                    send(self.ack_pub,{'plan_id':id,'success':True,'arm_parked':True,'gripper_released':True,'simulated_gripper':True})
            except Exception as exc:
                # Conservative: a rejected later stage can leave an earlier stage executed.
                partial = requires_reconciliation(exc, self.backend.execution_epoch != epoch, self.scene.attached is not None, self.blocked, self.backend.stop.is_set())
                self.blocked = partial
                self.event('failed' if partial else 'rejected',request_id=id,reason=str(exc),physical_state_confirmed=False,reconciliation_required=partial)
                if chess_plan:
                    send(self.ack_pub,{'plan_id':id,'success':False,'arm_parked':False,'gripper_released':False,'reason':str(exc)})
            finally:
                self.busy, self.active_id = False, ''
                self.lock.release()
        threading.Thread(target=run,daemon=True).start()

    def on_plan(self, msg):
        data = None
        try:
            if not self.get_parameter('accept_move_plans').value:
                raise ValueError('chess_execution_disabled_in_this_mode')
            data = json.loads(msg.data)
            if not isinstance(data,dict):
                raise ValueError('plan_must_be_object')
            board = chess.Board(data['before_fen'])
            if self.confirmed_fen and board.fen()!=chess.Board(self.confirmed_fen).fen():
                raise ValueError('plan_does_not_match_confirmed_board')
            validated = describe_move(board,chess.Move.from_uci(data['uci'])).to_dict()
            for key in ['actions','expected_fen','side','piece']:
                if data.get(key)!=validated[key]:
                    raise ValueError('inconsistent_chess_plan_'+key)
            self.submit(data,self.execute_plan,True)
        except Exception as exc:
            self.event('rejected',reason=str(exc))
            try:
                if isinstance(data,dict) and data.get('plan_id'):
                    send(self.ack_pub,{'plan_id':data['plan_id'],'success':False,'arm_parked':False,'gripper_released':False,'reason':str(exc)})
            except UnboundLocalError:
                pass

    def on_square(self, msg):
        try:
            data = json.loads(msg.data)
            if not isinstance(data,dict):
                raise ValueError('command_must_be_object')
            self.submit(data,self.execute_square)
        except Exception as exc:
            self.event('rejected',reason=str(exc))

    def check_targets(self, arm, targets):
        for label, point in targets:
            solutions = self.model.solutions(arm,np.asarray(point))
            if not solutions:
                raise MotionError(f'unreachable_{label}: world={list(map(float,point))}; check board/support/mount geometry')
            if 'approach' in label or label.startswith('capture_'):
                from moveit_msgs.msg import RobotState
                contacts = []
                for solution in solutions:
                    state = self.backend.state()
                    values = dict(zip(state.joint_state.name,state.joint_state.position))
                    values.update({f'{arm}_joint{i}':v for i,v in enumerate(solution['q'],1)})
                    state.joint_state = copy.deepcopy(state.joint_state)
                    state.joint_state.position = [float(values[n]) for n in state.joint_state.name]
                    ok, contacts = self.backend.valid(state)
                    if ok:
                        break
                else:
                    raise MotionError(f'collision_at_{label}: {contacts}')

    def execute_square(self, d):
        arm = d.get('arm','robot1')
        if arm not in ['robot1','robot2']:
            raise ValueError('invalid_arm')
        execute = d.get('execute') is True
        if d.get('home') is True:
            self.backend.park(arm,execute)
            return
        square = d['square']
        g = self.geometry
        approach = g.square(square,g.travel())
        target = g.square(square,g.grasp('p'))
        if d.get('height') is not None:
            approach = g.square(square)
            approach[2] = float(d['height'])  # backwards-compatible world-Z override
        if d.get('offset') is not None:
            approach = g.square(square,float(d['offset']))
        probe = d.get('probe') is True
        self.check_targets(arm,[('approach',approach)]+([('probe',target)] if probe else []))
        self.event('target',square=square,world=approach.tolist(),base_frame=arm+'_base_link',base=(np.linalg.inv(self.model.matrix(arm+'_base_link',self.backend.joints())) @ np.array([*approach,1.]))[:3].tolist(),execute=execute)
        self.backend.move(arm,approach,execute)
        if probe and execute:
            self.backend.vertical(arm,target)
            self.backend.vertical(arm,approach)

    def preflight_plan(self, plan, arm):
        g = self.geometry
        tilt = float(np.arccos(np.clip(g.T[2,2],-1.,1.)))
        if tilt > g.raw['motion']['max_board_tilt']:
            raise MotionError('board_tilt_exceeds_position_only_scara_grasp_limit')
        expected = {chess.square_name(i):p.symbol() for i,p in chess.Board(plan['before_fen']).piece_map().items()}
        observed = {sq:self.scene.pieces[name]['symbol'] for sq,name in self.scene.at.items()}
        if expected != observed:
            raise MotionError('piece_scene_does_not_match_validated_before_fen')
        bins = self.bin_counts[arm]
        targets = []
        for action in plan['actions']:
            source_piece = chess.Board(plan['before_fen']).piece_at(chess.parse_square(action['source'])) if action.get('source') else None
            symbol = source_piece.symbol() if source_piece else plan['piece']
            if action['kind']=='remove':
                symbol = plan['captured']
            elif action['kind']=='promote':
                symbol = 'p'
            for key in ['source','target']:
                if action.get(key):
                    sq = action[key]
                    targets.extend([(sq+'_grasp',g.square(sq,g.grasp(symbol))),(sq+'_approach',g.square(sq,g.travel(g.piece_height(symbol))))])
            if action['kind'] in ['remove','promote']:
                local = g.bin_local(arm,bins)
                bins += 1
                targets.extend([('capture_place',g.world(local+[0,0,g.grasp(symbol)])),('capture_approach',g.world([local[0],local[1],g.raw['board']['surface_height']+g.travel(g.piece_height(symbol))]))])
        self.check_targets(arm,targets)

    def transfer(self, arm, source, target=None, bin_slot=None):
        g = self.geometry
        if source not in self.scene.at:
            raise MotionError('scene_piece_missing_'+source)
        name = self.scene.at[source]
        symbol = self.scene.pieces[name]['symbol']
        height = g.piece_height(symbol)
        src = np.array(self.scene.pieces[name]['local'])-[0,0,height/2]
        dst = g.local_square(target) if target else g.bin_local(arm,bin_slot)
        travel_z = max(g.raw['board']['surface_height']+g.travel(height),float(dst[2])+height+g.raw['motion']['approach_clearance'])
        above_src = g.world([src[0],src[1],travel_z])
        above_dst = g.world([dst[0],dst[1],travel_z])
        grasp_src = g.world(src+[0,0,g.grasp(symbol)])
        grasp_dst = g.world(dst+[0,0,g.grasp(symbol)+g.raw['motion']['place_clearance']])
        self.event('stage',stage='approach_pick',source=source,target=target)
        self.grip('open',arm)
        self.backend.move(arm,above_src)
        self.scene.touch(name,arm,True)
        self.backend.vertical(arm,grasp_src)
        self.event('stage',stage='grasp',piece=name)
        self.grip('close',arm)
        self.scene.attach(name,arm)
        self.scene.at.pop(source)
        self.backend.vertical(arm,above_src)
        self.event('stage',stage='transfer',piece=name)
        self.backend.move(arm,above_dst)
        self.backend.vertical(arm,grasp_dst)
        self.grip('open',arm)
        centre = dst+[0,0,height/2+g.raw['motion']['place_clearance']]
        self.scene.detach(name,arm,centre)
        if target:
            self.scene.at[target] = name
        self.event('stage',stage='retreat',piece=name)
        self.backend.vertical(arm,above_dst)
        self.scene.touch(name,arm,False)
        if target is None:
            self.scene.rename(name,f'chess_bin_{arm}_{bin_slot}')

    def execute_plan(self, plan):
        arm = 'robot1' if plan['side']=='white' else 'robot2'
        self.event('preflight',uci=plan['uci'])
        self.preflight_plan(plan,arm)
        ok, contacts = self.backend.valid()
        if not ok:
            raise MotionError('start_scene_collision: '+str(contacts))
        # The non-moving arm must be parked, not merely clear at a sampled endpoint.
        other = 'robot2' if arm=='robot1' else 'robot1'
        values = self.backend.joints()
        if any(abs(values[f'{other}_joint{i}']-v)>.01 for i,v in enumerate([.09,0.,0.],1)):
            raise MotionError('other_arm_not_parked')
        for action in plan['actions']:
            kind = action['kind']
            if kind=='move':
                self.transfer(arm,action['source'],action['target'])
            elif kind=='remove':
                self.transfer(arm,action['source'],bin_slot=self.bin_counts[arm])
                self.bin_counts[arm] += 1
            elif kind=='promote':
                sq = action['target']
                self.transfer(arm,sq,bin_slot=self.bin_counts[arm])
                self.bin_counts[arm] += 1
                self.backend.park(arm)
                self.operator_event.clear()
                self.operator_token = uuid.uuid4().hex
                self.event('operator_required',token=self.operator_token,square=sq,piece=action['piece'],detail='Replace pawn with requested promotion piece; explicitly confirm')
                deadline = time.monotonic()+self.geometry.raw['motion']['operator_timeout']
                while not self.operator_event.wait(.1):
                    if self.backend.stop.is_set() or time.monotonic()>deadline:
                        raise MotionError('promotion_operator_timeout_or_stop')
                self.operator_token = None
                symbol = action['piece'].upper() if plan['side']=='white' else action['piece'].lower()
                name = 'chess_promoted_'+plan['plan_id']
                local = self.geometry.local_square(sq,self.geometry.piece_height(symbol)/2)
                from moveit_msgs.msg import PlanningScene, PlanningSceneWorld
                self.backend.apply(PlanningScene(is_diff=True,world=PlanningSceneWorld(collision_objects=[self.scene.piece_object(name,symbol,local)])))
                self.scene.at[sq] = name
                self.scene.pieces[name] = {'symbol':symbol,'local':local.tolist()}
                self.scene.ids.add(name)
        self.backend.park(arm)
        self.grip('open',arm)
        self.event('execution_complete',uci=plan['uci'],awaiting_vision=True)


def main(args=None):
    rclpy.init(args=args)
    node = MotionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.backend.stop.set()
    finally:
        executor.shutdown(timeout_sec=3.)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
