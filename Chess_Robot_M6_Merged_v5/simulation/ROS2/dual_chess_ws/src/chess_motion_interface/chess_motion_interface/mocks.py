"""Explicit simulation sources. Never use mock vision/gripper as hardware feedback."""
import argparse
import json
import time
import uuid
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from chess_brain_handover.core import chess, SCHEMA_VERSION
from .node import send


def observation(fen):
    board = chess.Board(fen)
    return {'schema_version':SCHEMA_VERSION,'capture_id':uuid.uuid4().hex,'source':'mock_vision_simulation',
            'captured_at':time.time(),'stable':True,'calibrated':True,'confidence':1.,'orientation':'a8_top_left',
            'cells':[{'row':row,'col':col,'piece':board.piece_at(chess.square(col,7-row)).symbol() if board.piece_at(chess.square(col,7-row)) else '.', 'confidence':1.} for row in range(8) for col in range(8)]}


class MockVision(Node):
    def __init__(self):
        super().__init__('mock_chess_vision')
        self.declare_parameter('initial_fen',chess.STARTING_FEN)
        self.declare_parameter('confirm_after_ack',True)
        self.fen = self.get_parameter('initial_fen').value
        self.plans = {}
        self.pending_confirmation = None
        self.held_confirmation = None
        self.pub = self.create_publisher(String,'/chess/vision/board',10)
        self.command = self.create_publisher(String,'/chess/command',10)
        self.create_subscription(String,'/chess/move_plan',self.plan,10)
        self.create_subscription(String,'/chess/robot_ack',self.ack,10)
        self.create_timer(.5,self.tick)

    def plan(self,msg):
        try:
            d = json.loads(msg.data)
            self.plans[d['plan_id']] = d
        except (ValueError,KeyError,TypeError):
            pass

    def ack(self,msg):
        try:
            d = json.loads(msg.data)
            if d.get('success') is True and d.get('arm_parked') is True and d.get('gripper_released') is True:
                plan = self.plans.pop(d['plan_id'],None)
                if plan:
                    if self.get_parameter('confirm_after_ack').value:
                        self.pending_confirmation = (plan['expected_fen'],time.monotonic()+.3)
                    else:
                        self.held_confirmation = plan['expected_fen']
        except (ValueError,KeyError,TypeError):
            pass

    def tick(self):
        if self.held_confirmation:
            if not self.get_parameter('confirm_after_ack').value:
                return
            self.pending_confirmation = (self.held_confirmation,time.monotonic()+.3)
            self.held_confirmation = None
        if self.pending_confirmation and time.monotonic()<self.pending_confirmation[1]:
            return
        if self.pending_confirmation and time.monotonic()>=self.pending_confirmation[1]:
            self.fen = self.pending_confirmation[0]
            self.pending_confirmation = None
        send(self.pub,observation(self.fen))
        send(self.command,{'action':'sync'})


def spin(node):
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


def vision_main(args=None):
    rclpy.init(args=args)
    spin(MockVision())


def gripper_main(args=None):
    rclpy.init(args=args)
    node = Node('mock_gripper')
    node.declare_parameter('fail_next',False)
    pub = node.create_publisher(String,'/chess/gripper/status',10)
    def command(msg):
        try:
            d = json.loads(msg.data)
            failed = bool(node.get_parameter('fail_next').value)
            if failed:node.set_parameters([rclpy.parameter.Parameter('fail_next',value=False)])
            send(pub,{'request_id':d['request_id'],'success':not failed and d['operation'] in ['open','close'],'mock':True})
        except (ValueError,KeyError,TypeError):pass
    node.create_subscription(String,'/chess/gripper/command',command,10)
    spin(node)


def engine_main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--move','--uci',dest='move',required=True)
    ap.add_argument('--timeout',type=float,default=180.)
    ap.add_argument('--no-wait',action='store_true')
    args,ros_args = ap.parse_known_args()
    chess.Move.from_uci(args.move)
    rclpy.init(args=ros_args)
    node = Node('mock_chess_engine_command')
    states,events,acks,readiness,statuses = [],[],[],[],[]
    node.create_subscription(String,'/chess/motion/status',lambda m:statuses.append(json.loads(m.data)),10)
    node.create_subscription(Bool,'/chess/safety_ready',lambda m:readiness.append(m.data),10)
    node.create_subscription(String,'/chess/state',lambda m:states.append(json.loads(m.data)),10)
    node.create_subscription(String,'/chess/events',lambda m:events.append(json.loads(m.data)),10)
    node.create_subscription(String,'/chess/robot_ack',lambda m:acks.append(json.loads(m.data)),10)
    pub = node.create_publisher(String,'/chess/engine/move',10)
    deadline = time.monotonic()+args.timeout
    code = 1
    try:
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if statuses and statuses[-1]['status'] in ['initialization_failed','blocked']:
                raise RuntimeError('Motion interface blocked; inspect /chess/motion/status and launch log')
            if states and states[-1]['synchronized'] and readiness and readiness[-1] and pub.get_subscription_count():
                break
        else:raise RuntimeError('No synchronized chess brain; start simulation_pipeline.launch.py')
        initial = states[-1]['committed_plies']
        key = uuid.uuid4().hex
        send(pub,{'uci':args.move,'request_id':key})
        plan_id = None
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            errors = [e for e in events if e.get('event')=='command_error']
            if errors:raise RuntimeError(str(errors[-1]))
            responses = [e for e in events if e.get('request_id')==key]
            if responses:
                response = responses[-1]
                if not response.get('ok'):raise RuntimeError(str(response))
                plan_id = response.get('plan_id')
                if args.no_wait:
                    print(json.dumps(response));code=0;break
            failures = [a for a in acks if a.get('plan_id')==plan_id and a.get('success') is False]
            if failures:raise RuntimeError(str(failures[-1]))
            if states and states[-1]['committed_plies']>initial:
                print(json.dumps({'status':'committed_after_execution_and_mock_vision','state':states[-1]},indent=2));code=0;break
        else:raise RuntimeError('Move not committed before timeout; inspect motion status, operator step or vision confirmation')
    except Exception as exc:
        print(json.dumps({'status':'failed_or_rejected','reason':str(exc)}))
    finally:
        node.destroy_node();rclpy.shutdown()
    raise SystemExit(code)


def square_main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--arm',choices=['robot1','robot2'],default='robot1')
    ap.add_argument('--square',default='e4')
    ap.add_argument('--height',type=float,help='Legacy absolute world-Z override')
    ap.add_argument('--offset',type=float,help='Height above the configured board playing surface')
    ap.add_argument('--home',action='store_true')
    ap.add_argument('--execute',action='store_true')
    ap.add_argument('--probe',action='store_true',help='Straight descent to configured pawn grasp height then retreat; use on an empty square')
    ap.add_argument('--output')
    ap.add_argument('--timeout',type=float,default=120.)
    args,ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('chess_square_command')
    pub = node.create_publisher(String,'/chess/motion/square_command',10)
    statuses = []
    node.create_subscription(String,'/chess/motion/status',lambda m:statuses.append(json.loads(m.data)),10)
    key = uuid.uuid4().hex
    code = 1
    try:
        deadline = time.monotonic()+args.timeout
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            if statuses and statuses[-1]['status']=='blocked':raise RuntimeError('Motion interface is blocked')
            if pub.get_subscription_count() and statuses and statuses[-1]['status']=='idle':break
        else:raise RuntimeError('Motion interface is not ready/idle')
        send(pub,{'request_id':key,**{k:v for k,v in vars(args).items() if k not in ['output','timeout']}})
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
            done = [s for s in statuses if s.get('request_id')==key and s['status'] in ['succeeded','failed','rejected']]
            if done:
                report = done[-1]
                print(json.dumps(report,indent=2))
                if args.output:
                    from pathlib import Path
                    Path(args.output).write_text(json.dumps(report,indent=2)+'\n')
                code = 0 if report['status']=='succeeded' else 1
                break
        else:raise RuntimeError('Square command timed out; inspect motion status')
    except Exception as exc:
        print(json.dumps({'status':'failed','reason':str(exc)}))
    finally:
        node.destroy_node();rclpy.shutdown()
    raise SystemExit(code)


def input_main():
    """Publish a synthetic detected board pose or complete piece-position JSON list."""
    from geometry_msgs.msg import PoseStamped
    from scipy.spatial.transform import Rotation
    ap = argparse.ArgumentParser()
    ap.add_argument('--position',nargs=3,type=float,default=[0.,0.,.01])
    ap.add_argument('--rpy',nargs=3,type=float,default=[0.,0.,0.])
    ap.add_argument('--frame',default='world')
    ap.add_argument('--pieces-file')
    args,ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('mock_vision_input')
    try:
        if args.pieces_file:
            from pathlib import Path
            data = {'schema_version':1,'frame_id':args.frame,'captured_at':time.time(),'pieces':json.loads(Path(args.pieces_file).read_text())}
            pub = node.create_publisher(String,'/chess/vision/pieces',10)
            message = String(data=json.dumps(data))
        else:
            pub = node.create_publisher(PoseStamped,'/chess/vision/board_pose',10)
            message = PoseStamped()
            message.header.frame_id = args.frame
            message.pose.position.x,message.pose.position.y,message.pose.position.z = args.position
            message.pose.orientation.x,message.pose.orientation.y,message.pose.orientation.z,message.pose.orientation.w = map(float,Rotation.from_euler('xyz',args.rpy).as_quat())
        deadline = time.monotonic()+10
        while not pub.get_subscription_count() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not pub.get_subscription_count():raise RuntimeError('Motion interface unavailable')
        if hasattr(message,'header'):message.header.stamp=node.get_clock().now().to_msg()
        else:
            data['captured_at']=time.time();message.data=json.dumps(data)
        pub.publish(message)
        for _ in range(5):rclpy.spin_once(node,timeout_sec=.1)
        print('Published mock observation; inspect /chess/motion/status for acceptance.')
    finally:
        node.destroy_node();rclpy.shutdown()


def operator_main(args=None):
    """Explicit synthetic promotion operator, for unattended simulation tests only."""
    rclpy.init(args=args)
    node = Node('mock_promotion_operator')
    pub = node.create_publisher(String,'/chess/motion/operator_confirm',10)
    def status(msg):
        try:
            data = json.loads(msg.data)
            if data.get('status')=='operator_required':
                send(pub,{'plan_id':data['plan_id'],'token':data['token'],'confirmed':True,'mock':True})
        except (ValueError,TypeError,KeyError):pass
    node.create_subscription(String,'/chess/motion/status',status,10)
    spin(node)
