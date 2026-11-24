"""ROS is confined to this worker thread. Widgets receive immutable event data."""
import json
import queue
import threading
import time
import xml.etree.ElementTree as ET


class RosBridge:
    def __init__(self,domain):
        self.domain=domain;self.events=queue.SimpleQueue();self.commands=queue.Queue()
        self.generation=0;self.mode=None;self.thread=threading.Thread(target=self._run,daemon=True)
        self.thread.start()

    def submit(self, kind, data=None):self.commands.put((kind,data))
    def shutdown(self):self.submit('quit')
    def alive(self):return self.thread.is_alive()

    def _run(self):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile,DurabilityPolicy
        from std_msgs.msg import String,Bool,Float64
        from sensor_msgs.msg import JointState
        from rosgraph_msgs.msg import Clock
        from rcl_interfaces.srv import SetParametersAtomically
        from rclpy.parameter import Parameter
        context=Context();node=None;executor=None
        manual_pub=None;limits={};values={};initial={};pubs={};pending=[]
        try:
            rclpy.init(context=context,domain_id=self.domain)
            node=Node('dual_scara_desktop_bridge',context=context)
            executor=SingleThreadedExecutor(context=context);executor.add_node(node)
            def emit(kind,data,generation=None):self.events.put((self.generation if generation is None else generation,kind,data,time.monotonic()))
            def decode(kind,msg):
                try:
                    data=json.loads(msg.data)
                    if isinstance(data,dict):emit(kind,data)
                except (ValueError,TypeError):emit('log',f'Ignored malformed {kind} message.')
            for topic,kind in [('/chess/state','state'),('/chess/events','event'),('/chess/move_plan','plan'),
                               ('/chess/motion/status','motion'),('/chess/robot_ack','ack'),('/chess/auto/status','auto')]:
                node.create_subscription(String,topic,lambda m,k=kind:decode(k,m),20)
            qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
            node.create_subscription(String,'/chess/geometry',lambda m:decode('geometry',m),qos)
            def description(msg):
                nonlocal limits,values
                try:
                    root=ET.fromstring(msg.data);new={}
                    for j in root.findall('joint'):
                        lim=j.find('limit')
                        if j.get('type') in ['prismatic','revolute'] and lim is not None:
                            new[j.get('name')]={'min':float(lim.get('lower')),'max':float(lim.get('upper')),
                                                'unit':'m' if j.get('type')=='prismatic' else 'rad'}
                    limits=new;values={name:max(v['min'],min(v['max'],values.get(name,initial.get(name,(v['min']+v['max'])/2)))) for name,v in new.items()}
                    emit('limits',new)
                except Exception as exc:emit('log','Cannot parse model limits: '+str(exc))
            node.create_subscription(String,'/robot_description',description,qos)
            node.create_subscription(Bool,'/chess/safety_ready',lambda m:emit('safety',m.data),10)
            node.create_subscription(JointState,'/joint_states',lambda m:emit('joints',dict(zip(m.name,m.position))),10)
            node.create_subscription(Clock,'/clock',lambda m:emit('clock',{'sec':m.clock.sec,'nanosec':m.clock.nanosec}),10)
            for topic in ['/chess/command','/chess/motion/square_command','/chess/motion/operator_confirm']:
                pubs[topic]=node.create_publisher(String,topic,10)
            pubs['/chess/motion/estop']=node.create_publisher(Bool,'/chess/motion/estop',10)
            for arm in ['robot1','robot2']:
                for i in [1,2,3]:
                    topic=f'/{arm}/joint{i}/cmd_pos';pubs[topic]=node.create_publisher(Float64,topic,10)
            emit('connected',self.domain)
            last_graph=last_manual=0
            while context.ok():
                for _ in range(50):
                    try:kind,data=self.commands.get_nowait()
                    except queue.Empty:break
                    if kind=='quit':return
                    if kind=='activate':
                        if manual_pub:node.destroy_publisher(manual_pub);manual_pub=None
                        self.generation=data['generation'];self.mode=data['mode'];limits={};values={};initial=data.get('initial_joints',{})
                        if self.mode=='manual':manual_pub=node.create_publisher(JointState,'/joint_states',10)
                        # Request a fresh brain snapshot; no commands that advance turns here.
                        if self.mode in ['stockfish','mock','logic']:
                            pubs['/chess/command'].publish(String(data='{"action":"state"}'))
                    elif kind=='deactivate':
                        if manual_pub:node.destroy_publisher(manual_pub);manual_pub=None
                        self.mode=None;limits={};values={}
                    elif kind=='publish':
                        if data['generation']!=self.generation or self.mode is None:continue
                        topic,payload=data['topic'],data['payload']
                        pub=pubs[topic]
                        if topic=='/chess/motion/estop':pub.publish(Bool(data=bool(payload)))
                        elif topic.endswith('/cmd_pos'):pub.publish(Float64(data=float(payload)))
                        else:pub.publish(String(data=json.dumps(payload)))
                    elif kind=='joint':
                        if self.mode=='manual' and data['name'] in limits:
                            lim=limits[data['name']];values[data['name']]=max(lim['min'],min(lim['max'],float(data['value'])))
                    elif kind=='parameter':
                        if data['generation']!=self.generation or self.mode is None:continue
                        client=node.create_client(SetParametersAtomically,data['node']+'/set_parameters_atomically')
                        if not client.service_is_ready():
                            emit('parameter_result',{'ok':False,'reason':'Parameter service unavailable: '+data['node'],'request_id':data['request_id']},data['generation']);node.destroy_client(client)
                        else:
                            req=SetParametersAtomically.Request(parameters=[Parameter(data['name'],value=data['value']).to_parameter_msg()])
                            pending.append((client,client.call_async(req),time.monotonic()+5.,data))
                now=time.monotonic()
                if manual_pub and limits and now-last_manual>.05:
                    others=[p for p in node.get_publishers_info_by_topic('/joint_states') if p.node_name!='dual_scara_desktop_bridge']
                    if not others:
                        msg=JointState();msg.header.stamp=node.get_clock().now().to_msg();msg.name=list(values);msg.position=list(values.values());manual_pub.publish(msg)
                    else:emit('conflict','Another joint-state publisher exists. Manual output suspended.')
                    last_manual=now
                if now-last_graph>.5:
                    names=node.get_node_names()
                    command_publishers=[p.node_name for p in node.get_publishers_info_by_topic('/chess/command')]
                    emit('graph',{'nodes':names,'joint_publishers':[p.node_name for p in node.get_publishers_info_by_topic('/joint_states')],
                                  'command_publishers':command_publishers,
                                  'square_subscribers':pubs['/chess/motion/square_command'].get_subscription_count()})
                    last_graph=now
                for client,future,until,data in list(pending):
                    if future.done() or now>until:
                        try:
                            result=future.result().result if future.done() else None
                            emit('parameter_result',{'ok':bool(result and result.successful),'reason':result.reason if result else 'Parameter request timed out','name':data['name'],'request_id':data['request_id']},data['generation'])
                        except Exception as exc:emit('parameter_result',{'ok':False,'reason':str(exc),'request_id':data['request_id']},data['generation'])
                        pending.remove((client,future,until,data));node.destroy_client(client)
                executor.spin_once(timeout_sec=.02)
        except Exception as exc:
            import traceback
            self.events.put((self.generation,'fatal','ROS worker failed: '+traceback.format_exc(),time.monotonic()))
        finally:
            if executor:executor.shutdown()
            if node:node.destroy_node()
            if context.ok():context.shutdown()
