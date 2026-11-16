"""ROS 2 node wiring for the executor (the logic lives in controller.py and is unit-tested without ROS).

Topics (see docs/INTEGRATION_CONTRACT.md):
  sub  /chess/move_plan              std_msgs/String  JSON plan from the chess brain
  sub  /chess/executor/estop         std_msgs/Bool    true = emergency stop (software), false = release
  sub  /chess/executor/operator_confirm  std_msgs/Bool  true = operator did the manual step (e.g. promotion piece placed)
  sub  /chess/executor/recovered     std_msgs/Bool    true = both arms were parked by hand after an abort
  pub  /chess/robot_ack              std_msgs/String  JSON ack consumed by the brain
  pub  /chess/executor/status        std_msgs/String  JSON status/events
  pub  /chess/safety_ready           std_msgs/Bool    ONLY if publish_safety_ready:=true (idle, parked, no estop); software state, not an interlock
Default is DRY-RUN: commands are only logged, nothing moves and NO ack is sent (so the brain is never told that a move
physically happened). Real commands need  dry_run:=false sink:=cmd_pos  and a model whose joint names match config.
"""
import json
import os


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, Float64, String

    from .config import build_config, load_config_file
    from .controller import ExecutorController
    from .sinks import LogSink

    class CmdPosSink:
        """Publishes Float64 on <topic_pattern> per joint (the Gazebo stand-in / future controller topics)."""

        def __init__(self, node, cfg):
            s = cfg.raw.get('sink', {})
            self.names = s.get('joint_names', {'shoulder': 'shoulder_joint', 'elbow': 'elbow_joint',
                                               'z': 'z_joint', 'gripper': 'gripper_finger_joint'})
            pattern = s.get('topic_pattern', '/{arm}/{joint}/cmd_pos')
            self.pubs = {(arm, k): node.create_publisher(Float64, pattern.format(arm=arm, joint=j), 10)
                         for arm in cfg.arms for k, j in self.names.items()}

        def send(self, arm, cmd):
            for k, v in cmd.items():
                msg = Float64(); msg.data = float(v); self.pubs[(arm, k)].publish(msg)

        def stop(self, arm):
            pass            # position-command stand-in: the last command is simply no longer updated

    class ExecutorNode(Node):
        def __init__(self):
            super().__init__('chess_robot_executor')
            self.declare_parameter('config_file', '')
            self.declare_parameter('sink', 'log')
            self.declare_parameter('dry_run', True)
            self.declare_parameter('ack_in_dry_run', False)
            self.declare_parameter('tick_hz', 50.0)
            self.declare_parameter('status_hz', 2.0)
            self.declare_parameter('speed_scale', -1.0)
            self.declare_parameter('publish_safety_ready', False)
            path = self.get_parameter('config_file').value
            cfg = load_config_file(path) if path and os.path.isfile(path) else build_config()
            scale = float(self.get_parameter('speed_scale').value)
            if scale >= 0.0:
                cfg = build_config({**cfg.raw, 'limits': {**cfg.raw['limits'], 'speed_scale': scale}})
            self.dry_run = bool(self.get_parameter('dry_run').value)
            self.ack_in_dry_run = bool(self.get_parameter('ack_in_dry_run').value)
            use_cmd = self.get_parameter('sink').value == 'cmd_pos' and not self.dry_run
            sink = CmdPosSink(self, cfg) if use_cmd else LogSink(self.get_logger().info)
            self.pub_ack = self.create_publisher(String, '/chess/robot_ack', 10)
            self.pub_status = self.create_publisher(String, '/chess/executor/status', 10)
            self.ctrl = ExecutorController(cfg, sink, self._ack, self._status)
            self.pub_safety = (self.create_publisher(Bool, '/chess/safety_ready', 10)
                               if self.get_parameter('publish_safety_ready').value else None)
            self.create_subscription(String, '/chess/move_plan', self._on_plan, 10)
            self.create_subscription(Bool, '/chess/executor/estop', lambda m: self.ctrl.on_estop(bool(m.data)), 10)
            self.create_subscription(Bool, '/chess/executor/operator_confirm',
                                     lambda m: self.ctrl.on_operator_confirm() if m.data else None, 10)
            self.create_subscription(Bool, '/chess/executor/recovered',
                                     lambda m: self.ctrl.on_recovered() if m.data else None, 10)
            self.dt = 1.0 / float(self.get_parameter('tick_hz').value)
            self.create_timer(self.dt, lambda: self.ctrl.tick(self.dt))
            self.create_timer(1.0 / float(self.get_parameter('status_hz').value), self._periodic)
            self.get_logger().info(f'Robot executor started (dry_run={self.dry_run}, '
                                   f'sink={"cmd_pos" if use_cmd else "log"}); all geometry is PLACEHOLDER')

        def _periodic(self):
            self._status(self.ctrl.status())
            if self.pub_safety is not None:
                msg = Bool(); msg.data = bool(self.ctrl.safety_ready()); self.pub_safety.publish(msg)

        def _send(self, pub, payload):
            msg = String(); msg.data = json.dumps(payload, separators=(',', ':')); pub.publish(msg)

        def _status(self, payload):
            self._send(self.pub_status, payload)

        def _ack(self, payload):
            if self.dry_run and not self.ack_in_dry_run:
                self._status({'event': 'ack_suppressed_dry_run', **payload})
                return
            self._send(self.pub_ack, payload)

        def _on_plan(self, msg):
            try:
                self.ctrl.on_plan(json.loads(msg.data))
            except json.JSONDecodeError as exc:
                self._status({'event': 'plan_ignored', 'reason': f'invalid JSON: {exc}'})

    rclpy.init(args=args)
    node = None
    try:
        node = ExecutorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
