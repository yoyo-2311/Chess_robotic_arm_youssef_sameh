#!/usr/bin/env python3
"""Plan to a chess square using the exact CAD chain and MoveIt collision checking.

Requires demo.launch.py. Default: plan only. --execute runs the planned
trajectory through MoveIt's controller manager (mock hardware in the demo).
The 3-DoF arm controls position only; tool orientation follows the CAD chain.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
import rclpy
from rclpy.action import ActionClient
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.action import ExecuteTrajectory
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import xacro
from check_square_reach import Model


def wait(node, future, timeout=30):
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError('MoveIt request timed out')
    return future.result()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--arm', choices=['robot1', 'robot2'], default='robot1')
    ap.add_argument('--square', default='e4')
    ap.add_argument('--height', type=float, default=None, help='Optional absolute world-Z override; default is configured approach height')
    ap.add_argument('--execute', action='store_true')
    ap.add_argument('--home', action='store_true', help='Return this arm to its parked SRDF home before switching arms')
    ap.add_argument('--output')
    args = ap.parse_args()
    if not args.home and (len(args.square) != 2 or args.square[0] not in 'abcdefgh' or args.square[1] not in '12345678'):
        ap.error('square must be a1 through h8')
    rclpy.init()
    node = rclpy.create_node('plan_chess_square')
    latest = []
    sub = node.create_subscription(JointState, '/joint_states', lambda msg: latest.append(msg), 10)
    try:
        from std_msgs.msg import String
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from chess_motion_interface.geometry import Geometry
        import time
        snapshots, descriptions = [], []
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String,'/chess/geometry',lambda m:snapshots.append(json.loads(m.data)),qos)
        node.create_subscription(String,'/robot_description',lambda m:descriptions.append(m.data),qos)
        deadline = time.monotonic()+10
        while (not snapshots or not descriptions) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        if not snapshots or not descriptions:
            raise RuntimeError('Wait for demo.launch.py to initialize its live geometry')
        model = Model(ET.fromstring(descriptions[-1]))
        geometry = Geometry(snapshots[-1]['configuration'])
        if args.home:
            srdf = ET.parse(Path(get_package_share_directory('dual_chess_moveit_config')) / 'config/dual_chess.srdf').getroot()
            state = srdf.find(f"group_state[@name='home'][@group='{args.arm}']")
            solutions = [{'q': [float(j.get('value')) for j in state.findall('joint')]}]
            args.square = 'home'
        else:
            target = geometry.square(args.square, geometry.travel())
            if args.height is not None:
                target[2] = args.height
            solutions = model.solutions(args.arm, target)
        if not solutions:
            raise SystemExit('Target is outside the CAD joint limits')
        client = node.create_client(GetMotionPlan, '/plan_kinematic_path')
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError('Start demo.launch.py first')
        for _ in range(50):
            if latest:
                break
            rclpy.spin_once(node, timeout_sec=.1)
        if not latest:
            raise RuntimeError('No joint states; controllers must be active')
        req = GetMotionPlan.Request()
        req.motion_plan_request.group_name = args.arm
        req.motion_plan_request.pipeline_id = 'ompl'
        req.motion_plan_request.planner_id = 'RRTConnectkConfigDefault'
        req.motion_plan_request.allowed_planning_time = 5.
        req.motion_plan_request.num_planning_attempts = 5
        req.motion_plan_request.max_velocity_scaling_factor = .2
        req.motion_plan_request.max_acceleration_scaling_factor = .2
        req.motion_plan_request.start_state.joint_state = latest[-1]
        for solution in solutions:
            goal = Constraints()
            for i, v in enumerate(solution['q'], 1):
                goal.joint_constraints.append(JointConstraint(joint_name=f'{args.arm}_joint{i}', position=v,
                                                              tolerance_above=1e-4, tolerance_below=1e-4, weight=1.))
            req.motion_plan_request.goal_constraints.append(goal)
        for attempt in range(1, 6):
            result = wait(node, client.call_async(req)).motion_plan_response
            if result.error_code.val == 1 and result.trajectory.joint_trajectory.points:
                break
        report = {'arm': args.arm, 'square': args.square, 'height': args.height,
                  'planning_error_code': result.error_code.val, 'planning_time': result.planning_time, 'planning_requests': attempt,
                  'trajectory_points': len(result.trajectory.joint_trajectory.points), 'executed': False}
        if result.error_code.val != 1:
            raise RuntimeError(f'Planning failed: {report}')
        if args.execute:
            action = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')
            if not action.wait_for_server(timeout_sec=15):
                raise RuntimeError('MoveIt execution action is unavailable')
            goal = ExecuteTrajectory.Goal(trajectory=result.trajectory)
            handle = wait(node, action.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Execution was rejected')
            duration = result.trajectory.joint_trajectory.points[-1].time_from_start
            res = wait(node, handle.get_result_async(), timeout=duration.sec + 30).result
            report['execution_error_code'] = res.error_code.val
            if res.error_code.val != 1:
                raise RuntimeError(f'Execution failed: {report}')
            report['executed'] = True
        print(json.dumps(report, indent=2))
        if args.output:
            Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
    finally:
        node.destroy_subscription(sub)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
