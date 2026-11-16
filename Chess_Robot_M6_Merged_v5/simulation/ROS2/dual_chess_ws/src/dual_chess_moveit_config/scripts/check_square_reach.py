#!/usr/bin/env python3
"""Numerical IK from expanded CAD URDF; optional MoveIt whole-scene collision checks.

Run with --urdf /path/model.urdf or use the installed description via xacro.
Validates square centres at 30/80 mm world height, not grasp contact or paths.
The other arm stays at its configured mock home during collision checks.
"""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


from chess_motion_interface.model import Model, transform


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--urdf')
    parser.add_argument('--geometry-file')
    parser.add_argument('--heights', nargs='+', type=float, default=[.03,.08])
    parser.add_argument('--relative', action='store_true', help='Interpret heights above the playing surface along the board normal')
    parser.add_argument('--output')
    parser.add_argument('--collision', action='store_true')
    parser.add_argument('--plan', action='store_true', help='Also check collision-free paths from both arms parked; implies --collision')
    args = parser.parse_args()
    args.collision = args.collision or args.plan
    if args.urdf:
        root = ET.parse(args.urdf).getroot()
    else:
        import xacro
        from ament_index_python.packages import get_package_share_directory
        root = ET.fromstring(xacro.process_file(str(Path(get_package_share_directory('dual_chess_description')) / 'urdf/dual_chess.urdf.xacro'), mappings={'geometry_file':args.geometry_file} if args.geometry_file else {}).toxml())
    from chess_motion_interface.geometry import Geometry
    from ament_index_python.packages import get_package_share_directory
    config_path = args.geometry_file or str(Path(get_package_share_directory('dual_chess_description'))/'config/chess_geometry.yaml')
    geometry = Geometry.load(config_path)
    model = Model(root)
    node = None
    if args.collision:
        import rclpy
        from moveit_msgs.srv import GetStateValidity
        rclpy.init()
        node = rclpy.create_node('square_reach_check')
        from rclpy.qos import QoSProfile, DurabilityPolicy
        from std_msgs.msg import String
        snapshots, descriptions = [], []
        qos = QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String,'/chess/geometry',lambda m:snapshots.append(json.loads(m.data)),qos)
        node.create_subscription(String,'/robot_description',lambda m:descriptions.append(m.data),qos)
        import time
        deadline = time.monotonic()+10
        while (not snapshots or not descriptions) and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.1)
        if not snapshots or not descriptions:
            raise RuntimeError('Live geometry/robot description unavailable; wait for demo.launch.py to initialize')
        geometry = Geometry(snapshots[-1]['configuration'])
        if not args.urdf:
            model = Model(ET.fromstring(descriptions[-1]))
        client = node.create_client(GetStateValidity, '/check_state_validity')
        if not client.wait_for_service(timeout_sec=30):
            raise RuntimeError('Start demo.launch.py first: /check_state_validity is unavailable')

        if args.plan:
            from moveit_msgs.srv import GetMotionPlan
            from moveit_msgs.msg import Constraints, JointConstraint
            planner = node.create_client(GetMotionPlan, '/plan_kinematic_path')
            if not planner.wait_for_service(timeout_sec=30):
                raise RuntimeError('MoveIt planning service is unavailable')

            def plan_from_home(arm, solutions):
                req = GetMotionPlan.Request()
                m = req.motion_plan_request
                m.group_name, m.pipeline_id, m.planner_id = arm, 'ompl', 'RRTConnectkConfigDefault'
                m.allowed_planning_time = 3.
                m.num_planning_attempts = 3
                m.max_velocity_scaling_factor = .2
                m.max_acceleration_scaling_factor = .2
                m.start_state.joint_state.name = [f'{a}_joint{i}' for a in ['robot1', 'robot2'] for i in [1, 2, 3]]
                m.start_state.joint_state.position = [.09, 0., 0.] * 2
                for solution in solutions:
                    goal = Constraints()
                    for i, value in enumerate(solution['q'], 1):
                        goal.joint_constraints.append(JointConstraint(joint_name=f'{arm}_joint{i}', position=value, tolerance_above=1e-4, tolerance_below=1e-4, weight=1.))
                    m.goal_constraints.append(goal)
                for attempt in range(1, 6):
                    f = planner.call_async(req)
                    rclpy.spin_until_future_complete(node, f, timeout_sec=20)
                    if not f.done():
                        raise RuntimeError('Planning service timed out')
                    result = f.result().motion_plan_response
                    points = len(result.trajectory.joint_trajectory.points)
                    if result.error_code.val == 1 and points:
                        return 1, points, attempt
                return result.error_code.val, points, attempt

        def collision_free(arm, values):
            req = GetStateValidity.Request()
            req.robot_state.is_diff = False
            req.robot_state.joint_state.name = [f'{a}_joint{i}' for a in ['robot1', 'robot2'] for i in [1, 2, 3]]
            req.robot_state.joint_state.position = values + [.09, 0., 0.] if arm == 'robot1' else [.09, 0., 0.] + values
            future = client.call_async(req)
            rclpy.spin_until_future_complete(node, future, timeout_sec=5)
            if not future.done():
                raise RuntimeError('Collision service timed out')
            res = future.result()
            return res.valid, sorted({c.contact_body_1 + ' / ' + c.contact_body_2 for c in res.contacts})
    report = {'scope': 'Configured square-centre reach; simulation-only geometry and uncalibrated geometric TCP.', 'configuration': geometry.raw, 'heights':args.heights, 'relative_to_surface':args.relative, 'collision_checked': args.collision, 'paths_from_home_checked': args.plan, 'arms': {}}
    failed = False
    try:
        for arm in ['robot1', 'robot2']:
            rows = []
            for file in 'abcdefgh':
                for rank in '12345678':
                    square = file + rank
                    xy = geometry.square(square)[:2]
                    for z in args.heights:
                        if args.collision and snapshots[-1]['revision'] != snapshots[0]['revision']:
                            raise RuntimeError('geometry_changed_during_validation: retry with the scene idle')
                        target = geometry.square(square,z) if args.relative else np.array([*xy,z])
                        sols = model.solutions(arm, target)
                        valid = []
                        contacts = []
                        for s in sols:
                            if args.collision:
                                ok, pairs = collision_free(arm, s['q'])
                                contacts.extend(pairs)
                                if not ok:
                                    continue
                            valid.append(s)
                        row = {'square': square, 'height': z, 'world':target.tolist(), 'reachable': bool(sols), 'valid': bool(valid), 'solutions': valid, 'contacts': sorted(set(contacts))}
                        if args.plan and valid:
                            code, points, attempts = plan_from_home(arm, valid)
                            row['planning_error_code'], row['trajectory_points'], row['planning_attempts'] = code, points, attempts
                            row['valid'] = code == 1 and points > 0
                        rows.append(row)
            count = sum(r['valid'] for r in rows)
            report['arms'][arm] = {'passed': count, 'total': len(rows), 'targets': rows}
            print(f'{arm}: {count}/{len(rows)} valid endpoints', flush=True)
            failed |= count != len(rows)
        if args.output:
            Path(args.output).write_text(json.dumps(report, indent=2) + '\n')
    finally:
        if node:
            node.destroy_node()
            rclpy.shutdown()
    raise SystemExit(1 if failed else 0)


if __name__ == '__main__':
    main()
