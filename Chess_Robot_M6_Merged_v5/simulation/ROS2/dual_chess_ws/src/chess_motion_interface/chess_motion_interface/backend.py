"""MoveIt service/action client. Execution succeeds only after controller result + feedback."""
import threading
import time
import numpy as np
from scipy.spatial.transform import Rotation
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from moveit_msgs.msg import Constraints, JointConstraint, RobotState
from moveit_msgs.srv import GetMotionPlan, GetCartesianPath, ApplyPlanningScene, GetStateValidity
from moveit_msgs.action import ExecuteTrajectory
from rclpy.action import ActionClient


class MotionError(RuntimeError):
    pass


def requires_reconciliation(error, execution_started=False, attached=False, blocked=False, stopped=False):
    """A stop or ambiguous gripper result must not be cleared by worker cleanup."""
    return bool(execution_started or attached or blocked or stopped or str(error).startswith(('gripper_', 'estop')))


class Backend:
    def __init__(self, node, model, geometry, event):
        self.node, self.model, self.geometry, self.event = node, model, geometry, event
        self.latest = None
        self.received_at = 0.
        self.stop = threading.Event()
        self.active_goal = None
        self.execution_epoch = 0
        node.create_subscription(JointState, '/joint_states', self._state, 10, callback_group=node.io_group)
        self.plan_client = node.create_client(GetMotionPlan, '/plan_kinematic_path', callback_group=node.io_group)
        self.cart_client = node.create_client(GetCartesianPath, '/compute_cartesian_path', callback_group=node.io_group)
        self.scene_client = node.create_client(ApplyPlanningScene, '/apply_planning_scene', callback_group=node.io_group)
        self.valid_client = node.create_client(GetStateValidity, '/check_state_validity', callback_group=node.io_group)
        self.execute_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory', callback_group=node.io_group)

    def _state(self, msg):
        self.latest, self.received_at = msg, time.monotonic()

    def wait(self, future, timeout=30, motion=False):
        until = time.monotonic() + timeout
        while not future.done():
            if motion and self.stop.is_set():
                if self.active_goal:
                    self.active_goal.cancel_goal_async()
                raise MotionError('estop: execution state requires reconciliation')
            if time.monotonic() > until:
                if self.active_goal:
                    self.active_goal.cancel_goal_async()
                raise MotionError('service_or_action_timeout')
            time.sleep(.01)
        if future.exception():
            raise MotionError(str(future.exception()))
        return future.result()

    def state(self):
        if self.latest is None or time.monotonic()-self.received_at > 2.:
            raise MotionError('missing_or_stale_joint_feedback')
        state = RobotState()
        state.joint_state = self.latest
        state.is_diff = True  # retain currently attached objects from the planning scene
        return state

    def joints(self):
        s = self.state().joint_state
        return dict(zip(s.name, s.position))

    def position(self, arm):
        return self.model.fk(arm + '_tool0', self.joints())

    def apply(self, scene):
        if not self.scene_client.wait_for_service(timeout_sec=10):
            raise MotionError('planning_scene_service_unavailable')
        if not self.wait(self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene))).success:
            raise MotionError('planning_scene_update_rejected')

    def valid(self, state=None):
        req = GetStateValidity.Request(robot_state=state or self.state())
        response = self.wait(self.valid_client.call_async(req))
        return response.valid, [c.contact_body_1 + '/' + c.contact_body_2 for c in response.contacts]

    def park(self, arm, execute=True):
        return self.joint_goal(arm, [[.09, 0., 0.]], execute=execute)

    def move(self, arm, target, execute=True):
        solutions = self.model.solutions(arm, np.asarray(target))
        if not solutions:
            raise MotionError(f'unreachable_target: {arm} {np.asarray(target).tolist()}')
        return self.joint_goal(arm, [s['q'] for s in solutions], execute)

    def joint_goal(self, arm, solutions, execute=True):
        if self.stop.is_set():
            raise MotionError('estop_active')
        req = GetMotionPlan.Request()
        m = req.motion_plan_request
        m.group_name, m.pipeline_id, m.planner_id = arm, 'ompl', 'RRTConnectkConfigDefault'
        m.allowed_planning_time, m.num_planning_attempts = 5., 3
        m.max_velocity_scaling_factor = self.geometry.raw['motion']['velocity_scale']
        m.max_acceleration_scaling_factor = self.geometry.raw['motion']['acceleration_scale']
        m.start_state = self.state()
        for values in solutions:
            goal = Constraints()
            for i, value in enumerate(values, 1):
                goal.joint_constraints.append(JointConstraint(joint_name=f'{arm}_joint{i}', position=float(value), tolerance_above=1e-4, tolerance_below=1e-4, weight=1.))
            m.goal_constraints.append(goal)
        if not self.plan_client.wait_for_service(timeout_sec=10):
            raise MotionError('moveit_planning_unavailable')
        for attempt in range(5):
            result = self.wait(self.plan_client.call_async(req), motion=True).motion_plan_response
            if result.error_code.val == 1 and result.trajectory.joint_trajectory.points:
                break
        else:
            raise MotionError(f'planning_failed: code={result.error_code.val}')
        if execute:
            self.execute(result.trajectory)
        return result.trajectory

    def vertical(self, arm, target):
        """Collision-checked straight Cartesian descent/lift, preserving actual tool orientation."""
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.start_state = self.state()
        req.group_name, req.link_name = arm, arm + '_tool0'
        req.avoid_collisions = True
        req.max_step = self.geometry.raw['motion']['cartesian_step']
        req.prismatic_jump_threshold, req.revolute_jump_threshold = .02, .2
        req.max_velocity_scaling_factor = self.geometry.raw['motion']['velocity_scale']
        req.max_acceleration_scaling_factor = self.geometry.raw['motion']['acceleration_scale']
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, target)
        rot = self.model.matrix(arm + '_tool0', self.joints())[:3, :3]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, Rotation.from_matrix(rot).as_quat())
        req.waypoints = [pose]
        result = self.wait(self.cart_client.call_async(req), motion=True)
        if result.error_code.val != 1 or result.fraction < self.geometry.raw['motion']['minimum_cartesian_fraction']:
            raise MotionError(f'cartesian_path_incomplete: fraction={result.fraction:.4f} code={result.error_code.val}')
        self.execute(result.solution)

    def execute(self, trajectory):
        self.execution_epoch += 1
        if self.stop.is_set():
            raise MotionError('estop_active')
        if not self.execute_client.wait_for_server(timeout_sec=10):
            raise MotionError('execution_controller_unavailable')
        handle = self.wait(self.execute_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory)), motion=True)
        if not handle.accepted:
            raise MotionError('trajectory_execution_rejected')
        self.active_goal = handle
        duration = trajectory.joint_trajectory.points[-1].time_from_start
        try:
            result = self.wait(handle.get_result_async(), duration.sec + duration.nanosec/1e9 + 20., motion=True).result
            if result.error_code.val != 1:
                raise MotionError(f'controller_execution_failed: code={result.error_code.val}')
            expected = dict(zip(trajectory.joint_trajectory.joint_names, trajectory.joint_trajectory.points[-1].positions))
            until = time.monotonic()+2.
            while True:
                actual = self.joints()
                if all(abs(actual[name]-value) < (.001 if name.endswith('joint1') else .01) for name, value in expected.items()):
                    break
                if time.monotonic() > until:
                    raise MotionError('controller_result_feedback_mismatch')
                time.sleep(.02)
        finally:
            self.active_goal = None
