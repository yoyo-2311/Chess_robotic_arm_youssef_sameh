"""MoveIt service/action client. Execution succeeds only after controller result + feedback."""
import copy
import threading
import uuid
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


class PreflightRejected(MotionError):
    """Pickup infeasible and its scoped planning-scene permission restored."""
    pass


def requires_reconciliation(error, execution_started=False, attached=False, blocked=False, stopped=False):
    """A stop or ambiguous gripper result must not be cleared by worker cleanup."""
    return bool(execution_started or attached or blocked or stopped or str(error).startswith(('gripper_', 'estop', 'service_or_action_timeout')))


def proven_safe_rejection(error, execution_before, execution_after, gripper_before, gripper_after, partial):
    return (isinstance(error,PreflightRejected) and not partial
            and execution_before==execution_after and gripper_before==gripper_after)


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

    def wait(self, future, timeout=30, motion=False, operation=None):
        until = time.monotonic() + timeout
        operation_id = uuid.uuid4().hex
        if operation:
            self.event('operation', operation=operation, operation_id=operation_id,
                       phase='waiting', timeout_seconds=float(timeout))
        try:
            while not future.done():
                if motion and self.stop.is_set():
                    if self.active_goal:
                        self.active_goal.cancel_goal_async()
                    raise MotionError('estop: execution state requires reconciliation')
                if time.monotonic() > until:
                    if self.active_goal:
                        self.active_goal.cancel_goal_async()
                    raise MotionError('service_or_action_timeout: '+str(operation or 'ROS service'))
                time.sleep(.01)
            if future.exception():
                raise MotionError(str(future.exception()))
            result = future.result()
        except Exception as exc:
            if operation:
                self.event('operation', operation=operation, operation_id=operation_id,
                           phase='failed', reason=str(exc))
            raise
        if operation:
            self.event('operation', operation=operation, operation_id=operation_id, phase='completed')
        return result

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
        if not self.wait(self.scene_client.call_async(ApplyPlanningScene.Request(scene=scene)), operation='apply collision scene').success:
            raise MotionError('planning_scene_update_rejected')

    def valid(self, state=None):
        if not self.valid_client.wait_for_service(timeout_sec=10):
            raise MotionError('state_validity_service_unavailable')
        req = GetStateValidity.Request(robot_state=state or self.state())
        response = self.wait(self.valid_client.call_async(req))
        return response.valid, [c.contact_body_1 + '/' + c.contact_body_2 for c in response.contacts]

    def park(self, arm, execute=True):
        return self.joint_goal(arm, [self.geometry.home(arm)], execute=execute)

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
            result = self.wait(self.plan_client.call_async(req), motion=True, operation=f'plan {arm} approach/home (attempt {attempt+1}/5)').motion_plan_response
            if result.error_code.val == 1 and result.trajectory.joint_trajectory.points:
                break
        else:
            raise MotionError(f'planning_failed: code={result.error_code.val}')
        if execute:
            self.execute(result.trajectory)
        return result.trajectory

    def _cartesian_request(self, arm, target, start):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'world'
        req.start_state = start
        req.group_name, req.link_name = arm, arm + '_tool0'
        req.avoid_collisions = True
        req.max_step = self.geometry.raw['motion']['cartesian_step']
        req.prismatic_jump_threshold, req.revolute_jump_threshold = .02, .2
        req.max_velocity_scaling_factor = self.geometry.raw['motion']['velocity_scale']
        req.max_acceleration_scaling_factor = self.geometry.raw['motion']['acceleration_scale']
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, target)
        joints = dict(zip(start.joint_state.name, start.joint_state.position))
        rot = self.model.matrix(arm + '_tool0', joints)[:3, :3]
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, Rotation.from_matrix(rot).as_quat())
        req.waypoints = [pose]
        return req

    def descent_candidates(self, arm, above, grasp):
        """Plan only to IK branches whose complete descent is collision-free.

        A reachable high endpoint does not establish a safe low grasp. Probe each
        branch in the current collision scene, including attached payloads and
        the other arm. These synthetic start states are NEVER executed. Replan
        the actual descent from controller feedback after arriving above it.
        """
        feasible, diagnostics = [], []
        for solution in self.model.solutions(arm, np.asarray(above)):
            state = copy.deepcopy(self.state())
            joints = dict(zip(state.joint_state.name, state.joint_state.position))
            joints.update({f'{arm}_joint{i}':v for i,v in enumerate(solution['q'],1)})
            state.joint_state.position = [float(joints[n]) for n in state.joint_state.name]
            request = self._cartesian_request(arm, grasp, state)
            result = self.wait(self.cart_client.call_async(request), motion=True,
                               operation=f'check {arm} descent branch {len(diagnostics)+1}')
            item = {'approach_joints':solution['q'], 'fraction':float(result.fraction),
                    'code':int(result.error_code.val)}
            if result.error_code.val == 1 and result.fraction >= self.geometry.raw['motion']['minimum_cartesian_fraction']:
                feasible.append(solution['q'])
            else:
                # Keep diagnostics on THIS branch. Mixing every endpoint IK
                # solution into each candidate incorrectly attributes contacts.
                _, start_contacts = self.valid(state)
                item['start_contacts'] = start_contacts
                endpoints = self.model.solutions(arm,np.asarray(grasp))
                if endpoints:
                    candidate = min(endpoints,key=lambda s:float(np.linalg.norm(np.asarray(s['q'])[1:]-np.asarray(solution['q'])[1:])))
                    end = copy.deepcopy(state)
                    values = dict(zip(end.joint_state.name,end.joint_state.position))
                    values.update({f'{arm}_joint{i}':v for i,v in enumerate(candidate['q'],1)})
                    end.joint_state.position = [float(values[n]) for n in end.joint_state.name]
                    _, contacts = self.valid(end)
                    item['endpoint_joints'] = candidate['q']
                    item['endpoint_contacts'] = contacts
            diagnostics.append(item)
        self.event('descent_checked', arm=arm, above=list(map(float,above)), target=list(map(float,grasp)),
                   feasible_branches=len(feasible), candidates=diagnostics)
        if not feasible:
            contacts = sorted({c for d in diagnostics for c in d.get('endpoint_contacts',[])})
            raise MotionError(f'no_collision_free_descent: {arm} target={list(map(float,grasp))}; '
                              f'contacts={contacts}; fractions={[round(d["fraction"],4) for d in diagnostics]}')
        return feasible

    def approach(self, arm, above, grasp, execute=True):
        return self.joint_goal(arm,self.descent_candidates(arm,above,grasp),execute)

    def trajectory_end_state(self, trajectory):
        state=copy.deepcopy(self.state())
        values=dict(zip(state.joint_state.name,state.joint_state.position))
        values.update(dict(zip(trajectory.joint_trajectory.joint_names,trajectory.joint_trajectory.points[-1].positions)))
        state.joint_state.position=[float(values[n]) for n in state.joint_state.name]
        return state

    def plan_vertical(self, arm, target, start):
        req=self._cartesian_request(arm,target,start)
        result=self.wait(self.cart_client.call_async(req),motion=True,operation=f'plan {arm} vertical motion')
        if result.error_code.val != 1 or result.fraction < self.geometry.raw['motion']['minimum_cartesian_fraction']:
            raise MotionError(f'cartesian_path_incomplete: {arm} target={list(map(float,target))} '
                              f'fraction={result.fraction:.4f} code={result.error_code.val}')
        return result.solution

    def vertical(self, arm, target):
        """Check the full actual Cartesian path; partial paths never execute."""
        self.execute(self.plan_vertical(arm,target,self.state()))

    def execute(self, trajectory):
        self.execution_epoch += 1
        if self.stop.is_set():
            raise MotionError('estop_active')
        if not self.execute_client.wait_for_server(timeout_sec=10):
            raise MotionError('execution_controller_unavailable')
        handle = self.wait(self.execute_client.send_goal_async(ExecuteTrajectory.Goal(trajectory=trajectory)), motion=True, operation='controller goal acceptance')
        if not handle.accepted:
            raise MotionError('trajectory_execution_rejected')
        self.active_goal = handle
        duration = trajectory.joint_trajectory.points[-1].time_from_start
        try:
            result = self.wait(handle.get_result_async(), duration.sec + duration.nanosec/1e9 + 20., motion=True, operation='controller execution result').result
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
