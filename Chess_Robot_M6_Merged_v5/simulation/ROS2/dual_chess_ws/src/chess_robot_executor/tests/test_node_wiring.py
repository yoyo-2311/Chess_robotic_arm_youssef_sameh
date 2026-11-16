"""Exercises chess_robot_executor.node.main() against a hand-made stub of rclpy/std_msgs (no ROS needed).
This checks the WIRING (parameters, topics, timers, ack suppression in dry-run). It does not replace a test on a ROS machine."""
import json
import sys
import types

import pytest

chess = pytest.importorskip('chess')


class _Msg:
    def __init__(self, data=None):
        self.data = data


class _Param:
    def __init__(self, value):
        self.value = value


def install_fake_ros(overrides, scenario):
    state = types.SimpleNamespace(published=[], subs={}, timers=[], params={}, logs=[], node=None)

    class FakeNode:
        def __init__(self, name):
            state.node = self

        def declare_parameter(self, name, default):
            state.params[name] = overrides.get(name, default)

        def get_parameter(self, name):
            return _Param(state.params[name])

        def create_publisher(self, typ, topic, depth):
            return types.SimpleNamespace(publish=lambda m, t=topic: state.published.append((t, m.data)))

        def create_subscription(self, typ, topic, cb, depth):
            state.subs[topic] = cb

        def create_timer(self, period, cb):
            state.timers.append((period, cb))

        def get_logger(self):
            return types.SimpleNamespace(info=lambda s: state.logs.append(s), warn=lambda s: state.logs.append(s))

        def destroy_node(self):
            pass

    rclpy = types.ModuleType('rclpy')
    rclpy.init = lambda args=None: None
    rclpy.ok = lambda: False
    rclpy.shutdown = lambda: None
    rclpy.spin = lambda node: scenario(state)
    node_mod = types.ModuleType('rclpy.node'); node_mod.Node = FakeNode
    std = types.ModuleType('std_msgs'); msgs = types.ModuleType('std_msgs.msg')
    msgs.Bool, msgs.Float64, msgs.String = _Msg, _Msg, _Msg
    sys.modules.update({'rclpy': rclpy, 'rclpy.node': node_mod, 'std_msgs': std, 'std_msgs.msg': msgs})
    return state


@pytest.fixture(autouse=True)
def _restore_modules():
    saved = {k: sys.modules.get(k) for k in ('rclpy', 'rclpy.node', 'std_msgs', 'std_msgs.msg')}
    yield
    for k, v in saved.items():
        if v is None:
            sys.modules.pop(k, None)
        else:
            sys.modules[k] = v


def run_main(overrides, scenario):
    from chess_robot_executor import node
    state = install_fake_ros(overrides, scenario)
    node.main([])
    return state


def tick_until_done(state, max_ticks=20000):
    tick = next(cb for p, cb in state.timers if abs(p - 0.02) < 1e-9)
    for _ in range(max_ticks):
        tick()
        if any('plan_done' in d for t, d in state.published if t == '/chess/executor/status' and isinstance(d, str)):
            return


def plan_json(uci='e2e4', fen=chess.STARTING_FEN, pid='n1'):
    from chess_brain_handover import core
    p = core.describe_move(chess.Board(fen), chess.Move.from_uci(uci)).to_dict()
    p['plan_id'] = pid
    return json.dumps(p)


def test_dry_run_default_logs_but_never_acks():
    def scenario(st):
        st.subs['/chess/move_plan'](_Msg(plan_json()))
        tick_until_done(st)
    st = run_main({}, scenario)
    topics = [t for t, _ in st.published]
    assert '/chess/robot_ack' not in topics
    assert any('ack_suppressed_dry_run' in d for t, d in st.published if t == '/chess/executor/status')
    assert any('[dry-run]' in l for l in st.logs) and any('PLACEHOLDER' in l for l in st.logs)
    assert not any(t.endswith('/cmd_pos') for t in topics)


def test_real_mode_publishes_commands_and_ack():
    def scenario(st):
        st.subs['/chess/move_plan'](_Msg(plan_json()))
        tick_until_done(st)
    st = run_main({'dry_run': False, 'sink': 'cmd_pos'}, scenario)
    acks = [json.loads(d) for t, d in st.published if t == '/chess/robot_ack']
    assert len(acks) == 1 and acks[0]['success'] and acks[0]['plan_id'] == 'n1'
    cmd_topics = {t for t, _ in st.published if t.endswith('/cmd_pos')}
    assert cmd_topics == {f'/robot1/{j}/cmd_pos' for j in ('shoulder_joint', 'elbow_joint', 'z_joint', 'gripper_finger_joint')}


def test_invalid_json_and_estop():
    def scenario(st):
        st.subs['/chess/move_plan'](_Msg('{not json'))
        st.subs['/chess/executor/estop'](_Msg(True))
        st.subs['/chess/move_plan'](_Msg(plan_json(pid='e1')))
    st = run_main({'dry_run': False, 'sink': 'cmd_pos'}, scenario)
    status = [json.loads(d) for t, d in st.published if t == '/chess/executor/status']
    assert any(s.get('event') == 'plan_ignored' and 'invalid JSON' in s['reason'] for s in status)
    acks = [json.loads(d) for t, d in st.published if t == '/chess/robot_ack']
    assert len(acks) == 1 and not acks[0]['success'] and acks[0]['reason'] == 'estop_or_recovery_required'


def test_speed_scale_parameter_overrides_config():
    def scenario(st):
        st.subs['/chess/move_plan'](_Msg(plan_json()))
        tick_until_done(st)
    slow = run_main({'dry_run': False, 'sink': 'cmd_pos', 'speed_scale': 0.25}, scenario)
    fast = run_main({'dry_run': False, 'sink': 'cmd_pos', 'speed_scale': 1.0}, scenario)
    n = lambda s: sum(1 for t, _ in s.published if t == '/robot1/shoulder_joint/cmd_pos')
    assert n(slow) > 1.5 * n(fast)


def test_safety_ready_is_opt_in_and_follows_state():
    seen = []

    def scenario(st):
        periodic = next(cb for p, cb in st.timers if abs(p - 0.5) < 1e-9)
        tick = next(cb for p, cb in st.timers if abs(p - 0.02) < 1e-9)
        periodic(); seen.append(('idle', [d for t, d in st.published if t == '/chess/safety_ready'][-1]))
        st.subs['/chess/move_plan'](_Msg(plan_json(pid='s1')))
        for _ in range(100):
            tick()
        periodic(); seen.append(('busy', [d for t, d in st.published if t == '/chess/safety_ready'][-1]))
        st.subs['/chess/executor/estop'](_Msg(True))
        periodic(); seen.append(('estop', [d for t, d in st.published if t == '/chess/safety_ready'][-1]))

    run_main({'dry_run': False, 'sink': 'cmd_pos', 'publish_safety_ready': True}, scenario)
    assert seen == [('idle', True), ('busy', False), ('estop', False)]
    off = run_main({}, lambda s: next(cb for p, cb in s.timers if abs(p - 0.5) < 1e-9)())
    assert not any(t == '/chess/safety_ready' for t, _ in off.published)
