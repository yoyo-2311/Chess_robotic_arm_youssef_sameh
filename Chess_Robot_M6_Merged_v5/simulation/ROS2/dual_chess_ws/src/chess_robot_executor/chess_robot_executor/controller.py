"""Executor state machine (pure Python, deterministic, driven by tick(dt)).

Input : move-plan dicts (the brain's /chess/move_plan), estop / operator / recovery events.
Output: joint commands to a sink, an acknowledgement dict (the brain's /chess/robot_ack format) and status dicts.

Safety properties (see docs/06_control_architecture.md):
  * one plan at a time, one arm moves at a time, the other arm must be parked;
  * every target is checked with IK and joint/z limits BEFORE any motion (pre-flight); a plan that fails pre-flight is
    acknowledged as failed and nothing moves;
  * all axes are rate- and acceleration-limited (trapezoidal profiles); vertical and horizontal moves are never mixed;
  * estop aborts immediately and blocks further plans until `on_recovered()`;
  * acknowledgements are derived from COMMANDED state (no hardware feedback exists yet): see the limitation in the docs.
This is NOT a physical safety interlock.
"""
from __future__ import annotations
import collections
import enum
from dataclasses import dataclass

from .config import ExecutorConfig
from .task_planner import PlanError, TaskPlan, plan_tasks
from .trajectory import Segment, plan_segment


class State(enum.Enum):
    IDLE = 'idle'
    EXECUTING = 'executing'
    WAIT_OPERATOR = 'wait_operator'
    ESTOP = 'estop'


@dataclass
class _Item:
    kind: str                       # 'move' | 'grip' | 'operator'
    duration: float = 0.0
    segment: Segment | None = None
    w0: float = 0.0
    w1: float = 0.0
    note: str = ''


class ExecutorController:
    def __init__(self, cfg: ExecutorConfig, sink, publish_ack, publish_status=None):
        self.cfg, self.sink = cfg, sink
        self._ack = publish_ack
        self._status = publish_status or (lambda s: None)
        self.state = State.IDLE
        self.time = 0.0
        park = cfg.raw['park']
        self.park_q = (park['q1'], park['q2'])
        self.q = {a: [park['q1'], park['q2'], cfg.travel_z - cfg.mounts[a].base_z] for a in cfg.arms}
        self.width = {a: cfg.raw['gripper']['open_width'] for a in cfg.arms}
        self.parked = {a: True for a in cfg.arms}
        self.needs_recovery = False
        self.bin_counts: dict = {}
        self._items: list = []
        self._idx = 0
        self._t = 0.0
        self._task: TaskPlan | None = None
        self._wait = 0.0
        self._done = collections.deque(maxlen=50)
        for a in cfg.arms:                       # the park pose itself must be legal
            if not cfg.scara.within_limits(park['q1'], park['q2'], self.q[a][2]):
                raise ValueError('park pose violates the SCARA joint/z limits')

    # ------------------------------------------------------------------ events
    def on_plan(self, plan) -> None:
        pid = plan.get('plan_id') if isinstance(plan, dict) else None
        if not isinstance(plan, dict):
            self._event('plan_ignored', reason='plan is not a JSON object'); return
        if pid in self._done or (self._task and pid == self._task.plan_id):
            self._event('plan_ignored', plan_id=pid, reason='duplicate plan_id'); return
        if self.state in (State.EXECUTING, State.WAIT_OPERATOR):
            self._event('plan_ignored', plan_id=pid, reason='busy with another plan'); return
        if self.state is State.ESTOP or self.needs_recovery or not all(self.parked.values()):
            self._finish_failed(pid, 'estop_or_recovery_required', parked=False, moved=False); return
        try:
            task = plan_tasks(plan, self.cfg, self.bin_counts)
            items = self._build_items(task)
        except PlanError as exc:
            self._finish_failed(pid, exc.reason, parked=True, moved=False, detail=str(exc)); return
        self._task, self._items, self._idx, self._t = task, items, 0, 0.0
        self.parked[task.arm] = False
        self.state = State.EXECUTING
        self._event('plan_started', plan_id=pid, arm=task.arm, summary=task.summary,
                    items=len(items), estimated_s=round(sum(i.duration for i in items), 2))

    def on_estop(self, active: bool) -> None:
        if active:
            if self.state in (State.EXECUTING, State.WAIT_OPERATOR) and self._task:
                arm, pid = self._task.arm, self._task.plan_id
                self.sink.stop(arm)
                self._task_done(pid)
                self.needs_recovery = True
                self.state = State.ESTOP
                self._send_ack(pid, arm, False, False, abs(self.width[arm] - self.cfg.raw['gripper']['open_width']) < 1e-9,
                               'estop')
                self._event('estop', plan_id=pid)
            elif self.state is State.IDLE:
                self.state = State.ESTOP
                self._event('estop')
        elif self.state is State.ESTOP:
            self.state = State.IDLE
            self._event('estop_released', needs_recovery=self.needs_recovery)

    def on_recovered(self) -> None:
        """Operator confirms both arms were parked by hand and the grippers are open."""
        if self.state is State.IDLE and self.needs_recovery:
            for a in self.cfg.arms:
                self.q[a] = [self.park_q[0], self.park_q[1], self.cfg.travel_z - self.cfg.mounts[a].base_z]
                self.width[a] = self.cfg.raw['gripper']['open_width']
                self.parked[a] = True
            self.needs_recovery = False
            self._event('recovered')

    def on_operator_confirm(self) -> None:
        if self.state is State.WAIT_OPERATOR:
            self._idx += 1
            self.state = State.EXECUTING
            self._event('operator_confirmed')

    # --------------------------------------------------------------------- time
    def tick(self, dt: float) -> None:
        self.time += dt
        if self.state is State.WAIT_OPERATOR:
            self._wait += dt
            if self._wait > self.cfg.raw['timing']['operator_timeout'] and self._task:
                arm, pid = self._task.arm, self._task.plan_id
                self._task_done(pid)
                self.state = State.IDLE
                self.needs_recovery = True
                self._send_ack(pid, arm, False, False, True, 'operator_timeout')
                self._event('operator_timeout', plan_id=pid)
            return
        if self.state is not State.EXECUTING:
            return
        while dt > 1e-12 and self.state is State.EXECUTING:
            if self._idx >= len(self._items):
                self._complete(); return
            it = self._items[self._idx]
            if it.kind == 'operator':
                self.state, self._wait = State.WAIT_OPERATOR, 0.0
                self._event('wait_operator', plan_id=self._task.plan_id, message=it.note); return
            step = min(dt, it.duration - self._t)
            self._t += step
            dt -= step
            self._emit(it)
            if self._t >= it.duration - 1e-12:
                self._idx += 1
                self._t = 0.0
        if self.state is State.EXECUTING and self._idx >= len(self._items):
            self._complete()

    # ---------------------------------------------------------------- internals
    def _build_items(self, task: TaskPlan) -> list:
        cfg, mount = self.cfg, self.cfg.mounts[task.arm]
        q = list(self.q[task.arm]); width = self.width[task.arm]
        vmax, amax = cfg.vmax(), cfg.amax()
        minseg = cfg.raw['timing']['min_segment']
        gtime = cfg.raw['gripper']['time']
        items: list = []
        for st in task.steps:
            if st.kind == 'grip':
                if abs(st.width - width) > 1e-9:
                    items.append(_Item('grip', gtime, w0=width, w1=st.width, note=st.note)); width = st.width
                continue
            if st.kind == 'operator':
                items.append(_Item('operator', note=st.note)); continue
            if st.kind == 'travel':
                xr, yr, _ = mount.to_robot(st.x, st.y)
                sol = cfg.scara.ik_best(xr, yr, q[2], seed=(q[0], q[1]), preference=cfg.elbow_preference)
                if sol is None:
                    raise PlanError('unreachable', f'{st.note} at board ({st.x:.3f}, {st.y:.3f}) for {task.arm}')
                target = [sol.q1, sol.q2, q[2]]
            elif st.kind == 'z':
                zr = st.z - mount.base_z
                if not (cfg.scara.z_min - 1e-9 <= zr <= cfg.scara.z_max + 1e-9):
                    raise PlanError('z_out_of_range', f'{st.note}: z={st.z:.3f} for {task.arm}')
                target = [q[0], q[1], zr]
            elif st.kind == 'park':
                target = [self.park_q[0], self.park_q[1], q[2]]
            else:
                raise PlanError('unknown_step', st.kind)
            seg = plan_segment(q, target, vmax, amax, minseg if any(abs(a - b) > 1e-9 for a, b in zip(q, target)) else 0.0)
            if seg.duration > 0.0:
                items.append(_Item('move', seg.duration, segment=seg, note=st.note))
            q = target
        return items

    def _emit(self, it: _Item) -> None:
        arm = self._task.arm
        if it.kind == 'move':
            self.q[arm] = it.segment.position(self._t)
        elif it.kind == 'grip':
            f = min(1.0, self._t / it.duration) if it.duration > 0 else 1.0
            self.width[arm] = it.w0 + (it.w1 - it.w0) * f
        if self._t >= it.duration - 1e-12:
            if it.kind == 'move':
                self.q[arm] = list(it.segment.q_end)
            elif it.kind == 'grip':
                self.width[arm] = it.w1
        self.sink.send(arm, {'shoulder': self.q[arm][0], 'elbow': self.q[arm][1], 'z': self.q[arm][2],
                             'gripper': self.width[arm]})

    def _complete(self) -> None:
        task = self._task
        arm = task.arm
        released = abs(self.width[arm] - self.cfg.raw['gripper']['open_width']) < 1e-9
        parked = (abs(self.q[arm][0] - self.park_q[0]) < 1e-6 and abs(self.q[arm][1] - self.park_q[1]) < 1e-6)
        self.parked[arm] = parked
        self.bin_counts[arm] = task.bin_count
        self._task_done(task.plan_id)
        self.state = State.IDLE
        self._send_ack(task.plan_id, arm, parked and released, parked, released, None if parked and released else 'not_parked_or_not_released')
        self._event('plan_done', plan_id=task.plan_id, arm=arm)

    def _task_done(self, pid) -> None:
        self._done.append(pid)
        self._task, self._items, self._idx, self._t = None, [], 0, 0.0

    def _finish_failed(self, pid, reason, *, parked, moved, detail='') -> None:
        if pid:
            self._done.append(pid)
        self._send_ack(pid, None, False, parked, True, reason, detail)
        self._event('plan_failed', plan_id=pid, reason=reason, detail=detail)

    def _send_ack(self, pid, arm, success, parked, released, reason, detail='') -> None:
        self._ack({'plan_id': pid, 'success': bool(success), 'arm_parked': bool(parked),
                   'gripper_released': bool(released), 'reason': reason, 'detail': detail, 'arm': arm,
                   'source': 'chess_robot_executor'})

    def _event(self, name: str, **kw) -> None:
        self._status({'event': name, **kw, **self.status()})

    def safety_ready(self) -> bool:
        """Software state only: idle, no estop, no pending recovery, both arms parked. NOT a physical safety interlock."""
        return self.state is State.IDLE and not self.needs_recovery and all(self.parked.values())

    def status(self) -> dict:
        n = len(self._items)
        return {'state': self.state.value, 'plan_id': self._task.plan_id if self._task else None,
                'arm': self._task.arm if self._task else None, 'item': self._idx, 'items': n,
                'progress': round(self._idx / n, 3) if n else None, 'needs_recovery': self.needs_recovery,
                'parked': dict(self.parked), 'bin_counts': dict(self.bin_counts), 'time': round(self.time, 3)}
