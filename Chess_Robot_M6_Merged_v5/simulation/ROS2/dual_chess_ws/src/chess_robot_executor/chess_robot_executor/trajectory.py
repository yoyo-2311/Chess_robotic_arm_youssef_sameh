"""Time-synchronised trapezoidal joint trajectories (pure Python).

Every axis moves with a symmetric trapezoidal (or triangular) velocity profile; all axes start and stop together
(duration = the slowest axis). Velocity and acceleration never exceed the per-axis limits.
"""
from __future__ import annotations
import math
from dataclasses import dataclass


def axis_min_time(distance: float, vmax: float, amax: float) -> float:
    d = abs(distance)
    if d < 1e-12:
        return 0.0
    if vmax <= 0.0 or amax <= 0.0:
        raise ValueError('vmax and amax must be positive')
    if d <= vmax * vmax / amax:                      # triangular
        return 2.0 * math.sqrt(d / amax)
    return d / vmax + vmax / amax                    # trapezoidal


@dataclass(frozen=True)
class _Axis:
    q0: float
    sign: float
    d: float
    v: float
    a: float
    ta: float

    def pos(self, t: float, T: float) -> float:
        if self.d < 1e-12:
            return self.q0
        if t <= 0.0:
            s = 0.0
        elif t >= T:
            s = self.d
        elif t <= self.ta:
            s = 0.5 * self.a * t * t
        elif t <= T - self.ta:
            s = 0.5 * self.a * self.ta ** 2 + self.v * (t - self.ta)
        else:
            s = self.d - 0.5 * self.a * (T - t) ** 2
        return self.q0 + self.sign * s

    def vel(self, t: float, T: float) -> float:
        if self.d < 1e-12 or t <= 0.0 or t >= T:
            return 0.0
        if t <= self.ta:
            v = self.a * t
        elif t <= T - self.ta:
            v = self.v
        else:
            v = self.a * (T - t)
        return self.sign * v


@dataclass(frozen=True)
class Segment:
    """A synchronised move q0 -> q1 lasting `duration` seconds."""
    duration: float
    axes: tuple[_Axis, ...]
    q_end: tuple[float, ...]

    def position(self, t: float) -> list[float]:
        return [ax.pos(t, self.duration) for ax in self.axes]

    def velocity(self, t: float) -> list[float]:
        return [ax.vel(t, self.duration) for ax in self.axes]


def plan_segment(q0, q1, vmax, amax, min_duration: float = 0.0) -> Segment:
    n = len(q0)
    if not (len(q1) == len(vmax) == len(amax) == n):
        raise ValueError('q0, q1, vmax, amax must have the same length')
    T = max([axis_min_time(q1[i] - q0[i], vmax[i], amax[i]) for i in range(n)] + [min_duration])
    axes = []
    for i in range(n):
        d = abs(q1[i] - q0[i])
        a = amax[i]
        if d < 1e-12 or T <= 0.0:
            axes.append(_Axis(q0[i], 1.0, 0.0, 0.0, a, 0.0))
            continue
        disc = max(0.0, a * a * T * T - 4.0 * a * d)
        v = (a * T - math.sqrt(disc)) / 2.0
        axes.append(_Axis(q0[i], 1.0 if q1[i] >= q0[i] else -1.0, d, v, a, v / a))
    return Segment(T, tuple(axes), tuple(q1))


def sample(segment: Segment, dt: float):
    """[(t, q), ...] every dt seconds including the exact end point."""
    out, k = [], 0
    while k * dt < segment.duration:
        out.append((k * dt, segment.position(k * dt)))
        k += 1
    out.append((segment.duration, list(segment.q_end)))
    return out
