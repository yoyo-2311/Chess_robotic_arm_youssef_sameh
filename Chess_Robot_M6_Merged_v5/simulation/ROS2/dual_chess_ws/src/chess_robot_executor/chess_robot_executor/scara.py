"""Analytic SCARA kinematics (two vertical-axis revolute joints + prismatic Z), pure Python.

Robot frame R: shoulder axis at the origin, +x forward at q1 = q2 = 0.
  x = l1 cos q1 + l2 cos(q1+q2),  y = l1 sin q1 + l2 sin(q1+q2),  z = tool height (prismatic joint).
Tool yaw = q1 + q2 (the gripper is assumed rotation-symmetric for chess pieces).
Elbow 'left' means q2 > 0, 'right' means q2 < 0.
"""
from __future__ import annotations
import math
from dataclasses import dataclass

_TOL = 1e-9


def wrap(angle: float) -> float:
    """Wrap to (-pi, pi]."""
    a = math.fmod(angle + math.pi, 2.0 * math.pi)
    if a <= 0.0:
        a += 2.0 * math.pi
    return a - math.pi


@dataclass(frozen=True)
class Solution:
    q1: float
    q2: float
    z: float
    elbow: str


@dataclass(frozen=True)
class ScaraParams:
    l1: float
    l2: float
    q1_min: float
    q1_max: float
    q2_min: float
    q2_max: float
    z_min: float
    z_max: float
    reach_margin: float = 0.003      # keep this far from full extension / full fold (singularities)

    def fk(self, q1: float, q2: float, z: float) -> tuple[float, float, float]:
        return (self.l1 * math.cos(q1) + self.l2 * math.cos(q1 + q2),
                self.l1 * math.sin(q1) + self.l2 * math.sin(q1 + q2), z)

    @property
    def r_max(self) -> float:
        return self.l1 + self.l2 - self.reach_margin

    @property
    def r_min(self) -> float:
        return abs(self.l1 - self.l2) + self.reach_margin

    def within_limits(self, q1: float, q2: float, z: float) -> bool:
        return (self.q1_min - _TOL <= q1 <= self.q1_max + _TOL and self.q2_min - _TOL <= q2 <= self.q2_max + _TOL
                and self.z_min - _TOL <= z <= self.z_max + _TOL)

    def ik(self, x: float, y: float, z: float) -> list[Solution]:
        """All joint-limit-valid solutions (0, 1 or 2), left elbow first. Empty if unreachable."""
        if not (self.z_min - _TOL <= z <= self.z_max + _TOL):
            return []
        r = math.hypot(x, y)
        if r > self.r_max or r < self.r_min:
            return []
        c2 = (r * r - self.l1 ** 2 - self.l2 ** 2) / (2.0 * self.l1 * self.l2)
        c2 = max(-1.0, min(1.0, c2))
        out = []
        for elbow, sign in (('left', 1.0), ('right', -1.0)):
            q2 = sign * math.acos(c2)
            q1 = wrap(math.atan2(y, x) - math.atan2(self.l2 * math.sin(q2), self.l1 + self.l2 * math.cos(q2)))
            if self.within_limits(q1, q2, z):
                out.append(Solution(q1, q2, z, elbow))
        return out

    def ik_best(self, x: float, y: float, z: float, seed: tuple[float, float] | None = None,
                preference: str = 'left') -> Solution | None:
        """Pick one solution: nearest to `seed` (joint space) if given, else the preferred elbow, else any."""
        sols = self.ik(x, y, z)
        if not sols:
            return None
        if seed is not None:
            return min(sols, key=lambda s: (s.q1 - seed[0]) ** 2 + (s.q2 - seed[1]) ** 2)
        for s in sols:
            if s.elbow == preference:
                return s
        return sols[0]

    def reachable(self, x: float, y: float, z: float) -> bool:
        return bool(self.ik(x, y, z))
