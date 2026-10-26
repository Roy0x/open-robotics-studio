"""Planar two-link kinematics. Lengths in metres, angles in radians."""
import math
from dataclasses import dataclass


@dataclass(frozen=True)
class Arm:
    l1: float = 0.6
    l2: float = 0.4
    limits: tuple = ((-math.pi, math.pi), (-math.pi, math.pi))

    def __post_init__(self):
        if not all(math.isfinite(v) and v > 0 for v in (self.l1, self.l2)):
            raise ValueError('Link lengths must be finite and positive')
        if len(self.limits) != 2 or any(
            len(pair) != 2 or not all(math.isfinite(v) for v in pair) or pair[0] > pair[1]
            for pair in self.limits
        ):
            raise ValueError('Each joint needs finite, ordered [min, max] limits')
        if any(lo < -math.pi or hi > math.pi for lo, hi in self.limits):
            raise ValueError('Joint limits must lie within [-pi, pi]')

    def forward(self, q1, q2):
        elbow = (self.l1 * math.cos(q1), self.l1 * math.sin(q1))
        tip = (elbow[0] + self.l2 * math.cos(q1 + q2),
               elbow[1] + self.l2 * math.sin(q1 + q2))
        return elbow, tip

    def solve(self, x, y):
        if not all(math.isfinite(v) for v in (x, y)):
            raise ValueError('Target coordinates must be finite')
        radius = math.hypot(x, y)
        result = {'target_m': [x, y], 'links_m': [self.l1, self.l2],
                  'joint_limits_rad': self.limits, 'solutions': []}
        if radius > self.l1 + self.l2 + 1e-12 or radius < abs(self.l1 - self.l2) - 1e-12:
            return dict(result, status='unreachable')
        # Equal links folded onto the origin have infinitely many shoulder angles.
        if radius < 1e-12 and abs(self.l1 - self.l2) < 1e-12:
            return dict(result, status='degenerate', reason='Infinitely many shoulder angles; choose another target')
        c2 = (x*x + y*y - self.l1*self.l1 - self.l2*self.l2) / (2*self.l1*self.l2)
        angle = math.acos(max(-1.0, min(1.0, c2)))
        for q2 in (angle, -angle):
            q1 = math.atan2(y, x) - math.atan2(self.l2*math.sin(q2), self.l1+self.l2*math.cos(q2))
            q1 = (q1 + math.pi) % (2*math.pi) - math.pi
            # Treat +/- pi as equivalent at a joint-limit boundary.
            choices = []
            for q, (lo, hi) in zip((q1, q2), self.limits):
                equivalent = [q + k*2*math.pi for k in (-1, 0, 1)]
                choices.append(next((v for v in equivalent if lo-1e-12 <= v <= hi+1e-12), None))
            if None in choices:
                continue
            if any(all(abs(a-b) < 1e-10 for a,b in zip(choices, s['joints_rad'])) for s in result['solutions']):
                continue
            elbow, tip = self.forward(*choices)
            result['solutions'].append({'joints_rad': choices,
                'joints_deg': [math.degrees(q) for q in choices],
                'elbow_m': elbow, 'tip_m': tip,
                'position_error_m': math.hypot(tip[0]-x, tip[1]-y)})
        return dict(result, status='reachable' if result['solutions'] else 'joint_limit_blocked')
