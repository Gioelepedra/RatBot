"""Rendering geometry adapters; no control state or Pygame dependency."""
from dataclasses import dataclass
from functools import lru_cache
import math
import warnings

from scipy.optimize import fsolve
from kinematics_solver import k_solver


@dataclass(frozen=True)
class LegGeometry:
    """Link graph in mm about the mount centre, with positive y down.

    A replacement mechanism supplies its points, links and mount/foot indices
    through the same interface without changing the GUI or projection.
    """
    points: tuple
    links: tuple
    mounts: tuple
    foot: int


class Q8FiveBarGeometry:
    def __init__(self):
        # Original Q8bot dimensions/calibration, independent of control's solver.
        self.solver = k_solver()

    @lru_cache(maxsize=4096)
    def solve(self, q1, q2):
        leg = self.solver
        angles = tuple(map(math.radians, (q1, q2)))
        a = (leg.d + leg.l1 * math.cos(angles[0]), leg.l1 * math.sin(angles[0]))
        b = (leg.l1p * math.cos(angles[1]), leg.l1p * math.sin(angles[1]))
        dx, dy = b[0] - a[0], b[1] - a[1]
        separation = math.hypot(dx, dy)
        if not abs(leg.l2 - leg.l2p) < separation < leg.l2 + leg.l2p:
            raise ValueError('Commanded linkage cannot close')

        def valid(foot):
            # Same assembly branch as the gait IK; reject nonconverged FK output.
            return (all(math.isfinite(v) for v in foot)
                    and abs(math.dist(a, foot) - leg.l2) < .002
                    and abs(math.dist(b, foot) - leg.l2p) < .002
                    and dx * (foot[1] - a[1]) - dy * (foot[0] - a[0]) < 0)

        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            foot = leg.fk_solve(q1, q2)
        if not valid(foot):
            # The legacy [10, 60] seed fails at folded rearing entry. Retry its
            # *unchanged equations* from the outward side of the knee midpoint.
            seed = ((a[0] + b[0]) / 2 + dy / separation * leg.l2,
                    (a[1] + b[1]) / 2 - dx / separation * leg.l2)
            foot, _, _, _ = fsolve(leg._fk_calc, seed, args=angles, full_output=True)
            if not valid(foot):
                raise ValueError('Cannot render commanded linkage')

        points = ((leg.d, 0), (0, 0), a, b, tuple(foot))
        return LegGeometry(tuple((float(x - leg.d / 2), float(y)) for x, y in points),
                           ((0, 2), (1, 3), (2, 4), (3, 4)), (0, 1), 4)
