"""Quintic (5th-order) polynomial trajectories for smooth limb motion."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from pymunk import Vec2d


@dataclass(frozen=True)
class QuinticTrajectory1D:
    """Position trajectory p(t) = c0 + c1*t + ... + c5*t^5 on [0, T]."""

    coefficients: np.ndarray
    duration: float

    @staticmethod
    def from_boundary_conditions(
        p0: float,
        v0: float,
        a0: float,
        pT: float,
        vT: float,
        aT: float,
        duration: float,
    ) -> "QuinticTrajectory1D":
        coeffs = solve_quintic_coefficients(p0, v0, a0, pT, vT, aT, duration)
        return QuinticTrajectory1D(coefficients=coeffs, duration=duration)

    def _t(self, t: float) -> float:
        return max(0.0, min(float(t), self.duration))

    def position(self, t: float) -> float:
        t = self._t(t)
        c = self.coefficients
        return float(
            c[0]
            + c[1] * t
            + c[2] * t * t
            + c[3] * t**3
            + c[4] * t**4
            + c[5] * t**5
        )

    def velocity(self, t: float) -> float:
        t = self._t(t)
        c = self.coefficients
        return float(
            c[1]
            + 2 * c[2] * t
            + 3 * c[3] * t * t
            + 4 * c[4] * t**3
            + 5 * c[5] * t**4
        )

    def acceleration(self, t: float) -> float:
        t = self._t(t)
        c = self.coefficients
        return float(
            2 * c[2]
            + 6 * c[3] * t
            + 12 * c[4] * t * t
            + 20 * c[5] * t**3
        )


@dataclass(frozen=True)
class QuinticTrajectory2D:
    x: QuinticTrajectory1D
    y: QuinticTrajectory1D

    @property
    def duration(self) -> float:
        return self.x.duration

    @classmethod
    def from_boundary_conditions(
        cls,
        p0: Vec2d,
        v0: Vec2d,
        a0: Vec2d,
        pT: Vec2d,
        vT: Vec2d,
        aT: Vec2d,
        duration: float,
    ) -> "QuinticTrajectory2D":
        return cls(
            x=QuinticTrajectory1D.from_boundary_conditions(
                p0.x, v0.x, a0.x, pT.x, vT.x, aT.x, duration
            ),
            y=QuinticTrajectory1D.from_boundary_conditions(
                p0.y, v0.y, a0.y, pT.y, vT.y, aT.y, duration
            ),
        )

    def position(self, t: float) -> Vec2d:
        return Vec2d(self.x.position(t), self.y.position(t))

    def velocity(self, t: float) -> Vec2d:
        return Vec2d(self.x.velocity(t), self.y.velocity(t))

    def acceleration(self, t: float) -> Vec2d:
        return Vec2d(self.x.acceleration(t), self.y.acceleration(t))


def solve_quintic_coefficients(
    p0: float,
    v0: float,
    a0: float,
    pT: float,
    vT: float,
    aT: float,
    duration: float,
) -> np.ndarray:
    """Solve for [c0..c5] given six boundary conditions over duration T."""
    if duration <= 1e-9:
        raise ValueError("duration must be positive")

    c0 = p0
    c1 = v0
    c2 = a0 / 2.0

    t = duration
    t2 = t * t
    t3 = t2 * t
    t4 = t3 * t
    t5 = t4 * t

    a_matrix = np.array(
        [
            [t3, t4, t5],
            [3 * t2, 4 * t3, 5 * t4],
            [6 * t, 12 * t2, 20 * t3],
        ],
        dtype=float,
    )
    b_vector = np.array(
        [
            pT - c0 - c1 * t - c2 * t2,
            vT - c1 - a0 * t,
            aT - a0,
        ],
        dtype=float,
    )
    c3, c4, c5 = np.linalg.solve(a_matrix, b_vector)
    return np.array([c0, c1, c2, c3, c4, c5], dtype=float)


def estimate_limb_catch_duration(
    foot_pos: Vec2d,
    target_pos: Vec2d,
    foot_vel: Vec2d,
    *,
    min_duration: float,
    max_duration: float,
    extension_speed: float,
    rotation_speed: float,
    typical_reach: float,
) -> float:
    """Heuristic catch horizon from geometry and actuator limits."""
    dist = (target_pos - foot_pos).length
    closing_speed = max(
        foot_vel.length,
        extension_speed,
        rotation_speed * max(typical_reach, 1e-6),
        1e-6,
    )
    duration = dist / closing_speed
    return max(min_duration, min(max_duration, duration))
