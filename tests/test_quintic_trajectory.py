import math
import unittest

from pymunk import Vec2d

from monkey_bot.quintic_trajectory import (
    QuinticTrajectory1D,
    QuinticTrajectory2D,
    estimate_limb_catch_duration,
)


class QuinticTrajectoryTest(unittest.TestCase):
    def test_boundary_conditions_at_start_and_end(self):
        traj = QuinticTrajectory1D.from_boundary_conditions(
            p0=1.0,
            v0=0.5,
            a0=0.2,
            pT=4.0,
            vT=-0.3,
            aT=0.0,
            duration=2.0,
        )
        self.assertAlmostEqual(traj.position(0.0), 1.0)
        self.assertAlmostEqual(traj.velocity(0.0), 0.5)
        self.assertAlmostEqual(traj.acceleration(0.0), 0.2)
        self.assertAlmostEqual(traj.position(2.0), 4.0)
        self.assertAlmostEqual(traj.velocity(2.0), -0.3)
        self.assertAlmostEqual(traj.acceleration(2.0), 0.0)

    def test_2d_matches_independent_axes(self):
        traj = QuinticTrajectory2D.from_boundary_conditions(
            p0=Vec2d(0, 1),
            v0=Vec2d(1, 0),
            a0=Vec2d(0, 0),
            pT=Vec2d(3, 4),
            vT=Vec2d(0, 0),
            aT=Vec2d(0, 0),
            duration=1.5,
        )
        self.assertAlmostEqual(traj.position(0.0).x, 0.0)
        self.assertAlmostEqual(traj.position(0.0).y, 1.0)
        self.assertAlmostEqual(traj.position(1.5).x, 3.0)
        self.assertAlmostEqual(traj.position(1.5).y, 4.0)
        self.assertAlmostEqual(traj.velocity(1.5).length, 0.0, places=6)

    def test_rest_to_rest_is_smooth(self):
        traj = QuinticTrajectory1D.from_boundary_conditions(
            p0=0.0,
            v0=0.0,
            a0=0.0,
            pT=10.0,
            vT=0.0,
            aT=0.0,
            duration=1.0,
        )
        samples = [traj.velocity(t / 20.0) for t in range(21)]
        self.assertTrue(all(math.isfinite(v) for v in samples))

    def test_estimate_duration_respects_bounds(self):
        duration = estimate_limb_catch_duration(
            Vec2d(0, 0),
            Vec2d(30, 0),
            Vec2d(0, 0),
            min_duration=0.2,
            max_duration=1.0,
            extension_speed=5.0,
            rotation_speed=2.0,
            typical_reach=10.0,
        )
        self.assertGreaterEqual(duration, 0.2)
        self.assertLessEqual(duration, 1.0)


if __name__ == "__main__":
    unittest.main()
