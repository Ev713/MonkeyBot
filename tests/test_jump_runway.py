import unittest

from pymunk import Vec2d

from monkey_bot.robot_controller import compute_jump_takeoff_point


class JumpRunwayTest(unittest.TestCase):
    def test_takeoff_respects_min_extension(self):
        foot1 = Vec2d(0.0, 0.0)
        foot2 = Vec2d(0.0, 20.0)
        start = Vec2d(30.0, 10.0)
        vec = Vec2d(100.0, 0.0)
        min_ext = 8.0
        max_ext = 60.0

        takeoff = compute_jump_takeoff_point(
            start,
            vec,
            foot1,
            foot2,
            dt=0.05,
            min_extension=min_ext,
            max_extension=max_ext,
        )

        for foot in (foot1, foot2):
            length = (foot - takeoff).length
            self.assertGreaterEqual(length, min_ext - 1e-6)
            self.assertLessEqual(length, max_ext + 1e-6)

    def test_takeoff_stays_inside_runway_envelope_along_path(self):
        foot1 = Vec2d(200.0, 500.0)
        foot2 = Vec2d(200.0, 520.0)
        start = Vec2d(169.0, 517.0)
        vec = Vec2d(25.0, -86.0)
        min_ext = 4.0
        max_ext = 66.0
        dt = 1 / 240

        takeoff = compute_jump_takeoff_point(
            start,
            vec,
            foot1,
            foot2,
            dt=dt,
            min_extension=min_ext,
            max_extension=max_ext,
        )

        pos = start
        while (pos - takeoff).dot(vec) > 0:
            for foot in (foot1, foot2):
                length = (foot - pos).length
                self.assertGreaterEqual(length, min_ext - 1e-6)
                self.assertLessEqual(length, max_ext + 1e-6)
            pos = pos + dt * vec


if __name__ == "__main__":
    unittest.main()
