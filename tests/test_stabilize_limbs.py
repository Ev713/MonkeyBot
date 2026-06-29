import unittest

from pymunk import Vec2d

from monkey_bot.procedure import StabilizeLimbs
from monkey_bot.signals import empty_control_signal, StateSignal


class FakeCoordinator:
    epsilon = 0.1
    rotation_speed = 2.0
    angle_rotation_speed = 0.75
    extension_speed = 4.0
    move_center_speed = 4.0
    num_legs = 3
    dt = 1 / 120
    max_extension = 2.0
    min_extension = 0.2


class StabilizeLimbsTest(unittest.TestCase):
    def test_zeros_angle_speed_and_relaxes_springs(self):
        stabilizer = StabilizeLimbs(FakeCoordinator(), settle_frames=3)
        signal = empty_control_signal(3)
        signal.rotation = [1.0, -0.5, 0.25]
        state = StateSignal(
            center_pos=Vec2d(0, 0),
            feet_pos=[Vec2d(0, 0), Vec2d(1, 0), Vec2d(0, 1)],
            active_grips=[True, False, True],
            t=0,
            body_angular_velocity=0.0,
        )
        out = stabilizer.adjust_signal(signal, state)
        self.assertTrue(all(out.relax_spring))
        self.assertFalse(any(out.angle_lock))
        self.assertEqual(out.rotation, [0.0, 0.0, 0.0])
        self.assertTrue(out.damp_body)

    def test_smoothly_decays_extension_commands(self):
        stabilizer = StabilizeLimbs(FakeCoordinator(), settle_frames=3)
        signal = empty_control_signal(3)
        signal.extension = [2.0, -1.0, 0.5]
        state = StateSignal(
            center_pos=Vec2d(0, 0),
            feet_pos=[Vec2d(0, 0), Vec2d(1, 0), Vec2d(0, 1)],
            active_grips=[True, False, True],
            t=0,
            body_angular_velocity=0.0,
        )
        out = stabilizer.adjust_signal(signal, state)
        self.assertEqual(out.rotation, [0.0, 0.0, 0.0])
        self.assertNotEqual(out.extension, [0.0, 0.0, 0.0])
        for rate in out.extension:
            self.assertLess(abs(rate), 2.0)

    def test_waits_for_low_body_spin(self):
        stabilizer = StabilizeLimbs(FakeCoordinator(), settle_frames=3)
        state = StateSignal(
            center_pos=Vec2d(0, 0),
            feet_pos=[Vec2d(0, 0), Vec2d(1, 0), Vec2d(0, 1)],
            active_grips=[True, True, True],
            t=0,
            body_angular_velocity=1.0,
        )
        stabilizer.adjust_signal(empty_control_signal(3), state)
        self.assertFalse(stabilizer.is_finished(state))

        calm = StateSignal(
            center_pos=Vec2d(0, 0),
            feet_pos=[Vec2d(0, 0), Vec2d(1, 0), Vec2d(0, 1)],
            active_grips=[True, True, True],
            t=1,
            body_angular_velocity=0.0,
        )
        for _ in range(2):
            stabilizer.adjust_signal(empty_control_signal(3), calm)
            self.assertFalse(stabilizer.is_finished(calm))
        stabilizer.adjust_signal(empty_control_signal(3), calm)
        self.assertTrue(stabilizer.is_finished(calm))


if __name__ == "__main__":
    unittest.main()
