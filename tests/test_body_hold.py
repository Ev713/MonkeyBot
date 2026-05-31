import unittest

from pymunk import Vec2d

from monkey_bot.procedure import BodyHolder, normalize_angle
from monkey_bot.signals import ControlSignal, StateSignal


def make_state(body_angle, body_rate, active_grips):
    return StateSignal(
        center_pos=Vec2d(0, 0),
        feet_pos=[Vec2d(0, 0) for _ in active_grips],
        active_grips=active_grips,
        t=0,
        body_angle=body_angle,
        body_angular_velocity=body_rate,
    )


class FakeCoordinator:
    epsilon = 0.1
    rotation_speed = 4.0
    extension_speed = 4.0
    move_center_speed = 4.0
    num_legs = 3
    dt = 1 / 120
    max_extension = 2.0
    min_extension = 0.2


class BodyHolderTest(unittest.TestCase):
    def test_idle_signal_resets_reference(self):
        holder = BodyHolder(FakeCoordinator(), 1, 2)
        rotating = ControlSignal([0, 0, 0], [1.0, 0.0, 0.0], [0, 0, 0])
        idle = ControlSignal([0, 0, 0], [0, 0, 0], [0, 0, 0])
        state = make_state(0.2, 0.0, [False, True, True])
        holder.adjust_signal(rotating, state)
        self.assertIsNotNone(holder.ref_angle)

        holder.adjust_signal(idle, state)
        self.assertIsNone(holder.ref_angle)
        self.assertEqual(holder.ref_lengths, {})

    def test_compensates_all_active_holding_limbs(self):
        holder = BodyHolder(FakeCoordinator(), 1, 2, kp=10.0, kd=2.0, smooth=0.0)
        signal = ControlSignal([0, 0, 0], [1.0, 0.0, 0.0], [0, 0, 0])
        holder.adjust_signal(signal, make_state(body_angle=0.0, body_rate=0.0, active_grips=[False, True, True]))

        state = make_state(body_angle=0.1, body_rate=0.05, active_grips=[False, True, True])
        out = holder.adjust_signal(signal, state)
        for _ in range(30):
            out = holder.adjust_signal(signal, state)
        expected = -10.0 * normalize_angle(0.1) - 2.0 * 0.05
        self.assertEqual(out.rotation[0], 1.0)
        self.assertAlmostEqual(out.rotation[1], expected, places=2)
        self.assertAlmostEqual(out.rotation[2], expected, places=2)

    def test_skips_holding_limbs_that_are_not_gripped(self):
        holder = BodyHolder(FakeCoordinator(), 1, 2)
        signal = ControlSignal([0, 0, 0], [1.0, 0.0, 0.0], [0, 0, 0])
        state = make_state(0.1, 0.05, [False, False, True])
        out = holder.adjust_signal(signal, state)
        self.assertEqual(out.rotation[1], 0.0)
        self.assertNotEqual(out.rotation[2], 0.0)

    def test_extends_when_length_drops_below_initial(self):
        holder = BodyHolder(FakeCoordinator(), 1, 2, extension_kp=10.0, length_eps=0.01, smooth=0.0)
        signal = ControlSignal([0, 0, 0], [1.0, 0.0, 0.0], [0, 0, 0])
        start = make_state(body_angle=0.0, body_rate=0.0, active_grips=[False, True, True])
        start.feet_pos[1] = Vec2d(0, 40)
        start.feet_pos[2] = Vec2d(0, 40)
        holder.adjust_signal(signal, start)

        shorter = make_state(body_angle=0.0, body_rate=0.0, active_grips=[False, True, True])
        shorter.feet_pos[1] = Vec2d(0, 36)
        shorter.feet_pos[2] = Vec2d(0, 40)
        out = holder.adjust_signal(signal, shorter)
        self.assertGreater(out.extension[1], 0.0)
        self.assertEqual(out.extension[2], 0.0)

    def test_smoothing_reduces_command_jump(self):
        holder = BodyHolder(FakeCoordinator(), 1, kp=12.0, kd=2.5, smooth=0.75)
        signal = ControlSignal([0, 0, 0], [1.0, 0.0, 0.0], [0, 0, 0])
        holder.adjust_signal(signal, make_state(0.0, 0.0, [False, True, True]))
        out = holder.adjust_signal(signal, make_state(0.5, 0.0, [False, True, True]))
        self.assertLess(abs(out.rotation[1]), 12.0 * 0.5)


if __name__ == "__main__":
    unittest.main()
