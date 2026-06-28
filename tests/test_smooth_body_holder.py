import unittest

from pymunk import Vec2d

from monkey_bot.procedure import SmoothBodyHolder, normalize_angle
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


def make_state(body_angle=0.0, body_rate=0.0, active_grips=None):
    active_grips = active_grips or [False, True, True]
    return StateSignal(
        center_pos=Vec2d(0, 0),
        feet_pos=[Vec2d(0, 40), Vec2d(30, 0), Vec2d(-30, 0)],
        active_grips=active_grips,
        t=0,
        body_angle=body_angle,
        body_angular_velocity=body_rate,
    )


class SmoothBodyHolderTest(unittest.TestCase):
    def test_correction_is_bounded(self):
        holder = SmoothBodyHolder(FakeCoordinator(), 1, 2, smooth=0.0)
        signal = empty_control_signal(3)
        signal.rotation[0] = 0.75
        holder.adjust_signal(signal, make_state(body_angle=0.0, body_rate=0.0))
        out = holder.adjust_signal(signal, make_state(body_angle=0.8, body_rate=0.0))
        max_rate = FakeCoordinator().angle_rotation_speed * 0.35
        self.assertLessEqual(abs(out.rotation[1]), max_rate + 1e-6)
        self.assertLessEqual(abs(out.rotation[2]), max_rate + 1e-6)

    def test_deadband_suppresses_small_errors(self):
        holder = SmoothBodyHolder(FakeCoordinator(), 1, 2, smooth=0.0)
        signal = empty_control_signal(3)
        signal.rotation[0] = 0.75
        holder.adjust_signal(signal, make_state())
        out = holder.adjust_signal(signal, make_state(body_angle=0.02, body_rate=0.01))
        self.assertEqual(out.rotation[1], 0.0)
        self.assertEqual(out.rotation[2], 0.0)

    def test_smooth_ramp_limits_per_frame_change(self):
        holder = SmoothBodyHolder(FakeCoordinator(), 1, 2, smooth=0.0)
        signal = empty_control_signal(3)
        signal.rotation[0] = 0.75
        holder.adjust_signal(signal, make_state())
        first = holder.adjust_signal(signal, make_state(body_angle=0.5, body_rate=0.0))
        second = holder.adjust_signal(signal, make_state(body_angle=0.5, body_rate=0.0))
        max_step = holder.max_correction_rate * holder.dt
        self.assertLessEqual(
            abs(second.rotation[1] - first.rotation[1]),
            max_step + 1e-6,
        )

    def test_fades_out_when_free_leg_stops(self):
        holder = SmoothBodyHolder(FakeCoordinator(), 1, 2)
        rotating = empty_control_signal(3)
        rotating.rotation[0] = 0.75
        holder.adjust_signal(rotating, make_state(body_angle=0.2, body_rate=0.0))

        idle = empty_control_signal(3)
        out = idle
        for _ in range(120):
            out = holder.adjust_signal(idle, make_state(body_angle=0.2, body_rate=0.0))
        self.assertLess(abs(out.rotation[1]), 0.05)


if __name__ == "__main__":
    unittest.main()
