import unittest

from pymunk import Vec2d

from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.procedure import ReleaseAtPoint
from monkey_bot.signals import StateSignal


def _coordinator(cell_size=80.0, *, epsilon=0.1, release_tolerance=0.625):
    instance = MonkeyBotProblemInstance(
        name="t",
        max_extension=3.0,
        grid_size_x=10.0,
        grid_size_y=10.0,
        gripping_points=[(1.0, 1.0)],
        goal_point=(5.0, 5.0),
        init_center=(2.0, 2.0),
        init_feet=[(1.0, 1.0), (3.0, 1.0), (1.0, 3.0)],
    )
    robot = RobotConfig(
        epsilon=epsilon,
        release_tolerance=release_tolerance,
        extension_speed=4,
        rotation_speed=4,
        move_center_speed=1,
        body_mass=5,
        foot_mass=0.2,
        leg_mass=0.2,
        leg_spring_stiffness=400,
        leg_spring_damping=1.25,
        simplified_problem=True,
    )
    sim = SimConfig(screen_width=800, screen_height=600, fps=240, cell_size=cell_size)
    return InstanceSimulationConfig(instance, sim, robot)


class ReleaseAtPointTest(unittest.TestCase):
    def test_releases_within_configured_tolerance_not_epsilon(self):
        coordinator = _coordinator(cell_size=80.0, epsilon=0.1, release_tolerance=0.625)
        goal = Vec2d(400, 300)
        proc = ReleaseAtPoint(goal, coordinator)
        self.assertAlmostEqual(proc.tolerance, 50.0)

        # 30px away: inside release window, outside MoveCenter epsilon (8px).
        state = StateSignal(
            center_pos=goal + Vec2d(30, 0),
            feet_pos=[goal, goal, goal],
            active_grips=[True, True, True],
            t=0,
        )
        from monkey_bot.signals import empty_control_signal

        signal = empty_control_signal(3)
        signal = proc.adjust_signal(signal, state)
        self.assertTrue(proc.released)
        self.assertEqual(signal.grip, [-1, -1, -1])

    def test_does_not_release_outside_tolerance(self):
        coordinator = _coordinator(cell_size=80.0, epsilon=0.1, release_tolerance=0.625)
        goal = Vec2d(400, 300)
        proc = ReleaseAtPoint(goal, coordinator)
        state = StateSignal(
            center_pos=goal + Vec2d(60, 0),
            feet_pos=[goal, goal, goal],
            active_grips=[True, True, True],
            t=0,
        )
        from monkey_bot.signals import empty_control_signal

        signal = empty_control_signal(3)
        signal = proc.adjust_signal(signal, state)
        self.assertFalse(proc.released)
        self.assertEqual(signal.grip, [0, 0, 0])


if __name__ == "__main__":
    unittest.main()
