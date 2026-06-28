import math
import random
import unittest

from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.goal_generator import generate_next_goal
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance, snapshot_instance


def make_instance(**overrides):
    base = MonkeyBotProblemInstance(
        name="test",
        max_extension=2,
        grid_size_x=10,
        grid_size_y=10,
        gripping_points=[(2, 2), (4, 2), (6, 4), (4, 6), (2, 8)],
        goal_point=(4, 4),
        init_center=(4, 4),
        init_feet=[(2, 2), (4, 2), (6, 4)],
        allowed_jump_configs=None,
    )
    if overrides:
        return snapshot_instance(
            base,
            init_center=overrides.get("init_center", base.init_center),
            init_feet=overrides.get("init_feet", base.init_feet),
            goal_point=overrides.get("goal_point", base.goal_point),
        )
    return base


class InfiniteDemoHelpersTest(unittest.TestCase):
    def test_screen_to_grid_round_trip(self):
        instance = make_instance()
        sim_config = SimConfig(screen_height=1000, screen_width=1000, fps=120, cell_size=20)
        robot_config = RobotConfig(
            epsilon=0.1,
            extension_speed=4,
            rotation_speed=4,
            move_center_speed=4,
            body_mass=5,
            foot_mass=0.2,
            leg_mass=0.2,
            simplified_problem=True,
            leg_spring_stiffness=500,
            leg_spring_damping=1.5,
        )
        coordinator = InstanceSimulationConfig(instance, sim_config, robot_config)

        for gx, gy in [(2, 2), (4, 4), (8, 6)]:
            screen = coordinator.grid_to_screen(gx, gy)
            back = coordinator.screen_to_grid(screen)
            self.assertEqual(back, (gx, gy))

    def test_generate_next_goal_is_near_gripping_points(self):
        instance = make_instance()
        rng = random.Random(0)
        goal = generate_next_goal(instance, current_center=(4, 4), rng=rng)
        nearby = [
            gp
            for gp in instance.gripping_points
            if math.hypot(gp[0] - goal[0], gp[1] - goal[1]) <= instance.max_extension
        ]
        self.assertGreaterEqual(len(nearby), 2)
        self.assertNotEqual(goal, (4, 4))


if __name__ == "__main__":
    unittest.main()
