import math
import os
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pymunk
from pymunk import Vec2d

from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.pymunk_simulator import MonkeyBotSimulator


class PymunkLimbEncodingTest(unittest.TestCase):
    def setUp(self):
        instance = load_instance("JumpTest", "instances")
        sim_config = SimConfig(screen_height=400, screen_width=400, fps=120, cell_size=20)
        robot_config = RobotConfig(
            epsilon=0.1,
            extension_speed=4,
            rotation_speed=4,
            move_center_speed=4,
            body_mass=5,
            body_radius=0.4,
            foot_mass=0.2,
            leg_mass=0.2,
            simplified_problem=True,
            leg_spring_stiffness=500,
            leg_spring_damping=1.5,
            min_extension=0.2,
        )
        self.coordinator = InstanceSimulationConfig(instance, sim_config, robot_config)
        self.simulator = MonkeyBotSimulator(sim_config)
        self.simulator.start_simulation(self.coordinator)

    def test_springs_connect_leg_to_foot_not_body_to_foot(self):
        body = self.simulator.body
        for spring in self.simulator.leg_springs:
            bodies = {spring.a, spring.b}
            self.assertIn(spring.a, [leg[0] for leg in self.simulator.legs])
            self.assertIn(spring.b, [foot[0] for foot in self.simulator.feet])
            self.assertNotIn(body, bodies)

    def test_leg_points_toward_foot_at_spawn(self):
        body_pos = Vec2d(*self.simulator.body.position)
        for leg_id, (leg_body, _) in enumerate(self.simulator.legs):
            foot_pos = self.simulator.get_foot_pos(leg_id)
            expected = (foot_pos - body_pos).normalized()
            actual = Vec2d(math.cos(leg_body.angle), math.sin(leg_body.angle))
            self.assertAlmostEqual(expected.cross(actual), 0.0, places=3)
            self.assertAlmostEqual(expected.dot(actual), 1.0, places=3)

    def test_foot_locks_exist(self):
        self.assertEqual(len(self.simulator.leg_angle_locks), self.coordinator.num_legs)


if __name__ == "__main__":
    unittest.main()
