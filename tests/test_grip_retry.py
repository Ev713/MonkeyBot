import os
import unittest

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

from pymunk import Vec2d

from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.procedure import AttachCatch, Grabber
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.signals import empty_control_signal, StateSignal


class GripRetryTest(unittest.TestCase):
    def setUp(self):
        instance = load_instance("GPT1", "instances")
        sim_config = SimConfig(screen_height=400, screen_width=400, fps=120, cell_size=20)
        robot_config = RobotConfig(
            epsilon=0.1,
            extension_speed=4,
            rotation_speed=2,
            angle_rotation_speed=0.75,
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

    def test_check_and_grip_does_not_raise_when_out_of_range(self):
        self.simulator.release_grip(0)
        foot_body, _ = self.simulator.feet[0]
        foot_body.position = (0, 0)
        self.assertFalse(self.simulator.check_and_grip(0))

    def test_attach_catch_not_finished_until_gripped(self):
        self.simulator.release_grip(0)
        gp = self.coordinator.grid_to_screen(6, 8)
        catcher = AttachCatch(0, gp, self.coordinator, (1, 2))
        state = self.simulator.get_state()
        self.assertFalse(catcher.is_finished(state))

    def test_grabber_requests_grip_only_when_close(self):
        gp = self.coordinator.grid_to_screen(6, 8)
        grabber = Grabber(0, gp, self.coordinator)
        far = StateSignal(
            center_pos=Vec2d(0, 0),
            feet_pos=[Vec2d(0, 0), Vec2d(10, 0), Vec2d(-10, 0)],
            active_grips=[False, True, True],
            t=0,
        )
        signal = empty_control_signal(3)
        out = grabber.adjust_signal(signal, far)
        self.assertEqual(out.grip[0], 0)


if __name__ == "__main__":
    unittest.main()
