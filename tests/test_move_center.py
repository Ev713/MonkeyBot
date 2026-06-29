import unittest

from pymunk import Vec2d

from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.procedure import MoveCenter
from monkey_bot.signals import empty_control_signal, StateSignal


def make_coordinator(cell_size=40.0):
    instance = MonkeyBotProblemInstance(
        name="test",
        max_extension=2.5,
        grid_size_x=8,
        grid_size_y=8,
        gripping_points=[(2, 4), (4, 2), (6, 4)],
        goal_point=(4, 4),
        init_center=(4, 4),
        init_feet=[(2, 4), (4, 2), (6, 4)],
    )
    sim_config = SimConfig(
        screen_height=800,
        screen_width=800,
        fps=120,
        gravity=1,
        cell_size=cell_size,
    )
    robot_config = RobotConfig(
        epsilon=0.15,
        extension_speed=4,
        rotation_speed=2,
        angle_rotation_speed=0.75,
        move_center_speed=4,
        body_mass=5,
        foot_mass=0.2,
        leg_mass=0.2,
        simplified_problem=True,
        leg_spring_stiffness=500,
        leg_spring_damping=1.5,
        min_extension=0.2,
        max_takeoff_speed=20,
        max_jump_dist=15,
    )
    return InstanceSimulationConfig(instance, sim_config, robot_config)


class MoveCenterTest(unittest.TestCase):
    def test_vertical_move_enables_angle_helpers(self):
        coordinator = make_coordinator()
        center = coordinator.grid_to_screen(4, 4)
        goal = coordinator.grid_to_screen(4, 5)

        move = MoveCenter(goal, coordinator)
        state = StateSignal(
            center_pos=center,
            feet_pos=[
                coordinator.grid_to_screen(2, 4),
                coordinator.grid_to_screen(4, 2),
                coordinator.grid_to_screen(6, 4),
            ],
            active_grips=[True, False, True],
            t=0,
        )
        out = move.adjust_signal(empty_control_signal(3), state)

        self.assertNotAlmostEqual(out.rotation[0], 0.0, places=5)
        self.assertNotAlmostEqual(out.rotation[2], 0.0, places=5)

    def test_horizontal_move_uses_extension_not_rotation(self):
        coordinator = make_coordinator()
        center = coordinator.grid_to_screen(4, 4)
        goal = coordinator.grid_to_screen(3, 4)

        move = MoveCenter(goal, coordinator)
        state = StateSignal(
            center_pos=center,
            feet_pos=[
                coordinator.grid_to_screen(2, 4),
                coordinator.grid_to_screen(4, 2),
                coordinator.grid_to_screen(6, 4),
            ],
            active_grips=[True, False, True],
            t=0,
        )
        out = move.adjust_signal(empty_control_signal(3), state)

        self.assertLess(out.extension[0], 0.0)
        self.assertGreater(out.extension[2], 0.0)
        self.assertEqual(out.rotation, [0.0, 0.0, 0.0])
        self.assertFalse(out.damp_body)

    def test_ignores_free_and_unlisted_legs(self):
        coordinator = make_coordinator()
        move = MoveCenter(coordinator.grid_to_screen(4, 5), coordinator, ignore_legs=[1])
        state = StateSignal(
            center_pos=coordinator.grid_to_screen(4, 4),
            feet_pos=[coordinator.grid_to_screen(x, y) for x, y in [(2, 4), (4, 2), (6, 4)]],
            active_grips=[True, True, True],
            t=0,
        )
        out = move.adjust_signal(empty_control_signal(3), state)
        self.assertEqual(out.extension[1], 0.0)


    def test_does_not_contract_past_min_extension(self):
        coordinator = make_coordinator()
        move = MoveCenter(coordinator.grid_to_screen(4, 12), coordinator)
        min_len = coordinator.min_extension
        foot = Vec2d(0.0, 0.0)
        center = Vec2d(min_len, 0.0)
        aim = Vec2d(0.0, 0.0)
        self.assertEqual(move._extension_drive(center, aim, foot), 0.0)

    def test_pull_up_coordination_balances_leg_growth(self):
        coordinator = make_coordinator()
        move = MoveCenter(coordinator.grid_to_screen(4, 12), coordinator)
        cell = coordinator.cell_size
        foot1 = Vec2d(5.19 * cell, 7.67 * cell)
        foot3 = Vec2d(4.0 * cell, 6.0 * cell)
        center = Vec2d(4.7 * cell, 7.0 * cell)
        target = coordinator.grid_to_screen(4, 12)
        aim = (center + target) / 2
        feet = [foot1, Vec2d(0.0, 0.0), foot3]
        foot_dist = (foot1 - foot3).length

        independent = {
            0: move._raw_desired_length(aim, foot1),
            2: move._raw_desired_length(aim, foot3),
        }
        coordinated = move._coordinated_pull_up_lengths(
            center, aim, feet, [0, 2], foot_dist
        )

        curr_len_0 = (center - foot1).length
        self.assertGreater(coordinated[0], curr_len_0)
        self.assertLess(coordinated[0], move.max_extension - 1e-3)
        self.assertLessEqual(coordinated[2], independent[2])


if __name__ == "__main__":
    unittest.main()
