import math
import random
import unittest

from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.goal_generator import (
    _goal_candidates,
    build_replan_instance,
    feet_grid_positions,
    generate_next_goal,
    plan_has_climbing_actions,
    pick_replan_instance,
)
from monkey_bot.coords import grid_distance
from monkey_bot.simplified_graph_planner import SimplifiedGraphPlanner
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance, snapshot_instance
from monkey_bot.signals import StateSignal
from pymunk import Vec2d


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

    def test_feet_grid_positions_snaps_drifted_gripped_foot(self):
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
        gp = (2, 2)
        screen_gp = coordinator.grid_to_screen(*gp)
        drifted = screen_gp + Vec2d(coordinator.cell_size * 0.35, 0)
        state = StateSignal(
            center_pos=coordinator.grid_to_screen(4, 4),
            feet_pos=[drifted, coordinator.grid_to_screen(4, 2), coordinator.grid_to_screen(6, 4)],
            active_grips=[True, True, True],
            t=0,
        )
        feet = feet_grid_positions(coordinator, state)
        self.assertEqual(feet[0], gp)

    def test_generate_next_goal_excludes_current_goal(self):
        instance = make_instance()
        rng = random.Random(0)
        goal = generate_next_goal(
            instance,
            current_center=(8, 16),
            rng=rng,
            exclude_goals={(8, 16), instance.goal_point},
        )
        self.assertNotEqual(goal, (8, 16))

    def test_build_replan_instance_after_goal(self):
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
        state = StateSignal(
            center_pos=coordinator.grid_to_screen(4, 5),
            feet_pos=coordinator.screen_init_feet(),
            active_grips=[True, True, True],
            t=0,
        )
        replanned = build_replan_instance(
            instance,
            coordinator,
            state,
            random.Random(0),
            previous_goal=instance.goal_point,
        )
        self.assertNotEqual(replanned.goal_point, instance.goal_point)
        self.assertEqual(len(replanned.init_feet), 3)

    def test_generate_next_goal_random_from_valid_pool(self):
        instance = make_instance()
        rng = random.Random(0)
        previous = (4, 4)
        center = (4, 5)
        goal = generate_next_goal(
            instance,
            current_center=center,
            rng=rng,
            previous_goal=previous,
        )
        self.assertNotEqual(goal, previous)
        valid = [
            g
            for g in _goal_candidates(instance)
            if g != previous and grid_distance(g, center) >= 1e-6
        ]
        self.assertIn(goal, valid)

    def test_feet_grid_positions_leaves_free_feet_unset(self):
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
        state = StateSignal(
            center_pos=coordinator.grid_to_screen(4, 5),
            feet_pos=[
                coordinator.grid_to_screen(2, 2),
                coordinator.grid_to_screen(6, 4),
                coordinator.grid_to_screen(4, 6),
            ],
            active_grips=[True, True, False],
            t=0,
        )
        feet = feet_grid_positions(coordinator, state)
        self.assertEqual(feet[0], (2.0, 2.0))
        self.assertEqual(feet[1], (6.0, 4.0))
        self.assertIsNone(feet[2])

    def test_planner_initial_state_respects_free_leg(self):
        instance = make_instance()
        snap = snapshot_instance(
            instance,
            init_center=(4.0, 6.0),
            init_feet=[(2.0, 2.0), (6.0, 4.0), None],
            goal_point=(4.0, 8.0),
        )
        planner = SimplifiedGraphPlanner(snap, [])
        initial = planner.initial_state
        self.assertIsNone(initial.feet[2])
        self.assertTrue(initial.close_enough[2])

        false_snap = snapshot_instance(
            instance,
            init_center=(4.0, 6.0),
            init_feet=[(2.0, 2.0), (6.0, 4.0), (4.0, 6.0)],
            goal_point=(4.0, 8.0),
        )
        good_releases = [
            action
            for _, action in planner.successors(initial)
            if action.startswith("release__")
        ]
        bad_releases = [
            action
            for _, action in SimplifiedGraphPlanner(false_snap, []).successors(
                SimplifiedGraphPlanner(false_snap, []).initial_state
            )
            if action.startswith("release__")
        ]
        self.assertEqual(good_releases, [])
        self.assertGreater(len(bad_releases), 0)

    def test_pick_replan_instance_requires_climbing_plan(self):
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
        state = StateSignal(
            center_pos=coordinator.grid_to_screen(4, 5),
            feet_pos=coordinator.screen_init_feet(),
            active_grips=[True, True, True],
            t=0,
        )
        replanned = pick_replan_instance(
            instance,
            coordinator,
            state,
            random.Random(0),
            previous_goal=instance.goal_point,
        )
        plan = SimplifiedGraphPlanner(replanned, []).solve()
        self.assertTrue(plan_has_climbing_actions(plan))


if __name__ == "__main__":
    unittest.main()
