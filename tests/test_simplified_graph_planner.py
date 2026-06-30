import unittest

from monkey_bot.action import parse_action
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.simplified_graph_planner import (
    SimplifiedGraphPlanner,
    compute_attach_config_components,
    state_str,
)


def make_instance(**overrides):
    base = MonkeyBotProblemInstance(
        name="test",
        max_extension=2,
        grid_size_x=10,
        grid_size_y=10,
        gripping_points=[(1, 1), (2, 1), (3, 1), (4, 1), (5, 1)],
        goal_point=(5, 1),
        init_center=(2, 1),
        init_feet=[(1, 1), (2, 1), (3, 1)],
        allowed_jump_configs=None,
    )
    if not overrides:
        return base
    return MonkeyBotProblemInstance(
        name=overrides.get("name", base.name),
        max_extension=overrides.get("max_extension", base.max_extension),
        grid_size_x=overrides.get("grid_size_x", base.grid_size_x),
        grid_size_y=overrides.get("grid_size_y", base.grid_size_y),
        gripping_points=overrides.get("gripping_points", base.gripping_points),
        goal_point=overrides.get("goal_point", base.goal_point),
        init_center=overrides.get("init_center", base.init_center),
        init_feet=overrides.get("init_feet", base.init_feet),
        allowed_jump_configs=base.allowed_jump_configs,
    )


class SimplifiedGraphPlannerTest(unittest.TestCase):
    def test_state_str_matches_upf_format(self):
        feet = ((1, 1), (3, 1), (2, 1))
        self.assertEqual(state_str(feet), "1_1_3_1_2_1")

    def test_goal_already_satisfied_returns_empty_plan(self):
        instance = make_instance(
            goal_point=(2, 1),
            init_feet=[(1, 1), (2, 1), (3, 1)],
        )
        plan = SimplifiedGraphPlanner(instance, transition_links=[]).solve()
        self.assertIsNotNone(plan)
        self.assertEqual(plan.actions, [])

    def test_finds_attach_release_plan_without_upf(self):
        instance = make_instance()
        plan = SimplifiedGraphPlanner(instance, transition_links=[]).solve()
        self.assertIsNotNone(plan)
        self.assertGreater(len(plan.actions), 0)
        for action in plan.actions:
            parsed = parse_action(action)
            self.assertTrue(
                parsed.name.startswith("release__")
                or parsed.name.startswith("attach__")
                or parsed.name.startswith("use_TL__")
            )

    def test_float_goal_and_gripping_points(self):
        instance = make_instance(
            gripping_points=[(1, 1), (2.5, 1), (4, 1), (5, 1)],
            goal_point=(3.25, 1),
            init_feet=[(1, 1), (2.5, 1), (4, 1)],
        )
        plan = SimplifiedGraphPlanner(instance, transition_links=[]).solve()
        self.assertIsNotNone(plan)

    def test_jump_edge_is_used_when_provided(self):
        instance = make_instance()
        transition_links = [((3, 1), (2, 1), ((4, 1), (5, 1), (5, 1)))]
        plan = SimplifiedGraphPlanner(instance, transition_links).solve()
        self.assertIsNotNone(plan)
        self.assertTrue(any("use_TL__" in action for action in plan.actions))

    def test_jump_action_encodes_multidigit_coords(self):
        from monkey_bot.coords import catch_points_str, parse_catch_points

        catch = ((10, 4), (10, 16), (6, 16))
        encoded = catch_points_str(catch)
        self.assertEqual(encoded, "10_4_10_16_6_16")
        self.assertEqual(parse_catch_points(encoded), catch)

    def test_mandatory_jump_instance_has_no_attach_only_plan(self):
        from monkey_bot.monkey_bot_problem_instance import load_instance

        for name in ("MandatoryJump", "JumpTwoClusters"):
            with self.subTest(instance=name):
                instance = load_instance(name, "instances")
                self.assertIsNone(SimplifiedGraphPlanner(instance, []).solve())
                if name == "JumpTwoClusters":
                    links = [((1, 1), (1, 3), ((7, 7), (7, 9), (9, 7)))]
                else:
                    links = [((2, 5), (5, 5), ((15, 14), (16, 13), (16, 14)))]
                plan = SimplifiedGraphPlanner(instance, links).solve()
                self.assertIsNotNone(plan)
                self.assertTrue(any("use_TL__" in action for action in plan.actions))

    def test_attach_config_components_partition_valid_configs(self):
        instance = make_instance()
        config_component, gps_by_component = compute_attach_config_components(instance)
        self.assertGreater(len(config_component), 0)
        self.assertGreaterEqual(len(gps_by_component), 1)
        init_cfg = frozenset(instance.init_feet)
        self.assertIn(init_cfg, config_component)


if __name__ == "__main__":
    unittest.main()
