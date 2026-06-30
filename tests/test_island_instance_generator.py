import random
import unittest

from monkey_bot.coords import grid_distance
from monkey_bot.island_instance_generator import (
    IslandGeneratorConfig,
    generate_island_instance,
    generate_island_points,
)


class IslandInstanceGeneratorTest(unittest.TestCase):
    def test_spacing_and_connectivity(self):
        config = IslandGeneratorConfig(
            num_islands=4,
            target_points_per_island_min=10,
            target_points_per_island_max=14,
            grid_size_x=40.0,
            grid_size_y=40.0,
        )
        islands = generate_island_points(config, random.Random(0))
        self.assertGreaterEqual(len(islands), 3)
        all_points = [p for island in islands for p in island]
        for i, p in enumerate(all_points):
            for q in all_points[i + 1 :]:
                self.assertGreaterEqual(
                    grid_distance(p, q),
                    config.min_spacing - 1e-9,
                    msg=f"points too close: {p}, {q}",
                )

    def test_generates_loadable_instance(self):
        instance = generate_island_instance(
            IslandGeneratorConfig(
                name="TestIslands",
                num_islands=5,
                target_points_per_island_min=8,
                target_points_per_island_max=12,
                grid_size_x=50.0,
                grid_size_y=50.0,
            ),
            seed=7,
        )
        self.assertGreater(len(instance.gripping_points), 30)
        self.assertEqual(len(instance.init_feet), 3)
        for foot in instance.init_feet:
            self.assertLessEqual(
                grid_distance(instance.init_center, foot),
                instance.max_extension + 1e-9,
            )


if __name__ == "__main__":
    unittest.main()
