import random

import monkey_bot.monkey_bot_problem_instance
from monkey_bot import upf_solver

"""name, 
            max_extension,
            grid_size_x,
            grid_size_y,
            gripping_points, 
            goal_point,
            init_center,
            init_feet"""

class InstanceGenerator:
    def __init__(self):
        name = f"i_{random.randint(1, 999999)}"
        self.instance = monkey_bot.monkey_bot_problem_instance.MonkeyBotProblemInstance(
            None, None, None, None, None, None, None, None)

    def set_basic_parameters(self, max_x, max_y, max_extension, name=None):
        self.instance.grid_size_x = max_x
        self.instance.grid_size_y = max_y
        self.instance.max_extension = max_extension
        if name is not None:
            self.instance.name = name

    def generate_random(self, max_x, max_y, name=None, density=0.5):
        init_center = random.randint(0, max_x), random.randint(0, max_y)
        goal = random.randint(1, max_x), random.randint(1, max_y)


        for i in range(self.instance.grid_size_x):
            for j in range(self.instance.grid_size_y):
                if random.random() < density:
                    self.instance.gripping_points.append((i+1, j+1))

    def check_is_doable(self, sim_runner):
        return upf_solver.get_plan(self.instance) is not None


