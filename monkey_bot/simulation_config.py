import math
from dataclasses import dataclass

from pymunk import Vec2d

from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance


@dataclass
class SimConfig:
    screen_height:int
    screen_width:int
    epsilon:float
    extension_speed:float
    rotation_speed:float
    move_center_speed:float
    fps:int
    body_mass:float
    foot_mass:float
    leg_mass:float
    leg_spring_stiffness:float
    leg_spring_damping:float
    min_extension:float=30
    angle_epsilon:float= 2*math.pi/24
    gravity:float=98

    @property
    def dt(self):
        return 1/self.fps

class InstanceSimulationCoordinator:
    def __init__(self, instance: MonkeyBotProblemInstance, sim_config: SimConfig):
        self.instance = instance
        self.config = sim_config

    def cell_size(self):
        total_grid_w = self.instance.grid_size_x + 2  # one margin cell per side
        total_grid_h = self.instance.grid_size_y + 2
        return min(self.config.screen_width / total_grid_w, self.config.screen_height / total_grid_h)

    @property
    def max_extension(self):
        return self.instance.leg_extension*self.cell_size()

    def grid_to_screen(self, x, y):
        """
        Convert grid coordinates (x, y) to screen coordinates (px, py).

        The grid is centered on the screen and scaled so that there is
        at least one grid cell margin on all sides.
        """
        cell_size = self.cell_size()

        center_x, center_y = self.config.screen_width / 2, self.config.screen_height / 2
        screen_x = center_x + cell_size * (x - self.instance.grid_size_x / 2)
        screen_y = center_y - cell_size * (y - self.instance.grid_size_y / 2)

        return screen_x, screen_y

    def gp_name_to_screen_point(self, gp_name):
        gp_x = int(gp_name.split('_')[1])
        gp_y = int(gp_name.split('_')[2])
        assert (gp_x, gp_y) in self.instance.gripping_points
        return Vec2d(*self.grid_to_screen(gp_x, gp_y))

    @property
    def num_legs(self):
        return len(self.instance.init_feet)

    def screen_grip_points(self):
        return [self.grid_to_screen(*gp) for gp in self.instance.gripping_points]

    def screen_goal_point(self):
        return self.grid_to_screen(*self.instance.goal_point)

    def screen_init_feet(self):
        return [self.grid_to_screen(*f) for f in self.instance.init_feet]

    def screen_init_center(self):
        return self.grid_to_screen(*self.instance.init_center)
    def screen_max_extension(self):
        return self.instance.leg_extension * self.cell_size()
