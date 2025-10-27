import math
from dataclasses import dataclass

from pymunk import Vec2d

from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance


@dataclass
class SimConfig:
    screen_height:int
    screen_width:int
    fps:int
    gravity:float=0.5

    @property
    def dt(self):
        return 1/self.fps

@dataclass
class RobotConfig:
    epsilon:float
    extension_speed:float
    rotation_speed:float
    move_center_speed:float
    body_mass:float
    foot_mass:float
    leg_mass:float
    leg_spring_stiffness:float
    leg_spring_damping:float
    min_extension:float=0.05
    angle_epsilon:float= 2*math.pi/24
    foot_radius:float = 0.01
    leg_thickness:float = 0.01
    body_radius:float = 0.1
    max_takeoff_speed:float = 10


class InstanceSimulationCoordinator:
    def __init__(self, instance: MonkeyBotProblemInstance, sim_config: SimConfig, robot_config: RobotConfig):
        self.instance = instance
        self.sim_config = sim_config
        self.robot_config = robot_config

    def cell_size(self):
        total_grid_w = self.instance.grid_size_x + 2  # one margin cell per side
        total_grid_h = self.instance.grid_size_y + 2
        return min(self.sim_config.screen_width / total_grid_w, self.sim_config.screen_height / total_grid_h)

    def grid_to_screen(self, x, y):
        """
        Convert grid coordinates (x, y) to screen coordinates (px, py).

        The grid is centered on the screen and scaled so that there is
        at least one grid cell margin on all sides.
        """
        cell_size = self.cell_size()

        center_x, center_y = self.sim_config.screen_width / 2, self.sim_config.screen_height / 2
        screen_x = center_x + cell_size * (x - self.instance.grid_size_x / 2)
        screen_y = center_y - cell_size * (y - self.instance.grid_size_y / 2)

        return Vec2d(screen_x, screen_y)

    def gp_name_to_screen_point(self, gp_name):
        gp_x = int(gp_name.split('_')[1])
        gp_y = int(gp_name.split('_')[2])
        assert (gp_x, gp_y) in self.instance.gripping_points
        return self.grid_to_screen(gp_x, gp_y)

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

    @property
    def max_extension(self):
        return self.instance.max_extension * self.cell_size() * 1.1

    @property
    def body_mass(self):
        return self.robot_config.body_mass

    @property
    def foot_mass(self):
        return self.robot_config.foot_mass

    @property
    def leg_mass(self):
        return self.robot_config.leg_mass

    @property
    def gravity(self):
        return self.sim_config.gravity*self.cell_size()

    @property
    def epsilon(self):
        return self.robot_config.epsilon*self.cell_size()

    @property
    def leg_thickness(self):
        return self.robot_config.leg_thickness*self.cell_size()

    @property
    def min_extension(self):
        return self.robot_config.min_extension * self.cell_size()

    @property
    def foot_radius(self):
        return self.robot_config.foot_radius*self.cell_size()

    @property
    def body_radius(self):
        return self.robot_config.body_radius*self.cell_size()

    @property
    def extension_speed(self):
        return self.robot_config.extension_speed*self.cell_size()

    @property
    def move_center_speed(self):
        return self.robot_config.move_center_speed * self.cell_size()

    @property
    def rotation_speed(self):
        return self.robot_config.rotation_speed

    @property
    def dt(self):
        return self.sim_config.dt

    @property
    def robot_mass(self):
        return self.body_mass + self.num_legs*(self.leg_mass+self.foot_mass)

    @property
    def stiffness(self):
        return self.robot_config.leg_spring_stiffness * self.cell_size()

    @property
    def damping(self):
        return self.robot_config.leg_spring_damping * self.cell_size()

    @property
    def max_jump_speed(self):
        return self.robot_config.max_takeoff_speed * self.cell_size()
