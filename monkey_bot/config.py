import math
from dataclasses import dataclass
from typing import Optional

from pymunk import Vec2d

from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.coords import Point2D, normalize_point, parse_coord


def fit_cell_size(
    instance: MonkeyBotProblemInstance,
    screen_width: float,
    screen_height: float,
    *,
    margin_px: float = 80,
) -> float:
    """Pick a cell size so the instance grid fills the screen with margin."""
    usable_w = max(screen_width - 2 * margin_px, 1)
    usable_h = max(screen_height - 2 * margin_px, 1)
    return min(usable_w / instance.grid_size_x, usable_h / instance.grid_size_y)


@dataclass
class SimConfig:
    screen_height:int
    screen_width:int
    fps:int
    gravity:float=0.5
    cell_size: Optional[float] = None
    render_scale: float = 1
    realtime: bool = True

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
    simplified_problem:bool
    min_extension:float=0.05
    angle_epsilon:float= 2*math.pi/24
    foot_radius:float = 0.01
    leg_thickness:float = 0.01
    body_radius:float = 0.1
    max_takeoff_speed:float = 10
    max_jump_dist: float = 3
    prune_short_jumps: bool = True
    prune_in_clique_jumps: bool = True
    prune_similar_jumps: bool = True
    angle_rotation_speed: Optional[float] = None



class InstanceSimulationConfig:
    def __init__(self, instance: MonkeyBotProblemInstance, sim_config: SimConfig, robot_config: RobotConfig):
        self.instance = instance
        self.sim_config = sim_config
        self.robot_config = robot_config

    def grid_to_screen(self, x, y):
        """
        Convert grid coordinates (x, y) to screen coordinates (px, py).

        The grid is centered on the screen and scaled so that there is
        at least one grid cell margin on all sides.
        """
        cell_size = self.cell_size

        center_x, center_y = self.sim_config.screen_width / 2, self.sim_config.screen_height / 2
        screen_x = center_x + cell_size * (x - self.instance.grid_size_x / 2)
        screen_y = center_y - cell_size * (y - self.instance.grid_size_y / 2)

        return Vec2d(screen_x, screen_y)

    def screen_to_grid(self, screen_pos: Vec2d) -> Point2D:
        """Convert screen coordinates to grid/world coordinates."""
        cell_size = self.cell_size
        center_x = self.sim_config.screen_width / 2
        center_y = self.sim_config.screen_height / 2
        gx = (screen_pos.x - center_x) / cell_size + self.instance.grid_size_x / 2
        gy = -(screen_pos.y - center_y) / cell_size + self.instance.grid_size_y / 2
        gx = max(0.0, min(float(self.instance.grid_size_x), gx))
        gy = max(0.0, min(float(self.instance.grid_size_y), gy))
        return normalize_point((gx, gy))

    @property
    def cell_size(self):
        explicit = self.sim_config.cell_size
        if explicit is not None:
            return explicit
        return fit_cell_size(
            self.instance,
            self.sim_config.screen_width,
            self.sim_config.screen_height,
        )

    def max_jump_dist(self):
        return self.robot_config.max_jump_dist*self.cell_size

    def gp_name_to_screen_point(self, gp_name):
        parts = gp_name.split("_")
        gp = (parse_coord(parts[1]), parse_coord(parts[2]))
        resolved = self.instance.resolve_grip_point(gp)
        if resolved is None:
            raise ValueError(f"Unknown gripping point {gp} in action name {gp_name}")
        return self.grid_to_screen(*resolved)

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
        return self.instance.max_extension * self.cell_size * 1.1

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
        return self.sim_config.gravity* self.cell_size

    @property
    def epsilon(self):
        return self.robot_config.epsilon* self.cell_size

    @property
    def leg_thickness(self):
        return self.robot_config.leg_thickness * self.cell_size

    @property
    def min_extension(self):
        return self.robot_config.min_extension * self.cell_size

    @property
    def foot_radius(self):
        return self.robot_config.foot_radius * self.cell_size

    @property
    def body_radius(self):
        return self.robot_config.body_radius * self.cell_size

    @property
    def extension_speed(self):
        return self.robot_config.extension_speed*self.cell_size

    @property
    def move_center_speed(self):
        return self.robot_config.move_center_speed * self.cell_size

    @property
    def rotation_speed(self):
        return self.robot_config.rotation_speed

    @property
    def angle_rotation_speed(self):
        if self.robot_config.angle_rotation_speed is not None:
            return self.robot_config.angle_rotation_speed
        return self.robot_config.rotation_speed

    @property
    def dt(self):
        return self.sim_config.dt

    @property
    def robot_mass(self):
        return self.body_mass + self.num_legs*(self.leg_mass+self.foot_mass)

    @property
    def stiffness(self):
        return self.robot_config.leg_spring_stiffness * self.cell_size
    @property
    def damping(self):
        return self.robot_config.leg_spring_damping * self.cell_size

    @property
    def max_jump_speed(self):
        return self.robot_config.max_takeoff_speed * self.cell_size
