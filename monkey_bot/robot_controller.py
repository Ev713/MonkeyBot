import math
import typing
from dataclasses import dataclass
from math import pi, hypot
from typing import List, Tuple
import shapely

from shapely.geometry import Polygon

from monkey_bot import monkey_bot_problem_instance, upf_solver
import monkey_bot.monkey_bot_problem_instance
import monkey_bot.upf_solver

@dataclass
class Action:
    name:str
    params:List[str]

class Controller:
    def __init__(self, instance: monkey_bot_problem_instance.MonkeyBotProblemInstance,
                 dt, extension_speed, twist_speed, move_center_speed, epsilon = 10, screen_width=1000, screen_height=800):
        self.instance = instance
        self.current_action = None
        self.feet_pos = []
        self.center_pos = None
        self.active_grips = []
        self.plan = None
        self.epsilon = epsilon

        self.screen_width = screen_width
        self.screen_height = screen_height

        self.grid_width = instance.grid_size_x
        self.grid_height = instance.grid_size_y
        self.extension_speed = extension_speed
        self.twist_speed = twist_speed
        self.move_center_speed = move_center_speed
        self.dt = dt


        self.last_grid_center_pos = self.instance.init_center

    def create_plan(self):
        plan = monkey_bot.upf_solver.get_plan(self.instance)
        self.plan = [a for a in plan.actions]

    def grid_to_screen(self, x, y):
        """
        Convert grid coordinates (x, y) to screen coordinates (px, py).

        The grid is centered on the screen and scaled so that there is
        at least one grid cell margin on all sides.
        """
        # total "virtual" grid dimensions (grid + margin cells on each side)
        total_grid_w = self.grid_width + 2  # one margin cell per side
        total_grid_h = self.grid_height + 2

        # scale factor = number of screen pixels per grid cell
        cell_size = min(self.screen_width / total_grid_w, self.screen_height / total_grid_h)

        center_x, center_y = self.screen_width / 2, self.screen_height / 2
        screen_x = center_x + cell_size * (x - self.grid_width/2)
        screen_y = center_y - cell_size * (y - self.grid_height / 2)

        return screen_x, screen_y

    def get_center_grid_delta(self, action_name):
        dirs = ['up', 'down', 'left', 'right']
        chosen_dir = None
        for d in dirs:
            if d in self.current_action.name:
                chosen_dir = d
                break

        # Direction deltas in grid space
        dir_deltas = {'up': (0, 1), 'down': (0, -1), 'left': (-1, 0), 'right': (1, 0)}
        return dir_deltas[chosen_dir]

    def move_center(self):
        """
        Compute directional control signals for moving the robot center.
        Returns a dict with angular (twist) and extension speeds for all legs.
        """
        # --- 3. For each leg, compute how geometry changes ---
        n_legs = len(self.feet_pos)
        hip_twist_speeds = [0.0 for i in range(n_legs)]
        leg_speeds = [0.0 for _ in range(n_legs)]
        perform_grip = [0 for _ in range(n_legs)]  # not used here but kept for consistency

        # --- 1. Determine direction from current action ---
        delta_x, delta_y = self.get_center_grid_delta(self.current_action.name)

        # --- 2. Compute desired center position (in screen space) ---
        longterm_desired_center_x = (self.last_grid_center_pos[0]
                            + delta_x)
        longterm_desired_center_y = (self.last_grid_center_pos[1]
                            + delta_y)

        longterm_desired_center_x, longterm_desired_center_y = self.grid_to_screen(longterm_desired_center_x, longterm_desired_center_y)

        if hypot(longterm_desired_center_x-self.center_pos[0], longterm_desired_center_y-self.center_pos[1]) < self.epsilon:
            self.last_grid_center_pos[0] += delta_x
            self.last_grid_center_pos[1] += delta_y
            self.current_action=None
            return {
                "hip_twist_speed": hip_twist_speeds,
                "leg_speeds": leg_speeds,
                "perform_grip": perform_grip
            }

        desired_center_x = self.center_pos[0] + (longterm_desired_center_x-self.center_pos[0]) * self.move_center_speed * self.dt

        desired_center_y = self.center_pos[1] + (longterm_desired_center_y - self.center_pos[1]) * self.move_center_speed * self.dt

        desired_center_x, desired_center_y = self.grid_to_screen(desired_center_x, desired_center_y)



        for i, (foot_x, foot_y) in enumerate(self.feet_pos):
            # Current geometry (relative to current center)
            dx = foot_x - self.center_pos[0]
            dy = foot_y - self.center_pos[1]
            current_length = math.hypot(dx, dy)
            current_angle = math.atan2(dy, dx)

            # Desired geometry (relative to desired center)
            dx_d = foot_x - desired_center_x
            dy_d = foot_y - desired_center_y
            desired_length = math.hypot(dx_d, dy_d)
            desired_angle = math.atan2(dy_d, dx_d)

            # Differences
            delta_length = desired_length - current_length
            delta_angle = desired_angle - current_angle

            # Normalize angle difference to [-pi, pi]
            delta_angle = (delta_angle + math.pi) % (2 * math.pi) - math.pi

            # Speeds: direction only (+1 or -1)
            leg_speeds[i] = 1.0 if delta_length > 0 else -1.0
            hip_twist_speeds[i] = 1.0 if delta_angle > 0 else -1.0

        # --- 4. Return control dictionary ---
        return {
            "hip_twist_speeds": hip_twist_speeds,
            "leg_speeds": leg_speeds,
            "perform_grip": perform_grip
        }

    def move_foot(self):
        """
            Compute directional control signals for moving a specific foot.
            Returns a dict with angular (twist) and extension speeds for all legs,
            and a perform_grip flag for each foot.
            """
        action = self.current_action
        foot_id = int(action.name.split("_")[-1])

        # --- Build command dictionary ---
        n_legs = len(self.feet_pos)
        hip_twist_speeds = [0.0 for _ in range(n_legs)]
        leg_speeds = [0.0 for _ in range(n_legs)]
        perform_grip = [0 for _ in range(n_legs)]
        if self.active_grips[foot_id] is not None:
            perform_grip[foot_id] = -1
            return {
                "hip_twist_speeds": hip_twist_speeds,
                "leg_speeds": leg_speeds,
                "perform_grip": perform_grip
            }

        # --- Current geometry ---
        foot_x, foot_y = self.feet_pos[foot_id]
        center_x, center_y = self.center_pos

        dx = foot_x - center_x
        dy = foot_y - center_y
        current_length = math.hypot(dx, dy)
        current_angle = math.atan2(dy, dx)

        # --- Desired geometry ---
        desired_gp = action.params[1].split('_')
        desired_x, desired_y = float(desired_gp[1]), float(desired_gp[2])

        dx_d = desired_x - center_x
        dy_d = desired_y - center_y
        desired_length = math.hypot(dx_d, dy_d)
        desired_angle = math.atan2(dy_d, dx_d)

        # --- Differences ---
        delta_length = desired_length - current_length
        delta_angle = desired_angle - current_angle

        # Normalize angle difference to [-pi, pi]
        delta_angle = (delta_angle + math.pi) % (2 * math.pi) - math.pi

        # --- Speeds ---
        # Extension speed: +1 = extend, -1 = retract
        ext_speed = 1.0 if delta_length > 0 else -1.0
        # Twist speed: +1 = rotate CCW, -1 = CW (shortest path)
        twist_speed = 1.0 if delta_angle > 0 else -1.0

        # Assign directional speeds to moving leg
        hip_twist_speeds[foot_id] = twist_speed
        leg_speeds[foot_id] = ext_speed

        # If target distance reached, issue grip command and finish
        # action by setting current action to None
        if abs(delta_length) < self.epsilon and abs(delta_angle) < math.radians(5):
            perform_grip[foot_id] = 1
            self.current_action = None

        return {
            "hip_twist_speeds": hip_twist_speeds,
            "leg_speeds": leg_speeds,
            "perform_grip": perform_grip
        }


    def get_signals(self):
        if self.current_action is None:
            new_action = str(self.plan.pop(0)).strip()
            if '(' not in new_action or ')' not in new_action:
                raise ValueError(f"Invalid action format: {new_action}")
            new_action_name, args_str = new_action.split('(', 1)
            args_str = args_str.rstrip(')')  # remove trailing parenthesis
            args = [arg.strip() for arg in args_str.split(',') if arg.strip()]
            new_action_name = new_action_name.strip()
            self.current_action = Action(new_action_name, args)

        if "move_center" in self.current_action.name:
            return self.move_center()
        elif "move_foot" in self.current_action.name:
            return self.move_foot()
        return None

