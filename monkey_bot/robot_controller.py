import math
import typing
from dataclasses import dataclass
from math import hypot
from typing import List

from pymunk import Vec2d

from monkey_bot import upf_solver
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
import monkey_bot.upf_solver
from monkey_bot.simulation_config import SimConfig, InstanceSimulationCoordinator


@dataclass
class Action:
    name:str
    params:List[str]

@dataclass
class Signal:
    extension:List[float]
    rotation:List[float]
    grip:List[int]

class Controller:
    def __init__(self, coordinator:InstanceSimulationCoordinator):
        self.plan = []
        self.coordinator = coordinator

        self.feet_pos = []
        self.center_pos = None
        self.active_grips = []
        self.t = 0
        self.actions_finished = 0

        self._dir_deltas = {'up': (0, 1), 'down': (0, -1), 'left': (-1, 0), 'right': (1, 0)}

    @property
    def _last_grid_center_pos(self):
        x, y = self.coordinator.instance.init_center
        for i, in range(self.actions_finished):
            action = self.plan[i]
            name = action.name
            if "move_center" in name:
                delta_x, delta_y = self.get_center_grid_delta(name)
                x, y = x + delta_x, y + delta_y

        return x, y

    def create_plan(self):
        plan = monkey_bot.upf_solver.get_plan(self.coordinator.instance)
        self.plan = []
        for action in plan.actions:
            action = str(action)
            if '(' not in action or ')' not in action:
                raise ValueError(f"Invalid action format: {action}")
            action_name, args_str = action.split('(', 1)
            args_str = args_str.rstrip(')')
            args = [arg.strip() for arg in args_str.split(',') if arg.strip()]
            action_name = action.strip()
            self.plan.append(Action(action_name, args))

    def get_center_grid_delta(self, action_name):
        chosen_dir = None
        for d in self._dir_deltas:
            if d in action_name:
                chosen_dir = d
                break
        return self._dir_deltas[chosen_dir]

    def move_center(self, action):
        last_x, last_y = self._last_grid_center_pos
        deltax, delta_y = self.get_center_grid_delta(action.name)
        goal_x, goal_y = last_x + deltax, last_y + delta_y
        screen_goal = Vec2d(*self.coordinator.grid_to_screen(goal_x, goal_y))
        curr_center = Vec2d(*self.center_pos)
        delta = screen_goal - curr_center
        delta = delta/delta.length*self.coordinator.config.move_center_speed
        next_center_pos = curr_center + delta
        for i, _ in enumerate(self.coordinator.num_legs):
            curr_foot_pos = Vec2d(*self.feet_pos[i])
            next_leg_length = (next_center_pos - curr_foot_pos).length



    def move_foot(self):
        raise NotImplementedError

    def get_sig(self):
        current_action = self.plan[self.actions_finished]
        if "move_center" in current_action.name:
            return self.move_center(current_action)
        elif "move_foot" in current_action.name:
            return self.move_foot(current_action)
        return None

    def update(self, state):
        self.feet_pos = state["feet_pos"]
        self.center_pos = state["center_pos"]
        self.t = state["t"]
        self.active_grips = state["active_grips"]

    @property
    def num_legs(self):
        return len(self.feet_pos)

class TestController(Controller):
    def get_sig(self):
        """Generate a simple timed signal sequence based on self.t."""
        n_legs = self.num_legs
        zero_ext = [0.0] * n_legs
        zero_rot = [0.0] * n_legs
        zero_grip = [0] * n_legs
        leg_id = 0
        t = self.t/60
        # 0–3s: do nothing
        # 3–3.1s: release one leg

        if t < 3:
            if self.active_grips[leg_id]:
                grip = zero_grip.copy()
                grip[leg_id] = -1  # release leg 0
                return Signal(zero_ext, zero_rot, grip)
            return Signal(zero_ext, zero_rot, zero_grip)

        # 3.1–6.1s: rotate left
        elif t < 6:
            rot = [1.0 if i == leg_id else 0.0 for i in range(n_legs)]
            return Signal(zero_ext, rot, zero_grip)

        # 6.1–9.1s: rotate right
        elif t < 9:
            rot = [-1.0 if i == leg_id else 0.0 for i in range(n_legs)]
            return Signal(zero_ext, rot, zero_grip)

        # 9.1–12.1s: extend
        elif t < 12:
            print("extending")
            ext = [30 if i == leg_id else 0.0 for i in range(n_legs)]
            return Signal(ext, zero_rot, zero_grip)

        # 12.1–15.1s: shorten
        elif t < 15:
            ext = [-30.0 if i == leg_id else 0.0 for i in range(n_legs)]
            return Signal(ext, zero_rot, zero_grip)

        if not self.active_grips[leg_id]:
            grip = zero_grip.copy()
            grip[leg_id] = 1  # release leg 0
            return Signal(zero_ext, zero_rot, grip)
        return Signal(zero_ext, zero_rot, zero_grip)
