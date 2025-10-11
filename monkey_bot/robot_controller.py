import math
import time
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
    args:List[str]

@dataclass
class Signal:
    extension:List[float]
    rotation:List[float]
    grip:List[int]


def parse_action(action):
    action = str(action)
    if '(' not in action or ')' not in action:
        raise ValueError(f"Invalid action format: {action}")
    action_name, args_str = action.split('(', 1)
    args_str = args_str.strip().rstrip(')')
    args = [arg.strip() for arg in args_str.split(',') if arg.strip()]
    action_name = action_name.strip()
    return Action(action_name, args)


class Controller:
    def __init__(self, coordinator:InstanceSimulationCoordinator):
        self.finished_plan = False
        self.plan = []
        self.coordinator = coordinator

        self.feet_pos = []
        self.center_pos = None
        self.active_grips = []
        self.t = 0
        self.actions_finished = 0

        self._dir_deltas = {'up': (0, 1), 'down': (0, -1), 'left': (-1, 0), 'right': (1, 0)}
        self.longest_action_time=0
        self._start_last_action_time=0


    @property
    def _last_grid_center_pos(self):
        x, y = self.coordinator.instance.init_center
        for i in range(self.actions_finished):
            action = self.plan[i]
            name = action.name
            if "move_center" in name:
                delta_x, delta_y = self.get_center_grid_delta(name)
                x, y = x + delta_x, y + delta_y

        return x, y

    def create_plan(self):
        plan = monkey_bot.upf_solver.get_plan(self.coordinator.instance)
        self.plan = [parse_action(a) for a in plan]

    def load_plan(self, filepath):
        with open(filepath) as f:
            self.plan = [parse_action(a) for a in f]


    def get_center_grid_delta(self, action_name):
        chosen_dir = None
        for d in self._dir_deltas:
            if d in action_name:
                chosen_dir = d
                break
        return self._dir_deltas[chosen_dir]

    def move_center(self, action):
        signal = self.get_empty_sig()
        last_x, last_y = self._last_grid_center_pos
        deltax, delta_y = self.get_center_grid_delta(action.name)
        goal_x, goal_y = last_x + deltax, last_y + delta_y
        screen_goal = Vec2d(*self.coordinator.grid_to_screen(goal_x, goal_y))
        curr_center = Vec2d(*self.center_pos)
        delta = screen_goal - curr_center
        if delta.length<self.coordinator.config.epsilon:
            self.finish_action()
            return signal
        delta = (delta/delta.length)*self.coordinator.config.move_center_speed*self.coordinator.config.dt
        next_center_pos = curr_center + delta
        for i in range(self.coordinator.num_legs):

            curr_foot_pos = Vec2d(*self.feet_pos[i])
            v1 = curr_foot_pos - curr_center  # current direction of leg
            v2 = curr_foot_pos - next_center_pos
            next_leg_length = v2.length
            curr_leg_length = v1.length
            extension = (next_leg_length - curr_leg_length )/self.coordinator.config.dt
            signal.extension[i] = extension
            angle_difference = v1.get_angle_between(v2)
            signal.rotation[i] = (
                -self.coordinator.config.twist_speed
                if angle_difference > 0
                else self.coordinator.config.twist_speed
            )
        return signal


    def move_foot(self, action):
        sig = self.get_empty_sig()

        foot_id = int(action.name.split("_")[-1])
        curr_foot_pos = Vec2d(*self.feet_pos[foot_id])
        grid_goal_x, grid_goal_y = [int(x) for x in action.args[-1].split("_") if x.isdigit()]
        goal_foot_pos = Vec2d(*self.coordinator.grid_to_screen(grid_goal_x, grid_goal_y))
        if (goal_foot_pos-curr_foot_pos).length<self.coordinator.config.epsilon:
            if self.active_grips[foot_id] is not None:
                self.actions_finished+=1
                return sig
            sig.grip[foot_id] = 1
            return sig

        if self.active_grips[foot_id] is not None:
            sig.grip[foot_id] = -1

        curr_center = Vec2d(*self.center_pos)
        v1 = curr_foot_pos - curr_center
        v2 = goal_foot_pos - curr_center
        angle_difference = v1.get_angle_between(v2)

        # rotate toward shortest direction
        sig.rotation[
            foot_id] = -self.coordinator.config.twist_speed if\
            angle_difference > 0 else self.coordinator.config.twist_speed
        curr_length = (curr_center - curr_foot_pos).length
        needed_length = (goal_foot_pos - curr_center).length
        if needed_length < curr_length:
            sig.extension[foot_id] = -self.coordinator.config.extension_speed
        else:
            sig.extension[foot_id] = self.coordinator.config.extension_speed
        return sig

    def get_empty_sig(self):
        return Signal([0 for _ in range(self.coordinator.num_legs)],
                      [0 for _ in range(self.coordinator.num_legs)],
                      [0 for _ in range(self.coordinator.num_legs)])

    def get_sig(self):
        if self._start_last_action_time == 0:
            self._start_last_action_time = time.perf_counter()
        self.longest_action_time = max(self.longest_action_time, time.perf_counter() - self._start_last_action_time)
        if self.actions_finished == len(self.plan):
            return self.get_empty_sig()
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

    def finish_action(self):
        self._start_last_action_time = 0
        #print(f"Finished Action: {self.plan[self.actions_finished]}")
        self.actions_finished+=1
        if len(self.plan)>self.actions_finished:
            pass
            #print(f"Starting action {self.plan[self.actions_finished]}")
        else:
            self.finished_plan =True
            #print(f"Goal achieved")


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
