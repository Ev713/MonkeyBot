import math
import os
import time
import typing
from dataclasses import dataclass
from logging import raiseExceptions
from math import hypot
from typing import List

from pymunk import Vec2d

from monkey_bot import upf_solver
from monkey_bot.action import parse_action, Action
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
import monkey_bot.upf_solver
from monkey_bot.procedure import MoveCenter, ReleaseGrip, Mixed, Tracker, Grabber, AdjustLength, DynamicCatch
from monkey_bot.signals import ControlSignal, StateSignal
from monkey_bot.simulation_config import SimConfig, InstanceSimulationCoordinator
from monkey_bot.trajectory_finder import JumpTrajectoryValidator




class Controller:
    def __init__(self, coordinator:InstanceSimulationCoordinator):
        self.finished_plan = False
        self.plan = []
        self.coordinator = coordinator

        self.t = 0
        self.actions_finished = 0

        self._dir_deltas = {'up': (0, 1), 'down': (0, -1), 'left': (-1, 0), 'right': (1, 0)}
        self.longest_action_time=0
        self._start_last_action_time=0
        self.procedures= None
        self.proc_id = 0

        self.jump_takeoff_instructions = {}


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

    def create_plan(self, plan_path=None):
        plan = monkey_bot.upf_solver.get_plan(self.coordinator.instance)
        if plan is None:
            raise ValueError("Planning problem is unsolvable")
        if plan_path:
            self.save_plan(plan, plan_path)
        self.plan = [parse_action(a) for a in plan.actions]

    def save_plan(self, plan, plan_path=None):
        with open(os.path.join(plan_path), 'w') as f:
            for a in plan.actions:
                f.write(f"{a}\n")

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


    def get_empty_sig(self):
        return ControlSignal([0 for _ in range(self.coordinator.num_legs)],
                      [0 for _ in range(self.coordinator.num_legs)],
                      [0 for _ in range(self.coordinator.num_legs)])

    def get_sig(self, state_info:StateSignal):
        sig = self.get_empty_sig()
        if self.actions_finished == len(self.plan):
            return sig

        if not self.procedures:
            current_action = self.plan[self.actions_finished]
            if "move_center" in current_action.name:
                self.setup_move_center_procedure_sequence(current_action)
            elif "move_foot" in current_action.name:
                self.setup_move_foot_procedure_sequence(current_action)
            elif "jump" in current_action.name:
                self.setup_jump_procedure_sequence(current_action, state_info)

        p = self.procedures[self.proc_id]
        if p.is_finished(state_info):
            self.proc_id += 1
            if self.proc_id == len(self.procedures):
                self.finish_action()
                self.procedures = None
        return p.adjust_signal(sig, state_info)

    def finish_action(self):
        self._start_last_action_time = 0
        #print(f"Finished Action: {self.plan[self.actions_finished]}")
        self.actions_finished+=1
        if len(self.plan)>self.actions_finished:
            #pass
            print(f"Starting action {self.plan[self.actions_finished]}")
        else:
            self.finished_plan =True
            print(f"Goal achieved")

    def read_plan(self, file_path):
        self.plan = [parse_action(l) for l in open(file_path)]

    def create_or_read_plan(self, folder):
        plan_path = f"{folder}/{self.coordinator.instance.name}.txt"

        if os.path.exists(plan_path):
            self.read_plan(plan_path)
            print(f"Loaded existing plan from {plan_path}")
        else:
            self.create_plan(plan_path=plan_path)
            print(f"Created and saved new plan to {plan_path}")

    def setup_move_center_procedure_sequence(self, current_action):
        goal_point = Vec2d(*self.coordinator.grid_to_screen(*tuple(Vec2d(*self._last_grid_center_pos)
                                        +Vec2d(*self.get_center_grid_delta(current_action.name)))))
        self.procedures = [MoveCenter(goal_point, self.coordinator)]
        self.proc_id = 0


    def setup_move_foot_procedure_sequence(self, current_action:Action):
        goal_point = self.coordinator.gp_name_to_screen_point(current_action.args[1])
        limb_id = int(current_action.name.split('_')[-1])
        self.procedures = []
        self.procedures.append(ReleaseGrip(limb_id, self.coordinator))
        catcher = Mixed([Tracker(limb_id, goal_point, self.coordinator),
                        Grabber(limb_id, goal_point, self.coordinator)], self.coordinator)
        self.procedures.append(catcher)

        self.proc_id = 0

    def calc_robot_mass(self):
        conf = self.coordinator.config
        return conf.body_mass+self.coordinator.num_legs*(conf.foot_mass+conf.body_mass)


    def check_jump_is_possible(self, from_1, from_2, p1, p2, p3, save_as):
        conf = self.coordinator.config
        jump_validator = JumpTrajectoryValidator(
            conf.gravity,
            conf.dt,
            [p1, p2, p3],
            self.coordinator.max_extension,
            [from_1, from_2],
        )

        start_point, takeoff_vector = jump_validator.suggest_trajectory_using_medians_and_ballistics()

        if jump_validator.setup_achieves_goal(start_point, takeoff_vector):
            self.jump_takeoff_instructions[save_as] = start_point, takeoff_vector
            return True
        else:
            return False


    def setup_jump_procedure_sequence(self, current_action:Action, state_info:StateSignal):
        jumping_leg_1_id = int(current_action.args[0])
        jumping_leg_2_id = int(current_action.args[1])
        jumping_leg_1_pos = state_info.feet_pos[jumping_leg_1_id]
        jumping_leg_2_pos = state_info.feet_pos[jumping_leg_2_id]

        catch_points = []
        for gp in current_action.args[2:]:
            if gp == '-1':
                catch_points.append(None)
            else:
                catch_points.append(self.coordinator.gp_name_to_screen_point(gp))

        ignore_legs = [leg_id for leg_id in range(self.coordinator.num_legs)
                       if leg_id not in (jumping_leg_1_id, jumping_leg_2_id)]

        keys = [jumping_leg_1_pos, jumping_leg_2_pos] + [tuple(p) for p in catch_points]
        key = tuple((round(k[0]), round(k[1])) for k in keys)
        if not self.check_jump_is_possible(jumping_leg_1_pos,jumping_leg_2_pos, *catch_points, key):
            raise Exception("Required jump is impossible")

        starting_point, jumping_vector = self.jump_takeoff_instructions[key]
        self.procedures = []
        take_off_point = starting_point
        while (state_info.feet_pos[jumping_leg_1_id] - take_off_point).length < self.coordinator.max_extension \
                and (state_info.feet_pos[jumping_leg_2_id] - take_off_point).length < self.coordinator.max_extension:
            take_off_point += self.coordinator.config.dt * jumping_vector
        take_off_point -= self.coordinator.config.dt * jumping_vector*5

        releasers = []
        for i in ignore_legs:
            releasers.append(ReleaseGrip(i, self.coordinator))

        self.procedures.append(Mixed(releasers, self.coordinator))
        retractors = []
        for i in ignore_legs:
            retractors.append(AdjustLength(i, self.coordinator.config.min_extension, self.coordinator))
        self.procedures.append(Mixed(retractors, self.coordinator))

        on_position = MoveCenter(tuple(starting_point), self.coordinator, ignore_legs)
        self.procedures.append(on_position)

        launch = MoveCenter(take_off_point, self.coordinator, ignore_legs)
        launch.move_center_speed = jumping_vector.length
        self.procedures.append(launch)

        takeoff_releasers = [ReleaseGrip(i, self.coordinator) for i in (jumping_leg_1_id, jumping_leg_2_id)]
        self.procedures.append(Mixed(takeoff_releasers, self.coordinator))
        catcher = DynamicCatch(catch_points, state_info, self.coordinator)
        self.procedures.append(catcher)
        self.proc_id = 0
