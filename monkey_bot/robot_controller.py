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
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
import monkey_bot.upf_solver
from monkey_bot.signals import ControlSignal, StateSignal
from monkey_bot.simulation_config import SimConfig, InstanceSimulationCoordinator


@dataclass
class Action:
    name:str
    args:List[str]


class Procedure:
    def __init__(self, coordinator: InstanceSimulationCoordinator):
        self.epsilon = coordinator.config.epsilon
        self.rotation_speed = coordinator.config.rotation_speed
        self.extension_speed = coordinator.config.extension_speed
        self.move_center_speed = coordinator.config.move_center_speed
        self.num_legs = coordinator.num_legs
        self.dt = coordinator.config.dt
        pass

    def adjust_signal(self, signal: ControlSignal, state_info):
        pass

    def is_finished(self, state_info:StateSignal):
        pass

class AdjustLength(Procedure):
    def __init__(self, limb_id, goal_extension, coordinator:InstanceSimulationCoordinator, custom_speed=None):
        super().__init__(coordinator)
        self.limb_id = limb_id
        self.goal_extension = goal_extension
        if custom_speed is not None:
            self.extension_speed = custom_speed

    def curr_length(self, state_info):
        return  (state_info.center_pos - state_info.feet_pos[self.limb_id]).length


    def adjust_signal(self, signal: ControlSignal, state_info):
        sign = 1 if self.goal_extension - self.curr_length(state_info) > 0 else -1
        signal.extension[self.limb_id] = signal.extension[self.limb_id] + sign*self.extension_speed
        return signal

    def is_finished(self, state_info:StateSignal):
        return abs(self.curr_length(state_info) - self.goal_extension) < self.epsilon

class ReleaseGrip(Procedure):
    def __init__(self, limb_id, coordinator):
        super().__init__(coordinator)
        self.limb_id = limb_id

    def adjust_signal(self, signal: ControlSignal, state_info):
        if state_info.active_grips[self.limb_id]:
            signal.grip[self.limb_id] = -1
        return signal

    def is_finished(self, state_info:StateSignal):
        return not state_info.active_grips[self.limb_id]


class AdjustAngle(Procedure):
    def __init__(self, limb_id, target_point: Vec2d,coordinator: InstanceSimulationCoordinator):
        super().__init__(coordinator)
        self.limb_id = limb_id
        self.target_point = target_point

    def adjust_signal(self, signal: ControlSignal, state_info):
        if self.rotation_speed is None:
            raise Exception("Rotation speed unfilled")
        d = self.curr_angle_difference(state_info)
        abs_rate = self.rotation_speed
        if abs(d) < abs_rate * self.dt:
            abs_rate = d*self.dt

        signal.rotation[self.limb_id] = - abs_rate if d > 0 else abs_rate
        return signal

    def curr_angle_difference(self, state_info):
        v1 = state_info.center_pos - state_info.feet_pos[self.limb_id]
        v2 = state_info.center_pos - self.target_point
        return v1.get_angle_between(v2)


    def is_finished(self, state_info:StateSignal):
        if self.target_point is None or self.epsilon is None:
            raise Exception("Target point or epsilon is unfilled")
        return abs(self.curr_angle_difference(state_info)) <= self.epsilon

class DynamicCatch(Procedure):
    def __init__(self, target_points, state ,coordinator: InstanceSimulationCoordinator):
        super().__init__(coordinator)
        self.target_points = target_points
        self.trackers = []
        self.prev_state = state
        self.grabbers = []

        for limb_id in range(self.num_legs):
            if target_points[limb_id] is None:
                continue
            tracker = Tracker(limb_id, target_points[limb_id], coordinator)
            grabber = Grabber(limb_id, target_points[limb_id], coordinator)
            self.grabbers.append(grabber)
            self.trackers.append(tracker)

    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        self.prev_state = state_info
        for grabber, tracker in zip(self.grabbers, self.trackers):
            signal = grabber.adjust_signal(signal, state_info)
            if not grabber.is_finished(state_info):
                signal = tracker.adjust_signal(signal, state_info)
        return signal

    def move_vector(self, state_info: StateSignal):
        if self.prev_state is None:
            self.prev_state = state_info
        return state_info.center_pos - self.prev_state.center_pos

    def move_leg_vector(self, limb_id, state_info):
        if self.prev_state is None:
            self.prev_state = state_info
        return state_info.feet_pos[limb_id] - self.prev_state.center_pos[limb_id] - self.move_vector(state_info)

    def is_finished(self, state_info:StateSignal):
        return all(g.is_finished(state_info) for g in self.grabbers)

class Grabber(Procedure):
    def __init__(self, limb_id, target_point: Vec2d, coordinator: InstanceSimulationCoordinator):
        super().__init__(coordinator)
        self.limb_id = limb_id
        self.target_point = target_point

    def adjust_signal(self, signal: ControlSignal, state_info):
        d = (state_info.feet_pos[self.limb_id] - self.target_point).length
        if abs(d) < self.epsilon:
            signal.grip[self.limb_id] = 1
        return signal

    def is_finished(self, state_info:StateSignal):
        return state_info.active_grips[self.limb_id] == 1

class Tracker(Procedure):
    def __init__(self, limb_id, target_point: Vec2d, coordinator:InstanceSimulationCoordinator):
        super().__init__(coordinator)
        self.limb_id = limb_id
        self.target_point = target_point
        self.angle_adjuster = AdjustAngle(limb_id, target_point, coordinator)
        self.length_adjuster = AdjustLength(limb_id, None, coordinator)

    def set_target_point(self, target_point: Vec2d):
        self.target_point = target_point
        self.angle_adjuster.target_point = target_point


    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        self.length_adjuster.goal_extension = (self.target_point - state_info.center_pos).length
        signal = self.length_adjuster.adjust_signal(signal, state_info)
        signal = self.angle_adjuster.adjust_signal(signal, state_info)
        return signal

    def is_finished(self, state_info:StateSignal):
        return True

class MoveCenter(Procedure):
    def __init__(self, goal_point, coordinator, ignore_legs=None):
        super().__init__(coordinator)
        self.goal_point = goal_point
        if ignore_legs is None:
            ignore_legs = []
        self.length_adjusters = [AdjustLength(limb_id, None, coordinator, None)
                                 for limb_id in range(self.num_legs) if limb_id not in ignore_legs]

    def _calc_next_point(self, state_info:StateSignal):
        move_vec = (self.goal_point - state_info.center_pos)
        return state_info.center_pos + move_vec/move_vec.length*self.dt*self.move_center_speed

    def adjust_signal(self, signal: ControlSignal, state_info):
        next_frame_center = self._calc_next_point(state_info)
        for a in self.length_adjusters:
            limb_id = a.limb_id
            foot_pos = state_info.feet_pos[limb_id]
            next_foot_vec = foot_pos - next_frame_center
            curr_foot_vec = foot_pos - state_info.center_pos
            self.length_adjusters[limb_id].goal_extension = next_foot_vec.length
            self.length_adjusters[limb_id].extension_speed = abs(next_foot_vec.length - curr_foot_vec.length)/self.dt
            signal.rotation[limb_id] = None

        for a in self.length_adjusters:#+self.angle_adjusters:
            signal = a.adjust_signal(signal, state_info)
        return signal


    def is_finished(self, state_info:StateSignal):
        return (state_info.center_pos - self.goal_point).length < self.epsilon

class Mixed(Procedure):
    def __init__(self, procs:List[Procedure], coordinator):
        super().__init__(coordinator)
        self.procs = procs
        self.finished = [False for _ in procs]

    def adjust_signal(self, signal: ControlSignal, state_info):
        for proc in self.procs:
            signal = proc.adjust_signal(signal, state_info)
        return signal

    def is_finished(self, state_info:StateSignal):
        for i, proc in enumerate(self.procs):
            if proc.is_finished(state_info):
                self.finished[i] = True
        return all(self.finished)


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

        self.t = 0
        self.actions_finished = 0

        self._dir_deltas = {'up': (0, 1), 'down': (0, -1), 'left': (-1, 0), 'right': (1, 0)}
        self.longest_action_time=0
        self._start_last_action_time=0
        self.procedures= None
        self.proc_id = 0


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
            with open(os.path.join(plan_path), 'w') as f:
                for a in plan.actions:
                    f.write(f"{a}\n")
        self.plan = [parse_action(a) for a in plan.actions]

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

    def setup_jump_procedure_sequence(self, current_action:Action, state_info:StateSignal):
        jumping_leg_1_id = int(current_action.args[0])
        jumping_leg_2_id = int(current_action.args[1])

        catch_points = []
        for gp in current_action.args[2:]:
            if gp == '-1':
                catch_points.append(None)
            else:
                catch_points.append(self.coordinator.gp_name_to_screen_point(gp))

        ignore_legs = [leg_id for leg_id in range(self.coordinator.num_legs)
                       if leg_id not in (jumping_leg_1_id, jumping_leg_2_id)]

        jumping_vector = Vec2d(0, -500)
        self.procedures = []
        take_off_point = state_info.center_pos
        while (state_info.feet_pos[jumping_leg_1_id] - take_off_point).length < self.coordinator.max_extension \
                and (state_info.feet_pos[jumping_leg_2_id] - take_off_point).length < self.coordinator.max_extension:
            take_off_point += self.coordinator.config.dt * jumping_vector
        take_off_point -= self.coordinator.config.dt * jumping_vector
        releasers = []
        for i in ignore_legs:
            releasers.append(ReleaseGrip(i, self.coordinator))

        self.procedures.append(Mixed(releasers, self.coordinator))
        retractors = []
        for i in ignore_legs:
            retractors.append(AdjustLength(i, self.coordinator.config.min_extension, self.coordinator))
        self.procedures.append(Mixed(retractors, self.coordinator))


        move_center = MoveCenter(take_off_point, self.coordinator, ignore_legs)
        move_center.move_center_speed = jumping_vector.length
        self.procedures.append(move_center)

        takeoff_releasers = [ReleaseGrip(i, self.coordinator) for i in (jumping_leg_1_id, jumping_leg_2_id)]
        self.procedures.append(Mixed(takeoff_releasers, self.coordinator))
        catcher = DynamicCatch(catch_points, state_info, self.coordinator)
        self.procedures.append(catcher)
        self.proc_id = 0
