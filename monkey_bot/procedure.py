from typing import List

from pymunk import Vec2d

from monkey_bot.signals import StateSignal, ControlSignal
from monkey_bot.simulation_config import InstanceSimulationCoordinator


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



