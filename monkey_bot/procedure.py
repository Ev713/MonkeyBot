import math
from typing import List

from pymunk import Vec2d

from monkey_bot.signals import StateSignal, ControlSignal
from monkey_bot.simulation_config import InstanceSimulationCoordinator


class Procedure:
    def __init__(self, coordinator: InstanceSimulationCoordinator):
        self.epsilon = coordinator.epsilon
        self.rotation_speed = coordinator.rotation_speed
        self.extension_speed = coordinator.extension_speed
        self.move_center_speed = coordinator.move_center_speed
        self.num_legs = coordinator.num_legs
        self.dt = coordinator.dt
        self.max_extension = coordinator.max_extension
        self.min_extension = coordinator.min_extension
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
        p = 0.5
        abs_rate = p*abs_rate+(1-p)*(abs(d)/math.pi)*abs_rate
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

class MultiOptionalDynamicCatch(Procedure):
    def __init__(self, admissable_catch_points, state, coordinator:InstanceSimulationCoordinator):
        super().__init__(coordinator)
        self.admissable_catch_points = admissable_catch_points
        self.chosen_catch_points = []
        self.update_catch_points(state)
        self.dynamic_catcher = DynamicCatch(self.chosen_catch_points, state, coordinator)

    def adjust_signal(self, signal: ControlSignal, state_info):
        self.update_catch_points(state_info)
        return self.dynamic_catcher.adjust_signal(signal, state_info)

    def is_finished(self, state_info:StateSignal):
        return self.dynamic_catcher.is_finished(state_info)

    def update_catch_points(self, state_info:StateSignal):
        self.chosen_catch_points = [None for _ in state_info.feet_pos]
        taken = set()

        # Precompute all distances
        pairs = []
        for i, foot in enumerate(state_info.feet_pos):
            for j, point in enumerate(self.admissable_catch_points):
                dist = (foot - point).length
                pairs.append((dist, i, j))

        # Sort by distance
        pairs.sort(key=lambda x: x[0])

        # Greedy assignment
        for dist, i, j in pairs:
            if self.chosen_catch_points[i] is None and j not in taken:
                self.chosen_catch_points[i] = self.admissable_catch_points[j]
                taken.add(j)

        return self.chosen_catch_points
class DynamicCatch(Procedure):
    def __init__(self, target_points, state ,coordinator: InstanceSimulationCoordinator):
        super().__init__(coordinator)
        self.target_points = target_points
        self.trackers = []
        self.prev_state = state
        self.grabbers = []
        self.move_center = MoveCenter(None, coordinator)

        for limb_id in range(self.num_legs):
            if target_points[limb_id] is None:
                continue
            tracker = Tracker(limb_id, target_points[limb_id], coordinator)
            grabber = Grabber(limb_id, target_points[limb_id], coordinator)
            self.grabbers.append(grabber)
            self.trackers.append(tracker)

    def get_closest_point(self, state_info:StateSignal):
        center = state_info.center_pos
        closest_point = None
        for p in self.target_points:
            if closest_point is None or (p-center).length < (closest_point - center).length:
                closest_point = p
        return closest_point


    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        self.prev_state = state_info
        for grabber, tracker in zip(self.grabbers, self.trackers):
            signal = grabber.adjust_signal(signal, state_info)
            if not grabber.is_finished(state_info):
                signal = tracker.adjust_signal(signal, state_info)
            finished_grabbers = [grabber for grabber in self.grabbers if grabber.is_finished(state_info)]
            if len(finished_grabbers) > 1:
                signal = self.adjust_signal_to_move_center_closer_to_closest_points(signal, state_info)
        return signal

    def adjust_signal_to_move_center_closer_to_closest_points(self, signal, state_info):
        ignore_legs = [tracker.limb_id for grabber, tracker in zip(self.grabbers, self.trackers) if
                       not grabber.is_finished(state_info)]
        self.move_center.goal_point = self.get_closest_point(state_info)
        self.move_center.move_center_speed = self.move_center_speed * 0.2
        self.move_center.ignore_leg_points = ignore_legs
        signal = self.move_center.adjust_signal(signal, state_info)
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
        self.ignore_legs = ignore_legs if ignore_legs is not None else []
        self.length_adjusters = [AdjustLength(limb_id, None, coordinator, None)
                                 for limb_id in range(self.num_legs)]
        self.angle_adjusters = [AdjustAngle(limb_id, None, coordinator)
                                 for limb_id in range(self.num_legs)]


    def _calc_next_point(self, state_info:StateSignal):
        move_vec = (self.goal_point - state_info.center_pos)
        return state_info.center_pos + move_vec/move_vec.length*self.dt*self.move_center_speed

    def get_adjuster(self, limb_id):
        for adjuster in self.length_adjusters:
            if adjuster.limb_id == limb_id:
                return adjuster
        raise Exception("No suitable adjuster")

    def active_legs(self):
        return [a.limb_id for a in self.length_adjusters if a.limb_id not in self.ignore_legs]

    def adjust_signal(self, signal: ControlSignal, state_info):
        next_frame_center = self._calc_next_point(state_info)
        active_legs = self.active_legs()


        for a in self.length_adjusters:
            limb_id = a.limb_id

            foot_pos = state_info.feet_pos[limb_id]
            next_foot_vec = foot_pos - next_frame_center
            curr_foot_vec = foot_pos - state_info.center_pos
            self.get_adjuster(limb_id).goal_extension = next_foot_vec.length
            self.get_adjuster(limb_id).extension_speed = abs(next_foot_vec.length - curr_foot_vec.length)/self.dt
            signal.rotation[limb_id] = None

        if len(active_legs) == 2:
            p1 = state_info.feet_pos[active_legs[0]]
            p2 = state_info.feet_pos[active_legs[1]]
            seg_vec = p2 - p1

            seg_len_sq = seg_vec.dot(seg_vec)
            if seg_len_sq == 0:
                # Degenerate case: x1 == x2
                distance = (next_frame_center - p1).length
            else:
                # Project p onto the line (parameterized by t)
                t = (next_frame_center - p1).dot(seg_vec) / seg_len_sq
                projection = p1 + t * seg_vec
                distance = (next_frame_center - projection).length
            if distance < self.epsilon*5:
                for a in self.angle_adjusters:
                    if a.limb_id in active_legs:
                        a.target_point=state_info.center_pos+(state_info.feet_pos[a.limb_id]- self.goal_point)
                        signal = a.adjust_signal(signal, state_info)

        for a in self.length_adjusters:
            if a.limb_id in active_legs:
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



