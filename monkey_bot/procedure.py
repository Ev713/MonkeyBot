import math
from typing import List

from pymunk import Vec2d

from monkey_bot.signals import StateSignal, ControlSignal
from monkey_bot.config import InstanceSimulationConfig


def normalize_angle(angle: float) -> float:
    return (angle + math.pi) % (2 * math.pi) - math.pi


def format_limb_id(limb_id: int) -> str:
    return str(limb_id + 1)


def format_limb_list(limb_ids) -> str:
    return ", ".join(format_limb_id(i) for i in limb_ids)


def format_grid_point(coordinator: InstanceSimulationConfig, point: Vec2d) -> str:
    gx, gy = coordinator.screen_to_grid(point)
    return f"({gx},{gy})"


def format_grid_distance(coordinator: InstanceSimulationConfig, a: Vec2d, b: Vec2d) -> str:
    cells = (a - b).length / coordinator.cell_size
    return f"{cells:.2f} cells"


class Procedure:
    def __init__(self, coordinator: InstanceSimulationConfig):
        self.coordinator = coordinator
        self.epsilon = coordinator.epsilon
        self.rotation_speed = coordinator.rotation_speed
        self.extension_speed = coordinator.extension_speed
        self.move_center_speed = coordinator.move_center_speed
        self.num_legs = coordinator.num_legs
        self.dt = coordinator.dt
        self.max_extension = coordinator.max_extension
        self.min_extension = coordinator.min_extension

    def adjust_signal(self, signal: ControlSignal, state_info):
        pass

    def is_finished(self, state_info:StateSignal):
        pass

    def describe_start(self, state_info: StateSignal) -> str | None:
        return None

    def describe_finish(self, state_info: StateSignal) -> str | None:
        return None

    def _log_phase(self, phase: str, message: str) -> None:
        if getattr(self, "_phase", None) != phase:
            self._phase = phase
            print(message, flush=True)

class GetCloser(Procedure):
    # Match DynamicCatch: gentle center shifts while legs stay gripped.
    MOVE_CENTER_SPEED_FACTOR = 0.2

    def __init__(
        self,
        target_point,
        coordinator: InstanceSimulationConfig,
        ignore_legs=None,
    ):
        super().__init__(coordinator)
        self.target_point = target_point
        self.move_center = MoveCenter(None, coordinator, ignore_legs=ignore_legs)
        self.move_center.move_center_speed *= self.MOVE_CENTER_SPEED_FACTOR
        self.move_center.max_length_adjust_rate = self.extension_speed * 0.35

    def is_active(self, state_info: StateSignal) -> bool:
        return (state_info.center_pos - self.target_point).length > self.max_extension * 0.5

    def is_finished(self, state_info:StateSignal):
        return True

    def adjust_signal(self, signal: ControlSignal, state_info):
        if self.is_active(state_info):
            self.move_center.goal_point = (state_info.center_pos + self.target_point) / 2
            signal = self.move_center.adjust_signal(signal, state_info)
        return signal

class AdjustLength(Procedure):
    def __init__(self, limb_id, goal_extension, coordinator:InstanceSimulationConfig, custom_speed=None):
        super().__init__(coordinator)
        self.limb_id = limb_id
        self.goal_extension = goal_extension
        if custom_speed is not None:
            self.extension_speed = custom_speed

    def curr_length(self, state_info):
        return  (state_info.center_pos - state_info.feet_pos[self.limb_id]).length

    def adjust_signal(self, signal: ControlSignal, state_info):
        error = self.goal_extension - self.curr_length(state_info)
        if abs(error) < self.epsilon:
            return signal
        drive = max(-self.extension_speed, min(self.extension_speed, 3.0 * error))
        signal.extension[self.limb_id] = signal.extension[self.limb_id] + drive
        return signal

    def is_finished(self, state_info:StateSignal):
        return abs(self.curr_length(state_info) - self.goal_extension) < self.epsilon


class GrooveSafeAdjustLength(AdjustLength):
    """Extension toward a target without slamming the foot out of the leg groove."""

    def __init__(
        self,
        limb_id,
        goal_extension,
        coordinator: InstanceSimulationConfig,
        *,
        speed_factor: float = 0.3,
        max_error_gain: float = 1.2,
    ):
        super().__init__(limb_id, goal_extension, coordinator)
        self.speed_factor = speed_factor
        self.max_error_gain = max_error_gain

    def adjust_signal(self, signal: ControlSignal, state_info):
        curr = self.curr_length(state_info)
        goal = min(self.goal_extension, self.max_extension * 0.92)
        error = goal - curr
        if abs(error) < self.epsilon:
            return signal
        max_rate = self.extension_speed * self.speed_factor
        if curr >= self.max_extension * 0.9 and error > 0:
            return signal
        drive = max(-max_rate, min(max_rate, self.max_error_gain * error))
        signal.extension[self.limb_id] = signal.extension[self.limb_id] + drive
        return signal

class ReleaseGrip(Procedure):
    def __init__(self, limb_id, coordinator):
        super().__init__(coordinator)
        self.limb_id = limb_id

    def describe_start(self, state_info: StateSignal) -> str | None:
        return f"releasing leg {format_limb_id(self.limb_id)}"

    def adjust_signal(self, signal: ControlSignal, state_info):
        if state_info.active_grips[self.limb_id]:
            signal.grip[self.limb_id] = -1
        return signal

    def is_finished(self, state_info:StateSignal):
        return not state_info.active_grips[self.limb_id]


class StabilizeLimbs(Procedure):
    """
    Damp body swing and relax leg springs before a release or other disruptive action.

    Sets rotation and extension motor rates to zero on all legs — no pymunk angle
    locks, which can snap when added or removed under load.
    """

    def __init__(
        self,
        coordinator: InstanceSimulationConfig,
        *,
        settle_frames: int = 36,
        angular_velocity_limit: float = 0.12,
        kp: float = 2.0,
        kd: float = 1.0,
        max_correction_rate: float | None = None,
        angle_deadband: float = 0.03,
        velocity_deadband: float = 0.05,
        smooth: float = 0.88,
    ):
        super().__init__(coordinator)
        self.settle_frames = settle_frames
        self.angular_velocity_limit = angular_velocity_limit
        self.kp = kp
        self.kd = kd
        self.max_correction_rate = (
            max_correction_rate
            if max_correction_rate is not None
            else coordinator.angle_rotation_speed * 0.4
        )
        self.angle_deadband = angle_deadband
        self.velocity_deadband = velocity_deadband
        self.smooth = smooth
        self._stable_frames = 0
        self.ref_angle = None
        self.prev_extension = [0.0] * self.num_legs
        self._was_settling = False

    def describe_start(self, state_info: StateSignal) -> str | None:
        gripped = [
            format_limb_id(i)
            for i, g in enumerate(state_info.active_grips)
            if g
        ]
        spin = state_info.body_angular_velocity
        legs = f"legs {', '.join(gripped)}" if gripped else "no gripped legs"
        return f"{legs} | body spin {spin:.3f} rad/s | need {self.settle_frames} calm frames"

    def describe_finish(self, state_info: StateSignal) -> str | None:
        spin = state_info.body_angular_velocity
        drift = normalize_angle(state_info.body_angle - self.ref_angle) if self.ref_angle is not None else 0.0
        return f"settled | spin {spin:.3f} rad/s | angle drift {math.degrees(drift):.1f}°"

    def _update_status(self, state_info: StateSignal) -> None:
        spin = abs(state_info.body_angular_velocity)
        if spin > self.angular_velocity_limit:
            if self._was_settling:
                self._log_phase(
                    "damping",
                    f"StabilizeLimbs: spin {spin:.3f} rad/s — damping, settle counter reset",
                )
            else:
                self._log_phase(
                    "damping",
                    f"StabilizeLimbs: damping body swing ({spin:.3f} rad/s)",
                )
            self._was_settling = False
            return

        if not self._was_settling:
            self._log_phase(
                "settling",
                f"StabilizeLimbs: settling ({self.settle_frames} frames below {self.angular_velocity_limit:.2f} rad/s)",
            )
            self._was_settling = True

        if self._stable_frames > 0 and self._stable_frames % 12 == 0:
            remaining = self.settle_frames - self._stable_frames
            if remaining > 0:
                self._log_phase(
                    f"settling-{self._stable_frames}",
                    f"StabilizeLimbs: {remaining} calm frames remaining (spin {spin:.3f} rad/s)",
                )

    def _smooth_step(self, previous: float, target: float, max_step: float) -> float:
        blended = self.smooth * previous + (1.0 - self.smooth) * target
        return previous + max(-max_step, min(max_step, blended - previous))

    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        self._update_status(state_info)

        if not signal.relax_spring:
            signal.relax_spring = [False] * self.num_legs
        if not signal.angle_lock:
            signal.angle_lock = [False] * self.num_legs

        if self.ref_angle is None:
            self.ref_angle = state_info.body_angle
            for limb_id in range(self.num_legs):
                self.prev_extension[limb_id] = signal.extension[limb_id]

        max_extension_step = self.extension_speed * self.dt

        for limb_id in range(self.num_legs):
            signal.relax_spring[limb_id] = True
            signal.angle_lock[limb_id] = False
            signal.rotation[limb_id] = 0.0

            ext = self._smooth_step(
                self.prev_extension[limb_id], 0.0, max_extension_step
            )
            self.prev_extension[limb_id] = ext
            signal.extension[limb_id] = ext

        signal.damp_body = True
        return signal

    def is_finished(self, state_info: StateSignal):
        if abs(state_info.body_angular_velocity) > self.angular_velocity_limit:
            self._stable_frames = 0
            return False
        self._stable_frames += 1
        return self._stable_frames >= self.settle_frames


class AdjustAngle(Procedure):
    def __init__(self, limb_id, target_point: Vec2d, coordinator: InstanceSimulationConfig):
        super().__init__(coordinator)
        self.limb_id = limb_id
        self.target_point = target_point
        self.angle_rotation_speed = coordinator.angle_rotation_speed

    def adjust_signal(self, signal: ControlSignal, state_info):
        if self.angle_rotation_speed is None:
            raise Exception("Rotation speed unfilled")
        active_grips = [g for g in state_info.active_grips if g]
        if len(active_grips) == 1:
            return signal
        d = self.curr_angle_difference(state_info)
        if abs(d) < 0.02:
            return signal
        sign = -1 if d > 0 else 1
        signal.rotation[self.limb_id] = sign * self.angle_rotation_speed
        return signal

    def curr_angle_difference(self, state_info):
        v1 = state_info.center_pos - state_info.feet_pos[self.limb_id]
        v2 = state_info.center_pos - self.target_point
        return v1.get_angle_between(v2)


    def is_finished(self, state_info:StateSignal):
        if self.target_point is None or self.epsilon is None:
            raise Exception("Target point or epsilon is unfilled")
        return abs(self.curr_angle_difference(state_info)) <= self.epsilon

class SmoothAdjustAngle(AdjustAngle):
    def __init__(self, limb_id, target_point: Vec2d, coordinator: InstanceSimulationConfig, acceleration_rate=0.35):
        super().__init__(limb_id, target_point, coordinator)
        self.prev_abs_rate = 0
        self.acceleration_rate = acceleration_rate

    def adjust_signal(self, signal: ControlSignal, state_info):
        if self.angle_rotation_speed is None:
            raise Exception("Rotation speed unfilled")

        d = self.curr_angle_difference(state_info)
        sign = -1 if d > 0 else 1
        desired_rate = self.angle_rotation_speed
        final_rate = desired_rate
        if self.acceleration_rate > 0:
            acceleration_limited_speed = self.prev_abs_rate + self.acceleration_rate * self.dt
            required_stop_abs_rate = math.sqrt(2 * self.acceleration_rate * abs(d))

            if self.prev_abs_rate > required_stop_abs_rate:
                decceleration_limited_rate = self.prev_abs_rate - self.angle_rotation_speed * self.dt
            else:
                decceleration_limited_rate = acceleration_limited_speed
            final_rate = min(desired_rate, acceleration_limited_speed, decceleration_limited_rate)

        self.prev_abs_rate = final_rate
        signal.rotation[self.limb_id] = sign * final_rate
        return signal

class MultiOptionalDynamicCatch(Procedure):
    def __init__(self, admissable_catch_points, state_info:StateSignal, coordinator:InstanceSimulationConfig):
        super().__init__(coordinator)
        self.admissable_catch_points = admissable_catch_points
        self.chosen_catch_points = []
        self.dynamic_catcher = DynamicCatch(admissable_catch_points[:len(state_info.feet_pos)], state_info, coordinator)
        self.update_catch_points(state_info)

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
            if len(taken) == len(self.chosen_catch_points):
                break
            if self.chosen_catch_points[i] is None and j not in taken:
                self.chosen_catch_points[i] = self.admissable_catch_points[j]
                taken.add(j)
        for tracker, grabber in zip(self.dynamic_catcher.trackers, self.dynamic_catcher.grabbers):
            tracker.set_target_point(self.chosen_catch_points[tracker.limb_id])
            grabber.target_point = self.chosen_catch_points[tracker.limb_id]

        return self.chosen_catch_points

class DynamicCatch(Procedure):
    def __init__(self, target_points, state, coordinator: InstanceSimulationConfig):
        super().__init__(coordinator)
        self.target_points = target_points
        self.trackers = []
        self.prev_state = state
        self.grabbers = []
        self.move_center = MoveCenter(None, coordinator)
        self.finished_legs = None
        self.get_closer = GetCloser(None, coordinator)

        for limb_id in range(self.num_legs):
            if target_points[limb_id] is None:
                continue
            tracker = Tracker(limb_id, target_points[limb_id], coordinator)
            grabber = Grabber(limb_id, target_points[limb_id], coordinator)
            self.grabbers.append(grabber)
            self.trackers.append(tracker)

    def get_closest_free_point(self, state_info: StateSignal):
        center = state_info.center_pos
        closest_point = None
        min_dist = float('inf')
        for p in self.target_points:
            if self.is_free(state_info, p):
                dist = (p - center).length
                if dist < min_dist:
                    min_dist = dist
                    closest_point = p
        if closest_point is None:
            return center
        return closest_point

    def is_free(self, state_info, p):
        foot_dists = [(foot - p).length for foot in state_info.feet_pos]
        return all([d > 2*self.epsilon for d in foot_dists])

    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        self.update_finished_legs(state_info)
        self.prev_state = state_info
        for grabber, tracker in zip(self.grabbers, self.trackers):
            signal = grabber.adjust_signal(signal, state_info)
            if not grabber.is_finished(state_info):
                signal = tracker.adjust_signal(signal, state_info)
        if len(self.finished_legs) > 0:
            p = self.get_closest_free_point(state_info)
            self.get_closer.target_point = p
            signal = self.get_closer.adjust_signal(signal, state_info)

        return signal

    def adjust_signal_to_move_center_closer_to_closest_points(self, signal, state_info):
        ignore_legs = [tracker.limb_id for tracker in self.trackers if tracker.limb_id not in self.finished_legs]
        self.move_center.goal_point = self.get_closest_free_point(state_info)
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

    def update_finished_legs(self, state_info:StateSignal):
        self.finished_legs = [tracker.limb_id for grabber, tracker in zip(self.grabbers, self.trackers) if grabber.is_finished(state_info)]

class Grabber(Procedure):
    def __init__(self, limb_id, target_point: Vec2d, coordinator: InstanceSimulationConfig):
        super().__init__(coordinator)
        self.limb_id = limb_id
        self.target_point = target_point

    def adjust_signal(self, signal: ControlSignal, state_info):
        if state_info.active_grips[self.limb_id]:
            return signal
        foot_pos = state_info.feet_pos[self.limb_id]
        d = (foot_pos - self.target_point).length
        if d <= self.epsilon:
            signal.grip[self.limb_id] = 1
        return signal

    def is_finished(self, state_info:StateSignal):
        return state_info.active_grips[self.limb_id]


class AttachCatch(Procedure):
    """
    Reach toward a gripping point and retry grabbing until the foot latches on.

    Runs in two sequential phases each attach:
    1. MoveCenter (via GetCloser) on gripped legs only — shift body toward the GP.
    2. Tracker + body hold + grab on the free leg — reach and latch.
    """

    def __init__(
        self,
        limb_id: int,
        target_point: Vec2d,
        coordinator: InstanceSimulationConfig,
        holding_limb_ids: tuple[int, ...] | list[int],
    ):
        super().__init__(coordinator)
        self.limb_id = limb_id
        self.get_closer = GetCloser(target_point, coordinator, ignore_legs=[limb_id])
        self.tracker = Tracker(limb_id, target_point, coordinator)
        self.tracker.length_adjuster = GrooveSafeAdjustLength(limb_id, None, coordinator)
        self.body_holder = SmoothBodyHolder(coordinator, *holding_limb_ids)
        self.grabber = Grabber(limb_id, target_point, coordinator)
        self.target_point = target_point
        self.holding_limb_ids = list(holding_limb_ids)
        self._status_frame = 0
        self._status_interval = 20
        self._reach_phase = False

    def _foot_dist(self, state_info: StateSignal) -> float:
        return (state_info.feet_pos[self.limb_id] - self.target_point).length

    def _foot_can_reach_gp(self, state_info: StateSignal) -> bool:
        foot_dist = self._foot_dist(state_info)
        center_dist = (state_info.center_pos - self.target_point).length
        grip_slack = self.epsilon + self.coordinator.foot_radius

        if foot_dist <= grip_slack:
            return True

        # Foot almost on GP — tracker can finish without shifting the body.
        if foot_dist <= self.coordinator.cell_size * 1.25:
            return True

        # Body close enough that a leg can span the GP without a center shift.
        return center_dist <= (self.max_extension - self.min_extension) * 0.92

    def _move_center_needed(self, state_info: StateSignal) -> bool:
        if self._foot_can_reach_gp(state_info):
            return False
        return self.get_closer.is_active(state_info)

    def _begin_reach_phase(self, state_info: StateSignal, reason: str) -> None:
        if self._reach_phase:
            return
        self._reach_phase = True
        foot_cells = format_grid_distance(
            self.coordinator,
            state_info.feet_pos[self.limb_id],
            self.target_point,
        )
        self._log_phase(
            "reach_start",
            f"AttachCatch: skipping MoveCenter ({reason}) — starting Tracker on leg "
            f"{format_limb_id(self.limb_id)} (foot {foot_cells} from GP)",
        )

    def _in_pull_up_phase(self, state_info: StateSignal) -> bool:
        if self._reach_phase:
            return False
        if not self._move_center_needed(state_info):
            if self._foot_can_reach_gp(state_info):
                self._begin_reach_phase(state_info, "foot already in reach")
            else:
                self._begin_reach_phase(state_info, "center close enough")
            return False
        return True

    def _apply_pull_up_support(self, signal: ControlSignal, state_info: StateSignal) -> ControlSignal:
        signal.rotation[self.limb_id] = 0.0
        signal.extension[self.limb_id] = 0.0
        return signal

    def _attach_mode(self, state_info: StateSignal) -> str:
        if self._in_pull_up_phase(state_info):
            return "pull-up (MoveCenter)"
        return "reach (Tracker)"

    def _format_leg_line(
        self, state_info: StateSignal, signal: ControlSignal, limb_id: int
    ) -> str:
        grip = "G" if state_info.active_grips[limb_id] else "F"
        length = (
            (state_info.center_pos - state_info.feet_pos[limb_id]).length
            / self.coordinator.cell_size
        )
        foot_gp = format_grid_point(self.coordinator, state_info.feet_pos[limb_id])
        return (
            f"L{format_limb_id(limb_id)}[{grip}] @ {foot_gp} "
            f"len={length:.2f} rot={signal.rotation[limb_id]:+.2f} ext={signal.extension[limb_id]:+.2f}"
        )

    def _build_status(self, state_info: StateSignal, signal: ControlSignal) -> str:
        mode = self._attach_mode(state_info)
        target = format_grid_point(self.coordinator, self.target_point)
        foot_dist = format_grid_distance(
            self.coordinator,
            state_info.feet_pos[self.limb_id],
            self.target_point,
        )
        center_dist = format_grid_distance(
            self.coordinator, state_info.center_pos, self.target_point
        )
        legs = " | ".join(
            self._format_leg_line(state_info, signal, i)
            for i in range(self.num_legs)
        )
        return (
            f"AttachCatch [{mode}] leg {format_limb_id(self.limb_id)} → {target} "
            f"(foot {foot_dist}, center {center_dist}) | "
            f"body spin {state_info.body_angular_velocity:+.3f} rad/s | {legs}"
        )

    def _record_status(self, state_info: StateSignal, signal: ControlSignal) -> None:
        import monkey_bot.runtime_debug as runtime_debug

        runtime_debug.procedure_status = self._build_status(state_info, signal)

    def _maybe_log_status(self, state_info: StateSignal, signal: ControlSignal) -> None:
        self._status_frame += 1
        if self._status_frame == 1 or self._status_frame % self._status_interval == 0:
            print(self._build_status(state_info, signal), flush=True)

    def describe_start(self, state_info: StateSignal) -> str | None:
        target = format_grid_point(self.coordinator, self.target_point)
        holding = format_limb_list(
            i for i in self.holding_limb_ids if state_info.active_grips[i]
        )
        foot_dist = format_grid_distance(
            self.coordinator,
            state_info.feet_pos[self.limb_id],
            self.target_point,
        )
        center_dist = format_grid_distance(
            self.coordinator, state_info.center_pos, self.target_point
        )
        move = "MoveCenter then Tracker" if self._move_center_needed(state_info) else "Tracker only (foot in reach)"
        return (
            f"leg {format_limb_id(self.limb_id)} → GP {target} "
            f"(foot {foot_dist} away, center {center_dist} away) | "
            f"bracing legs {holding} | plan: {move}"
        )

    def describe_finish(self, state_info: StateSignal) -> str | None:
        foot_dist = format_grid_distance(
            self.coordinator,
            state_info.feet_pos[self.limb_id],
            self.target_point,
        )
        return f"leg {format_limb_id(self.limb_id)} latched at {foot_dist} from GP"

    def _update_phase(self, state_info: StateSignal, signal: ControlSignal) -> None:
        if self.is_finished(state_info):
            return
        foot_dist = (state_info.feet_pos[self.limb_id] - self.target_point).length
        if self._in_pull_up_phase(state_info):
            center_dist = format_grid_distance(
                self.coordinator, state_info.center_pos, self.target_point
            )
            legs = " | ".join(
                self._format_leg_line(state_info, signal, i)
                for i in range(self.num_legs)
            )
            self._log_phase(
                "pull_up",
                f"AttachCatch: phase 1/2 MoveCenter — shifting body toward {format_grid_point(self.coordinator, self.target_point)} "
                f"(center {center_dist} away) | {legs}",
            )
        elif foot_dist <= self.epsilon:
            self._log_phase(
                "grab",
                f"AttachCatch: foot at GP, requesting grip on leg {format_limb_id(self.limb_id)}",
            )
        else:
            foot_cells = format_grid_distance(
                self.coordinator,
                state_info.feet_pos[self.limb_id],
                self.target_point,
            )
            legs = " | ".join(
                self._format_leg_line(state_info, signal, i)
                for i in range(self.num_legs)
            )
            self._log_phase(
                "reach",
                f"AttachCatch: phase 2/2 Tracker — leg {format_limb_id(self.limb_id)} reaching GP "
                f"({foot_cells} away) | {legs}",
            )

    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        if self.is_finished(state_info):
            return signal

        if self._in_pull_up_phase(state_info):
            signal = self.get_closer.adjust_signal(signal, state_info)
            signal = self._apply_pull_up_support(signal, state_info)
        else:
            signal = self.tracker.adjust_signal(signal, state_info)
            signal = self.body_holder.adjust_signal(signal, state_info)
            signal = self.grabber.adjust_signal(signal, state_info)
            if abs(state_info.body_angular_velocity) > 1.5:
                signal.damp_body = True

        self._update_phase(state_info, signal)
        self._record_status(state_info, signal)
        self._maybe_log_status(state_info, signal)
        return signal

    def is_finished(self, state_info: StateSignal):
        return self.grabber.is_finished(state_info)

class ReleaseAtPoint(Procedure):
    def __init__(self, tolerance, target_point, coordinator: InstanceSimulationConfig):
        super().__init__(coordinator)
        self.tolerance = tolerance
        self.target_point = target_point
        self.released = False

    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        center = state_info.center_pos
        perform_release = (self.target_point - center).length < self.tolerance
        if not perform_release:
            return signal
        for i, _ in enumerate(state_info.feet_pos):
            signal.grip[i] = -1
        self.released = True
        return signal

    def is_finished(self, state_info:StateSignal):
        return self.released

class Tracker(Procedure):
    def __init__(self, limb_id, target_point: Vec2d, coordinator:InstanceSimulationConfig):
        super().__init__(coordinator)
        self.limb_id = limb_id
        self.target_point = target_point
        self.angle_adjuster = SmoothAdjustAngle(limb_id, target_point, coordinator)
        self.length_adjuster = AdjustLength(limb_id, None, coordinator)

    def set_target_point(self, target_point: Vec2d):
        self.target_point = target_point
        self.angle_adjuster.target_point = target_point

    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        goal = (self.target_point - state_info.center_pos).length
        self.length_adjuster.goal_extension = min(goal, self.max_extension * 0.92)
        signal = self.angle_adjuster.adjust_signal(signal, state_info)
        signal = self.length_adjuster.adjust_signal(signal, state_info)
        return signal

    def is_finished(self, state_info:StateSignal):
        return True

class MoveCenter(Procedure):
    """
    Shift body center using gripped-leg extension.

    When the move direction is nearly perpendicular to a gripped leg (pull-up
    geometry), rotation helpers steer that leg as well.
    """

    SPEED_FACTOR = 0.2
    LENGTH_RATE_FACTOR = 0.12
    # Enable angle helpers when |leg/move angle - 90°| is within this band.
    PULL_UP_ANGLE_TOLERANCE = math.radians(35)

    def __init__(self, goal_point, coordinator, ignore_legs=None):
        super().__init__(coordinator)
        self.goal_point = goal_point
        self.ignore_legs = ignore_legs if ignore_legs is not None else []
        self.move_center_speed *= self.SPEED_FACTOR
        self.max_length_adjust_rate = coordinator.extension_speed * self.LENGTH_RATE_FACTOR
        self._angle_rotation_speed = coordinator.angle_rotation_speed

    def _calc_next_point(self, state_info: StateSignal):
        move_vec = self.goal_point - state_info.center_pos
        if move_vec.length == 0:
            return state_info.center_pos
        return state_info.center_pos + move_vec.normalized() * self.dt * self.move_center_speed

    def _needs_angle_helper(self, move_vec: Vec2d, leg_vec: Vec2d) -> bool:
        if move_vec.length < 1e-6 or leg_vec.length < 1e-6:
            return False
        angle = abs(move_vec.get_angle_between(leg_vec))
        return abs(angle - math.pi / 2) <= self.PULL_UP_ANGLE_TOLERANCE

    def _raw_desired_length(
        self,
        aim_center: Vec2d,
        foot_pos: Vec2d,
    ) -> float:
        desired_len = (foot_pos - aim_center).length
        return max(self.min_extension, min(self.max_extension, desired_len))

    def _grip_span_between(self, feet_pos: list[Vec2d], legs: list[int]) -> float:
        if len(legs) < 2:
            return 0.0
        return (feet_pos[legs[0]] - feet_pos[legs[1]]).length

    def _pull_up_geometry(
        self,
        move_vec: Vec2d,
        legs_to_drive: list[int],
        center_pos: Vec2d,
        feet_pos: list[Vec2d],
    ) -> bool:
        if len(legs_to_drive) < 2 or move_vec.length < 1e-6:
            return False
        return all(
            self._needs_angle_helper(move_vec, feet_pos[limb_id] - center_pos)
            for limb_id in legs_to_drive
        )

    def _coordinated_pull_up_lengths(
        self,
        center_pos: Vec2d,
        aim_center: Vec2d,
        feet_pos: list[Vec2d],
        legs_to_drive: list[int],
        foot_dist: float,
    ) -> dict[int, float]:
        """
        Share lengthening across gripped legs during pull-up so one leg does not
        race to max_extension while the other barely moves.
        """
        curr = {
            limb_id: (center_pos - feet_pos[limb_id]).length
            for limb_id in legs_to_drive
        }
        raw = {
            limb_id: (aim_center - feet_pos[limb_id]).length
            for limb_id in legs_to_drive
        }
        curr_sum = sum(curr.values())
        raw_sum = sum(raw.values())
        if raw_sum <= 1e-6:
            return {
                limb_id: self._raw_desired_length(aim_center, feet_pos[limb_id])
                for limb_id in legs_to_drive
            }

        target_sum = max(foot_dist, min(raw_sum, len(legs_to_drive) * self.max_extension))
        grow = max(0.0, target_sum - curr_sum)
        desired = {}
        for limb_id in legs_to_drive:
            share = raw[limb_id] / raw_sum
            length = curr[limb_id] + grow * share
            desired[limb_id] = max(
                self.min_extension,
                min(self.max_extension, length),
            )
        return desired

    def _extension_drive_to_length(self, curr_len: float, desired_len: float) -> float:
        delta_len = desired_len - curr_len
        margin = self.min_extension * 0.05

        if delta_len < 0 and curr_len <= self.min_extension + margin:
            return 0.0
        if delta_len > 0 and curr_len >= self.max_extension - margin:
            return 0.0
        if abs(delta_len) <= 1e-6:
            return 0.0

        rate = min(abs(delta_len) / self.dt, self.max_length_adjust_rate)
        return (1.0 if delta_len > 0 else -1.0) * rate

    def _extension_drive(
        self,
        center_pos: Vec2d,
        aim_center: Vec2d,
        foot_pos: Vec2d,
    ) -> float:
        """Extension rate toward the length needed at aim_center."""
        curr_len = (foot_pos - center_pos).length
        desired_len = self._raw_desired_length(aim_center, foot_pos)
        return self._extension_drive_to_length(curr_len, desired_len)

    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        center_pos = state_info.center_pos
        aim_center = self._calc_next_point(state_info)
        move_vec = aim_center - center_pos
        legs_to_drive = [
            i
            for i, gripped in enumerate(state_info.active_grips)
            if gripped and i not in self.ignore_legs
        ]

        foot_dist = self._grip_span_between(state_info.feet_pos, legs_to_drive)
        pull_up = self._pull_up_geometry(
            move_vec, legs_to_drive, center_pos, state_info.feet_pos
        )
        if pull_up:
            desired_lengths = self._coordinated_pull_up_lengths(
                center_pos,
                aim_center,
                state_info.feet_pos,
                legs_to_drive,
                foot_dist,
            )
        else:
            desired_lengths = {
                limb_id: self._raw_desired_length(
                    aim_center, state_info.feet_pos[limb_id]
                )
                for limb_id in legs_to_drive
            }

        for limb_id in legs_to_drive:
            foot_pos = state_info.feet_pos[limb_id]
            leg_vec = foot_pos - center_pos
            curr_len = leg_vec.length

            if not self._needs_angle_helper(move_vec, leg_vec):
                signal.extension[limb_id] = self._extension_drive_to_length(
                    curr_len, desired_lengths[limb_id]
                )
                continue

            next_foot_vec = foot_pos - aim_center
            curr_foot_vec = leg_vec
            angle_diff = next_foot_vec.get_angle_between(curr_foot_vec)
            if abs(angle_diff) > 1e-6:
                rot_rate = angle_diff / self.dt
                rot_rate = max(
                    -self._angle_rotation_speed,
                    min(self._angle_rotation_speed, rot_rate),
                )
                signal.rotation[limb_id] = rot_rate
            else:
                steer = move_vec.cross(leg_vec)
                if abs(steer) > 1e-6:
                    rot_rate = math.copysign(
                        self._angle_rotation_speed * 0.25,
                        steer,
                    )
                    signal.rotation[limb_id] = rot_rate

            signal.extension[limb_id] = self._extension_drive_to_length(
                curr_len, desired_lengths[limb_id]
            )

        return signal


    def is_finished(self, state_info:StateSignal):
        return (state_info.center_pos - self.goal_point).length < self.epsilon

def free_limb_is_rotating(signal: ControlSignal, state_info: StateSignal, eps: float = 1e-6) -> bool:
    return any(
        abs(rate) > eps and not state_info.active_grips[i]
        for i, rate in enumerate(signal.rotation)
    )


def limb_length(state_info: StateSignal, limb_id: int) -> float:
    return (state_info.center_pos - state_info.feet_pos[limb_id]).length


class BodyHolder(Procedure):
    def __init__(
        self,
        coordinator: InstanceSimulationConfig,
        *holding_limb_ids: int,
        kp: float = 12.0,
        kd: float = 2.5,
        extension_kp: float = 6.0,
        rotation_power: float = 1.0,
        extension_power: float = 1.0,
        smooth: float = 0.75,
        eps: float = 1e-6,
        length_eps: float = 0.3,
    ):
        super().__init__(coordinator)
        self.holding_limb_ids = holding_limb_ids
        self.kp = kp
        self.kd = kd
        self.extension_kp = extension_kp
        self.rotation_power = rotation_power
        self.extension_power = extension_power
        self.smooth = smooth
        self.eps = eps
        self.length_eps = length_eps
        self.ref_angle = None
        self.ref_lengths: dict[int, float] = {}
        self.prev_rotation = 0.0
        self.prev_extension: dict[int, float] = {}

    def _smooth_step(self, previous, target, max_step):
        blended = self.smooth * previous + (1.0 - self.smooth) * target
        return previous + max(-max_step, min(max_step, blended - previous))

    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        active_holders = [
            limb_id for limb_id in self.holding_limb_ids
            if state_info.active_grips[limb_id]
        ]
        if not active_holders or not free_limb_is_rotating(signal, state_info, self.eps):
            self.ref_angle = None
            self.ref_lengths.clear()
            self.prev_rotation = 0.0
            self.prev_extension.clear()
            return signal

        if self.ref_angle is None:
            self.ref_angle = state_info.body_angle

        max_rotation = self.rotation_speed * self.rotation_power
        max_extension = self.extension_speed * self.extension_power
        max_rotation_step = max_rotation * 3.0 * self.dt
        max_extension_step = max_extension * 3.0 * self.dt

        theta_err = normalize_angle(state_info.body_angle - self.ref_angle)
        target_rotation = -self.kp * theta_err - self.kd * state_info.body_angular_velocity
        target_rotation = max(-max_rotation, min(max_rotation, target_rotation))
        smoothed_rotation = self._smooth_step(self.prev_rotation, target_rotation, max_rotation_step)
        self.prev_rotation = smoothed_rotation

        for limb_id in active_holders:
            signal.rotation[limb_id] = smoothed_rotation

            current_length = limb_length(state_info, limb_id)
            if limb_id not in self.ref_lengths:
                self.ref_lengths[limb_id] = current_length

            length_deficit = self.ref_lengths[limb_id] - current_length
            prev_ext = self.prev_extension.get(limb_id, 0.0)
            if length_deficit > self.length_eps:
                target_extension = min(max_extension, self.extension_kp * length_deficit)
            else:
                target_extension = 0.0

            smoothed_extension = self._smooth_step(prev_ext, target_extension, max_extension_step)
            self.prev_extension[limb_id] = smoothed_extension
            signal.extension[limb_id] = signal.extension[limb_id] + smoothed_extension

        return signal

    def is_finished(self, state_info: StateSignal):
        return True


class SmoothBodyHolder(Procedure):
    """
    Gently and smoothly nudge gripped legs to counter body rotation while a free
    leg moves. Uses low gains, a deadband, and hard rate limits so holding legs
    stay in their grooves instead of fighting the physics.
    """

    def __init__(
        self,
        coordinator: InstanceSimulationConfig,
        *holding_limb_ids: int,
        kp: float = 1.5,
        kd: float = 0.5,
        max_correction_rate: float | None = None,
        angle_deadband: float = 0.05,
        velocity_deadband: float = 0.05,
        smooth: float = 0.9,
        eps: float = 1e-6,
    ):
        super().__init__(coordinator)
        self.holding_limb_ids = holding_limb_ids
        self.kp = kp
        self.kd = kd
        self.max_correction_rate = (
            max_correction_rate
            if max_correction_rate is not None
            else coordinator.angle_rotation_speed * 0.35
        )
        self.angle_deadband = angle_deadband
        self.velocity_deadband = velocity_deadband
        self.smooth = smooth
        self.eps = eps
        self.ref_angle = None
        self.prev_rotation = 0.0

    def _smooth_step(self, previous: float, target: float, max_step: float) -> float:
        blended = self.smooth * previous + (1.0 - self.smooth) * target
        return previous + max(-max_step, min(max_step, blended - previous))

    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        if signal.angle_lock:
            for limb_id in self.holding_limb_ids:
                signal.angle_lock[limb_id] = False

        active_holders = [
            limb_id for limb_id in self.holding_limb_ids
            if state_info.active_grips[limb_id]
        ]
        if not active_holders or not free_limb_is_rotating(signal, state_info, self.eps):
            self.ref_angle = None
            self.prev_rotation = self._smooth_step(
                self.prev_rotation, 0.0, self.max_correction_rate * self.dt
            )
            for limb_id in active_holders:
                signal.rotation[limb_id] = self.prev_rotation
            return signal

        if self.ref_angle is None:
            self.ref_angle = state_info.body_angle

        theta_err = normalize_angle(state_info.body_angle - self.ref_angle)
        if (
            abs(theta_err) < self.angle_deadband
            and abs(state_info.body_angular_velocity) < self.velocity_deadband
        ):
            target_rotation = 0.0
        else:
            target_rotation = -self.kp * theta_err - self.kd * state_info.body_angular_velocity
            target_rotation = max(
                -self.max_correction_rate,
                min(self.max_correction_rate, target_rotation),
            )

        max_rate_step = self.max_correction_rate * self.dt
        self.prev_rotation = self._smooth_step(
            self.prev_rotation, target_rotation, max_rate_step
        )

        for limb_id in active_holders:
            signal.rotation[limb_id] = self.prev_rotation

        return signal

    def is_finished(self, state_info: StateSignal):
        return True


class BodyHolderAngleLock(Procedure):
    """
    Keep gripped legs fixed relative to the body using pymunk angle locks,
    instead of driving rotation motors to compensate body spin.
    """

    def __init__(
        self,
        coordinator: InstanceSimulationConfig,
        *holding_limb_ids: int,
        extension_kp: float = 6.0,
        extension_power: float = 1.0,
        smooth: float = 0.75,
        eps: float = 1e-6,
        length_eps: float = 0.3,
    ):
        super().__init__(coordinator)
        self.holding_limb_ids = holding_limb_ids
        self.extension_kp = extension_kp
        self.extension_power = extension_power
        self.smooth = smooth
        self.eps = eps
        self.length_eps = length_eps
        self.ref_lengths: dict[int, float] = {}
        self.prev_extension: dict[int, float] = {}

    def _smooth_step(self, previous, target, max_step):
        blended = self.smooth * previous + (1.0 - self.smooth) * target
        return previous + max(-max_step, min(max_step, blended - previous))

    def adjust_signal(self, signal: ControlSignal, state_info: StateSignal):
        if not signal.angle_lock:
            signal.angle_lock = [False] * self.num_legs

        for limb_id in self.holding_limb_ids:
            signal.angle_lock[limb_id] = False
            signal.rotation[limb_id] = 0.0

        active_holders = [
            limb_id for limb_id in self.holding_limb_ids
            if state_info.active_grips[limb_id]
        ]
        if not active_holders or not free_limb_is_rotating(signal, state_info, self.eps):
            self.ref_lengths.clear()
            self.prev_extension.clear()
            return signal

        max_extension = self.extension_speed * self.extension_power
        max_extension_step = max_extension * 3.0 * self.dt

        for limb_id in active_holders:
            signal.angle_lock[limb_id] = True
            signal.rotation[limb_id] = 0.0

            current_length = limb_length(state_info, limb_id)
            if limb_id not in self.ref_lengths:
                self.ref_lengths[limb_id] = current_length

            length_deficit = self.ref_lengths[limb_id] - current_length
            prev_ext = self.prev_extension.get(limb_id, 0.0)
            if length_deficit > self.length_eps:
                target_extension = min(max_extension, self.extension_kp * length_deficit)
            else:
                target_extension = 0.0

            smoothed_extension = self._smooth_step(prev_ext, target_extension, max_extension_step)
            self.prev_extension[limb_id] = smoothed_extension
            signal.extension[limb_id] = signal.extension[limb_id] + smoothed_extension

        return signal

    def is_finished(self, state_info: StateSignal):
        return True


class Mixed(Procedure):
    def __init__(self, procs:List[Procedure], coordinator, finish_at_any = False):
        super().__init__(coordinator)
        self.procs = procs
        self.finished = [False for _ in procs]
        self.finish_at_any = finish_at_any

    def adjust_signal(self, signal: ControlSignal, state_info):
        for proc in self.procs:
            signal = proc.adjust_signal(signal, state_info)
        return signal

    def is_finished(self, state_info:StateSignal):
        for i, proc in enumerate(self.procs):
            if proc.is_finished(state_info):
                self.finished[i] = True
        if self.finish_at_any:
            return any (self.finished)
        return all(self.finished)



