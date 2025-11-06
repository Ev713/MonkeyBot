import math
from dataclasses import dataclass
from typing import List

import numpy as np
import optuna
from pymunk import Vec2d
from shapely.predicates import is_valid


def create_ellipse(p1, p2, L, n_samples = 100):
    # Midpoint and vector between foci
    c = (p1 + p2) / 2
    f = (p2 - p1) / 2
    c_dist = f.length  # distance from center to focus

    # Semi-major axis (a) = L / 2
    # Semi-minor axis (b) = sqrt(a^2 - c^2)
    a = L / 2
    if a <= c_dist:
        raise ValueError("L must be greater than distance between foci")

    b = math.sqrt(a ** 2 - c_dist ** 2)

    # Rotation angle of ellipse
    angle = math.atan2(f.y, f.x)

    # Sample points along ellipse
    t = np.linspace(0, 2 * math.pi, n_samples, endpoint=False)

    # Exclude values within pi/6 (~30°) of 0 or π
    mask = (
            (np.abs((t % (2 * math.pi)) - 0) > math.pi / 6) &
            (np.abs((t % (2 * math.pi)) - math.pi) > math.pi / 6)
    )

    t = t[mask]
    ellipse_points = []
    for theta in t:
        # Point in ellipse local coordinates
        x = a * math.cos(theta)
        y = b * math.sin(theta)
        # Rotate and translate
        px = c.x + x * math.cos(angle) - y * math.sin(angle)
        py = c.y + x * math.sin(angle) + y * math.cos(angle)
        ellipse_points.append(Vec2d(px, py))
    return ellipse_points


@dataclass
class Launcher:
    g:float
    dt:float
    catch_points: List[Vec2d]
    init_jump_leg_points:List[Vec2d]
    max_x: int
    max_y: int
    max_extension: float
    min_extension: float
    trajectory_fps: int = 20
    minimum_runway_length:float = 100
    minimum_catch_window_length:float = 100
    max_take_off_speed:float = 1000

    def approximate_trajectory(self, start_point, takeoff_vector, number_of_points = 9999):
        trajectory = [start_point]
        x_speed = takeoff_vector[0]
        y_speed = takeoff_vector[1]
        curr_point = start_point

        for _ in range(number_of_points):
            delta = (Vec2d(x_speed, 0)+Vec2d(0, y_speed))*self.trajectory_dt
            curr_point = curr_point+delta
            trajectory.append(curr_point)
            y_speed += self.g*self.trajectory_dt
            if curr_point.x >self.max_x or curr_point.y >self.max_y or curr_point.x<0 or curr_point.y<0:
                break
        return trajectory

    @property
    def check_every_n_frames(self):
        return (1/self.dt)/self.trajectory_fps

    @property
    def trajectory_dt(self):
        return 1/self.trajectory_fps

    def runway_is_long_enough(self, trajectory):
        runway_length = len(self.get_runway(trajectory))
        is_valid = runway_length > self.minimum_runway_length/self.check_every_n_frames
        return is_valid

    def catch_window_is_long_enough(self, trajectory):
        catch_window_length = len(self.get_catch_window(trajectory))
        is_valid = catch_window_length> self.minimum_catch_window_length/self.check_every_n_frames
        return is_valid

    def is_runway(self, p):
        return all((p - x).length < self.max_extension for x in self.init_jump_leg_points)

    def get_runway(self, trajectory):
        return [p for p in trajectory if self.is_runway(p)]

    def is_catchable(self, p):
        return all((p - x).length < self.max_extension for x in self.catch_points)

    def get_catch_window(self, trajectory):
        is_valid = lambda x: all([(p - x).length < self.max_extension for p in self.catch_points])
        catch_window = []
        for p in trajectory:
            if is_valid(p):
                catch_window.append(p)
        return catch_window

    def setup_achieves_goal(self, start_point, takeoff_vector):
        achieves_goal = self.starting_point_in_permitted_range(start_point)
        achieves_goal = achieves_goal and self.take_off_is_slow_enough(takeoff_vector)
        trajectory = self.approximate_trajectory(start_point, takeoff_vector)
        achieves_goal = achieves_goal and self.runway_is_long_enough(trajectory)
        achieves_goal = achieves_goal and self.catch_window_is_long_enough(trajectory)
        return achieves_goal

    def starting_point_in_permitted_range(self, start_point):
        for p in self.init_jump_leg_points:
            e = (start_point-p).length
            if e < self.min_extension or e > self.max_extension:
                return False
        return True

    def take_off_is_slow_enough(self, takeoff_vector):
        return takeoff_vector.length < self.max_take_off_speed

    def suggest_trajectory(self):
        pass


class GeometricLauncher(Launcher):

    up_bias:float=0.5

    def suggest_trajectory(self):
        start = self.choose_start()
        if start is None:
            return None, None
        return start, self.find_best_launch_vector(start, self.choose_goal_point())

    def choose_goal_point(self):
        goal_median = sum(self.catch_points, Vec2d(0, 0)) / len(self.catch_points)
        return Vec2d(goal_median.x, goal_median.y * (1 - self.up_bias) + min([p.y for p in self.catch_points]) * self.up_bias)

    def choose_start(self):
        n_samples = 12
        goal = self.choose_goal_point()
        leg_1_pos = self.init_jump_leg_points[0]
        leg_2_pos = self.init_jump_leg_points[0]
        max_ext = self.max_extension
        min_ext = self.min_extension

        # Generate circular samples around both legs
        angles = np.linspace(0, 2 * math.pi, n_samples, endpoint=False)
        circle_points = []
        for leg in [leg_1_pos, leg_2_pos]:
            for r in np.linspace(0.9*min_ext+0.1*max_ext, 0.1*min_ext+0.9*max_ext, 3):  # 3 radii (inner, mid, outer)
                for theta in angles:
                    p = leg + Vec2d(math.cos(theta), math.sin(theta)) * r
                    if self.starting_point_in_permitted_range(p):
                        circle_points.append(p)
        best_point = None
        best_length = -1.0

        for p in circle_points:
            v = self.find_best_launch_vector(p, goal)  # returns Vec2d velocity
            pos = p
            total_length = 0.0

            for _ in range(int(2*max_ext/self.trajectory_dt)):
                pos += v * self.trajectory_dt
                total_length += v.length * self.trajectory_dt

                d1, d2 = (pos - leg_1_pos).length, (pos - leg_2_pos).length
                if d1 < min_ext or d2 < min_ext:
                    # too close, invalid
                    break
                if d1 > max_ext or d2 > max_ext:
                    # exceeded allowed zone → valid trajectory
                    if total_length > best_length:
                        best_length = total_length
                        best_point = p
                    break

        return best_point


    def find_best_launch_vector(self, start, goal):
        dx = goal.x - start.x
        dy = -(goal.y - start.y)
        R = math.hypot(dx, dy)  # R is the straight-line distance, sqrt(dx^2 + dy^2)
        if R == 0:
            return Vec2d(0.0, 0.0)  # Already at the goal

        if dx == 0 and dy < 0:
            return Vec2d(0.0, 0.0)

        T_star = math.sqrt(2 / self.g) * (dx ** 2 + dy ** 2) ** 0.25

        v0x = dx / T_star
        v0y = dy / T_star + 0.5 * self.g * T_star
        return Vec2d(v0x, -v0y)


class OptunaLauncher(Launcher):

    def suggest_trajectory(self):
        n_trials = 4000
        speed_range = (0.0, self.max_take_off_speed)
        angle_range = (-math.pi, math.pi )
        start_bounds = ((max(p.x for p in self.init_jump_leg_points) - self.max_extension,
                        min(p.x for p in self.init_jump_leg_points) + self.max_extension),
                        (max(p.y for p in self.init_jump_leg_points) - self.max_extension,
                         min(p.y for p in self.init_jump_leg_points) + self.max_extension))


        def objective(trial):
            # --- 1. Sample parameters ---
            x = trial.suggest_float("x", *start_bounds[0])
            y = trial.suggest_float("y", *start_bounds[1])
            speed = trial.suggest_float("speed", *speed_range)
            angle = trial.suggest_float("angle", *angle_range)

            start_point = Vec2d(x, y)
            takeoff_vec = Vec2d(math.cos(angle) * speed, math.sin(angle) * speed)

            # --- 2. Check if setup is valid ---
            if not self.setup_is_legal(start_point, takeoff_vec):
                trial.set_user_attr("valid", False)
                return -100.0  # strong penalty for invalid setups

            # --- 3. Evaluate performance ---
            trajectory = self.approximate_trajectory(start_point, takeoff_vec)
            score = len(self.get_catch_window(trajectory)*self.check_every_n_frames)
            if score > 0:
                pass
            cw = self.get_catch_window(trajectory)

            trial.set_user_attr("valid", True)
            trial.set_user_attr("trajectory", trajectory)
            return score

        # --- 4. Run optimization ---
        study = optuna.create_study(direction="maximize")
        study.optimize(objective, n_trials=n_trials, show_progress_bar=True)

        # --- 5. Extract best trial ---
        best = study.best_trial
        best_x = best.params["x"]
        best_y = best.params["y"]
        best_angle = best.params["angle"]
        best_speed = best.params["speed"]

        best_start = Vec2d(best_x, best_y)
        best_vec = Vec2d(math.cos(best_angle) * best_speed, math.sin(best_angle) * best_speed)

        best_traj = self.approximate_trajectory(best_start, best_vec)
        best_score = len(self.get_catch_window(best_traj)*self.check_every_n_frames)

        return {
            "start": best_start,
            "vector": best_vec,
            "score": best_score,
            "study": study,
            "trajectory": best_traj,
        }
