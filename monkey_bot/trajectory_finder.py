import math
from dataclasses import dataclass
from typing import List

import optuna
from pymunk import Vec2d


@dataclass
class TrajectoryValidator:
    g:float
    dt:float
    catch_points: List[Vec2d]
    max_extension:float
    init_jump_leg_points:List[Vec2d]
    check_every_n_frames: int = 20
    minimum_runway_length:float = 100
    minimum_catch_window_length:float = 100
    max_take_off_speed:float = 1000

    @property
    def trajectory_dt(self):
        return self.dt*self.check_every_n_frames

    def take_off_is_valid(self, trajectory):
        runway_length = len(self.get_runway(trajectory))*self.check_every_n_frames
        #print(f"Runway length: {runway_length}/{self.minimum_runway_length}")
        return  runway_length> self.minimum_runway_length

    def catch_window_is_valid(self, trajectory):
        catch_window_length = len(self.get_catch_window(trajectory))*self.check_every_n_frames
        #print(f"Required catch window: {catch_window_length}/{self.minimum_catch_window_length}")
        return catch_window_length> self.minimum_catch_window_length

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

    def approximate_trajectory(self, start_point, takeoff_vector, number_of_points = 100):
        pass

    def setup_achieves_goal(self, start_point, takeoff_vector):
        trajectory = self.approximate_trajectory(start_point, takeoff_vector)
        return self.take_off_is_valid(trajectory) and self.catch_window_is_valid(trajectory)

    def starting_point_is_valid(self, start_point):
        return all([(start_point-p).length < self.max_extension for p in self.init_jump_leg_points])

    def takeoff_vector_is_valid(self, takeoff_vector):
        return takeoff_vector.length < self.max_take_off_speed

    def setup_is_valid(self, starting_point, takeoff_vector):
        return self.starting_point_is_valid(starting_point) and self.takeoff_vector_is_valid(takeoff_vector)

    def suggest_trajectory_using_medians_and_ballistics(self):
        median = sum(self.catch_points, Vec2d(0, 0)) / len(self.catch_points)
        start = sum(self.init_jump_leg_points, Vec2d(0, 0)) / len(self.init_jump_leg_points)
        goal = Vec2d(median.x, median.y*0.5+min([p.y for p in self.catch_points])*0.5)

        dx = goal.x - start.x
        dy = -(goal.y - start.y)
        R = math.hypot(dx, dy)  # R is the straight-line distance, sqrt(dx^2 + dy^2)
        if R == 0:
            return Vec2d(0.0, 0.0)  # Already at the goal

        T_star = math.sqrt((2.0 * (R + dy)) / self.g)

        v0x = dx / T_star
        v0y = dy / T_star + 0.5 * self.g * T_star

        return start, Vec2d(v0x, -v0y)

    def optuna_suggest_trajectory(self, n_trials=4000, ):
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
            if not self.setup_is_valid(start_point, takeoff_vec):
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


class JumpTrajectoryValidator(TrajectoryValidator):
    def approximate_trajectory(self, start_point, takeoff_vector, number_of_points = 1000):
        trajectory = [start_point]
        x_speed = takeoff_vector[0]
        y_speed = takeoff_vector[1]
        curr_point = start_point

        for _ in range(number_of_points):
            delta = (Vec2d(x_speed, 0)+Vec2d(0, y_speed))*self.trajectory_dt
            curr_point = curr_point+delta
            trajectory.append(curr_point)
            y_speed += self.g*self.trajectory_dt
        return trajectory

