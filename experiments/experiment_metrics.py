import argparse
import csv
import math
import os
import random
from dataclasses import dataclass
from pathlib import Path

import numpy as np

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame
from pymunk import Vec2d

from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.signals import ControlSignal
from monkey_bot.trajectory_finder import GeometricLauncher


class FastClock:
    def tick(self, fps):
        return None


@dataclass
class AngleTrialConfig:
    run_id: int
    limb_id: int
    target_degrees: float
    speed_degrees: float
    grid_size_x: int
    grid_size_y: int
    leg_extension: int


def default_configs(cell_size=20, render_scale=1):
    sim_config = SimConfig(
        screen_height=1000,
        screen_width=1000,
        fps=120,
        gravity=1,
        cell_size=cell_size,
        render_scale=render_scale,
    )
    robot_config = RobotConfig(
        epsilon=0.1,
        extension_speed=4,
        rotation_speed=4,
        move_center_speed=4,
        body_mass=5,
        body_radius=0.4,
        foot_mass=0.2,
        leg_mass=0.2,
        simplified_problem=True,
        leg_spring_stiffness=500,
        leg_spring_damping=1.5,
        min_extension=0.2,
        max_takeoff_speed=20,
        max_jump_dist=15,
        prune_short_jumps=True,
        prune_in_clique_jumps=True,
        prune_similar_jumps=True,
    )
    return sim_config, robot_config


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def angle_delta(current, start):
    return normalize_angle(current - start)


def mean(values):
    values = list(values)
    return float(np.mean(values)) if values else 0.0


def variance(values):
    values = list(values)
    return float(np.var(values)) if values else 0.0


def percentile(values, q):
    values = list(values)
    return float(np.percentile(values, q)) if values else 0.0


def zero_signal(num_legs):
    return ControlSignal(
        extension=[0.0 for _ in range(num_legs)],
        rotation=[0.0 for _ in range(num_legs)],
        grip=[0 for _ in range(num_legs)],
    )


def make_angle_instance(config):
    init_feet = [(1, 1), (3, 1), (1, 3)]
    return MonkeyBotProblemInstance(
        name=f"angle_random_{config.run_id}",
        max_extension=config.leg_extension,
        grid_size_x=config.grid_size_x,
        grid_size_y=config.grid_size_y,
        gripping_points=init_feet.copy(),
        goal_point=(config.grid_size_x - 2, config.grid_size_y - 2),
        init_center=(2, 2),
        init_feet=init_feet,
        allowed_jump_configs=None,
    )


def read_angles(simulator, limb_id):
    body_angle = simulator.body.angle
    limb_angle = simulator.legs[limb_id][0].angle
    relative_angle = normalize_angle(limb_angle - body_angle)
    return body_angle, limb_angle, relative_angle


def run_angle_trial(config, cell_size, max_seconds, settle_frames):
    sim_config, robot_config = default_configs(cell_size=cell_size)
    instance = make_angle_instance(config)
    coordinator = InstanceSimulationConfig(instance, sim_config, robot_config)
    simulator = MonkeyBotSimulator(sim_config)
    simulator.start_simulation(coordinator)
    simulator.clock = FastClock()

    release = zero_signal(coordinator.num_legs)
    release.grip[config.limb_id] = -1
    simulator.apply_signal(release)
    simulator.step()

    for _ in range(settle_frames):
        simulator.apply_signal(zero_signal(coordinator.num_legs))
        simulator.step()

    target_delta = math.radians(config.target_degrees)
    command_rate = math.copysign(
        math.radians(abs(config.speed_degrees)),
        target_delta if target_delta != 0 else 1.0,
    )
    start_body, start_limb, start_relative = read_angles(simulator, config.limb_id)
    prev_body, prev_limb, prev_relative = start_body, start_limb, start_relative

    abs_errors = []
    signed_errors = []
    actual_relative_rates = []
    actual_limb_world_rates = []
    actual_body_world_rates = []
    command_rates = []
    reached_target = False
    overshoot_rad = 0.0
    max_steps = int(max_seconds * sim_config.fps)

    for step in range(max_steps):
        signal = zero_signal(coordinator.num_legs)
        signal.rotation[config.limb_id] = command_rate
        simulator.apply_signal(signal)
        simulator.step()

        body, limb, relative = read_angles(simulator, config.limb_id)
        # Negative sign matches RotationMotor.actual_rate/body_rate convention.
        actual_relative_rate = -normalize_angle(relative - prev_relative) / sim_config.dt
        actual_limb_world_rate = -normalize_angle(limb - prev_limb) / sim_config.dt
        actual_body_world_rate = -normalize_angle(body - prev_body) / sim_config.dt

        err = command_rate - actual_relative_rate
        signed_errors.append(err)
        abs_errors.append(abs(err))
        actual_relative_rates.append(actual_relative_rate)
        actual_limb_world_rates.append(actual_limb_world_rate)
        actual_body_world_rates.append(actual_body_world_rate)
        command_rates.append(command_rate)

        motor_relative_delta = -angle_delta(relative, start_relative)
        if target_delta >= 0 and motor_relative_delta >= target_delta:
            reached_target = True
            overshoot_rad = motor_relative_delta - target_delta
            break
        if target_delta < 0 and motor_relative_delta <= target_delta:
            reached_target = True
            overshoot_rad = target_delta - motor_relative_delta
            break

        prev_body, prev_limb, prev_relative = body, limb, relative

    final_body, final_limb, final_relative = read_angles(simulator, config.limb_id)
    pygame.quit()

    duration = (step + 1) * sim_config.dt if max_steps else 0.0
    return {
        "run_id": config.run_id,
        "instance": instance.name,
        "limb_id": config.limb_id,
        "target_degrees": config.target_degrees,
        "speed_degrees": config.speed_degrees,
        "grid_size_x": config.grid_size_x,
        "grid_size_y": config.grid_size_y,
        "leg_extension": config.leg_extension,
        "samples": len(abs_errors),
        "duration_s": duration,
        "reached_target": int(reached_target),
        "final_limb_body_delta_deg": math.degrees(-angle_delta(final_relative, start_relative)),
        "final_limb_world_delta_deg": math.degrees(-angle_delta(final_limb, start_limb)),
        "final_body_world_delta_deg": math.degrees(-angle_delta(final_body, start_body)),
        "overshoot_deg": math.degrees(overshoot_rad),
        "command_rate_mean_deg_s": math.degrees(mean(command_rates)),
        "actual_relative_rate_mean_deg_s": math.degrees(mean(actual_relative_rates)),
        "actual_relative_rate_var": variance(actual_relative_rates),
        "actual_limb_world_rate_mean_deg_s": math.degrees(mean(actual_limb_world_rates)),
        "actual_body_world_rate_mean_deg_s": math.degrees(mean(actual_body_world_rates)),
        "signed_error_mean_deg_s": math.degrees(mean(signed_errors)),
        "signed_error_var": variance(signed_errors),
        "abs_error_mean_deg_s": math.degrees(mean(abs_errors)),
        "abs_error_median_deg_s": math.degrees(percentile(abs_errors, 50)),
        "abs_error_p90_deg_s": math.degrees(percentile(abs_errors, 90)),
        "abs_error_max_deg_s": math.degrees(max(abs_errors) if abs_errors else 0.0),
    }


def random_angle_config(run_id, rng):
    return AngleTrialConfig(
        run_id=run_id,
        limb_id=rng.randrange(3),
        target_degrees=rng.choice([-45, -30, -20, -15, 15, 20, 30, 45]),
        speed_degrees=rng.choice([15, 30, 45, 60, 90]),
        grid_size_x=rng.randint(8, 20),
        grid_size_y=rng.randint(8, 20),
        leg_extension=rng.choice([3, 4]),
    )


def sample_triplet_near(center, radius, grid_size, rng):
    points = set()
    attempts = 0
    while len(points) < 3 and attempts < 200:
        attempts += 1
        x = rng.randint(max(1, center[0] - radius), min(grid_size[0] - 1, center[0] + radius))
        y = rng.randint(max(1, center[1] - radius), min(grid_size[1] - 1, center[1] + radius))
        if math.hypot(x - center[0], y - center[1]) <= radius:
            points.add((x, y))
    return list(points) if len(points) == 3 else None


def run_jump_trial(run_id, rng, cell_size):
    sim_config, robot_config = default_configs(cell_size=cell_size)
    grid_size_x = rng.randint(12, 28)
    grid_size_y = rng.randint(12, 28)
    leg_extension = rng.choice([3, 4, 5])

    start_center = (rng.randint(4, grid_size_x - 5), rng.randint(4, grid_size_y - 5))
    jump_distance = rng.uniform(2.0, robot_config.max_jump_dist * 1.15)
    jump_angle = rng.uniform(-math.pi, math.pi)
    end_center = (
        round(start_center[0] + math.cos(jump_angle) * jump_distance),
        round(start_center[1] + math.sin(jump_angle) * jump_distance),
    )
    end_center = (
        min(max(3, end_center[0]), grid_size_x - 3),
        min(max(3, end_center[1]), grid_size_y - 3),
    )

    start_points = sample_triplet_near(start_center, leg_extension, (grid_size_x, grid_size_y), rng)
    end_points = sample_triplet_near(end_center, leg_extension, (grid_size_x, grid_size_y), rng)
    if start_points is None or end_points is None:
        return {
            "run_id": run_id,
            "possible": 0,
            "success": 0,
            "skip_reason": "could_not_sample_triplets",
        }

    instance = MonkeyBotProblemInstance(
        name=f"jump_random_{run_id}",
        max_extension=leg_extension,
        grid_size_x=grid_size_x,
        grid_size_y=grid_size_y,
        gripping_points=start_points + end_points,
        goal_point=end_center,
        init_center=start_center,
        init_feet=start_points,
        allowed_jump_configs=None,
    )
    coordinator = InstanceSimulationConfig(instance, sim_config, robot_config)
    start_screen = [coordinator.grid_to_screen(*p) for p in start_points]
    end_screen = [coordinator.grid_to_screen(*p) for p in end_points]

    launcher = GeometricLauncher(
        coordinator.gravity,
        coordinator.dt,
        end_screen,
        start_screen,
        coordinator.sim_config.screen_width,
        coordinator.sim_config.screen_height,
        max_extension=coordinator.max_extension,
        min_extension=coordinator.min_extension,
        max_take_off_speed=coordinator.max_jump_speed,
        max_average_distance=coordinator.max_jump_dist(),
    )

    average_start = sum(start_screen, Vec2d(0, 0)) / len(start_screen)
    average_end = sum(end_screen, Vec2d(0, 0)) / len(end_screen)
    start_point, takeoff_vector = launcher.suggest_trajectory()

    base = {
        "run_id": run_id,
        "instance": instance.name,
        "grid_size_x": grid_size_x,
        "grid_size_y": grid_size_y,
        "leg_extension": leg_extension,
        "start_points": repr(start_points),
        "end_points": repr(end_points),
        "avg_jump_distance_grid": (average_start - average_end).length / cell_size,
        "possible": 0,
        "success": 0,
        "skip_reason": "",
    }

    if start_point is None or takeoff_vector is None:
        base["skip_reason"] = "geometric_launcher_none"
        return base

    trajectory = launcher.approximate_trajectory(start_point, takeoff_vector)
    runway = launcher.get_runway(trajectory)
    catch_window = launcher.get_catch_window(trajectory)
    possible = launcher.setup_achieves_goal(start_point, takeoff_vector)
    success = possible and len(catch_window) > 0

    base.update(
        {
            "possible": int(possible),
            "success": int(success),
            "takeoff_speed_px_s": takeoff_vector.length,
            "takeoff_speed_grid_s": takeoff_vector.length / cell_size,
            "takeoff_x_grid_s": takeoff_vector.x / cell_size,
            "takeoff_y_grid_s": takeoff_vector.y / cell_size,
            "trajectory_points": len(trajectory),
            "runway_points": len(runway),
            "catch_window_points": len(catch_window),
            "runway_time_s": len(runway) * launcher.trajectory_dt,
            "catch_window_time_s": len(catch_window) * launcher.trajectory_dt,
            "start_body_to_avg_start_grid": (start_point - average_start).length / cell_size,
            "start_body_to_avg_end_grid": (start_point - average_end).length / cell_size,
        }
    )
    if not possible:
        base["skip_reason"] = "setup_does_not_achieve_goal"
    return base


def write_csv(path, rows):
    if not rows:
        return
    keys = sorted({key for row in rows for key in row})
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def summarize_numeric(rows, keys):
    summary = {}
    for key in keys:
        vals = [float(row[key]) for row in rows if key in row and row[key] != ""]
        summary[f"{key}_mean"] = mean(vals)
        summary[f"{key}_var"] = variance(vals)
        summary[f"{key}_min"] = min(vals) if vals else 0.0
        summary[f"{key}_max"] = max(vals) if vals else 0.0
        summary[f"{key}_p50"] = percentile(vals, 50)
        summary[f"{key}_p90"] = percentile(vals, 90)
    return summary


def main():
    parser = argparse.ArgumentParser(description="Run random angle and jump metric batches.")
    parser.add_argument("--angle-runs", type=int, default=30)
    parser.add_argument("--jump-runs", type=int, default=30)
    parser.add_argument("--seed", type=int, default=713)
    parser.add_argument("--out-dir", default="experiments/results/experiment_metrics")
    parser.add_argument("--cell-size", type=float, default=20)
    parser.add_argument("--angle-max-seconds", type=float, default=4)
    args = parser.parse_args()

    rng = random.Random(args.seed)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    angle_rows = []
    for run_id in range(args.angle_runs):
        config = random_angle_config(run_id, rng)
        angle_rows.append(
            run_angle_trial(
                config,
                cell_size=args.cell_size,
                max_seconds=args.angle_max_seconds,
                settle_frames=5,
            )
        )
        print(f"angle {run_id + 1}/{args.angle_runs}")

    jump_rows = []
    for run_id in range(args.jump_runs):
        jump_rows.append(run_jump_trial(run_id, rng, cell_size=args.cell_size))
        print(f"jump {run_id + 1}/{args.jump_runs}")

    write_csv(out_dir / "angle_runs.csv", angle_rows)
    write_csv(out_dir / "jump_runs.csv", jump_rows)

    angle_summary = {
        "runs": len(angle_rows),
        "target_reached_rate": mean(row["reached_target"] for row in angle_rows),
    }
    angle_summary.update(
        summarize_numeric(
            angle_rows,
            [
                "abs_error_mean_deg_s",
                "abs_error_p90_deg_s",
                "abs_error_max_deg_s",
                "signed_error_mean_deg_s",
                "actual_relative_rate_mean_deg_s",
                "final_limb_body_delta_deg",
                "overshoot_deg",
                "duration_s",
            ],
        )
    )

    possible_jumps = [row for row in jump_rows if row.get("possible") == 1]
    jump_summary = {
        "runs": len(jump_rows),
        "possible_rate": mean(row.get("possible", 0) for row in jump_rows),
        "success_rate_all_runs": mean(row.get("success", 0) for row in jump_rows),
        "success_rate_possible_only": mean(row.get("success", 0) for row in possible_jumps),
        "possible_count": len(possible_jumps),
    }
    jump_summary.update(
        summarize_numeric(
            jump_rows,
            [
                "avg_jump_distance_grid",
                "takeoff_speed_grid_s",
                "runway_time_s",
                "catch_window_time_s",
                "trajectory_points",
            ],
        )
    )

    write_csv(out_dir / "angle_summary.csv", [angle_summary])
    write_csv(out_dir / "jump_summary.csv", [jump_summary])

    print(f"wrote metrics to {out_dir}")
    print(f"angle target reached rate: {angle_summary['target_reached_rate']:.3f}")
    print(
        "jump possible rate: "
        f"{jump_summary['possible_rate']:.3f}, "
        "success among possible: "
        f"{jump_summary['success_rate_possible_only']:.3f}"
    )


if __name__ == "__main__":
    main()
