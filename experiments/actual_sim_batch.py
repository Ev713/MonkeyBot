import argparse
import csv
import math
import os
import random
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pygame
from pymunk import Vec2d

from monkey_bot.action import Action
from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.robot_controller import SimplifiedProblemController
from monkey_bot.signals import ControlSignal
from monkey_bot.trajectory_finder import GeometricLauncher


@dataclass
class JumpSetup:
    instance: MonkeyBotProblemInstance
    action: Action
    launch_key: tuple
    launch_start: Vec2d
    launch_vector: Vec2d
    predicted_trajectory: list[Vec2d]
    start_points: list[tuple[int, int]]
    end_points: list[tuple[int, int]]
    arbitrary_center: tuple[int, int]


def default_configs(cell_size=20, render_scale=1, screen_width=1000, screen_height=1000):
    sim_config = SimConfig(
        screen_height=screen_height,
        screen_width=screen_width,
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


def choose_render_scale(instance, args):
    if args.render_scale > 0:
        return args.render_scale
    margin = 80
    usable_width = max(1, args.screen_width - 2 * margin)
    usable_height = max(1, args.screen_height - 2 * margin)
    scale_x = usable_width / (instance.grid_size_x * args.cell_size)
    scale_y = usable_height / (instance.grid_size_y * args.cell_size)
    return max(1, min(args.max_auto_render_scale, scale_x, scale_y))


def hold_window(simulator, coordinator, seconds):
    steps = int(seconds * simulator.sim_config.fps)
    for _ in range(steps):
        if not simulator.run:
            break
        simulator.apply_signal(zero_signal(coordinator.num_legs))
        simulator.step()


def zero_signal(num_legs):
    return ControlSignal(
        extension=[0.0 for _ in range(num_legs)],
        rotation=[0.0 for _ in range(num_legs)],
        grip=[0 for _ in range(num_legs)],
    )


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


def write_csv(path, rows):
    if not rows:
        return
    keys = sorted({key for row in rows for key in row})
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=keys)
        writer.writeheader()
        writer.writerows(rows)


def parse_float_list(value):
    return [float(part.strip()) for part in value.split(",") if part.strip()]


def parse_range(value):
    parts = parse_float_list(value)
    if len(parts) != 2:
        raise ValueError(f"Expected two comma-separated numbers, got {value!r}")
    lo, hi = parts
    if lo > hi:
        lo, hi = hi, lo
    return lo, hi


def summarize_numeric(rows, keys):
    summary = {}
    for key in keys:
        vals = [
            float(row[key])
            for row in rows
            if key in row and row[key] != "" and math.isfinite(float(row[key]))
        ]
        summary[f"{key}_mean"] = mean(vals)
        summary[f"{key}_var"] = variance(vals)
        summary[f"{key}_min"] = min(vals) if vals else 0.0
        summary[f"{key}_max"] = max(vals) if vals else 0.0
        summary[f"{key}_p50"] = percentile(vals, 50)
        summary[f"{key}_p90"] = percentile(vals, 90)
    return summary


def nearest_trajectory_distance(point, trajectory):
    if not trajectory:
        return 0.0
    return min((point - p).length for p in trajectory)


def stage_stats(samples, cell_size):
    if not samples:
        return {
            "count": 0,
            "nearest_mean_grid": 0.0,
            "nearest_max_grid": 0.0,
            "time_aligned_mean_grid": 0.0,
            "time_aligned_max_grid": 0.0,
        }
    nearest = [sample["nearest_dist"] / cell_size for sample in samples]
    aligned = [sample["time_dist"] / cell_size for sample in samples]
    return {
        "count": len(samples),
        "nearest_mean_grid": mean(nearest),
        "nearest_max_grid": max(nearest),
        "time_aligned_mean_grid": mean(aligned),
        "time_aligned_max_grid": max(aligned),
    }


def count_sign_changes(values, threshold=1e-6):
    last_sign = 0
    changes = 0
    for value in values:
        if abs(value) <= threshold:
            continue
        sign = 1 if value > 0 else -1
        if last_sign and sign != last_sign:
            changes += 1
        last_sign = sign
    return changes


def sample_triplet_near(center, radius, grid_size, rng):
    points = set()
    attempts = 0
    while len(points) < 3 and attempts < 500:
        attempts += 1
        x = rng.randint(max(1, center[0] - radius), min(grid_size[0] - 1, center[0] + radius))
        y = rng.randint(max(1, center[1] - radius), min(grid_size[1] - 1, center[1] + radius))
        if math.hypot(x - center[0], y - center[1]) <= radius:
            points.add((x, y))
    return list(points) if len(points) == 3 else None


def find_arbitrary_center(points, leg_extension):
    min_x = min(p[0] for p in points)
    max_x = max(p[0] for p in points)
    min_y = min(p[1] for p in points)
    max_y = max(p[1] for p in points)
    best = None
    best_r = float("inf")
    for x in range(min_x, max_x + 1):
        for y in range(min_y, max_y + 1):
            dists = [math.hypot(x - px, y - py) for px, py in points]
            if all(0 < d <= leg_extension for d in dists):
                r = max(dists)
                if r < best_r:
                    best = (x, y)
                    best_r = r
    return best


def build_possible_jump_setup(run_id, rng, sim_config, robot_config, max_attempts=80):
    for attempt in range(max_attempts):
        grid_size_x = rng.randint(12, 26)
        grid_size_y = rng.randint(12, 26)
        leg_extension = rng.choice([3, 4, 5])
        start_center = (
            rng.randint(4, grid_size_x - 5),
            rng.randint(4, grid_size_y - 5),
        )
        jump_distance = rng.uniform(2.0, robot_config.max_jump_dist * 1.05)
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
            continue
        arbitrary_center = find_arbitrary_center(end_points, leg_extension)
        if arbitrary_center is None:
            continue

        instance = MonkeyBotProblemInstance(
            name=f"actual_jump_{run_id}_{attempt}",
            max_extension=leg_extension,
            grid_size_x=grid_size_x,
            grid_size_y=grid_size_y,
            gripping_points=start_points + end_points,
            goal_point=arbitrary_center,
            init_center=start_center,
            init_feet=start_points,
            allowed_jump_configs=None,
        )
        coordinator = InstanceSimulationConfig(instance, sim_config, robot_config)
        start_screen = [coordinator.grid_to_screen(*p) for p in start_points[:2]]
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
            min_runway_limb_alignment_deg=robot_config.min_runway_limb_alignment_deg,
            min_limb_vector_angle_deg=robot_config.min_limb_vector_angle_deg,
        )
        launch_start, launch_vector = launcher.suggest_trajectory()
        if launch_start is None or launch_vector is None:
            continue
        if not launcher.setup_achieves_goal(launch_start, launch_vector):
            continue
        predicted_trajectory = launcher.approximate_trajectory(launch_start, launch_vector)

        launch_key = (
            start_points[0],
            start_points[1],
            arbitrary_center,
        )
        action = Action(
            name=(
                f"use_TL__from__{start_points[0][0]}_{start_points[0][1]}"
                f"__{start_points[1][0]}_{start_points[1][1]}"
                f"__to__{end_points[0][0]}_{end_points[0][1]}_"
                f"{end_points[1][0]}_{end_points[1][1]}_"
                f"{end_points[2][0]}_{end_points[2][1]}"
            ),
            args=[],
        )
        return JumpSetup(
            instance=instance,
            action=action,
            launch_key=launch_key,
            launch_start=launch_start,
            launch_vector=launch_vector,
            predicted_trajectory=predicted_trajectory,
            start_points=start_points,
            end_points=end_points,
            arbitrary_center=arbitrary_center,
        )
    return None


def run_actual_jump(run_id, rng, args):
    sim_config, robot_config = default_configs(
        args.cell_size,
        1,
        screen_width=args.screen_width,
        screen_height=args.screen_height,
    )
    setup = build_possible_jump_setup(run_id, rng, sim_config, robot_config)
    if setup is None:
        return {
            "type": "jump",
            "run_id": run_id,
            "skipped": 1,
            "success": 0,
            "skip_reason": "no_possible_jump_sampled",
        }

    sim_config.render_scale = choose_render_scale(setup.instance, args)
    print(
        f"jump {run_id}: grid {setup.instance.grid_size_x}x{setup.instance.grid_size_y}, "
        f"render_scale={sim_config.render_scale:.2f}"
    )
    coordinator = InstanceSimulationConfig(setup.instance, sim_config, robot_config)
    simulator = MonkeyBotSimulator(sim_config)
    simulator.start_simulation(coordinator)
    controller = SimplifiedProblemController(coordinator, enable_transition_links=False)
    controller.launch_instructions[controller.hash_jump_params(*setup.launch_key)] = (
        setup.launch_start,
        setup.launch_vector,
    )
    controller.setup_jump_for_simplified_problem_procedure_sequence(setup.action, simulator.get_state())

    steps = 0
    success = False
    failure_reason = ""
    max_steps = int(args.jump_max_seconds * sim_config.fps)
    last_proc_id = controller.proc_id
    trajectory_started = False
    trajectory_start_step = None
    saw_all_detached = False
    saw_catch = False
    stage_samples = {"runway": [], "fly": [], "catch": []}

    try:
        while simulator.run and steps < max_steps:
            state = simulator.get_state()
            if controller.procedures is not None and controller.proc_id >= 3:
                if not trajectory_started:
                    trajectory_started = True
                    trajectory_start_step = steps

                active_grips = sum(state.active_grips)
                if active_grips == 0:
                    saw_all_detached = True
                elif saw_all_detached:
                    saw_catch = True

                if not saw_all_detached:
                    stage = "runway"
                elif not saw_catch:
                    stage = "fly"
                else:
                    stage = "catch"

                elapsed = (steps - trajectory_start_step) * sim_config.dt
                predicted_idx = min(
                    len(setup.predicted_trajectory) - 1,
                    max(0, round(elapsed / 0.05)),
                )
                predicted_point = setup.predicted_trajectory[predicted_idx]
                stage_samples[stage].append(
                    {
                        "nearest_dist": nearest_trajectory_distance(
                            state.center_pos, setup.predicted_trajectory
                        ),
                        "time_dist": (state.center_pos - predicted_point).length,
                    }
                )

            signal = zero_signal(coordinator.num_legs)
            if controller.procedures is None:
                success = True
                break
            proc = controller.procedures[controller.proc_id]
            if proc.is_finished(state):
                controller.proc_id += 1
                if controller.proc_id == len(controller.procedures):
                    controller.procedures = None
                    success = True
                    break
                if controller.proc_id != last_proc_id:
                    print(f"jump {run_id}: procedure {controller.proc_id + 1}/{len(controller.procedures)}")
                    last_proc_id = controller.proc_id
                proc = controller.procedures[controller.proc_id]
            signal = proc.adjust_signal(signal, state)
            simulator.apply_signal(signal)
            simulator.step()
            steps += 1
        if steps >= max_steps and not success:
            failure_reason = "timeout"
        if not simulator.run and not success:
            failure_reason = "simulator_stopped"
    except Exception as exc:
        failure_reason = f"{type(exc).__name__}: {exc}"
    finally:
        final_state = simulator.get_state()
        hold_window(simulator, coordinator, args.post_run_hold_seconds)
        pygame.quit()

    end_screen = [coordinator.grid_to_screen(*p) for p in setup.end_points]
    foot_to_end_dists = []
    for foot in final_state.feet_pos:
        foot_to_end_dists.append(min((foot - p).length for p in end_screen) / args.cell_size)
    runway_stats = stage_stats(stage_samples["runway"], args.cell_size)
    fly_stats = stage_stats(stage_samples["fly"], args.cell_size)
    catch_stats = stage_stats(stage_samples["catch"], args.cell_size)

    row = {
        "type": "jump",
        "run_id": run_id,
        "skipped": 0,
        "success": int(success),
        "failure_reason": failure_reason,
        "instance": setup.instance.name,
        "grid_size_x": setup.instance.grid_size_x,
        "grid_size_y": setup.instance.grid_size_y,
        "leg_extension": setup.instance.max_extension,
        "start_points": repr(setup.start_points),
        "end_points": repr(setup.end_points),
        "duration_s": steps * sim_config.dt,
        "steps": steps,
        "final_active_grips": sum(final_state.active_grips),
        "max_foot_to_end_dist_grid": max(foot_to_end_dists),
        "mean_foot_to_end_dist_grid": mean(foot_to_end_dists),
        "takeoff_speed_grid_s": setup.launch_vector.length / args.cell_size,
        "trajectory_started": int(trajectory_started),
        "saw_all_detached": int(saw_all_detached),
        "saw_catch": int(saw_catch),
    }
    for stage, stats in (("runway", runway_stats), ("fly", fly_stats), ("catch", catch_stats)):
        for key, value in stats.items():
            row[f"{stage}_{key}"] = value
    return row


def make_angle_instance(run_id, rng):
    init_feet = [(1, 1), (3, 1), (1, 3)]
    grid_size_x = rng.randint(8, 18)
    grid_size_y = rng.randint(8, 18)
    leg_extension = rng.choice([3, 4])
    return MonkeyBotProblemInstance(
        name=f"actual_angle_{run_id}",
        max_extension=leg_extension,
        grid_size_x=grid_size_x,
        grid_size_y=grid_size_y,
        gripping_points=init_feet.copy(),
        goal_point=(grid_size_x - 2, grid_size_y - 2),
        init_center=(2, 2),
        init_feet=init_feet,
        allowed_jump_configs=None,
    )


def read_angles(simulator, limb_id):
    body_angle = simulator.body.angle
    limb_angle = simulator.legs[limb_id][0].angle
    relative_angle = normalize_angle(limb_angle - body_angle)
    return body_angle, limb_angle, relative_angle


def run_actual_angle(run_id, rng, args):
    sim_config, robot_config = default_configs(
        args.cell_size,
        1,
        screen_width=args.screen_width,
        screen_height=args.screen_height,
    )
    instance = make_angle_instance(run_id, rng)
    sim_config.render_scale = choose_render_scale(instance, args)
    print(
        f"angle {run_id}: grid {instance.grid_size_x}x{instance.grid_size_y}, "
        f"render_scale={sim_config.render_scale:.2f}"
    )
    coordinator = InstanceSimulationConfig(instance, sim_config, robot_config)
    simulator = MonkeyBotSimulator(sim_config)
    simulator.start_simulation(coordinator)

    limb_id = rng.randrange(coordinator.num_legs)
    target_magnitude = rng.uniform(*args.angle_target_abs_range)
    target_degrees = target_magnitude * rng.choice([-1, 1])
    speed_degrees = rng.uniform(*args.angle_speed_range)
    target_delta = math.radians(target_degrees)
    command_rate = math.copysign(math.radians(speed_degrees), target_delta)

    release = zero_signal(coordinator.num_legs)
    release.grip[limb_id] = -1
    simulator.apply_signal(release)
    simulator.step()

    start_body, start_limb, start_relative = read_angles(simulator, limb_id)
    prev_body, prev_limb, prev_relative = start_body, start_limb, start_relative
    unwrapped_body_delta = 0.0
    unwrapped_limb_delta = 0.0
    unwrapped_relative_delta = 0.0
    abs_errors = []
    signed_errors = []
    body_rates = []
    limb_world_rates = []
    relative_rates = []
    reached = False
    overshoot = 0.0
    steps = 0
    max_steps = int(args.angle_max_seconds * sim_config.fps)

    try:
        while simulator.run and steps < max_steps:
            signal = zero_signal(coordinator.num_legs)
            signal.rotation[limb_id] = command_rate
            simulator.apply_signal(signal)
            simulator.step()
            steps += 1

            body, limb, relative = read_angles(simulator, limb_id)
            body_step_delta = normalize_angle(body - prev_body)
            limb_step_delta = normalize_angle(limb - prev_limb)
            relative_step_delta = normalize_angle(relative - prev_relative)
            unwrapped_body_delta += -body_step_delta
            unwrapped_limb_delta += -limb_step_delta
            unwrapped_relative_delta += -relative_step_delta
            actual_relative_rate = -relative_step_delta / sim_config.dt
            actual_limb_world_rate = -limb_step_delta / sim_config.dt
            actual_body_rate = -body_step_delta / sim_config.dt
            error = command_rate - actual_relative_rate

            abs_errors.append(abs(error))
            signed_errors.append(error)
            relative_rates.append(actual_relative_rate)
            limb_world_rates.append(actual_limb_world_rate)
            body_rates.append(actual_body_rate)

            if target_delta >= 0 and unwrapped_relative_delta >= target_delta:
                reached = True
                overshoot = unwrapped_relative_delta - target_delta
                break
            if target_delta < 0 and unwrapped_relative_delta <= target_delta:
                reached = True
                overshoot = target_delta - unwrapped_relative_delta
                break
            prev_body, prev_limb, prev_relative = body, limb, relative
    finally:
        hold_window(simulator, coordinator, args.post_run_hold_seconds)
        pygame.quit()
    abs_relative_rates = [abs(v) for v in relative_rates]
    speed_accels = [
        (abs_relative_rates[i] - abs_relative_rates[i - 1]) / sim_config.dt
        for i in range(1, len(abs_relative_rates))
    ]
    jerk = [
        (speed_accels[i] - speed_accels[i - 1]) / sim_config.dt
        for i in range(1, len(speed_accels))
    ]

    return {
        "type": "angle",
        "run_id": run_id,
        "success": int(reached),
        "instance": instance.name,
        "limb_id": limb_id,
        "target_degrees": target_degrees,
        "speed_degrees": speed_degrees,
        "duration_s": steps * sim_config.dt,
        "steps": steps,
        "final_limb_body_delta_deg": math.degrees(unwrapped_relative_delta),
        "final_limb_world_delta_deg": math.degrees(unwrapped_limb_delta),
        "final_body_world_delta_deg": math.degrees(unwrapped_body_delta),
        "overshoot_deg": math.degrees(overshoot),
        "abs_error_mean_deg_s": math.degrees(mean(abs_errors)),
        "abs_error_var": variance(abs_errors),
        "abs_error_p90_deg_s": math.degrees(percentile(abs_errors, 90)),
        "abs_error_max_deg_s": math.degrees(max(abs_errors) if abs_errors else 0.0),
        "signed_error_mean_deg_s": math.degrees(mean(signed_errors)),
        "signed_error_var": variance(signed_errors),
        "actual_relative_rate_mean_deg_s": math.degrees(mean(relative_rates)),
        "actual_limb_world_rate_mean_deg_s": math.degrees(mean(limb_world_rates)),
        "actual_body_rate_mean_deg_s": math.degrees(mean(body_rates)),
        "abs_speed_mean_deg_s": math.degrees(mean(abs_relative_rates)),
        "abs_speed_var": variance(abs_relative_rates),
        "abs_speed_min_deg_s": math.degrees(min(abs_relative_rates) if abs_relative_rates else 0.0),
        "abs_speed_max_deg_s": math.degrees(max(abs_relative_rates) if abs_relative_rates else 0.0),
        "speed_accel_std_deg_s2": math.degrees(float(np.std(speed_accels)) if speed_accels else 0.0),
        "speed_accel_sign_changes": count_sign_changes(speed_accels, math.radians(1)),
        "jerk_std_deg_s3": math.degrees(float(np.std(jerk)) if jerk else 0.0),
    }


def main():
    parser = argparse.ArgumentParser(description="Run visible alternating actual jump/angle simulations.")
    parser.add_argument("--pairs", type=int, default=50)
    parser.add_argument("--seed", type=int, default=713)
    parser.add_argument("--out-dir", default="experiments/results/actual_sim_batch")
    parser.add_argument("--cell-size", type=float, default=20)
    parser.add_argument("--render-scale", type=float, default=0)
    parser.add_argument("--max-auto-render-scale", type=float, default=8)
    parser.add_argument("--screen-width", type=int, default=1000)
    parser.add_argument("--screen-height", type=int, default=1000)
    parser.add_argument(
        "--angle-target-abs-range",
        default="45,120",
        help="Uniform range for absolute target angle in degrees; sign is random.",
    )
    parser.add_argument(
        "--angle-speed-range",
        default="20,60",
        help="Uniform motor command speed range in degrees/second.",
    )
    parser.add_argument("--angle-max-seconds", type=float, default=5)
    parser.add_argument("--jump-max-seconds", type=float, default=25)
    parser.add_argument("--post-run-hold-seconds", type=float, default=0.5)
    args = parser.parse_args()
    args.angle_target_abs_range = parse_range(args.angle_target_abs_range)
    args.angle_speed_range = parse_range(args.angle_speed_range)

    rng = random.Random(args.seed)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    angle_rows = []
    jump_rows = []
    for run_id in range(args.pairs):
        print(f"pair {run_id + 1}/{args.pairs}: jump")
        jump_rows.append(run_actual_jump(run_id, rng, args))
        write_csv(out_dir / "jump_runs.csv", jump_rows)

        print(f"pair {run_id + 1}/{args.pairs}: angle")
        angle_rows.append(run_actual_angle(run_id, rng, args))
        write_csv(out_dir / "angle_runs.csv", angle_rows)

    angle_summary = {
        "runs": len(angle_rows),
        "success_rate": mean(row["success"] for row in angle_rows),
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
                "abs_speed_var",
                "speed_accel_std_deg_s2",
                "speed_accel_sign_changes",
                "jerk_std_deg_s3",
            ],
        )
    )

    attempted_jumps = [row for row in jump_rows if row.get("skipped") != 1]
    jump_summary = {
        "runs": len(jump_rows),
        "attempted_runs": len(attempted_jumps),
        "skipped_runs": len(jump_rows) - len(attempted_jumps),
        "simulation_success_rate_attempted": mean(row["success"] for row in attempted_jumps),
        "simulation_success_rate_all": mean(row["success"] for row in jump_rows),
    }
    jump_summary.update(
        summarize_numeric(
            attempted_jumps,
            [
                "duration_s",
                "max_foot_to_end_dist_grid",
                "mean_foot_to_end_dist_grid",
                "takeoff_speed_grid_s",
                "final_active_grips",
                "runway_nearest_mean_grid",
                "runway_time_aligned_mean_grid",
                "fly_nearest_mean_grid",
                "fly_time_aligned_mean_grid",
                "catch_nearest_mean_grid",
                "catch_time_aligned_mean_grid",
                "runway_count",
                "fly_count",
                "catch_count",
            ],
        )
    )

    write_csv(out_dir / "angle_summary.csv", [angle_summary])
    write_csv(out_dir / "jump_summary.csv", [jump_summary])
    print(f"wrote actual simulation metrics to {out_dir}")
    print(f"angle success rate: {angle_summary['success_rate']:.3f}")
    print(
        "jump simulation success rate: "
        f"{jump_summary['simulation_success_rate_attempted']:.3f} attempted, "
        f"{jump_summary['simulation_success_rate_all']:.3f} overall"
    )


if __name__ == "__main__":
    main()
