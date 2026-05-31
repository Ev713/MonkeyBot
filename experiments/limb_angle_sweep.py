import argparse
import math
import os
from pathlib import Path

import numpy as np

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pygame

from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.signals import ControlSignal


def parse_float_list(value):
    return [float(part.strip()) for part in value.split(",") if part.strip()]


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def unwrap_delta(current, start):
    return normalize_angle(current - start)


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


def zero_signal(num_legs):
    return ControlSignal(
        extension=[0.0 for _ in range(num_legs)],
        rotation=[0.0 for _ in range(num_legs)],
        grip=[0 for _ in range(num_legs)],
    )


def read_angles(simulator, limb_id):
    body_angle = simulator.body.angle
    limb_angle = simulator.legs[limb_id][0].angle
    relative_angle = normalize_angle(limb_angle - body_angle)
    return body_angle, limb_angle, relative_angle


def run_limb_angle_trial(
    instance_name,
    instances_folder,
    limb_id,
    degrees,
    speed_deg_per_s,
    out_dir,
    max_seconds,
    settle_frames,
    release_limb,
    cell_size,
    render_scale,
):
    sim_config, robot_config = default_configs(cell_size=cell_size, render_scale=render_scale)
    instance = load_instance(instance_name, instances_folder)
    coordinator = InstanceSimulationConfig(instance, sim_config, robot_config)
    simulator = MonkeyBotSimulator(sim_config)
    simulator.start_simulation(coordinator)

    if release_limb:
        signal = zero_signal(coordinator.num_legs)
        signal.grip[limb_id] = -1
        simulator.apply_signal(signal)
        simulator.step()

    for _ in range(settle_frames):
        simulator.apply_signal(zero_signal(coordinator.num_legs))
        simulator.step()

    target_delta = math.radians(degrees)
    speed = math.radians(abs(speed_deg_per_s))
    signed_rate = math.copysign(speed, target_delta if target_delta != 0 else 1.0)
    max_steps = int(max_seconds * sim_config.fps)

    start_body, start_limb, start_relative = read_angles(simulator, limb_id)
    rows = []
    reached_target = False

    for step in range(max_steps):
        body_angle, limb_angle, relative_angle = read_angles(simulator, limb_id)
        relative_delta = unwrap_delta(relative_angle, start_relative)
        rows.append(
            {
                "t": step * sim_config.dt,
                "body_world": body_angle,
                "limb_world": limb_angle,
                "limb_body": relative_angle,
                "body_world_delta": unwrap_delta(body_angle, start_body),
                "limb_world_delta": unwrap_delta(limb_angle, start_limb),
                "limb_body_delta": relative_delta,
                "command_rate": signed_rate,
            }
        )

        if target_delta >= 0 and relative_delta >= target_delta:
            reached_target = True
            break
        if target_delta < 0 and relative_delta <= target_delta:
            reached_target = True
            break

        signal = zero_signal(coordinator.num_legs)
        signal.rotation[limb_id] = signed_rate
        simulator.apply_signal(signal)
        simulator.step()

    pygame.quit()

    safe_degrees = str(degrees).replace("-", "neg").replace(".", "p")
    safe_speed = str(speed_deg_per_s).replace(".", "p")
    stem = f"{instance_name}_limb{limb_id}_deg{safe_degrees}_speed{safe_speed}"
    csv_path = out_dir / f"{stem}.csv"
    plot_path = out_dir / f"{stem}.png"

    with csv_path.open("w") as f:
        f.write(
            "t,body_world,limb_world,limb_body,body_world_delta,"
            "limb_world_delta,limb_body_delta,command_rate\n"
        )
        for row in rows:
            f.write(
                ",".join(
                    str(row[key])
                    for key in (
                        "t",
                        "body_world",
                        "limb_world",
                        "limb_body",
                        "body_world_delta",
                        "limb_world_delta",
                        "limb_body_delta",
                        "command_rate",
                    )
                )
                + "\n"
            )

    t = np.array([row["t"] for row in rows])
    body_delta = np.degrees([row["body_world_delta"] for row in rows])
    limb_world_delta = np.degrees([row["limb_world_delta"] for row in rows])
    limb_body_delta = np.degrees([row["limb_body_delta"] for row in rows])
    target_line = np.full_like(t, degrees, dtype=float)

    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True, constrained_layout=True)
    axes[0].plot(t, limb_body_delta, label="limb relative to body")
    axes[0].plot(t, target_line, "--", label="requested delta")
    axes[0].set_ylabel("degrees")
    axes[0].set_title(
        f"{instance_name}: limb {limb_id}, target {degrees:g} deg, speed {speed_deg_per_s:g} deg/s"
    )
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(t, body_delta, label="body relative to world")
    axes[1].plot(t, limb_world_delta, label="limb relative to world")
    axes[1].set_xlabel("time (s)")
    axes[1].set_ylabel("degrees from start")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.savefig(plot_path, dpi=160)
    plt.close(fig)

    return {
        "csv": csv_path,
        "plot": plot_path,
        "samples": len(rows),
        "duration": rows[-1]["t"] if rows else 0.0,
        "reached_target": reached_target,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Run dumb constant-rate limb angle simulations and save plots."
    )
    parser.add_argument("--instance", default="TestAngle")
    parser.add_argument("--instances-folder", default="instances")
    parser.add_argument("--limb", type=int, default=0)
    parser.add_argument("--degrees", default="15,-15,30")
    parser.add_argument("--speeds", default="30,60")
    parser.add_argument("--out-dir", default="experiments/results/limb_angle_sweep")
    parser.add_argument("--max-seconds", type=float, default=5.0)
    parser.add_argument("--settle-frames", type=int, default=5)
    parser.add_argument("--cell-size", type=float, default=20)
    parser.add_argument("--render-scale", type=float, default=1)
    parser.add_argument("--keep-gripped", action="store_true")
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for degrees in parse_float_list(args.degrees):
        for speed in parse_float_list(args.speeds):
            result = run_limb_angle_trial(
                instance_name=args.instance,
                instances_folder=args.instances_folder,
                limb_id=args.limb,
                degrees=degrees,
                speed_deg_per_s=speed,
                out_dir=out_dir,
                max_seconds=args.max_seconds,
                settle_frames=args.settle_frames,
                release_limb=not args.keep_gripped,
                cell_size=args.cell_size,
                render_scale=args.render_scale,
            )
            results.append(result)
            status = "reached" if result["reached_target"] else "maxed"
            print(
                f"{status}: {result['plot']} "
                f"({result['samples']} samples, {result['duration']:.3f}s)"
            )

    print(f"wrote {len(results)} plot(s) to {out_dir}")


if __name__ == "__main__":
    main()
