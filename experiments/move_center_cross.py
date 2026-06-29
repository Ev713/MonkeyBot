#!/usr/bin/env python3
"""
MoveCenter-only cross test: two gripped legs, body traces a cross via extension only.

Legs 0 and 2 stay gripped. Leg 1 is released. Each waypoint is driven only by
MoveCenter (gripped-leg extension commands — no rotation, damp, or stabilizer).

  python experiments/move_center_cross.py
"""

from __future__ import annotations

import argparse
import csv
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

if "--headless" in sys.argv:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import pygame

from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.procedure import MoveCenter, limb_length
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.signals import empty_control_signal


def default_configs(cell_size=40, render_scale=1):
    sim_config = SimConfig(
        screen_height=800,
        screen_width=800,
        fps=120,
        gravity=1,
        cell_size=cell_size,
        render_scale=render_scale,
        realtime=True,
    )
    robot_config = RobotConfig(
        epsilon=0.15,
        extension_speed=4,
        rotation_speed=2,
        angle_rotation_speed=0.75,
        move_center_speed=2,
        body_mass=5,
        body_radius=0.4,
        foot_mass=0.2,
        leg_mass=0.2,
        simplified_problem=True,
        leg_spring_stiffness=600,
        leg_spring_damping=20.0,
        min_extension=0.3,
        max_takeoff_speed=20,
        max_jump_dist=15,
        prune_short_jumps=True,
        prune_in_clique_jumps=True,
        prune_similar_jumps=True,
    )
    return sim_config, robot_config


def grid_label(coordinator: InstanceSimulationConfig, pos) -> str:
    gx, gy = coordinator.screen_to_grid(pos)
    return f"({gx:.2f}, {gy:.2f})"


def cross_waypoints(home: tuple[float, float], delta: float = 1.0):
    x, y = home
    return [
        ("home", (x, y)),
        ("up", (x, y + delta)),
        ("home", (x, y)),
        ("down", (x, y - delta)),
        ("home", (x, y)),
        ("left", (x - delta, y)),
        ("home", (x, y)),
        ("right", (x + delta, y)),
        ("home", (x, y)),
    ]


def release_free_leg(simulator: MonkeyBotSimulator, free_limb_id: int):
    signal = empty_control_signal(len(simulator.feet))
    signal.grip[free_limb_id] = -1
    simulator.apply_signal(signal)
    simulator.step()


def run_move_segment(
    simulator: MonkeyBotSimulator,
    coordinator: InstanceSimulationConfig,
    segment_name: str,
    target_grid: tuple[float, float],
    *,
    max_seconds: float,
    free_limb_id: int,
    rows: list[dict],
):
    target_screen = coordinator.grid_to_screen(*target_grid)
    move = MoveCenter(target_screen, coordinator)
    max_steps = int(max_seconds * coordinator.sim_config.fps)
    num_legs = coordinator.num_legs

    start_center = simulator.get_center_pos()
    start_dist = (start_center - target_screen).length
    max_spin = 0.0
    finished = False
    steps_taken = 0

    for step in range(max_steps):
        steps_taken = step + 1
        if not simulator.run:
            break

        state = simulator.get_state()
        signal = move.adjust_signal(empty_control_signal(num_legs), state)

        simulator.apply_signal(signal)
        simulator.step()

        dist = (state.center_pos - target_screen).length
        max_spin = max(max_spin, abs(state.body_angular_velocity))
        rows.append(
            {
                "segment": segment_name,
                "step": step,
                "t": step * coordinator.dt,
                "center_gx": coordinator.screen_to_grid(state.center_pos)[0],
                "center_gy": coordinator.screen_to_grid(state.center_pos)[1],
                "goal_gx": target_grid[0],
                "goal_gy": target_grid[1],
                "dist_cells": dist / coordinator.cell_size,
                "body_spin": state.body_angular_velocity,
                "leg0_len": limb_length(state, 0),
                "leg1_len": limb_length(state, 1),
                "leg2_len": limb_length(state, 2),
                "ext0": signal.extension[0],
                "ext1": signal.extension[1],
                "ext2": signal.extension[2],
                "grip0": int(state.active_grips[0]),
                "grip1": int(state.active_grips[1]),
                "grip2": int(state.active_grips[2]),
            }
        )

        if move.is_finished(state):
            finished = True
            break

    end_center = simulator.get_center_pos()
    end_dist = (end_center - target_screen).length
    return {
        "segment": segment_name,
        "target": target_grid,
        "start": grid_label(coordinator, start_center),
        "end": grid_label(coordinator, end_center),
        "start_dist_cells": start_dist / coordinator.cell_size,
        "end_dist_cells": end_dist / coordinator.cell_size,
        "finished": finished,
        "steps": steps_taken,
        "max_spin": max_spin,
    }


def write_csv(path: Path, rows: list[dict]):
    path.parent.mkdir(parents=True, exist_ok=True)
    if not rows:
        return
    with path.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        writer.writeheader()
        writer.writerows(rows)


def print_summary(summaries: list[dict], epsilon_cells: float):
    print()
    print("MoveCenter cross — segment summary")
    print("-" * 72)
    print(f"{'segment':<8} {'target':<14} {'start':<16} {'end':<16} {'err':>6} {'done':>5}")
    print("-" * 72)
    for s in summaries:
        target = f"({s['target'][0]:.1f},{s['target'][1]:.1f})"
        ok = "yes" if s["finished"] else "no"
        err = s["end_dist_cells"]
        flag = " !" if err > epsilon_cells else ""
        print(
            f"{s['segment']:<8} {target:<14} {s['start']:<16} {s['end']:<16} "
            f"{err:>5.2f}{flag} {ok:>5}"
        )
    print("-" * 72)
    print(f"epsilon = {epsilon_cells:.2f} cells  (! = missed target)")


def main():
    parser = argparse.ArgumentParser(description="MoveCenter-only cross-pattern test.")
    parser.add_argument("--instance", default="MoveCenterCross")
    parser.add_argument("--instances-folder", default="instances")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--cell-size", type=float, default=40)
    parser.add_argument("--cross-delta", type=float, default=1.0)
    parser.add_argument("--segment-seconds", type=float, default=8.0)
    parser.add_argument("--free-leg", type=int, default=1)
    parser.add_argument(
        "--out",
        type=Path,
        default=Path("experiments/results/move_center_cross.csv"),
    )
    args = parser.parse_args()

    instance = load_instance(args.instance, args.instances_folder)
    sim_config, robot_config = default_configs(cell_size=args.cell_size)
    if args.headless:
        sim_config.realtime = False
    coordinator = InstanceSimulationConfig(instance, sim_config, robot_config)

    simulator = MonkeyBotSimulator(sim_config)
    simulator.start_simulation(coordinator)
    if not args.headless:
        pygame.display.set_caption(f"{args.instance}: MoveCenter-only cross")

    home = instance.init_center
    waypoints = cross_waypoints(home, delta=args.cross_delta)
    rows: list[dict] = []
    summaries: list[dict] = []

    print(f"Instance: {args.instance} | gripped legs 0+2 | free leg {args.free_leg}")
    print("MoveCenter only — extension on gripped legs, nothing else.")
    release_free_leg(simulator, args.free_leg)

    for name, target in waypoints:
        if not simulator.run:
            break
        simulator.update_goal_point(coordinator.grid_to_screen(*target))
        if not args.headless:
            pygame.display.set_caption(f"{args.instance}: MoveCenter -> {name} {target}")
        print(f"  -> {name} target {target}")
        summaries.append(
            run_move_segment(
                simulator,
                coordinator,
                name,
                target,
                max_seconds=args.segment_seconds,
                free_limb_id=args.free_leg,
                rows=rows,
            )
        )

    write_csv(args.out, rows)
    print_summary(summaries, coordinator.epsilon / coordinator.cell_size)
    print(f"\nWrote trace to {args.out}")

    if not args.headless:
        pygame.quit()


if __name__ == "__main__":
    main()
