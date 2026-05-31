"""
Record body and all limb angles while one released leg spins at constant rate.

Braced (gripped) legs receive rotation rate 0; the active leg receives a fixed
command rate. Outputs CSV + plot under experiments/results/body_rotation_record/.
"""

import argparse
import csv
import math
import os
import sys
from pathlib import Path

if "--visible" not in sys.argv and "--demo" not in sys.argv:
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pygame

from monkey_bot.procedure import BodyHolder, Tracker, limb_length
from monkey_bot.config import InstanceSimulationConfig, RobotConfig, SimConfig
from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.pymunk_simulator import MonkeyBotSimulator
from monkey_bot.signals import ControlSignal


def normalize_angle(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def angle_delta(current, start):
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


def compute_render_scale(instance, cell_size, screen_width, screen_height, margin=80, max_scale=16):
    usable_width = max(1, screen_width - 2 * margin)
    usable_height = max(1, screen_height - 2 * margin)
    scale_x = usable_width / (instance.grid_size_x * cell_size)
    scale_y = usable_height / (instance.grid_size_y * cell_size)
    return max(1.0, min(max_scale, scale_x, scale_y))


def zero_signal(num_legs):
    return ControlSignal(
        extension=[0.0 for _ in range(num_legs)],
        rotation=[0.0 for _ in range(num_legs)],
        grip=[0 for _ in range(num_legs)],
    )


def read_all_angles(simulator):
    body = simulator.body.angle
    limbs_world = [leg[0].angle for leg in simulator.legs]
    limbs_body = [normalize_angle(l - body) for l in limbs_world]
    return body, limbs_world, limbs_body


def grip_point_for_limb(coordinator, limb_id):
    x, y = coordinator.instance.init_feet[limb_id]
    return coordinator.grid_to_screen(x, y)


def stance_holder(coordinator, limb_id, enabled):
    if not enabled:
        return None
    holding_limb_ids = [i for i in range(coordinator.num_legs) if i != limb_id]
    return BodyHolder(coordinator, *holding_limb_ids)


def apply_stance_holder(signal, state, body_holder):
    if body_holder is not None:
        signal = body_holder.adjust_signal(signal, state)
    return signal


def regrip_limb(simulator, coordinator, limb_id, body_hold=True, max_seconds=5.0):
    goal = grip_point_for_limb(coordinator, limb_id)
    body_holder = stance_holder(coordinator, limb_id, body_hold)
    tracker = Tracker(limb_id, goal, coordinator)
    max_steps = int(max_seconds * coordinator.sim_config.fps)

    for _ in range(max_steps):
        if not simulator.run:
            return False
        if simulator.active_grips[limb_id] is not None:
            return True

        signal = zero_signal(coordinator.num_legs)
        state = simulator.get_state()
        signal = tracker.adjust_signal(signal, state)
        if (state.feet_pos[limb_id] - goal).length < coordinator.epsilon:
            signal.grip[limb_id] = 1
        if body_holder is not None:
            signal = apply_stance_holder(signal, state, body_holder)
        simulator.apply_signal(signal)
        simulator.step()

    return simulator.active_grips[limb_id] is not None


def ensure_only_limb_free(simulator, coordinator, limb_id, body_hold=True):
    for i in range(coordinator.num_legs):
        if i == limb_id:
            continue
        if simulator.active_grips[i] is None:
            pygame.display.set_caption(f"Re-gripping limb {i} before releasing limb {limb_id}")
            print(f"  -> re-gripping limb {i}")
            if not regrip_limb(simulator, coordinator, i, body_hold=body_hold):
                print(f"warning: could not re-grip limb {i} before releasing limb {limb_id}")
                return False
            pause_segment(simulator, coordinator, pause_s=0.4)
    return True


def run_spin_segment(
    simulator,
    coordinator,
    limb_id,
    speed_deg_per_s,
    duration_s,
    body_hold=False,
    settle_frames=30,
    release_first=True,
):
    num_legs = coordinator.num_legs

    if not ensure_only_limb_free(simulator, coordinator, limb_id, body_hold=body_hold):
        return False

    if release_first:
        release = zero_signal(num_legs)
        release.grip[limb_id] = -1
        simulator.apply_signal(release)
        simulator.step()

    for _ in range(settle_frames):
        if not simulator.run:
            return False
        simulator.apply_signal(zero_signal(num_legs))
        simulator.step()

    command_rate = math.radians(speed_deg_per_s)
    body_holder = stance_holder(coordinator, limb_id, body_hold)
    free_leg_ref_length = None
    max_steps = int(duration_s * coordinator.sim_config.fps)

    for step in range(max_steps):
        if not simulator.run:
            return False

        signal = zero_signal(num_legs)
        signal.rotation[limb_id] = command_rate
        state = simulator.get_state()

        if free_leg_ref_length is None:
            free_leg_ref_length = limb_length(state, limb_id)
        length_error = free_leg_ref_length - limb_length(state, limb_id)
        if abs(length_error) > coordinator.epsilon:
            length_drive = max(
                -coordinator.extension_speed,
                min(coordinator.extension_speed, 3.0 * length_error),
            )
            signal.extension[limb_id] = length_drive

        signal = apply_stance_holder(signal, state, body_holder)
        simulator.apply_signal(signal)
        simulator.step()

    return True


def pause_segment(simulator, coordinator, pause_s=0.8):
    num_legs = coordinator.num_legs
    steps = int(pause_s * coordinator.sim_config.fps)
    for _ in range(steps):
        if not simulator.run:
            return False
        simulator.apply_signal(zero_signal(num_legs))
        simulator.step()
    return True


def run_visible_demo(
    instance_name,
    instances_folder,
    cell_size,
    render_scale,
    body_hold,
    screen_width=1000,
    screen_height=1000,
):
    instance = load_instance(instance_name, instances_folder)
    if render_scale <= 0:
        render_scale = compute_render_scale(instance, cell_size, screen_width, screen_height)

    sim_config, robot_config = default_configs(
        cell_size=cell_size,
        render_scale=render_scale,
    )
    sim_config.realtime = True
    sim_config.screen_width = screen_width
    sim_config.screen_height = screen_height
    coordinator = InstanceSimulationConfig(instance, sim_config, robot_config)

    simulator = MonkeyBotSimulator(sim_config)
    simulator.start_simulation(coordinator)
    pygame.display.set_caption(
        f"{instance_name} body rotation demo (render_scale={render_scale:.1f}x)"
    )

    segments = [
        ("Limb 0  +45 deg/s  2.0s", 0, 45.0, 2.0),
        ("Limb 0  -60 deg/s  1.5s", 0, -60.0, 1.5),
        ("Limb 2  +30 deg/s  2.5s", 2, 30.0, 2.5),
        ("Limb 2  -90 deg/s  1.2s", 2, -90.0, 1.2),
        ("Limb 0  +75 deg/s  1.8s", 0, 75.0, 1.8),
    ]

    print(f"render_scale: {render_scale:.2f}x")
    print(f"running {len(segments)} segments (close window to stop early)")

    for label, limb_id, speed_deg, duration_s in segments:
        if not simulator.run:
            break
        pygame.display.set_caption(f"{instance_name}: {label}")
        print(f"  -> {label}")
        if not run_spin_segment(
            simulator,
            coordinator,
            limb_id,
            speed_deg,
            duration_s,
            body_hold=body_hold,
            settle_frames=20,
            release_first=True,
        ):
            break
        if not pause_segment(simulator, coordinator, pause_s=0.8):
            break

    if simulator.run:
        for limb_id in range(coordinator.num_legs):
            if simulator.active_grips[limb_id] is None:
                pygame.display.set_caption(f"{instance_name}: re-gripping limb {limb_id}")
                print(f"  -> re-gripping limb {limb_id}")
                regrip_limb(simulator, coordinator, limb_id, body_hold=body_hold)
                pause_segment(simulator, coordinator, pause_s=0.5)

    pygame.quit()
    print("demo finished")


def run_trial(
    instance_name,
    instances_folder,
    limb_id,
    speed_deg_per_s,
    duration_s,
    settle_frames,
    out_dir,
    cell_size,
    render_scale,
    body_hold=False,
    visible=False,
):
    sim_config, robot_config = default_configs(cell_size=cell_size, render_scale=render_scale)
    if visible:
        sim_config.realtime = True
    instance = load_instance(instance_name, instances_folder)
    coordinator = InstanceSimulationConfig(instance, sim_config, robot_config)
    num_legs = coordinator.num_legs

    simulator = MonkeyBotSimulator(sim_config)
    simulator.start_simulation(coordinator)

    release = zero_signal(num_legs)
    release.grip[limb_id] = -1
    simulator.apply_signal(release)
    simulator.step()

    for _ in range(settle_frames):
        simulator.apply_signal(zero_signal(num_legs))
        simulator.step()

    command_rate = math.radians(speed_deg_per_s)
    body_holder = stance_holder(coordinator, limb_id, body_hold)
    start_body, start_limbs_world, start_limbs_body = read_all_angles(simulator)
    prev_body, prev_limbs_world, prev_limbs_body = start_body, start_limbs_world, start_limbs_body

    max_steps = int(duration_s * sim_config.fps)
    rows = []

    for step in range(max_steps):
        if not simulator.run:
            break
        body, limbs_world, limbs_body = read_all_angles(simulator)
        body_rate = -angle_delta(body, prev_body) / sim_config.dt
        limb_world_rates = [
            -angle_delta(lw, plw) / sim_config.dt
            for lw, plw in zip(limbs_world, prev_limbs_world)
        ]
        limb_body_rates = [
            -angle_delta(lb, plb) / sim_config.dt
            for lb, plb in zip(limbs_body, prev_limbs_body)
        ]

        row = {
            "step": step,
            "t": step * sim_config.dt,
            "body_world": body,
            "body_world_delta": angle_delta(body, start_body),
            "body_rate": body_rate,
            "command_rate": command_rate if step > 0 else 0.0,
        }
        for i in range(num_legs):
            row[f"limb{i}_world"] = limbs_world[i]
            row[f"limb{i}_body"] = limbs_body[i]
            row[f"limb{i}_world_delta"] = angle_delta(limbs_world[i], start_limbs_world[i])
            row[f"limb{i}_body_delta"] = angle_delta(limbs_body[i], start_limbs_body[i])
            row[f"limb{i}_world_rate"] = limb_world_rates[i]
            row[f"limb{i}_body_rate"] = limb_body_rates[i]
            row[f"limb{i}_gripped"] = int(simulator.active_grips[i] is not None)
        rows.append(row)

        signal = zero_signal(num_legs)
        signal.rotation[limb_id] = command_rate
        state = simulator.get_state()
        signal = apply_stance_holder(signal, state, body_holder)
        simulator.apply_signal(signal)
        simulator.step()

        prev_body, prev_limbs_world, prev_limbs_body = body, limbs_world, limbs_body

    if not rows:
        pygame.quit()
        raise RuntimeError("Simulation ended before recording any samples.")

    if not visible:
        out_dir.mkdir(parents=True, exist_ok=True)
        stem = (
            f"{instance_name}_limb{limb_id}_"
            f"speed{str(speed_deg_per_s).replace('.', 'p')}_"
            f"{duration_s:.1f}s"
            f"{'_bodyhold' if body_hold else ''}"
        )
        csv_path = out_dir / f"{stem}.csv"
        plot_path = out_dir / f"{stem}.png"

        fieldnames = list(rows[0].keys())
        with csv_path.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)

        t = np.array([r["t"] for r in rows])
        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True, constrained_layout=True)

        axes[0].plot(t, np.degrees([r["body_world_delta"] for r in rows]), label="body")
        for i in range(num_legs):
            axes[0].plot(
                t,
                np.degrees([r[f"limb{i}_world_delta"] for r in rows]),
                label=f"limb {i} world",
                linestyle="--" if i == limb_id else "-",
            )
        axes[0].set_ylabel("deg from start")
        axes[0].set_title(
            f"World angles: limb {limb_id} @ {speed_deg_per_s:g} deg/s for {duration_s:g}s"
        )
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(ncol=2, fontsize=8)

        axes[1].plot(t, np.degrees([r["body_rate"] for r in rows]), label="body rate")
        axes[1].axhline(math.degrees(command_rate), color="C1", ls=":", label="command rate")
        for i in range(num_legs):
            axes[1].plot(
                t,
                np.degrees([r[f"limb{i}_body_rate"] for r in rows]),
                label=f"limb {i} rel rate",
                alpha=0.8,
            )
        axes[1].set_ylabel("deg/s")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(ncol=2, fontsize=8)

        axes[2].plot(t, np.degrees([r["body_world_delta"] for r in rows]), label="body drift")
        for i in range(num_legs):
            if i != limb_id:
                axes[2].plot(
                    t,
                    np.degrees([r[f"limb{i}_body_delta"] for r in rows]),
                    label=f"braced limb {i} rel angle",
                )
        axes[2].set_xlabel("time (s)")
        axes[2].set_ylabel("deg")
        axes[2].set_title("Body drift vs braced limb relative angles (should stay flat if rate=0 holds)")
        axes[2].grid(True, alpha=0.3)
        axes[2].legend(ncol=2, fontsize=8)

        fig.savefig(plot_path, dpi=160)
        plt.close(fig)
    else:
        csv_path = None
        plot_path = None

    pygame.quit()

    body_drift_deg = math.degrees(angle_delta(rows[-1]["body_world"], start_body))
    braced_rel_drift = [
        math.degrees(angle_delta(rows[-1][f"limb{i}_body"], start_limbs_body[i]))
        for i in range(num_legs)
        if i != limb_id
    ]
    active_rel_delta = math.degrees(
        angle_delta(rows[-1][f"limb{limb_id}_body"], start_limbs_body[limb_id])
    )

    return {
        "csv": csv_path,
        "plot": plot_path,
        "samples": len(rows),
        "body_drift_deg": body_drift_deg,
        "active_limb_rel_delta_deg": active_rel_delta,
        "braced_limb_rel_drift_deg": braced_rel_drift,
    }


def main():
    parser = argparse.ArgumentParser(
        description="Spin one released leg and record all body/limb angles."
    )
    parser.add_argument("--instance", default="TestAngle")
    parser.add_argument("--instances-folder", default="instances")
    parser.add_argument("--limb", type=int, default=0)
    parser.add_argument("--speed-deg", type=float, default=45.0)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument("--settle-frames", type=int, default=5)
    parser.add_argument("--out-dir", default="experiments/results/body_rotation_record")
    parser.add_argument("--cell-size", type=float, default=20)
    parser.add_argument("--render-scale", type=float, default=1, help="Zoom factor for debug draw; <=0 auto-fits instance.")
    parser.add_argument("--body-hold", action="store_true")
    parser.add_argument("--visible", action="store_true", help="Show pygame window at real-time speed.")
    parser.add_argument(
        "--demo",
        action="store_true",
        help="Visible multi-segment demo: several limbs, speeds, and directions.",
    )
    args = parser.parse_args()

    if args.demo:
        if not args.visible:
            args.visible = True
        run_visible_demo(
            instance_name=args.instance,
            instances_folder=args.instances_folder,
            cell_size=args.cell_size,
            render_scale=args.render_scale,
            body_hold=args.body_hold,
        )
        return

    result = run_trial(
        instance_name=args.instance,
        instances_folder=args.instances_folder,
        limb_id=args.limb,
        speed_deg_per_s=args.speed_deg,
        duration_s=args.duration,
        settle_frames=args.settle_frames,
        out_dir=Path(args.out_dir),
        cell_size=args.cell_size,
        render_scale=args.render_scale,
        body_hold=args.body_hold,
        visible=args.visible,
    )

    if args.visible:
        print(f"samples: {result['samples']}")
        print(f"body drift: {result['body_drift_deg']:.3f} deg")
        print(f"active limb rel delta: {result['active_limb_rel_delta_deg']:.3f} deg")
        print(f"braced limb rel drift: {[f'{d:.3f}' for d in result['braced_limb_rel_drift_deg']]}")
        return

    print(f"csv:  {result['csv']}")
    print(f"plot: {result['plot']}")
    print(f"samples: {result['samples']}")
    print(f"body drift: {result['body_drift_deg']:.3f} deg")
    print(f"active limb rel delta: {result['active_limb_rel_delta_deg']:.3f} deg")
    print(f"braced limb rel drift: {[f'{d:.3f}' for d in result['braced_limb_rel_drift_deg']]}")


if __name__ == "__main__":
    main()
