#!/usr/bin/env python3
"""Run the infinite climbing demo."""

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# This demo requires a visible window; do not inherit headless dummy SDL from the shell.
if os.environ.get("SDL_VIDEODRIVER", "").lower() == "dummy":
    del os.environ["SDL_VIDEODRIVER"]

from monkey_bot.config import RobotConfig, SimConfig
from monkey_bot.infinite_demo_runner import InfiniteDemoRunner
from monkey_bot.monkey_bot_problem_instance import load_instance


def default_configs(
    cell_size=None,
    render_scale=1,
    screen_width=1000,
    screen_height=1000,
    *,
    disable_assist: bool = False,
    debug_move_center: bool = True,
    debug_attach: bool = False,
):
    sim_config = SimConfig(
        screen_height=screen_height,
        screen_width=screen_width,
        fps=240,
        gravity=1,
        cell_size=cell_size,
        render_scale=render_scale,
        realtime=True,
    )
    # Primary tuning block — values are in grid-cell units unless noted as degrees.
    robot_config = RobotConfig(
        epsilon=0.2,
        release_tolerance=0.625,
        extension_speed=4,
        rotation_speed=4,
        # Limb aim rate during catch/attach (SmoothAdjustAngle); primary in-air knob.
        angle_rotation_speed=0.75,
        move_center_speed=1,
        move_center_speed_factor=1.0,
        move_center_length_rate_factor=1.0,
        pull_up_angle_tolerance_deg=35.0,
        get_closer_speed_factor=0.2,
        get_closer_length_rate_factor=0.35,
        jump_runway_speed_scale=1.0,
        min_runway_limb_alignment_deg=15.0,
        min_limb_vector_angle_deg=20.0,
        body_mass=5,
        body_radius=0.5,
        body_moment=1000.0,
        foot_mass=0.2,
        leg_mass=0.2,
        simplified_problem=True,
        leg_spring_stiffness=400,
        leg_spring_damping=1.25,
        min_extension=0.4,
        max_takeoff_speed=20,
        max_jump_dist=15,
        prune_short_jumps=True,
        prune_in_clique_jumps=True,
        prune_similar_jumps=True,
        disable_assist=disable_assist,
        debug_move_center=debug_move_center,
        debug_attach=debug_attach,
    )
    return sim_config, robot_config


def main():
    parser = argparse.ArgumentParser(
        description="Run the infinite MonkeyBot climbing demo."
    )
    parser.add_argument("--instance", default="InfiniteDemoQuick")
    parser.add_argument("--instances-folder", default="instances")
    parser.add_argument(
        "--cell-size",
        type=float,
        default=None,
        help="Grid cell size in pixels (default: auto-fit to screen).",
    )
    parser.add_argument("--render-scale", type=float, default=1)
    parser.add_argument("--screen-width", type=int, default=800)
    parser.add_argument("--screen-height", type=int, default=600)
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument(
        "--disable-assist",
        action="store_true",
        help="Turn off simulation assists (e.g. foot snap to GP on grip).",
    )
    parser.add_argument(
        "--no-debug-move-center",
        action="store_true",
        help="Disable per-frame jump runway MoveCenter trace (on by default).",
    )
    parser.add_argument(
        "--debug-attach",
        action="store_true",
        help="Verbose StabilizeLimbs / AttachCatch phase logging.",
    )
    args = parser.parse_args()

    instance = load_instance(args.instance, args.instances_folder)
    sim_config, robot_config = default_configs(
        cell_size=args.cell_size,
        render_scale=args.render_scale,
        screen_width=args.screen_width,
        screen_height=args.screen_height,
        disable_assist=args.disable_assist,
        debug_move_center=not args.no_debug_move_center,
        debug_attach=args.debug_attach,
    )

    runner = InfiniteDemoRunner(
        instance,
        sim_config=sim_config,
        robot_config=robot_config,
        seed=args.seed,
    )
    runner.execute_infinite_demo()


if __name__ == "__main__":
    main()
