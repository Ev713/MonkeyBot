import itertools
import math
import random
from typing import List, Optional

from pymunk import Vec2d

from monkey_bot.config import InstanceSimulationConfig
from monkey_bot.coords import Point2D, grid_distance, normalize_point
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance, snapshot_instance
from monkey_bot.signals import StateSignal


def nearest_gripping_point(
    coordinator: InstanceSimulationConfig,
    screen_pos: Vec2d,
    *,
    max_distance: Optional[float] = None,
) -> Optional[Point2D]:
    """Return the nearest gripping point to *screen_pos*, or None if too far."""
    if max_distance is None:
        max_distance = coordinator.epsilon * 2
    best_gp = None
    best_dist = float("inf")
    for gp in coordinator.instance.gripping_points:
        screen_gp = coordinator.grid_to_screen(*gp)
        dist = (screen_gp - screen_pos).length
        if dist < best_dist:
            best_dist = dist
            best_gp = gp
    if best_gp is None or best_dist > max_distance:
        return None
    return best_gp


def feet_grid_positions(
    coordinator: InstanceSimulationConfig,
    state: StateSignal,
) -> List[Point2D]:
    """Map each simulated foot to the gripping point it is currently on."""
    feet = []
    for foot_pos in state.feet_pos:
        gp = nearest_gripping_point(coordinator, foot_pos)
        if gp is None:
            raise ValueError("Foot is not on a known gripping point")
        feet.append(gp)
    return feet


def _goal_candidates(instance: MonkeyBotProblemInstance) -> list[Point2D]:
    """Sample goal locations from GPs, midpoints, and small offsets."""
    candidates: set[Point2D] = set()
    max_ext = instance.max_extension
    gps = instance.gripping_points

    for gp in gps:
        candidates.add(normalize_point(gp))

    for a, b in itertools.combinations(gps, 2):
        mid = normalize_point(((a[0] + b[0]) / 2, (a[1] + b[1]) / 2))
        candidates.add(mid)

    for gp in gps:
        for dx, dy in ((0.5, 0), (-0.5, 0), (0, 0.5), (0, -0.5)):
            candidates.add(normalize_point((gp[0] + dx, gp[1] + dy)))

    viable = []
    for goal in candidates:
        nearby = [gp for gp in gps if grid_distance(gp, goal) <= max_ext]
        if len(nearby) >= 2:
            viable.append(goal)
    return viable


def generate_next_goal(
    instance: MonkeyBotProblemInstance,
    current_center: Point2D,
    rng: random.Random,
    *,
    min_nearby_grips: int = 2,
) -> Point2D:
    """Pick a goal point within reach of several gripping points."""
    max_ext = instance.max_extension
    candidates: list[tuple[Point2D, float]] = []

    for goal in _goal_candidates(instance):
        if grid_distance(goal, current_center) < 1e-6:
            continue
        nearby = [gp for gp in instance.gripping_points if grid_distance(gp, goal) <= max_ext]
        if len(nearby) < min_nearby_grips:
            continue
        candidates.append((goal, grid_distance(goal, current_center)))

    if not candidates:
        return instance.goal_point

    candidates.sort(key=lambda item: item[1], reverse=True)
    pool_size = max(3, len(candidates) // 4)
    goal, _ = rng.choice(candidates[:pool_size])
    return goal


def build_replan_instance(
    base: MonkeyBotProblemInstance,
    coordinator: InstanceSimulationConfig,
    state: StateSignal,
    rng: random.Random,
) -> MonkeyBotProblemInstance:
    """Snapshot the live simulation state and attach a freshly generated goal."""
    init_center = coordinator.screen_to_grid(state.center_pos)
    init_feet = feet_grid_positions(coordinator, state)
    goal_point = generate_next_goal(base, init_center, rng)
    return snapshot_instance(
        base,
        init_center=init_center,
        init_feet=init_feet,
        goal_point=goal_point,
    )
