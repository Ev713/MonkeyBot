import itertools
import random
from typing import Callable, List, Optional

from pymunk import Vec2d

from monkey_bot.config import InstanceSimulationConfig
from monkey_bot.coords import Point2D, grid_distance, normalize_point
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance, snapshot_instance
from monkey_bot.signals import StateSignal
from monkey_bot.simplified_graph_planner import GraphPlan, SimplifiedGraphPlanner


def _foot_snap_tolerance(coordinator: InstanceSimulationConfig) -> float:
    """Screen-space slack for mapping a simulated foot onto a gripping point."""
    return max(
        coordinator.epsilon * 2,
        coordinator.foot_radius * 3,
        coordinator.cell_size * 0.5,
    )


def nearest_gripping_point(
    coordinator: InstanceSimulationConfig,
    screen_pos: Vec2d,
    *,
    max_distance: Optional[float] = None,
) -> Optional[Point2D]:
    """Return the nearest gripping point to *screen_pos*, or None if too far."""
    if max_distance is None:
        max_distance = _foot_snap_tolerance(coordinator)
    elif max_distance == float("inf"):
        max_distance = None
    best_gp = None
    best_dist = float("inf")
    for gp in coordinator.instance.gripping_points:
        screen_gp = coordinator.grid_to_screen(*gp)
        dist = (screen_gp - screen_pos).length
        if dist < best_dist:
            best_dist = dist
            best_gp = gp
    if best_gp is None or (max_distance is not None and best_dist > max_distance):
        return None
    return best_gp


def feet_grid_positions(
    coordinator: InstanceSimulationConfig,
    state: StateSignal,
) -> List[Optional[Point2D]]:
    """Map each gripped foot to its GP; ungripped feet are ``None``."""
    loose_tol = _foot_snap_tolerance(coordinator)
    feet: List[Optional[Point2D]] = []
    for i, foot_pos in enumerate(state.feet_pos):
        if not state.active_grips[i]:
            feet.append(None)
            continue
        gp = nearest_gripping_point(coordinator, foot_pos, max_distance=float("inf"))
        if gp is None:
            gp = nearest_gripping_point(coordinator, foot_pos, max_distance=loose_tol)
        if gp is None:
            raise ValueError(
                f"Gripped foot {i + 1} is not on a known gripping point"
            )
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


def plan_has_climbing_actions(plan: GraphPlan | None) -> bool:
    """True when the plan does more than release feet to fake goal satisfaction."""
    if plan is None:
        return False
    return any("attach" in action or "use_TL" in action for action in plan.actions)


def _valid_goal_pool(
    instance: MonkeyBotProblemInstance,
    current_center: Point2D,
    *,
    min_nearby_grips: int,
    excluded: set[Point2D],
) -> list[Point2D]:
    max_ext = instance.max_extension
    pool: list[Point2D] = []
    for goal in _goal_candidates(instance):
        goal = normalize_point(goal)
        if goal in excluded:
            continue
        if grid_distance(goal, current_center) < 1e-6:
            continue
        nearby = [
            gp for gp in instance.gripping_points if grid_distance(gp, goal) <= max_ext
        ]
        if len(nearby) < min_nearby_grips:
            continue
        pool.append(goal)
    return pool


def generate_next_goal(
    instance: MonkeyBotProblemInstance,
    current_center: Point2D,
    rng: random.Random,
    *,
    min_nearby_grips: int = 2,
    exclude_goals: set[Point2D] | None = None,
    previous_goal: Point2D | None = None,
) -> Point2D:
    """Pick a random goal with at least *min_nearby_grips* GPs within leg reach."""
    excluded = {normalize_point(g) for g in (exclude_goals or set())}
    if previous_goal is not None:
        excluded.add(normalize_point(previous_goal))

    pool = _valid_goal_pool(
        instance,
        current_center,
        min_nearby_grips=min_nearby_grips,
        excluded=excluded,
    )
    if not pool:
        raise ValueError("No alternative goal point available for replanning")
    return rng.choice(pool)


def pick_replan_instance(
    base: MonkeyBotProblemInstance,
    coordinator: InstanceSimulationConfig,
    state: StateSignal,
    rng: random.Random,
    *,
    previous_goal: Point2D,
    max_attempts: int = 64,
    plan_validator: Callable[[MonkeyBotProblemInstance], bool] | None = None,
) -> MonkeyBotProblemInstance:
    """
    Snapshot the live robot state and pick a random new goal.

    Goals must have at least two gripping points within leg length and must pass
    *plan_validator* (defaults to requiring a non-trivial attach-only graph plan).
    """
    init_center = coordinator.screen_to_grid(state.center_pos)
    init_feet = feet_grid_positions(coordinator, state)
    prev = normalize_point(previous_goal)

    if plan_validator is None:
        def plan_validator(instance: MonkeyBotProblemInstance) -> bool:
            plan = SimplifiedGraphPlanner(instance, []).solve()
            return plan_has_climbing_actions(plan)

    pool = _valid_goal_pool(
        base,
        init_center,
        min_nearby_grips=2,
        excluded={prev},
    )
    if not pool:
        raise ValueError("No alternative goal point available for replanning")

    rng.shuffle(pool)
    attempts = pool[:max_attempts]
    for goal_point in attempts:
        candidate = snapshot_instance(
            base,
            init_center=init_center,
            init_feet=init_feet,
            goal_point=goal_point,
        )
        if plan_validator(candidate):
            return candidate

    raise ValueError(
        f"Could not find a reachable goal different from the previous one ({prev})"
    )


def build_replan_instance(
    base: MonkeyBotProblemInstance,
    coordinator: InstanceSimulationConfig,
    state: StateSignal,
    rng: random.Random,
    *,
    previous_goal: Point2D,
) -> MonkeyBotProblemInstance:
    """Snapshot the live simulation state and attach a freshly generated goal."""
    return pick_replan_instance(
        base,
        coordinator,
        state,
        rng,
        previous_goal=previous_goal,
    )
