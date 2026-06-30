"""
Graph-based planner for the simplified leg-attachment problem.

Mirrors ``get_simplified_problem`` but builds an explicit state graph and
searches with Dijkstra (unit edge costs). Avoids loading unified-planning.
"""

from __future__ import annotations

import heapq
import itertools
from dataclasses import dataclass
from typing import Callable, Iterable, Optional

import smallestenclosingcircle

from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance
from monkey_bot.coords import Point2D, grid_distance, normalize_point, parse_coord, parse_point_tokens, catch_points_str, parse_catch_points, tuple_to_str


@dataclass(frozen=True)
class PlannerState:
    feet: tuple[Optional[Point2D], ...]
    close_enough: tuple[bool, ...]


@dataclass
class GraphPlan:
    actions: list[str]


def state_str(feet: tuple[Optional[Point2D], ...]) -> str:
    parts: list[str] = []
    for foot in feet:
        if foot is None:
            parts.append("None")
        else:
            parts.append(tuple_to_str(foot))
    return "_".join(parts)


def _gp_close_to_goal(
    gripping_points: Iterable[Point2D],
    goal_point: Point2D,
    leg_extension: float,
) -> dict[Point2D, bool]:
    return {
        gp: grid_distance(gp, goal_point) <= leg_extension
        for gp in gripping_points
    }


def _build_valid_configs(
    gripping_points: list[Point2D],
    leg_extension: float,
    num_legs: int,
    min_attached: int = 2,
) -> set[frozenset[Point2D]]:
    valid: set[frozenset[Point2D]] = set()
    for size in range(min_attached, num_legs + 1):
        for combo in itertools.combinations(gripping_points, size):
            _, _, radius = smallestenclosingcircle.make_circle(combo)
            if radius <= leg_extension:
                valid.add(frozenset(combo))
    return valid


def _build_valid_feet_states(
    valid_configs: set[frozenset[Point2D]],
    num_legs: int,
) -> dict[tuple[Optional[Point2D], ...], frozenset[Point2D]]:
    valid_states: dict[tuple[Optional[Point2D], ...], frozenset[Point2D]] = {}
    for cfg in valid_configs:
        cfg_list = list(cfg)
        for perm in itertools.permutations(cfg_list):
            for leg_slots in itertools.combinations(range(num_legs), len(cfg_list)):
                feet: list[Optional[Point2D]] = [None] * num_legs
                for slot, gp in zip(leg_slots, perm):
                    feet[slot] = gp
                key = tuple(feet)
                valid_states[key] = frozenset(cfg)
    return valid_states


class SimplifiedGraphPlanner:
    """
    Drop-in replacer for ``get_simplified_problem`` + ``solve_problem``.

    States are foot attachment configurations plus per-foot "close enough to
    goal" flags. Edges are release, attach, and jump (transition link) actions
    with unit cost; shortest path is found via Dijkstra.
    """

    def __init__(
        self,
        instance: MonkeyBotProblemInstance,
        transition_links: Optional[Iterable[tuple]] = None,
        *,
        min_attached: int = 2,
    ):
        self.instance = instance
        self.transition_links = list(transition_links or [])
        self.min_attached = min_attached
        self.num_legs = len(instance.init_feet)

        self.gp_is_close_enough = _gp_close_to_goal(
            instance.gripping_points,
            instance.goal_point,
            instance.max_extension,
        )
        self.valid_configs = _build_valid_configs(
            instance.gripping_points,
            instance.max_extension,
            self.num_legs,
            min_attached=min_attached,
        )
        self.valid_feet_states = _build_valid_feet_states(
            self.valid_configs,
            self.num_legs,
        )

    @property
    def initial_state(self) -> PlannerState:
        feet: list[Optional[Point2D]] = []
        close_enough: list[bool] = []
        for gp in self.instance.init_feet:
            if gp is None:
                feet.append(None)
                close_enough.append(True)
            else:
                feet.append(gp)
                close_enough.append(self.gp_is_close_enough[gp])
        return PlannerState(tuple(feet), tuple(close_enough))

    def is_goal(self, state: PlannerState) -> bool:
        return all(state.close_enough)

    def successors(self, state: PlannerState) -> list[tuple[PlannerState, str]]:
        results: list[tuple[PlannerState, str]] = []
        feet = state.feet
        close = state.close_enough
        attached = {gp for gp in feet if gp is not None}
        attached_count = len(attached)

        if attached_count > self.min_attached:
            for leg_id, gp in enumerate(feet):
                if gp is None:
                    continue
                new_feet = list(feet)
                new_feet[leg_id] = None
                new_close = list(close)
                new_close[leg_id] = True
                action = f"release__{leg_id + 1}__{state_str(feet)}"
                results.append(
                    (PlannerState(tuple(new_feet), tuple(new_close)), action)
                )

        if attached_count < self.num_legs:
            for leg_id, gp in enumerate(feet):
                if gp is not None:
                    continue
                for point in self.instance.gripping_points:
                    if frozenset(attached | {point}) not in self.valid_configs:
                        continue
                    new_feet = list(feet)
                    new_feet[leg_id] = point
                    new_close = list(close)
                    new_close[leg_id] = self.gp_is_close_enough[point]
                    action = (
                        f"attach__{leg_id + 1}__{tuple_to_str(point)}__{state_str(feet)}"
                    )
                    results.append(
                        (PlannerState(tuple(new_feet), tuple(new_close)), action)
                    )

        foot_set = set(feet)
        for p_jump1, p_jump2, p_catch in self.transition_links:
            if p_jump1 not in foot_set or p_jump2 not in foot_set:
                continue
            catch = tuple(p_catch)
            new_close = tuple(self.gp_is_close_enough[p] for p in catch)
            p_catch_str = catch_points_str(catch)
            action = (
                f"use_TL__from__{tuple_to_str(p_jump1)}__{tuple_to_str(p_jump2)}"
                f"__to__{p_catch_str}"
            )
            results.append((PlannerState(catch, new_close), action))

        return results

    def solve(self, timeout: Optional[float] = None) -> Optional[GraphPlan]:
        return dijkstra_plan(
            start=self.initial_state,
            is_goal=self.is_goal,
            successors=self.successors,
            timeout=timeout,
        )


def dijkstra_plan(
    *,
    start: PlannerState,
    is_goal: Callable[[PlannerState], bool],
    successors: Callable[[PlannerState], list[tuple[PlannerState, str]]],
    timeout: Optional[float] = None,
) -> Optional[GraphPlan]:
    """Shortest-path search with unit edge weights (Dijkstra)."""
    if is_goal(start):
        return GraphPlan(actions=[])

    frontier: list[tuple[int, int, PlannerState, list[str]]] = []
    counter = 0
    heapq.heappush(frontier, (0, counter, start, []))
    best_cost: dict[PlannerState, int] = {start: 0}

    if timeout is not None:
        import time

        deadline = time.perf_counter() + timeout
    else:
        deadline = None

    while frontier:
        if deadline is not None and time.perf_counter() > deadline:
            return None

        cost, _, state, path = heapq.heappop(frontier)
        if cost != best_cost.get(state):
            continue
        if is_goal(state):
            return GraphPlan(actions=path)

        for next_state, action in successors(state):
            next_cost = cost + 1
            if next_cost >= best_cost.get(next_state, 1 << 60):
                continue
            best_cost[next_state] = next_cost
            counter += 1
            heapq.heappush(frontier, (next_cost, counter, next_state, path + [action]))

    return None


def solve_simplified_problem(
    instance: MonkeyBotProblemInstance,
    transition_links: Optional[Iterable[tuple]] = None,
    timeout: Optional[float] = None,
) -> Optional[GraphPlan]:
    """Convenience wrapper matching the old ``solve_problem`` call shape."""
    return SimplifiedGraphPlanner(instance, transition_links).solve(timeout=timeout)


def _config_neighbors(
    cfg: frozenset[Point2D],
    valid_configs: set[frozenset[Point2D]],
    *,
    min_attached: int,
    num_legs: int,
    gripping_points: list[Point2D],
) -> set[frozenset[Point2D]]:
    neighbors: set[frozenset[Point2D]] = set()
    if len(cfg) > min_attached:
        for gp in cfg:
            smaller = frozenset(cfg - {gp})
            if smaller in valid_configs:
                neighbors.add(smaller)
    if len(cfg) < num_legs:
        for gp in gripping_points:
            if gp in cfg:
                continue
            larger = frozenset(cfg | {gp})
            if larger in valid_configs:
                neighbors.add(larger)
    return neighbors


def compute_attach_config_components(
    instance: MonkeyBotProblemInstance,
    *,
    min_attached: int = 2,
) -> tuple[dict[frozenset[Point2D], int], dict[int, set[Point2D]]]:
    """
    Partition valid attachment configs into connected components using only
    attach/release moves (no jumps).
    """
    planner = SimplifiedGraphPlanner(instance, [], min_attached=min_attached)
    valid_configs = planner.valid_configs
    config_component: dict[frozenset[Point2D], int] = {}
    gps_by_component: dict[int, set[Point2D]] = {}
    next_id = 0

    for seed in valid_configs:
        if seed in config_component:
            continue
        queue = [seed]
        config_component[seed] = next_id
        component_gps: set[Point2D] = set(seed)
        while queue:
            cfg = queue.pop()
            for nbr in _config_neighbors(
                cfg,
                valid_configs,
                min_attached=min_attached,
                num_legs=planner.num_legs,
                gripping_points=instance.gripping_points,
            ):
                if nbr not in config_component:
                    config_component[nbr] = next_id
                    component_gps |= nbr
                    queue.append(nbr)
        gps_by_component[next_id] = component_gps
        next_id += 1

    return config_component, gps_by_component


def goal_satisfying_components(
    instance: MonkeyBotProblemInstance,
    config_component: dict[frozenset[Point2D], int],
    *,
    min_attached: int = 2,
) -> set[int]:
    planner = SimplifiedGraphPlanner(instance, [], min_attached=min_attached)
    goal_components: set[int] = set()
    for cfg in planner.valid_configs:
        if len(cfg) != planner.num_legs:
            continue
        if all(planner.gp_is_close_enough[gp] for gp in cfg):
            goal_components.add(config_component[cfg])
    return goal_components


def is_solvable_with_links(
    instance: MonkeyBotProblemInstance,
    transition_links: Iterable[tuple],
    *,
    min_attached: int = 2,
) -> bool:
    return (
        SimplifiedGraphPlanner(instance, transition_links, min_attached=min_attached).solve()
        is not None
    )


def analyze_reachability(
    instance: MonkeyBotProblemInstance,
    transition_links: Optional[Iterable[tuple]] = None,
) -> dict[str, int | bool]:
    """Count reachable states and whether any goal state exists."""
    planner = SimplifiedGraphPlanner(instance, transition_links)
    seen: set[PlannerState] = {planner.initial_state}
    queue = [planner.initial_state]
    goal_states = 0
    while queue:
        state = queue.pop()
        if planner.is_goal(state):
            goal_states += 1
        for next_state, _ in planner.successors(state):
            if next_state not in seen:
                seen.add(next_state)
                queue.append(next_state)
    return {
        "reachable_states": len(seen),
        "goal_states": goal_states,
        "solvable": goal_states > 0,
        "transition_links": len(list(transition_links or [])),
    }
