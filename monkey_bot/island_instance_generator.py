"""Procedural generator for multi-island gripping-point layouts."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from monkey_bot.coords import Point2D, grid_distance, normalize_point
from monkey_bot.monkey_bot_problem_instance import MonkeyBotProblemInstance


@dataclass
class IslandGeneratorConfig:
    name: str = "InfiniteIslands"
    grid_size_x: float = 100.0
    grid_size_y: float = 100.0
    num_islands: int = 14
    # Initial scattered seeds per island before expansion.
    seeds_per_island: int = 3
    target_points_per_island_min: int = 12
    target_points_per_island_max: int = 28
    min_spacing: float = 1.2
    max_link_distance: float = 2.8
    # Keep island seed clusters apart so landmasses stay separate.
    island_seed_separation: float = 15.0
    margin: float = 6.0
    leg_extension: float = 3.0
    max_growth_attempts_per_point: int = 80


def _in_bounds(
    point: Point2D,
    width: float,
    height: float,
    margin: float,
) -> bool:
    x, y = point
    return margin <= x <= width - margin and margin <= y <= height - margin


def _far_enough(candidate: Point2D, points: Sequence[Point2D], min_spacing: float) -> bool:
    return all(grid_distance(candidate, p) >= min_spacing for p in points)


def _linked_to_island(
    candidate: Point2D,
    island_points: Sequence[Point2D],
    max_link_distance: float,
) -> bool:
    if not island_points:
        return True
    return min(grid_distance(candidate, p) for p in island_points) <= max_link_distance


def _sample_near_existing(
    anchor: Point2D,
    rng: random.Random,
    *,
    min_spacing: float,
    max_link_distance: float,
) -> Point2D:
    angle = rng.uniform(0.0, 2.0 * math.pi)
    dist = rng.uniform(min_spacing, max_link_distance)
    return normalize_point(
        (anchor[0] + dist * math.cos(angle), anchor[1] + dist * math.sin(angle))
    )


def _place_island_seeds(
    rng: random.Random,
    config: IslandGeneratorConfig,
    existing_points: List[Point2D],
) -> Optional[Point2D]:
    width, height = config.grid_size_x, config.grid_size_y
    for _ in range(400):
        candidate = normalize_point(
            (
                rng.uniform(config.margin, width - config.margin),
                rng.uniform(config.margin, height - config.margin),
            )
        )
        if not _far_enough(candidate, existing_points, config.island_seed_separation):
            continue
        return candidate
    return None


def _seed_cluster(
    center: Point2D,
    rng: random.Random,
    config: IslandGeneratorConfig,
) -> List[Point2D]:
    points: List[Point2D] = [center]
    for _ in range(config.seeds_per_island - 1):
        anchor = rng.choice(points)
        for _ in range(config.max_growth_attempts_per_point):
            candidate = _sample_near_existing(
                anchor,
                rng,
                min_spacing=config.min_spacing,
                max_link_distance=config.max_link_distance,
            )
            if _far_enough(candidate, points, config.min_spacing):
                points.append(candidate)
                break
    return points


def _grow_island(
    island_points: List[Point2D],
    rng: random.Random,
    config: IslandGeneratorConfig,
    all_other_points: List[Point2D],
    *,
    target_points: int,
) -> List[Point2D]:
    points = list(island_points)
    failures = 0
    max_failures = target_points * 4

    while len(points) < target_points and failures < max_failures:
        # Bias toward recent points for irregular, branching shapes.
        recent = points[-min(8, len(points)) :]
        anchor = rng.choice(recent)
        added = False
        for _ in range(config.max_growth_attempts_per_point):
            candidate = _sample_near_existing(
                anchor,
                rng,
                min_spacing=config.min_spacing,
                max_link_distance=config.max_link_distance,
            )
            if not _in_bounds(
                candidate,
                config.grid_size_x,
                config.grid_size_y,
                config.margin,
            ):
                continue
            if not _far_enough(candidate, points, config.min_spacing):
                continue
            if not _far_enough(candidate, all_other_points, config.min_spacing):
                continue
            if not _linked_to_island(candidate, points, config.max_link_distance):
                continue
            points.append(candidate)
            added = True
            failures = 0
            break
        if not added:
            failures += 1
    return points


def generate_island_points(
    config: IslandGeneratorConfig,
    rng: random.Random,
) -> List[List[Point2D]]:
    islands: List[List[Point2D]] = []
    seed_centers: List[Point2D] = []

    for _ in range(config.num_islands):
        center = _place_island_seeds(rng, config, seed_centers)
        if center is None:
            break
        seed_centers.append(center)
        target = rng.randint(
            config.target_points_per_island_min,
            config.target_points_per_island_max,
        )
        cluster = _seed_cluster(center, rng, config)
        other = [p for isl in islands for p in isl]
        grown = _grow_island(cluster, rng, config, other, target_points=target)
        islands.append(grown)
    return islands


def _pick_start_triangle(
    points: Sequence[Point2D],
    leg_extension: float,
) -> Optional[Tuple[Point2D, Point2D, Point2D, Point2D]]:
    """Return init feet (3) and body center for a valid attach on one island."""
    pts = list(points)
    for i, p1 in enumerate(pts):
        for p2 in pts[i + 1 :]:
            if grid_distance(p1, p2) > leg_extension:
                continue
            for p3 in pts:
                if p3 in (p1, p2):
                    continue
                if grid_distance(p1, p3) > leg_extension:
                    continue
                if grid_distance(p2, p3) > leg_extension:
                    continue
                center = normalize_point(
                    ((p1[0] + p2[0] + p3[0]) / 3.0, (p1[1] + p3[1] + p2[1]) / 3.0)
                )
                if all(grid_distance(center, p) <= leg_extension for p in (p1, p2, p3)):
                    return p1, p2, p3, center
    return None


def _pick_goal_point(
    gripping_points: Sequence[Point2D],
    init_center: Point2D,
    leg_extension: float,
    rng: random.Random,
) -> Point2D:
    far = [
        p
        for p in gripping_points
        if grid_distance(p, init_center) > leg_extension * 2.0
    ]
    if far:
        return normalize_point(rng.choice(far))
    return normalize_point(
        max(gripping_points, key=lambda p: grid_distance(p, init_center))
    )


def generate_island_instance(
    config: Optional[IslandGeneratorConfig] = None,
    *,
    seed: Optional[int] = None,
) -> MonkeyBotProblemInstance:
    config = config or IslandGeneratorConfig()
    rng = random.Random(seed)
    islands = generate_island_points(config, rng)
    if not islands:
        raise RuntimeError("failed to place any islands")

    gripping_points = [p for island in islands for p in island]
    start_island = min(islands, key=lambda isl: sum(p[1] for p in isl) / len(isl))
    start = _pick_start_triangle(start_island, config.leg_extension)
    if start is None:
        raise RuntimeError("no valid start triangle on lowest island")
    p1, p2, p3, init_center = start
    goal_point = _pick_goal_point(
        gripping_points, init_center, config.leg_extension, rng
    )

    return MonkeyBotProblemInstance(
        name=config.name,
        max_extension=config.leg_extension,
        grid_size_x=config.grid_size_x,
        grid_size_y=config.grid_size_y,
        gripping_points=gripping_points,
        goal_point=goal_point,
        init_center=init_center,
        init_feet=[p1, p2, p3],
    )
