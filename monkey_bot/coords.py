"""Shared coordinate helpers for grid / plan / simulation."""

from __future__ import annotations

import math
from typing import Optional, Tuple, Union

Point2D = Tuple[float, float]
Coord = Union[int, float]


def as_float(value: Coord) -> float:
    return float(value)


def normalize_point(point: Optional[Point2D], *, precision: int = 6) -> Optional[Point2D]:
    if point is None:
        return None
    return (round(float(point[0]), precision), round(float(point[1]), precision))


def format_coord(value: Coord) -> str:
    """Encode a coordinate for use in plan action names (no dots)."""
    x = float(value)
    if math.isclose(x, round(x), rel_tol=0, abs_tol=1e-9):
        return str(int(round(x)))
    text = format(x, ".6g")
    return text.replace(".", "d").replace("-", "m")


def parse_coord(text: str) -> float:
    if text == "None":
        raise ValueError("unexpected None coord")
    if "d" in text or "m" in text or "e" in text or "E" in text:
        return float(text.replace("d", ".").replace("m", "-"))
    if "." in text:
        return float(text)
    return float(int(text))


def parse_point_tokens(tokens: list[str]) -> Point2D:
    if len(tokens) == 2:
        return parse_coord(tokens[0]), parse_coord(tokens[1])
    if len(tokens) % 2 == 0:
        return parse_coord(tokens[0]), parse_coord(tokens[1])
    raise ValueError(f"cannot parse point tokens: {tokens}")


def catch_points_str(points: list[Point2D] | tuple[Point2D, ...]) -> str:
    """Encode multiple grid points for jump action names (flat x_y_x_y_...)."""
    coords: list[Coord] = []
    for point in points:
        coords.extend(point)
    return tuple_to_str(coords)


def parse_catch_points(catch_text: str, *, num_points: int = 3) -> tuple[Point2D, ...]:
    tokens = catch_text.split("_")
    expected = 2 * num_points
    if len(tokens) != expected:
        raise ValueError(
            f"expected {expected} catch coord tokens, got {len(tokens)}: {catch_text!r}"
        )
    return tuple(
        parse_point_tokens(tokens[i : i + 2]) for i in range(0, expected, 2)
    )


def tuple_to_str(parts, sep: str = "_") -> str:
    if parts is None:
        return "None"
    return sep.join(format_coord(p) if p is not None else "None" for p in parts)


def grid_distance(p1: Point2D, p2: Point2D) -> float:
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])


def points_close(p1: Point2D, p2: Point2D, *, tol: float = 1e-6) -> bool:
    return grid_distance(p1, p2) <= tol
