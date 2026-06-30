"""Mutable runtime debug state for simulator overlays and halt diagnostics."""

from typing import Optional

# (start_xy, end_xy) in simulation/screen physics coordinates
MoveCenterLine = tuple[tuple[float, float], tuple[float, float]]
# (center_xy, radius_px)
AttachTarget = tuple[tuple[float, float], float]

procedure_status: str | None = None
move_center_line: Optional[MoveCenterLine] = None
attach_target: Optional[AttachTarget] = None


def _xy(point) -> tuple[float, float]:
    if hasattr(point, "x") and hasattr(point, "y"):
        return float(point.x), float(point.y)
    return float(point[0]), float(point[1])


def set_move_center_line(start, end) -> None:
    global move_center_line
    sx, sy = _xy(start)
    ex, ey = _xy(end)
    move_center_line = ((sx, sy), (ex, ey))


def clear_move_center_line() -> None:
    global move_center_line
    move_center_line = None


def set_attach_target(center, radius: float) -> None:
    global attach_target
    cx, cy = _xy(center)
    attach_target = ((cx, cy), float(radius))


def clear_attach_target() -> None:
    global attach_target
    attach_target = None


def clear_procedure_overlays() -> None:
    clear_move_center_line()
    clear_attach_target()
