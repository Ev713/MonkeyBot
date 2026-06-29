import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional, Tuple

from monkey_bot.coords import Point2D, as_float, format_coord, grid_distance, normalize_point, parse_coord, parse_point_tokens, tuple_to_str


@dataclass
class MonkeyBotProblemInstance:
    name: str
    max_extension: float
    grid_size_x: float
    grid_size_y: float
    gripping_points: List[Point2D]
    goal_point: Point2D
    init_center: Point2D
    init_feet: List[Optional[Point2D]]
    allowed_jump_configs: Optional[list] = None
    _gp_index: dict[Point2D, Point2D] = field(default_factory=dict, repr=False)

    def __post_init__(self):
        self.max_extension = float(self.max_extension)
        self.grid_size_x = float(self.grid_size_x)
        self.grid_size_y = float(self.grid_size_y)
        self.goal_point = normalize_point(self.goal_point)
        self.init_center = normalize_point(self.init_center)
        self.gripping_points = [normalize_point(p) for p in self.gripping_points]
        self.init_feet = [
            normalize_point(p) if p is not None else None for p in self.init_feet
        ]
        self._gp_index = {gp: gp for gp in self.gripping_points}

    @classmethod
    def from_dict(cls, data: dict) -> "MonkeyBotProblemInstance":
        return cls(
            name=str(data["name"]),
            max_extension=as_float(data["leg_extension"]),
            grid_size_x=as_float(data["grid_size_x"]),
            grid_size_y=as_float(data["grid_size_y"]),
            goal_point=tuple(data["goal_point"]),
            init_center=tuple(data["init_center"]),
            init_feet=[tuple(p) for p in data["init_feet"]],
            gripping_points=[tuple(p) for p in data["gripping_points"]],
            allowed_jump_configs=None,
        )

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "leg_extension": self.max_extension,
            "grid_size_x": self.grid_size_x,
            "grid_size_y": self.grid_size_y,
            "gripping_points": [list(p) for p in self.gripping_points],
            "goal_point": list(self.goal_point),
            "init_center": list(self.init_center),
            "init_feet": [list(p) if p is not None else None for p in self.init_feet],
        }

    def resolve_grip_point(self, point: Point2D) -> Optional[Point2D]:
        key = normalize_point(point)
        return self._gp_index.get(key)

    @classmethod
    def _from_json(cls, s: str) -> "MonkeyBotProblemInstance":
        data = json.loads(s)
        return cls.from_dict(data)

    @classmethod
    def from_json_file(cls, path: Path | str) -> "MonkeyBotProblemInstance":
        with open(path) as f:
            return cls._from_json(f.read())

    def to_json(self, **kwargs) -> str:
        return json.dumps(self.to_dict(), **kwargs)

    def to_json_file(self, path: Path | str, **kwargs):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2, **kwargs)


def load_instance(name: str, instances_folder: str | Path = "instances") -> MonkeyBotProblemInstance:
    path = Path(instances_folder) / f"{name}.json"
    return MonkeyBotProblemInstance.from_json_file(path)


def snapshot_instance(
    base: MonkeyBotProblemInstance,
    *,
    init_center: Point2D,
    init_feet: List[Optional[Point2D]],
    goal_point: Point2D,
) -> MonkeyBotProblemInstance:
    """Return a copy of *base* with updated start state and goal."""
    return MonkeyBotProblemInstance(
        name=base.name,
        max_extension=base.max_extension,
        grid_size_x=base.grid_size_x,
        grid_size_y=base.grid_size_y,
        gripping_points=list(base.gripping_points),
        goal_point=goal_point,
        init_center=init_center,
        init_feet=list(init_feet),
        allowed_jump_configs=base.allowed_jump_configs,
    )


def scale_instance(instance: MonkeyBotProblemInstance, factor: float) -> MonkeyBotProblemInstance:
    """Scale all spatial fields uniformly (coordinates and leg reach)."""
    if factor <= 0:
        raise ValueError("scale factor must be positive")
    return MonkeyBotProblemInstance(
        name=instance.name,
        max_extension=instance.max_extension * factor,
        grid_size_x=instance.grid_size_x * factor,
        grid_size_y=instance.grid_size_y * factor,
        gripping_points=[(x * factor, y * factor) for x, y in instance.gripping_points],
        goal_point=(instance.goal_point[0] * factor, instance.goal_point[1] * factor),
        init_center=(instance.init_center[0] * factor, instance.init_center[1] * factor),
        init_feet=[
            (p[0] * factor, p[1] * factor) if p is not None else None
            for p in instance.init_feet
        ],
        allowed_jump_configs=instance.allowed_jump_configs,
    )


def increase_scale(instance: MonkeyBotProblemInstance, c: float) -> MonkeyBotProblemInstance:
    return scale_instance(instance, c)


def list_instances(instances_folder: str | Path = "instances") -> list[str]:
    return [p.stem for p in Path(instances_folder).glob("*.json")]
