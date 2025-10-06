import json
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, cast


@dataclass
class MonkeyBotProblemInstance:
    leg_extension: int
    grid_size_x: int
    grid_size_y: int
    gripping_points: List[Tuple[int, int]]
    goal_point: Tuple[int, int]
    init_center: Tuple[int, int]
    init_feet: List[Tuple[int, int]]

    @classmethod
    def from_dict(cls, data: dict) -> "MonkeyBotProblemInstance":
        return cls(
            leg_extension=int(data["leg_extension"]),
            grid_size_x=int(data["grid_size_x"]),
            grid_size_y=int(data["grid_size_y"]),
            goal_point=cast(Tuple[int, int], tuple(data["goal_point"])),
            init_center=cast(Tuple[int, int], tuple(data["init_center"])),
            init_feet=[cast(Tuple[int, int], tuple(p)) for p in data["init_feet"]],
            gripping_points=[cast(Tuple[int, int], tuple(p)) for p in data["gripping_points"]]
        )


    def to_dict(self) -> dict:
        return {
            "leg_extension": self.leg_extension,
            "grid_size_x": self.grid_size_x,
            "grid_size_y": self.grid_size_y,
            "gripping_points": [list(p) for p in self.gripping_points],
            "goal_point": list(self.goal_point),
            "init_center": list(self.init_center),
            "init_feet": [list(p) for p in self.init_feet],
        }

    @classmethod
    def from_json(cls, s: str) -> "MonkeyBotProblemInstance":
        data = json.loads(s)
        return cls.from_dict(data)

    @classmethod
    def from_json_file(cls, path: Path | str) -> "MonkeyBotProblemInstance":
        with open(path) as f:
            return cls.from_json(f.read())

    def to_json(self, **kwargs) -> str:
        return json.dumps(self.to_dict(), **kwargs)

    def to_json_file(self, path: Path | str, **kwargs):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, **kwargs)
