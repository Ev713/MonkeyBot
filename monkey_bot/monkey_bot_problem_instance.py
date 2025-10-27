import json
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple, cast


@dataclass
class MonkeyBotProblemInstance:
    name:str
    max_extension: int
    grid_size_x: int
    grid_size_y: int
    gripping_points: List[Tuple[int, int]]
    goal_point: Tuple[int, int]
    init_center: Tuple[int, int]
    init_feet: List[Tuple[int, int]]
    allowed_jump_configs: List

    @classmethod
    def from_dict(cls, data: dict) -> "MonkeyBotProblemInstance":
        return cls(
            name = str(data["name"]),
            max_extension=int(data["leg_extension"]),
            grid_size_x=int(data["grid_size_x"]),
            grid_size_y=int(data["grid_size_y"]),
            goal_point=cast(Tuple[int, int], tuple(data["goal_point"])),
            init_center=cast(Tuple[int, int], tuple(data["init_center"])),
            init_feet=[cast(Tuple[int, int], tuple(p)) for p in data["init_feet"]],
            gripping_points=[cast(Tuple[int, int], tuple(p)) for p in data["gripping_points"]],
            allowed_jump_configs = None
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
            "init_feet": [list(p) for p in self.init_feet],
        }

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
            json.dump(self.to_dict(), f, **kwargs)

def load_instance(name: str, instances_folder: str | Path | None = None) -> MonkeyBotProblemInstance:
    path = Path(instances_folder) / f"{name}.json"
    return MonkeyBotProblemInstance.from_json_file(path)

def increase_scale(instance:MonkeyBotProblemInstance, c):
    instance.grid_size_x *= c
    instance.grid_size_y *= c
    instance.gripping_points = [(x*c, y*c) for x, y in instance.gripping_points ]
    instance.init_center = (instance.init_center[0]*c, instance.init_center[1]*c)
    instance.goal_point = (instance.goal_point[0]*c, instance.goal_point[1]*c)
    instance.max_extension *= c
    instance.init_feet = [(x*c, y*c) for x, y in instance.init_feet ]
    return instance

def list_instances(instances_folder: str | Path | None = None) -> list[str]:
    return [p.stem for p in Path(instances_folder).glob("*.json")]
