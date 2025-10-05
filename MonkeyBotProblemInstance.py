from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class MonkeyBotProblemInstance:
    gripping_points: List[Tuple[float, float]]
    goal_point: Tuple[float, float]
    init_center: Tuple[float, float]
    init_feet: Tuple[float, float]
