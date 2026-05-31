from dataclasses import dataclass
from typing import List

from pymunk import Vec2d
from unified_planning.shortcuts import Bool


@dataclass
class ControlSignal:
    extension:List[float]
    rotation:List[float]
    grip:List[int]

@dataclass
class StateSignal:
    center_pos:Vec2d
    feet_pos:List[Vec2d]
    active_grips:List[Bool]
    t:int
    body_angle:float=0.0
    body_angular_velocity:float=0.0

