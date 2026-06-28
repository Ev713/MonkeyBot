from dataclasses import dataclass, field
from typing import List

from pymunk import Vec2d
from unified_planning.shortcuts import Bool


def empty_control_signal(num_legs: int) -> "ControlSignal":
    return ControlSignal(
        extension=[0.0] * num_legs,
        rotation=[0.0] * num_legs,
        grip=[0] * num_legs,
        angle_lock=[False] * num_legs,
        relax_spring=[False] * num_legs,
        damp_body=False,
    )


@dataclass
class ControlSignal:
    extension:List[float]
    rotation:List[float]
    grip:List[int]
    angle_lock: List[bool] = field(default_factory=list)
    relax_spring: List[bool] = field(default_factory=list)
    damp_body: bool = False

@dataclass
class StateSignal:
    center_pos:Vec2d
    feet_pos:List[Vec2d]
    active_grips:List[Bool]
    t:int
    body_angle:float=0.0
    body_angular_velocity:float=0.0

