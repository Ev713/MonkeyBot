import math
import typing
from math import pi
from typing import List, Tuple
import shapely

from shapely.geometry import Polygon

class Block:

    polygon: Polygon
    sliding_coeff: float
    is_goal: bool

    def __init__(self, polygon: Polygon, sliding_coeff: float, is_goal: bool):
        self.polygon = polygon
        self.sliding_coeff = sliding_coeff
        self.is_goal = is_goal


class Robot:

    shoulder_angles: List[float]
    arm_extensions: List[float]
    hand_positions: List[Tuple[float, float]]

    def __init__(self):
        self.shoulder_angles = [pi/2, pi*7/6, pi*11/6]
        self.arm_extensions = [1, 1, 1]
        self.hand_positions = self.infer_hand_positions()

    def infer_hand_positions(self) -> list[tuple[float, float]]:
        h_pos = [(math.cos(theta)*r, math.sin(theta)*r)for theta, r in zip(self.shoulder_angles, self.arm_extensions)]
        self.hand_positions = h_pos
        return h_pos

    def set_hand_positions(self, h_pos: List[Tuple[float, float]]) -> tuple[list[float], list[float]]:
        for i, (x, y) in enumerate(h_pos):
            theta = math.atan2(y, x)
            r = math.sqrt(x**2 + y**2)
            if r > 1:
                raise Exception('hand position error')
            self.shoulder_angles[i] = theta
            self.arm_extensions[i] = r
        return self.shoulder_angles, self.arm_extensions


class State:
    robot: Robot
    blocks: List[Block]

    def __init__(self):
        self.robot = Robot()
        self.blocks = []


