import math

from dataclasses import dataclass

from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q


@dataclass
class Link:
    a: float = 0.0
    alpha: float = 0.0
    d: float = 0.0
    teta: float = 0.0
    link_number: int = 1

    initial_position = DQ(D0=Q(), D1=Q())
    end_postion = DQ(D0=Q(), D1=Q())

