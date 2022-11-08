import math

from dataclasses import dataclass

from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q
from mathlib.vector import scalar_vector


@dataclass
class Link:
    a: float = 0.0
    alpha: float = 0.0
    d: float = 0.0
    teta: float = 0.0
    number: int = 1
    name: str = 'link',
    joint_type: str = 'rotation'

    __CS_position = DQ()

    @property
    def CS_position(self) -> DQ:
        return self.__CS_position

    @CS_position.setter
    def CS_position(self, cs: DQ):
        if isinstance(cs, DQ):
            self.__CS_position = cs
        else:
            print('CS_postion wron parameter')