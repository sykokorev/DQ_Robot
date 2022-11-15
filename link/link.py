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
    __CS_position_BRF = DQ()
    __CS_orientation_BRF = DQ()

    @property
    def CS_position(self) -> DQ:
        return self.__CS_position

    @CS_position.setter
    def CS_position(self, cs: DQ):
        if isinstance(cs, DQ):
            self.__CS_position = cs
        else:
            print("Wrong argument for CS position in Local reference frame")
    
    @property
    def CS_position_BRF(self):
        return self.__CS_position_BRF

    @CS_position_BRF.setter
    def CS_position_BRF(self, cs: DQ):
        if isinstance(cs, DQ):
            self.__CS_position_BRF = cs
        else:
            print("Wrong argumet for CS position in Base reference frame.")

    @property
    def position_BRF(self):
        return (self.CS_position_BRF.Real.conjugate().mult(self.CS_position_BRF.Dual)).vector

    @property
    def euler_angles(self) -> list:
        q0 = self.CS_position_BRF.Real.q0
        q1 = self.CS_position_BRF.Real.q1
        q2 = self.CS_position_BRF.Real.q2
        q3 = self.CS_position_BRF.Real.q3
        roll = math.atan2(2 * (q0 * q1 + q2 * q3), (1 - 2 * (q1**2 + q2**2)))
        pitch = math.asin(2 * (q0 * q2 - q3 * q1))
        yaw = math.atan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2**2 + q3**2)))
    
        return [roll, pitch, yaw]

    @property
    def rotation_matrix(self):
        q0 = self.CS_position_BRF.Dual.q0
        q1 = self.CS_position_BRF.Dual.q1
        q2 = self.CS_position_BRF.Dual.q2
        q3 = self.CS_position_BRF.Dual.q3
        return [
            [1 - 2 * (q2**2 + q3**2), 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2), 0],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * (q1**2 + q3**2), 2 * (q2 * q3 - q0 * q1), 0],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * (q1**2 + q2**2), 0],
            [0.0, 0.0, 0.0, 1.0]
        ]
