"""
    structs lib provides convinient way to store and
    representation of vectors.
        Class Veloctiy stores and shows velocity
        Class EulerAngles stores shows Euler angles
        Class Position stores and shows point coordinates
"""

import math

from dataclasses import dataclass


@dataclass
class Velocity:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    velocity_type: str = "Velocity"

    @property
    def velocity(self) -> list:
        return [self.vx, self.vy, self.vz]

    def showDetails(self) -> None:
        print(f'\t\t{self.velocity_type} [x, y, z] : ', end="")
        print([round(vi, 5) for vi in self.velocity])


@dataclass
class EulerAngles:
    alpha: float = 0.0
    beta: float = 0.0
    gamma: float = 0.0

    @property
    def angles(self) -> list:
        return [self.alpha, self.beta, self.gamma]

    @property
    def deg_angles(self) -> list:
        return [math.degrees(a) for a in self.angles]

    def showDetails(self) -> None:
        print('\t\tEuler Angles : ', end="")
        print(f'alpha={round(math.degrees(self.alpha), 2)}\t'\
                f' beta={round(math.degrees(self.beta), 2)}\t'\
                    f' gamma={round(math.degrees(self.gamma), 2)}')



@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @property
    def point(self) -> list:
        return [self.x, self.y, self.z]

    def showDetails(self) -> None:
        print('\t\tPosition : ', end="")
        print(f'x={round(self.x, 4)}\ty={round(self.y, 4)}\tz={round(self.z, 4)}')