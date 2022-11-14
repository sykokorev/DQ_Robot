import math
import matplotlib.pyplot as plt

from copy import deepcopy

import mathlib.vector as vec
from link.link import Link
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q
from chart.chart import PlotData


if __name__ == "__main__":


    # DH-Parameters from UR Supportwebsite
    DH = {
        'link1': {'a': 0.0, 'alpha': 1.570796327, 'd': 0.1625, 'teta': 5.27089341},
        'link2': {'a': -0.425, 'alpha': 0.0, 'd': 0.0, 'teta': 3.316125579},
        'link3': {'a': -0.3922, 'alpha': 0.0, 'd': 0.0, 'teta': 1.029744259},
        'link4': {'a': 0.0, 'alpha': 1.570796327, 'd': 0.1333, 'teta': 3.473205211},
        'link5': {'a': 0.0, 'alpha': -1.57079633, 'd': 0.0997, 'teta': 2.094395102},
        'link6': {'a': 0.0, 'alpha': 0.0, 'd': 0.0996, 'teta': 1.570796327}
    }

    links = []
    for i, (link, pars) in enumerate(DH.items(), 1):
        pars['name'], pars['number'] = link, i
        links.append(Link(**pars))


    # point_local = [2.0, 1.0, 0.0]
    # cs_local = [0.0, 1.0, 2.0]
    # axis = [0.0, 0.0, 1.0]

    point_local = [-3.0, 4.0, 5.0]
    cs_local = [4.0, 2.0, 6.0]
    axis = [1.0, 0.0, 0.0]

    phi = math.radians(45)
    rot_vector = vec.scalar_vector(scalar=math.sin(phi), vector=axis)
    rot_scalar = math.cos(phi)
    tr_vector = vec.scalar_vector(scalar=1.0, vector=cs_local)

    InitPoint = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), D1=Q(scalar=0.0, vector=point_local))

    Rot = DQ(D0=Q(scalar=rot_scalar, vector=rot_vector), D1=Q(scalar=0.0, vector=[0.0, 0.0, 0.0]))
    Tr = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), D1=Q(scalar=0.0, vector=tr_vector))

    Sum = Tr.mult(Rot)
    Sum.inverse()
    Point = Sum.mult(InitPoint)
    print(Point)

    q0 = Point.Real.q0
    q1 = Point.Real.q1
    q2 = Point.Real.q2
    q3 = Point.Real.q3
    roll = math.atan2(2 * (q0 * q1 + q2 * q3), (1 - 2 * (q1**2 + q2**2)))
    pitch = math.asin(2 * (q0 * q2 - q3 * q1))
    yaw = math.atan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2**2 + q3**2)))
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
    print(f'{roll=}\t{pitch=}\t{yaw=}')
