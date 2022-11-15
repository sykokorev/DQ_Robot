import math
import matplotlib.pyplot as plt
import sys

import mathlib.vector as vec
from link.link import Link
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q
from chart.chart import PlotData


if __name__ == "__main__":


    # DH-Parameters from UR Supportwebsite
    DH = {
        'link1': {'a': 0.0, 'alpha': 1.570796327, 'd': 0.1625, 'teta': 2.879793266},
        'link2': {'a': -0.425, 'alpha': 0.0, 'd': 0.0, 'teta': 5.201081171},
        'link3': {'a': -0.3922, 'alpha': 0.0, 'd': 0.0, 'teta': 1.151917306},
        'link4': {'a': 0.0, 'alpha': 1.570796327, 'd': 0.1333, 'teta': 4.328416545},
        'link5': {'a': 0.0, 'alpha': -1.57079633, 'd': 0.0997, 'teta': 0.715584993},
        'link6': {'a': 0.0, 'alpha': 0.0, 'd': 0.0996, 'teta': 5.340707511}
    }

    links = []
    for i, (link, pars) in enumerate(DH.items(), 1):
        pars['name'], pars['number'] = link, i
        links.append(Link(**pars))

    xaxis = [1.0, 0.0, 0.0]
    yaxis = [0.0, 1.0, 0.0]
    zaxis = [0.0, 0.0, 1.0]

    RotZ = DQ(D0=Q(), D1=Q())
    RotX = DQ(D0=Q(), D1=Q())
    TrZ = DQ(D0=Q(scalar=1.0), D1=Q())
    TrX = DQ(D0=Q(scalar=1.0), D1=Q())

    Origin = DQ(D0=Q(scalar=1.0), D1=Q())

    for i, link in enumerate(links):
        trx = [link.a, 0.0, 0.0]
        trz = [0.0, 0.0, link.d]
        scalarx = math.cos(link.alpha/2)
        vectorx = vec.scalar_vector(scalar=math.sin(link.alpha/2), vector=xaxis)
        scalarz = math.cos(link.teta/2)
        vectorz = vec.scalar_vector(scalar=math.sin(link.teta/2), vector=zaxis)

        RotX.Real.vector, RotZ.Real.vector = vectorx, vectorz
        RotX.Real.scalar, RotZ.Real.scalar = scalarx, scalarz
        TrX.Dual.vector, TrZ.Dual.vector = trx, trz

        Sum = TrX.mult(RotX).mult(TrZ).mult(RotZ)
        Sum.normed()
        link.CS_position = Sum.mult(Origin)

        if not i:
            Sum1 = Sum
            Sum1.normed()
            link.CS_position_BRF = link.CS_position
        else:
            Sum1 = Sum.mult(Sum1)
            Sum1.normed()
            link.CS_position_BRF = Origin.mult(Sum1)
    for link in links:
        print(f"{link.name}\tposition [x, y, z]:", end="\t")
        print(*link.position_BRF)
    print("********************************")
    for link in links:
        print(f"{link.name}\tEuler angles", end="\t")
        print(*[round(math.degrees(a), 4) for a in link.euler_angles])
    print("********************************")
    for link in links:
        print(f"{link.name}\t Rotation matrix")
        for row in link.rotation_matrix:
            print([round(n, 4) for n in row])
        print()


    #######################################
    # Test simple Translation then Rotation
    point_local = [3.0, 4.0, 5.0]
    cs_local = [4.0, 2.0, 6.0]
    axis = [1.0, 0.0, 0.0]

    phi = math.radians(180)
    rot_vector = vec.scalar_vector(scalar=math.sin(phi/2), vector=axis)
    rot_scalar = math.cos(phi/2)
    tr_vector = vec.scalar_vector(scalar=1.0, vector=cs_local)

    InitPoint = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), D1=Q(scalar=0.0, vector=point_local))

    Rot = DQ(D0=Q(scalar=rot_scalar, vector=rot_vector), D1=Q(scalar=0.0, vector=[0.0, 0.0, 0.0]))
    Tr = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), D1=Q(scalar=0.0, vector=tr_vector))

    EndPoint = InitPoint.mult(Tr).mult(Rot)
    end_point = (EndPoint.Real.conjugate()).mult(EndPoint.Dual)

    q0 = EndPoint.Real.q0
    q1 = EndPoint.Real.q1
    q2 = EndPoint.Real.q2
    q3 = EndPoint.Real.q3
    roll = math.atan2(2 * (q0 * q1 + q2 * q3), (1 - 2 * (q1**2 + q2**2)))
    pitch = math.asin(2 * (q0 * q2 - q3 * q1))
    yaw = math.atan2(2 * (q0 * q3 + q1 * q2), (1 - 2 * (q2**2 + q3**2)))
    roll = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw = math.degrees(yaw)
