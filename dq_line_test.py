import math
import matplotlib.pyplot as plt

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


    point_local = [1.0, 2.0, 0.0]
    cs_local = [0.0, 1.0, 2.0]
    phi = math.radians(90)
    axis = [0.0, 0.0, 1.0]

    Origin = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0 ,0.0]), D1=Q(scalar=0.0, vector=[0.0, 0.0, 0.0]))

    PointLocal = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), D1=Q(scalar=0.0, vector=point_local))
    CSLocal = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), D1=Q(scalar=0.0, vector=cs_local))
    CSRot = DQ(D0=Q(scalar=math.cos(phi), vector=vec.scalar_vector(scalar=math.sin(phi), vector=axis)),
               D1=Q(scalar=0.0, vector=[0.0, 0.0, 0.0]))
    
    # CSLocal = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), 
    #              D1=Q(scalar=0.0, vector=vec.scalar_vector(scalar=0.5, vector=cs_local)))
    # CSRot = DQ(D0=Q(scalar=math.cos(phi/2), vector=vec.scalar_vector(scalar=math.sin(phi/2), vector=axis)),
    #            D1=Q(scalar=0.0, vector=[0.0, 0.0, 0.0]))

    CSLocal.normed()
    CSRot.normed()

    DQSum = CSRot.mult(CSLocal)
    PointBRF = DQSum.mult(PointLocal)
    print(PointBRF)
