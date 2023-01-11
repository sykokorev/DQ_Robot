import math
import matplotlib.pyplot as plt

import mathlib.vector as vec
from forward_kinematics.link import Link
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q
from chart.chart import PlotData



if __name__ == "__main__":

    DH = {
        'link1': {'a': 0.0, 'alpha': 1.570796327, 'd': 0.1625, 'teta': 5.27089341},
        'link2': {'a': -0.425, 'alpha': 0.0, 'd': 0.0, 'teta': 3.316125579},
        'link3': {'a': -0.3922, 'alpha': 0.0, 'd': 0.0, 'teta': 3.089232776},
        'link4': {'a': 0.0, 'alpha': 1.570796327, 'd': 0.1333, 'teta': 3.473205211},
        'link5': {'a': 0.0, 'alpha': -1.57079633, 'd': 0.0997, 'teta': 2.094395102},
        'link6': {'a': 0.0, 'alpha': 0.0, 'd': 0.0996, 'teta': 1.570796327}
    }

    links = []
    for i, (link, link_pars) in enumerate(DH.items()):
        link_pars['name'], link_pars['number'] = link, i
        links.append(Link(**link_pars))


    RotZ = DQ(D0=Q(), D1=Q())
    RotX = DQ(D0=Q(), D1=Q())
    TrZ = DQ(D0=Q(scalar=1.0), D1=Q())
    TrX = DQ(D0=Q(scalar=1.0), D1=Q())
    position = DQ(D0=Q(scalar=1.0), D1=Q())

    for i, link in enumerate(links):
        TrX.Dual.vector = vec.scalar_vector(scalar=link.a/2, vector=[1.0, 0.0, 0.0])
        TrZ.Dual.vector = vec.scalar_vector(scalar=link.d/2, vector=[0.0, 0.0, 1.0])
        RotZ.Real.scalar = math.cos(link.teta/2)
        RotZ.Real.vector = vec.scalar_vector(scalar=math.sin(link.teta/2), vector=[0.0, 0.0, 1.0])
        RotX.Real.scalar = math.cos(link.alpha/2)
        RotX.Real.vector = vec.scalar_vector(scalar=math.sin(link.alpha/2), vector=[1.0, 0.0, 0.0])
        if not i:
            DQSum = TrZ.mult(RotZ).mult(TrX).mult(RotX)
        else:
            DQSum = DQSum.mult(TrZ).mult(RotZ).mult(TrX).mult(RotX)
        DQposition = DQSum.mult(position).mult(DQSum.conjugate())
        DQorientation = DQSum.mult(position)
        link.CS_position_BRF = DQposition
        link.CS_orientation_BRF = DQorientation

        print(*link.CS_position_BRF.Dual.vector)
        print(*link.euler_angles)
        print(*[math.degrees(angle) for angle in link.euler_angles])
        print()
