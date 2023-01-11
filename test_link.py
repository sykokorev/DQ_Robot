import math
import matplotlib.pyplot as plt
import sys

from forward_kinematics.link import Link
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q
from mathlib.vector import scalar_vector
from chart.chart import PlotData


if __name__ == "__main__":

    """
        DH: dict('link_name': {
            'a': float, 'alpha': float (radians), 'd': float, 'teta': float (radians)
        })
        All DH-Parameters were taken from the web-site https://www.universal-robots.com
        Direct link: https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/
    """
    # UR5e
    # DH = {
    #     'link1': {'a': 0.0, 'alpha': math.pi/2, 'd': 0.1615, 'teta': 0.0},
    #     'link2': {'a': -0.24355, 'alpha': 0.0, 'd': 0.0, 'teta': 0.0},
    #     'link3': {'a': -0.2132, 'alpha': 0.0, 'd': 0.0, 'teta': 0.0},
    #     'link4': {'a': 0.0, 'alpha': math.pi/2, 'd': 0.13105, 'teta': 0.0},
    #     'link5': {'a': 0.0, 'alpha': -math.pi/2, 'd': 0.08535, 'teta': 0.0},
    #     'link6': {'a': 0.0921, 'alpha': 0.0, 'd': 0.0, 'teta': 0.0}
    # }
    # DH-Parameters from UR Supportwebsite
    DH = {
        'link1': {'a': 0.0, 'alpha': 1.570796327, 'd': 0.1625, 'teta': 3.577924967},
        'link2': {'a': -0.425, 'alpha': 0.0, 'd': 0.0, 'teta': 3.316125579},
        'link3': {'a': -0.3922, 'alpha': 0.0, 'd': 0.0, 'teta': 1.640609497},
        'link4': {'a': 0.0, 'alpha': 1.570796327, 'd': 0.1333, 'teta': 3.473205211},
        'link5': {'a': 0.0, 'alpha': -1.57079633, 'd': 0.0997, 'teta': 2.094395102},
        'link6': {'a': 0.0, 'alpha': 0.0, 'd': 0.0996, 'teta': 2.49582083}
    }
    links = []
    for i, (link, dh) in enumerate(DH.items(), 1):
        dh['number'], dh['name'] = i, link
        links.append(Link(**dh))

    DQTrX = DQ(D0=Q(scalar=1.0), D1=Q(vector=[0.0, 0.0, 0.0]))
    DQTrZ = DQ(D0=Q(scalar=1.0), D1=Q(vector=[0.0, 0.0, 0.0]))
    DQRotX = DQ(D0=Q(scalar=0.0, vector=[1.0, 0.0, 0.0]), D1=Q())
    DQRotZ = DQ(D0=Q(scalar=0.0, vector=[0.0, 0.0 ,1.0]), D1=Q())

    for i, link in enumerate(links):
        DQTrX.Dual.vector = [link.a/2, 0.0, 0.0]
        DQTrZ.Dual.vector = [0.0, 0.0, link.d/2]
        DQRotX.Real.scalar = math.cos(link.alpha/2)
        DQRotX.Real.vector = scalar_vector(scalar=math.sin(link.alpha/2), vector=[1.0, 0.0, 0.0])
        DQRotZ.Real.scalar = math.cos(link.teta/2)
        DQRotZ.Real.vector = scalar_vector(scalar=math.sin(link.teta/2), vector=[0.0, 0.0, 1.0])
        # Transformation DQ in a local reference frame
        DQSum = DQTrZ.mult(DQRotZ).mult(DQTrX).mult(DQRotX)
        
        # Compute DQ link coordinate system in a local reference frame
        link.CS_position = DQSum.mult(DQ(D0=Q(scalar=1.0), D1=Q())).mult(DQSum.conjugate())

        # Compute DQ link coordinate system in the base reference frame
        if not i:
            DQSum1 = DQSum
            link.CS_position_BRF = link.CS_position
        else:
            # Transformation DQ in the base referece frame
            DQSum1 = DQSum1.mult(DQSum)
            link.CS_position_BRF = DQSum1.mult(link.CS_position_BRF).mult(DQSum1.conjugate())
            link.CS_orientation_BRF = DQSum1.mult(link.CS_position_BRF)

    # for link in links:
    #     print(f'Link {link.number}')
    #     print(link)
    #     print("### DQ CS position in Local reference Frame ###")
    #     print(link.CS_position)
    #     print("### DQ CS position in Base reference Frame ###")
    #     print(link.CS_position_BRF)

    print("Cartesian coordinates:")
    for link in links:
        print(*link.CS_position_BRF.Dual.vector)
    
    print("Euler angles")
    for link in links:
        print([math.degrees(angle) for angle in link.euler_angles])

    # Plotting Robot links
    data = [
        [link.CS_position_BRF.Dual.vector, links[i+1].CS_position_BRF.Dual.vector]
        for i, link in enumerate(links[:-1])
    ]
    plt_robot = PlotData(data=[[0.0, 0.0, 0.0], links[0].CS_position_BRF.Dual.vector])
    fig, ax = plt_robot.plt_3Dgraph()
    line_color = ['r', 'g', 'b', 'black', 'purple', 'r']
    for color, link in enumerate(data):
        plt_robot.add_3Dgraph(ax=ax, data=link, color=line_color[color])

    plt.show()
