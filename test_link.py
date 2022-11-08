import math


from link.link import Link
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q
from mathlib.vector import scalar_vector


if __name__ == "__main__":

    """
        DH: dict('link_name': {
            'a': float, 'alpha': float (radians), 'd': float, 'teta': float (radians)
        })
    """
    DH = {
        'link1': {'a': 0.0, 'alpha': math.pi/2, 'd': 0.1615, 'teta': 0.0},
        'link2': {'a': -0.24355, 'alpha': 0.0, 'd': 0.0, 'teta': 0.0},
        'link3': {'a': -0.2132, 'alpha': 0.0, 'd': 0.0, 'teta': 0.0},
        'link4': {'a': 0.0, 'alpha': math.pi/2, 'd': 0.13105, 'teta': 0.0},
        'link5': {'a': 0.0, 'alpha': -math.pi/2, 'd': 0.08535, 'teta': 0.0},
        'link6': {'a': 0.0921, 'alpha': 0.0, 'd': 0.0, 'teta': 0.0}
    }
    links = []
    for i, (link, dh) in enumerate(DH.items(), 1):
        dh['number'], dh['name'] = i, link
        links.append(Link(**dh))

    for i, link in enumerate(links):
        DQTrZ = DQ(
            D0=Q(scalar=1.0), 
            D1=Q(vector=scalar_vector(scalar=link.d/2, vector=[0.0, 0.0, 1.0]))
        )
        DQRotZ = DQ(
            D0=Q(scalar=math.cos(link.teta/2), 
                vector=scalar_vector(scalar=math.sin(link.teta/2), vector=[0.0, 0.0, 1.0])),
            D1=Q()
        )
        DQTrX = DQ(
            D0=Q(scalar=1.0), 
            D1=Q(vector=scalar_vector(scalar=link.a/2, vector=[1.0, 0.0, 0.0]))
        )
        DQRotX = DQ(
            D0=Q(scalar=math.cos(link.alpha/2), 
                vector=scalar_vector(scalar=math.sin(link.alpha/2), vector=[1.0, 0.0, 0.0])),
            D1=Q()
        )
        DQSum = DQTrZ.mult(DQRotZ).mult(DQTrX).mult(DQRotX)
        link.CS_position = DQSum.mult(DQ(D0=Q(scalar=1.0), D1=Q())).mult(DQSum.conjugate())

    for link in links:
        print(f'Link {link.number}:\t{link.name}')
        print(link.CS_position)
        if link.number == 1:
            DQEndEffector = link.CS_position
        else:
            DQEndEffector = link.CS_position.mult(DQEndEffector)

    print("DQ End Effector")
    print(DQEndEffector)
