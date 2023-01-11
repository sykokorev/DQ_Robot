import math


from forward_kinematics.link import Link
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion
from mathlib.vector import *


if __name__ == "__main__":

    # DH [d, teta, a, alfa]
    DH = {
        'link1': [0.1625, 0.0, 0.0, math.pi / 2],
        'link2': [0.0, 0.0, -0.425, 0.0],
        'link3': [0.0, 0.0, -0.3922, 0.0],
        'link4': [0.1333, 0.0, 0.0, math.pi / 2],
        'link5': [0.0997, 0.0, 0.0, -math.pi / 2],
        'link6': [0.0996, 0.0, 0.0, 0.0]
    }
    links = []
    for link in DH.values():
        links.append(Link(DH=link))
       

    for i, link in enumerate(links, 1):
        print(f'{i}\t{link}')

    print()

    DQlink2CSTrX = DQ(
        D0=Quaternion(scalar=1.0), D1=Quaternion(
            vector=scalar_vector(scalar=DH['link2'][2], vector=[1.0, 0.0, 0.0])
        )
    )
    DQlink2CSTrZ = DQ(
        D0=Quaternion(scalar=1.0), D1=Quaternion(
            vector=scalar_vector(scalar=DH['link2'][0], vector=[1.0, 0.0, 0.0])
        )
    )
    DQlink2CSRotX = DQ(D0=Quaternion(
                scalar=math.cos(DH['link2'][3]), 
                vector=scalar_vector(scalar=math.sin(DH['link2'][3]), vector=[1.0, 0.0, 0.0])), 
        D1=Quaternion()
    )
    DQlink2CSRotZ = DQ(D0=Quaternion(
                scalar=math.cos(DH['link2'][1]), 
                vector=scalar_vector(scalar=math.sin(DH['link2'][1]), vector=[1.0, 0.0, 0.0])), 
        D1=Quaternion()
    )
    DQlink2CS = DQlink2CSTrX.mult(DQlink2CSTrZ).mult(DQlink2CSRotX).mult(DQlink2CSRotZ)
    print('DQ Position CS of the Link2 in CS of the Link1')
    print(DQlink2CS)

    DQlink1CS = DQ(
        D0=Quaternion(scalar=1.0),
        D1=Quaternion(vector=scalar_vector(scalar=DH['link1'][0]/2, vector=[0.0, 0.0, 1.0]))
    )
    DQlink1CSRotX = DQ(D0=Quaternion(
            scalar=math.cos(DH['link1'][3]/2), 
            vector=scalar_vector(scalar=math.sin(DH['link1'][3]/2), vector=[1.0, 0.0, 0.0])),
        D1=Quaternion()
    )
    DQlink1CSRotZ = DQ(D0=Quaternion(
            scalar=math.cos(DH['link1'][1]/2),
            vector=scalar_vector(scalar=math.sin(DH['link1'][1]/2), vector=[0.0, 0.0, 1.0])),
        D1=Quaternion()
    )

    DQlink2Sum = DQlink1CSRotX.mult(DQlink1CS).mult(DQlink1CSRotZ)
    DQlink2Sum.normed()
    DQlink2CSBase = (DQlink2Sum.conjugate()).mult(DQlink2CS).mult(DQlink2Sum)
    print('DQ Position CS of the Link2 in CS of the Base Reference Frame')
    print(DQlink2CSBase)
