import math


from link.link import Link
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion
from mathlib.vector import *


if __name__ == "__main__":

    # x = [1.0, 0.0, 0.0]
    # DQx = DQ(D0=Quaternion(scalar=1.0), D1=Quaternion(vector=x))
    # DQRot = DQ(
    #     D0=Quaternion(scalar=math.cos(math.pi/4), 
    #         vector=scalar_vector(scalar=math.sin(math.pi/4), vector=[0.0, 0.0, 1.0])),
    #     D1=Quaternion()
    # )
    # DQTr = DQ(
    #     D0=Quaternion(scalar=1.0), 
    #     D1=Quaternion(vector=scalar_vector(scalar=1.5/2, vector=[0.0, 0.0, 1.0]))
    # )
    # DQSum = DQTr.mult(DQRot)#.mult(DQTr)
    # x_ = DQSum.mult(DQx).mult(DQSum.conjugate())
    # x_1 = DQSum.conjugate().mult(DQx).mult(DQSum)
    # print(x_)
    # print(x_1)

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

    DQlink1CS = DQ(
        D0=Quaternion(scalar=1.0),
        D1=Quaternion(vector=scalar_vector(scalar=DH['link1'][0]/2, vector=[0.0, 0.0, 1.0]))
    )
    DQlink2CS = DQ(
        D0=Quaternion(scalar=1.0), D1=Quaternion(
            vector=scalar_vector(scalar=DH['link2'][2], vector=[1.0, 0.0, 0.0])
        )
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
    DQlink2CSBase = DQlink2Sum.mult(DQlink2CS).mult(DQlink2Sum.conjugate())
    DQlink2CSBase_ = (DQlink2Sum.conjugate()).mult(DQlink2CS).mult(DQlink2Sum)

    print(DQlink2CSBase)
    print(DQlink2CSBase_)
