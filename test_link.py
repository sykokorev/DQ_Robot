import math


from link.link import Link
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q


if __name__ == "__main__":

    """
        DH: dict('link_name': {
            'a': float, 'alpha': float (radians), 'd': float, 'teta': float (radians)
        })
    """
    DH = {
        'link1': {'a': 0.0, 'alpha': math.pi/2, 'd': 0.0, 'teta': 0.0},
        'link2': {'a': -0.24355, 'alpha': 0.0, 'd': 0.0, 'teta': 0.0},
        'link3': {'a': -0.2132, 'alpha': 0.0, 'd': 0.0, 'teta': 0.0},
        'link4': {'a': 0.0, 'alpha': math.pi/2, 'd': 0.13105, 'teta': 0.0},
        'link5': {'a': 0.0, 'alpha': -math.pi/2, 'd': 0.08535, 'teta': 0.0},
        'link6': {'a': 0.0921, 'alpha': 0.0, 'd': 0.0, 'teta': 0.0}
    }
    links = []
    for i, (link, dh) in enumerate(DH.items(), 1):
        links.append(Link(
            a=dh['a'], alpha=dh['alpha'], d=dh['d'], teta=dh['teta'], link_number=i
        ))

    for link in links:
        print(link)
