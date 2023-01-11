"""
    Class Link provides storage of information 
    about link.
    
    Link:
        ---------
        Property:
            d - offset along previous z to common normal;
            teta - angle in radians about previous z, from old x to new x;
            a - length of the common normal. Assuming a revolute joint, this is 
                radius about previous z;
            alpha - angle in raidans about common normal, the old z axis to new z axis;
        ---------
        Method:
            showDetails: return None. Print link's parameters.
"""

from dataclasses import dataclass


@dataclass
class Link:

    def __init__(self, *args):
        self.d = args[0]
        self.teta = args[1]
        self.a = args[2]
        self.alpha = args[3]

    def showDetails(self, link: str='Link'):
        print(f"\t{link} : ",  end="")
        print([round(par, 4) for par in [self.d, self.teta, self.a, self.alpha]])

    def __str__(self):
        return f"\t{round(self.d, 4)}\t{round(self.teta, 4)}\t{round(self.a, 4)}\t"\
            f"{round(self.alpha, 4)}\n"
