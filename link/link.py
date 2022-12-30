import math

from dataclasses import dataclass

from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q


@dataclass
class EulerAngles:
    alpha: float = 0.0
    beta: float = 0.0
    gamma: float = 0.0

    @property
    def angles(self):
        return [self.alpha, self.beta, self.gamma]

    @property
    def deg_angles(self):
        return [math.degrees(a) for a in self.angles]

    def showDetails(self):
        print('\t\tEuler Angles : ', end="")
        print(f'alpha={round(math.degrees(self.alpha), 2)}\t'\
                f' beta={round(math.degrees(self.beta), 2)}\t'\
                    f' gamma={round(math.degrees(self.gamma), 2)}')



@dataclass
class Position:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    @property
    def point(self):
        return [self.x, self.y, self.z]

    def showDetails(self):
        print('\t\tPosition : ', end="")
        print(f'x={round(self.x, 4)}\ty={round(self.y, 4)}\tz={round(self.z, 4)}')


# Link Leaf Class
@dataclass
class Link:

    # constructor
    def __init__(self, *args):
        self.d: float = args[0]
        self.theta: float = args[1]
        self.a: float = args[2]
        self.alpha: float = args[3]
        self.parameters = args

    def showDetails(self, link: str='Link'):
        print(f"\t{link} : ",  end="")
        print([round(par, 4) for par in self.parameters])

    def __str__(self):
        return f"\t{round(self.d, 4)}\t{round(self.theta, 4)}\t{round(self.a, 4)}\t"\
            f"{round(self.alpha, 4)}\n"


# Forward Kinematics Composite Class
class Forwardkinematics:

    def __init__(self, *args):
        self.robot = args[0]
        self.__numLinks = None
        self.__links = []

        # Position parameters
        self.__local = []
        self.__base = []

        # Temporal data
        self.__RotX = DQ()
        self.__TrX = DQ(D0=Q(scalar=1.0))
        self.__RotZ = DQ()
        self.__TrZ = DQ(D0=Q(scalar=1.0))
        self.__ResDQ = DQ()

    @property
    def numLinks(self):
        self.__numLinks = len(self.__links) + 1
        return self.__numLinks

    # Setting Robot manipulators
    def addLink(self, link: Link) -> None:
        if isinstance(link, Link):
            self.__links.append(link)

    def removeLink(self, sid: int) -> Link:
        try:
            link = self.__links.pop(sid)
            return link
        except (IndexError, TypeError):
            return None

    def insertLink(self, sid: int, link: Link):
        if isinstance(link, Link):
            try:
                self.__links.insert(sid, link)
            except TypeError:
                pass

    # Representation
    def showDetails(self):
        print(self.robot)
        for i, (link, position, angles) in enumerate(
            zip(self.__links, self.getPositions(), self.getListEulerAngles()), 1):
            link.showDetails(link=f'Link{i}')
            position.showDetails()
            angles.showDetails()

    # Setting up Robot Initial Position
    def setInitialPosition(self):
        for i, link in enumerate(self.__links):
            self.__RotX.Real.scalar = math.cos(link.alpha/2)
            self.__RotX.Real.vector = [math.sin(link.alpha/2), 0.0, 0.0]
            self.__TrX.Dual.vector = [link.a, 0.0, 0.0]
            self.__RotZ.Real.scalar = math.cos(link.theta/2)
            self.__RotZ.Real.vector = [0.0, 0.0, math.sin(link.theta/2)]
            self.__TrZ.Dual.vector = [0.0, 0.0, link.d]

            self.__ResDQ = self.__TrX.mult(self.__RotX).mult(self.__TrZ).mult(self.__RotZ)
            self.__local.append(self.__ResDQ)

            if not i:
                self.__base.append(self.__ResDQ)
            else:
                self.__base.append(self.__ResDQ.mult(self.__base[i-1]))

    # Getter methods
    def getLink(self, sid: int) -> Link:
        try:
            return self.__links[sid]
        except (IndexError, TypeError):
            return None

    def getLinks(self) -> list:
        return self.__links

    def getPosition(self, sid: int) -> Position:
        try:
            tmpQ = self.__base[sid].Real.conjugate()
            tmpQ = tmpQ.mult(self.__base[sid].Dual)
            position = Position(*tmpQ.vector)
            return position
        except (IndexError, TypeError):
            return None

    def getPositions(self) -> list:
        positions = []
        for i, link in enumerate(self.__links):
            positions.append(self.getPosition(i))
        
        return positions

    def getRotationMatrix(self, sid: int) -> int:
        try:
            return self.__base[sid].DQtoRotMatrix()
        except (TypeError, IndexError):
            return None

    def getRotationMatrices(self) -> list:
        matrices = []
        for i, link in enumerate(self.__links):
            matrices.append(self.getRotationMatrix(i))
        return matrices

    def getEulerAngles(self, sid: int) -> EulerAngles:
        try:
            R = self.getRotationMatrix(sid)
            euler_angles = EulerAngles()
            sy = (R[0][0] ** 2 + R[1][0] ** 2) ** 0.5
            euler_angles.beta = math.atan2(sy, (-1) * R[2][0])
            euler_angles.alpha = math.atan2(R[0][0] / math.cos(euler_angles.beta),
                                            R[1][0] / math.cos(euler_angles.beta))
            if sy > 1e-6:
                euler_angles.gamma = math.atan2(R[2][2] / math.cos(euler_angles.beta),
                                                R[2][1] / math.cos(euler_angles.beta))
            else:
                euler_angles.gamma = 0.0
            return euler_angles
        except (TypeError, IndexError) as ex:
            return None

    def getListEulerAngles(self) -> list:
        angles = []
        for i, link in enumerate(self.__links):
            angles.append(self.getEulerAngles(i))
        return angles

    def __repr__(self) -> str:
        strout = f'{self.robot} : ['
        for link in self.__links[:-1]:
            strout += str(link.parameters) + ','
        strout += str(self.__links[-1].parameters) + ']\n'
        return strout
