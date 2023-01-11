import copy
import math


from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q
from forward_kinematics.link import Link
from forward_kinematics import structs


"""
    class ForwardKinematics provides common methods for getting coordinates,
    Euler angles, rotation matrix, angular and linear velocities of the links.
    --------
    Property:
        numLinks - int. Number of robot's links
        resultDQ - list of DualQuaternion. List of resulting transformation dual quaternion.
    ------
    Method:
        addLink(Link) - return None. Adding dataclass Link into a robot
        removeLink(sid: int) - return None. Remove Link with index sid from a robot
        insertLink(sid: int, link: Link) - return None. Insert Link 
                                           into a Robot at sid position
        setPosition - return None. Setting self.__base (list of resulting Dual 
                      Quaternions respect to Base Reference Frame), self.__ResDQs same as self.__base,
                      self.__local list of transformation Dual Quaternions in local reference frame
    ----------------
    Getting mehtods:
        getLink(sid: int) - return Link dataclass. Get link at sid index
        getLinks() - return list of Link dataclass.
        getPosition(sid: int) - return Position dataclass. Get current postion of link at index sid
        getpositions() - return list of Position dataclass. Get list of current positions of each link
        getRotationMatrix(sid: int) - return list. Get rotation matrix of link at sid position
        getRotationsMatrices() - return list. Get list of Rotation matrices of each link
        getEulerAngle(sid: int) - return EulerAngles dataclass. Get current Euler angles of link at sid index
        getListEulerAngles() - return list of EulerAngles dataclass. Get list of Euler angles of each link
        getAngularVelocity(init: list, rotated: list, del_t: float) - return list of Velocity dataclass. 
                        Get list of angular velocity of each link
        getLinearVelocity(init_point: list, rotated_point: list, del_t: float) - retutn list of Velocity dataclass. 
                        Get list of linear velocity of each link
    -----------------
    Operation methods:
        rotateLink(sid: int, delta_teta: float) - return None. Rotate link at sid position and setPosition.
                        delta_teta is a rotation angle about z axis of local reference frame;
        roatateLinks(delta_teta: list) - return None. Rotate links. delta_teta list of rotation angles
                        about z axis of local reference frame;
"""

class Forwardkinematics:

    def __init__(self, *args):
        self.robot = args[0]
        self.__numLinks = None
        self.__links = []
        self.__baseDQ = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), D1=Q(scalar=0.0, vector=[0.0, 0.0, 0.0]))

        # Position parameters
        self.__local = []
        self.__base = []
        self.__ResDQs = []

        # Temporal data
        self.__RotX = DQ()
        self.__TrX = DQ(D0=Q(scalar=1.0))
        self.__RotZ = DQ()
        self.__TrZ = DQ(D0=Q(scalar=1.0))
        self.__ResDQ = DQ()

    @property
    def numLinks(self) -> int:
        self.__numLinks = len(self.__links) + 1
        return self.__numLinks

    @property
    def resultDQ(self) -> list:
        return self.__ResDQs

    # Set up Robot manipulators
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

    # Some usefule methods
    @staticmethod
    def fromPointToDQ(point: list) -> DQ:
        if hasattr(point, '__iter__'):
            return DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), D1=Q(scalar=0.0, vector=point))
        else:
            return None

    @staticmethod
    def setRotationDQ(angle: float, axis: list) -> DQ:
        if isinstance(angle, (int, float)) and hasattr(axis, '__iter__'):
            return DQ(D0=Q(scalar=math.cos(angle/2), vector=[math.sin(angle/2) * c for c in axis]),
                      D1=Q(scalar=0.0, vector=[0.0, 0.0, 0.0]))
        else:
            return None

    @staticmethod
    def setTranslateDQ(distance: float, axis: list) -> DQ:
        if isinstance(distance, (int, float)) and hasattr(axis, '__iter__'):
            return DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]),
                      D1=Q(scalar=0.0, vector=[a * distance/2 for a in axis]))

    # Representation
    def showDetails(self):
        print(self.robot)
        for i, (link, position, angles) in enumerate(
            zip(self.__links, self.getPositions(), self.getListEulerAngles()), 1):
            link.showDetails(link=f'Link{i}')
            position.showDetails()
            angles.showDetails()

    # Setting up Robot Initial Position
    def setPosition(self):
        self.__base.clear()
        self.__ResDQs.clear()
        self.__local.clear()
        for i, link in enumerate(self.__links):
            self.__RotX.Real.scalar = math.cos(link.alpha/2)
            self.__RotX.Real.vector = [math.sin(link.alpha/2), 0.0, 0.0]
            self.__TrX.Dual.vector = [link.a/2, 0.0, 0.0]
            self.__RotZ.Real.scalar = math.cos(link.teta/2)
            self.__RotZ.Real.vector = [0.0, 0.0, math.sin(link.teta/2)]
            self.__TrZ.Dual.vector = [0.0, 0.0, link.d/2]

            self.__ResDQ = self.__TrX.mult(self.__RotX).mult(self.__TrZ).mult(self.__RotZ)
            self.__local.append(self.__ResDQ)

            if not i:
                self.__base.append(self.__ResDQ)
                self.__ResDQs.append(self.__ResDQ)
            else:
                self.__ResDQs.append(self.__ResDQ.mult(self.__ResDQs[i-1]))
                self.__base.append(self.__ResDQ.mult(self.__base[i-1]))

    # Getter methods
    def getLink(self, sid: int) -> Link:
        try:
            return self.__links[sid]
        except (IndexError, TypeError):
            return None

    def getLinks(self) -> list:
        return self.__links

    def getPositionDQ(self) -> list :
        return copy.copy(self.__ResDQs)

    def getPosition(self, sid: int) -> structs.Position:
        try:
            tmpDQ = self.__ResDQs[sid].conjugate()
            tmpDQ = tmpDQ.mult(self.__baseDQ).mult(self.__ResDQs[sid])
            position = structs.Position(*tmpDQ.Dual.vector)
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

    def getEulerAngles(self, sid: int) -> structs.EulerAngles:
        try:
            R = self.getRotationMatrix(sid)
            euler_angles = structs.EulerAngles()
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

    def getAngularVelocity(self, init: list, rotated: list, del_t: float) -> list:
        
        """
            Parameters:
            ----------
                init - list of DualQuaternion class. Initial resulting transformation
                       dual quaternion.
                rotated - list of DualQuaternion class. Resulting transformation dual quaternion
                        after transformation.
                del_t - delta time
            Return
            ------
                List of Velocity dataclass
        """

        res = []
        for initDQ, rotatedDQ in zip(init, rotated):
            initDQ.normed()
            rotatedDQ.normed()
            vel = structs.Velocity(*[
                (2 / del_t) * (initDQ.Real.w * rotatedDQ.Real.q1 - initDQ.Real.q1 * rotatedDQ.Real.w - \
                    initDQ.Real.q2 * rotatedDQ.Real.q3 + initDQ.Real.q3 * rotatedDQ.Real.q2),
                (2 / del_t) * (initDQ.Real.w * rotatedDQ.Real.q2 + initDQ.Real.q1 * rotatedDQ.Real.q3 - \
                    initDQ.Real.q2 * rotatedDQ.Real.w - initDQ.Real.q3 * rotatedDQ.Real.q1),
                (2 / del_t) * (initDQ.Real.w * rotatedDQ.Real.q3 - initDQ.Real.q1 * rotatedDQ.Real.q2 + \
                    initDQ.Real.q2 * rotatedDQ.Real.q1 - initDQ.Real.q3 * rotatedDQ.Real.w)
            ])
            vel.velocity_type = "Angulat Velocity"
            res.append(vel)
        
        return res

    def getLinearVelocity(self, init_point: list, rotated_point: list, del_t: float) -> list:
        """
            Parameters:
            -----------
                init_point - list of points ([x, y, z]) of initial position of each link
                rotated_point - list of point ([x, y, z]) of each link after transformation
            Return
            ------
                List of Velocity dataclass
        """

        res = []
        dq = self.fromPointToDQ(point=[0.0, 0.0, 0.0])
        dq_ = self.fromPointToDQ(point=[0.0, 0.0, 0.0])
        for init, rotated in zip(init_point, rotated_point):
            dq.Dual.vector = init
            dq_.Dual.vector = rotated
            vel = structs.Velocity(*DQ.derivate(dq=dq, deldq=dq_, del_arg=del_t).Dual.vector)
            vel.velocity_type = "Linear Velocity"
            res.append(vel)
        
        return res

    def getLinearAcceleration(self, init: list, rotated: list, del_t: float) -> list:
        pass

    def getAngularAcceleration(self, init: list, rotated: list, del_t: float) -> list:
        pass

    # Robot operations
    def rotateLink(self, sid: int, delta_teta: float) -> None:
        self.__links[sid].teta = delta_teta + self.__links[sid].teta
        self.setPosition()

    def rotateLinks(self, delta_teta: list) -> None:
        for i, link in enumerate(self.__links):
            self.__links[i].teta = link.teta + delta_teta[i]
        self.setPosition()

    def __repr__(self) -> str:
        strout = f'{self.robot} : ['
        for link in self.__links[:-1]:
            strout += str(link.parameters) + ','
        strout += str(self.__links[-1].parameters) + ']\n'
        return strout
