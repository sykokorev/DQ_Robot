import math


import mathlib.vector as vec
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion


class Link:
    def __init__(self, DH: list, length: float=0.0):
        self.__DH = DH
        self.__d = DH[0]
        self.__teta = DH[1]
        self.__a = DH[2]
        self.__alfa = DH[3]

        self.__l = length


    @property
    def DH(self):
        return self.__DH
    
    @property
    def d(self):
        return self.__d

    @property
    def teta(self):
        return self.__teta

    @property
    def a(self):
        return self.__a
    
    @property
    def alfa(self):
        return self.__alfa

    @DH.setter
    def DH(self, DH: list):
        if hasattr(DH, '__iter__'):
            if all([isinstance(val, float) for val in DH]):
                self.__init__(DH=DH)
    
    @property
    def length(self):
        return self.__l

    @d.setter
    def d(self, d: float):
        if isinstance(d, float):
            self.__d = d
            self.__DH[0] = d

    @teta.setter
    def teta(self, teta: float):
        if isinstance(teta, float):
            self.__teta = teta
            self.__DH[1] = teta

    @a.setter
    def a(self, a: float):
        if isinstance(a, float):
            self.__a = a
            self.__DH[2] = a

    @alfa.setter
    def alfa(self, alfa: float):
        if isinstance(alfa, float):
            self.__alfa = alfa
            self.__DH[3] = alfa

    @length.setter
    def length(self, length):
        if isinstance(length, float):
            self.__l = length

    def __repr__(self):
        return f'{self.__class__.__name__}:\nDH:\t' \
            f'[d={round(self.d, 5)}; teta={round(math.degrees(self.teta), 5)}; ' \
                f'a={round(self.a, 5)}; alfa={round(math.degrees(self.alfa), 5)}]'
