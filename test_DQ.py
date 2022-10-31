from copyreg import dispatch_table
import math
from sys import displayhook
import matplotlib.pyplot as plt


from mathlib.math import linspace
from mathlib.vector import magnitude, scalar_vector, cross
from chart.chart import PlotData
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q



if __name__ == "__main__":

    teta = math.radians(30) # [rad]
    rot_time = 30 # [s]
    del_teta = teta / rot_time # [1/s]
    del_t = 1 # [s]

    point = [1.0, 0.0, 0.0]
    axis = [0.0, -1.0, 0.0]
    displacement = magnitude(point)

    DQRotInit = DQ(
        D0=Q(scalar=math.cos(0.0), vector=scalar_vector(scalar=math.sin(0.0), vector=axis)),
        D1=Q()
    )
    DQRotY = DQ()
    DQInitialPoint = DQ(D0=Q(scalar=1.0), D1=Q(vector=point))

    DQTr = DQ(D0=Q(scalar=1.0), D1=Q())

    points = [DQInitialPoint.D1.vector]
    DQPoints = [DQInitialPoint]

    velocities = []
    DQVelocities = []
    vels = []

    for t in range(1, rot_time+1):

        DQRotY.D0.scalar = math.cos(del_teta/2)
        DQRotY.D0.vector = scalar_vector(scalar=math.sin(del_teta/2), vector=axis)
        DQDif = DQ.derivate(dq=DQRotInit, deldq=DQRotY, del_arg=del_t)

        DQVel = (DQDif.mult(DQInitialPoint).mult(DQRotY.conjugate())).addition((DQRotY.mult(DQInitialPoint).mult(DQDif.conjugate())))
        DQPoint = DQRotY.mult(DQInitialPoint).mult(DQRotY.conjugate())

        DQInitialPoint = DQPoint

        points.append(DQPoint.D1.vector)
        velocities.append(DQVel.D1.vector)
        DQPoints.append(DQPoint)
        DQVelocities.append(DQVel)

        DQVel.D0.scalar = 1.0
        DQTr.D1.vector = scalar_vector(scalar=displacement/2, vector=DQPoint.D1.vector)
        DQVelTr = DQTr.mult(DQVel).mult(DQTr.conjugate())
        vels.append([DQPoint.D1.vector, DQVelTr.D1.vector])

    for dq_v in DQVelocities:
        print(dq_v)
    
    plt_data = PlotData(data=points)
    fig, ax = plt_data.plt_3Dgraph()
    for i, v in enumerate(vels):
        if not i % 2:
            plt_data.add_3Dgraph(ax=ax, data=v)

    plt.show()
