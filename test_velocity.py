import math
import matplotlib.pyplot as plt


from mathlib.math import linspace
from mathlib.vector import scalar_vector, cross, magnitude
from chart.chart import PlotData
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q


if __name__ == "__main__":

    teta = math.radians(30)
    point_init = [1.0, 0.0, 0.0]
    displacement = magnitude(vector=point_init)
    axis = [0.0, -1.0, 0.0]
    rot_time = 30
    del_teta = teta / rot_time
    del_t = 1

    QRotYinit = Q(scalar=math.cos(0.0), vector=scalar_vector(scalar=math.sin(0.0), vector=axis))
    QPointinit = Q(vector=point_init)
    QRotY = Q()
    DQTr = DQ(D0=Q(scalar=1.0))

    points = [QPointinit.vector]
    QPoints = [QPointinit]
    velocities = []
    QVelocities = []
    vel = []

    for t in range(1, rot_time+1):

        QRotY.scalar = math.cos(del_teta/2)
        QRotY.vector = scalar_vector(scalar=math.sin(del_teta/2), vector=axis)
        QDif = Q.derivate(q=QRotYinit, delq=QRotY, del_arg=del_t)

        QVel = (QDif.mult(QPointinit).mult(QRotY.conjugate())).addition((QRotY.mult(QPointinit).mult(QDif.conjugate())))
        QPoint = QRotY.mult(QPointinit).mult(QRotY.conjugate())
        QPointinit = QPoint

        points.append(QPoint.vector)
        velocities.append(QVel.vector)
        QPoints.append(QPoint)
        QVelocities.append(QVel)

        DQVel = DQ(D0=Q(scalar=1.0), D1=Q(vector=scalar_vector(scalar=2, vector=QVel.vector)))
        DQTr.D1.vector = scalar_vector(scalar=displacement/2, vector=QPoint.vector)
        DQVelTr = DQTr.mult(DQVel).mult(DQTr.conjugate())
        vel.append([QPoint.vector, DQVelTr.D1.vector])

    for qvel in QVelocities:
        print(qvel)

    for p in points:
        print(*p)
    print()
    for v in vel:
        print(*v[0])
        print(*v[1])
        print()

    plt_data = PlotData(data=points)
    fig, ax = plt_data.plt_3Dgraph()
    for i, point in enumerate(vel):
        if not i % 2:
            plt_data.add_3Dgraph(ax=ax, data=point)

    plt.show()
