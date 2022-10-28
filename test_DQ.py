import math
import matplotlib.pyplot as plt


from mathlib.math import linspace
from mathlib.vector import scalar_vector
from chart.chart import PlotData
from mathlib.dual_quaternion import DualQuaternion as DQ
from mathlib.quaternion import Quaternion as Q



if __name__ == "__main__":
    
    point = [2.0, 0.0, 0.0]
    DQPoint = DQ(D0=Q(scalar=1.0), D1=Q(vector=point))

    time = 10
    teta = math.radians(90)

    del_teta = teta / time
    axis = [0.0, 1.0, 0.0]

    DQRotY = DQ(D0=Q(
        scalar=math.cos(0), vector=scalar_vector(scalar=math.sin(0), vector=axis)
    ), D1=Q())

    current_point = point
    DQcurPoint = DQ(D0=Q(scalar=1.0), D1=Q(vector=current_point))

    points = [current_point]
    velocities = []
    for t in range(time):

        DQRotY_temp = DQRotY
        DQRotY.D0 = Q(scalar=math.cos(del_teta/2), vector=scalar_vector(scalar=math.sin(del_teta/2), vector=axis))
        derivate = DQRotY_temp.D0.derivate(delq=DQRotY.D0, delt=1)
        DQRotY_temp.D0 = derivate
        vel = DQRotY_temp.mult(DQRotY)
        velocities.append(vel)
        DQRotY.normed()
        DQNewPoint = DQRotY.mult(DQcurPoint).mult(DQRotY.quaternion_conjugate())



        DQcurPoint.D1.vector = DQNewPoint.D1.vector
        points.append(DQNewPoint.D1.vector)

    for vel in velocities:
        print(vel)
    
    for point in points:
        print(point)

    plot = PlotData(data=points)
    plot.plt_3Dgraph()

    # plt.show()
