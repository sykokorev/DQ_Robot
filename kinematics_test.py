import math
from link.link import *


def show_matrix(m: list):
    for row in m:
        print([round(e, 4) for e in row])


def show_matrices(ms: list):
    for m in ms:
        show_matrix(m)
        print()


def show_euler_angles(angles: EulerAngles):
    print(angles.angles)


def show_arr_euler_angles(angles: list):
    print('alpha, beta, gamma')
    for a in angles:
        show_euler_angles(a)


if __name__ == "__main__":

    # [d, theta, a, alpha]
    links_parameters = [
        [0.1625, math.radians(302), 0.0, math.pi/2],
        [0.0, math.radians(190), -0.425, 0.0],
        [0.0, math.radians(94), -0.3922, 0.0],
        [0.13105, math.radians(199), 0.0, math.pi/2],
        [0.0997, math.radians(120), 0.0, -math.pi/2],
        [0.0996, math.radians(90), 0.0, 0.0]
    ]

    sixAxesRobot = Forwardkinematics("6AXIS")
    for parameter in links_parameters:
        link = Link(*parameter)
        sixAxesRobot.addLink(link)

    sixAxesRobot.setInitialPosition()

    positions = sixAxesRobot.getPositions()
    rotation_matrices = sixAxesRobot.getRotationMatrices()
    euler_angles = sixAxesRobot.getListEulerAngles()

    sixAxesRobot.showDetails()

    # Simple Robot
    print("*"*10, ' SimpleRobot ', "*"*10)

    links_par = [
        [2, math.radians(90.0), 0.0, math.radians(90.0)],
        [1, math.radians(0), 0.0, math.radians(0.0)]
    ]

    simple_robot = Forwardkinematics("SimpleRobot")
    for par in links_par:
        link = Link(*par)
        simple_robot.addLink(link)

    simple_robot.setInitialPosition()
    init = simple_robot.getPositionDQ()
    init_position = simple_robot.getPositions()
    euler_angles = simple_robot.getListEulerAngles()
    print("Before rotation")
    simple_robot.showDetails()

    delta_teta = [math.radians(2.0), math.radians(1.0)]
    delta_t = 0.1

    simple_robot.rotateLinks(delta_teta)
    rotated = simple_robot.getPositionDQ()
    rotated_position = simple_robot.getPositions()

    print("After rotation")
    simple_robot.showDetails()

    angular_velocities = simple_robot.getAngularVelocity(init=init, rotated=rotated, del_t=delta_t)
    baseDQ = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), D1=Q(scalar=0.0, vector=[0.0, 0.0, 0.0]))
    resDQ = DQ()
    lhs = DQ()
    rhs = DQ()
    diffDQ = DQ()
    pointDQ = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), D1=Q(scalar=0.0, vector=[0.0, 0.0, 0.0]))
    pointDQ_ = DQ(D0=Q(scalar=1.0, vector=[0.0, 0.0, 0.0]), D1=Q(scalar=0.0, vector=[0.0, 0.0, 0.0]))

    for i, (idq, rdq, point, rpoint, w) in enumerate(
        zip(init, rotated, init_position, rotated_position, angular_velocities), 1):

        print(f'Link {i}')
        diffDQ = DQ.derivate(dq=idq, deldq=rdq, del_arg=delta_t)
        pointDQ.Dual.vector = point.point
        pointDQ_.Dual.vector = rpoint.point
        resDQ = DQ.derivate(dq=pointDQ, deldq=pointDQ_, del_arg=delta_t)
        print("Velocities [vx, vy, vz] : ", resDQ.Dual.vector)
        print("Angulat velocities [wx, wy, wz] : ", w)
