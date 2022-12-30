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
