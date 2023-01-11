import math


from forward_kinematics import link, structs, forward_kinematics as fk
from mathlib import dual_quaternion as DQ, quaternion as Q


def show_matrix(m: list):
    for row in m:
        print([round(e, 4) for e in row])


def show_matrices(ms: list):
    for m in ms:
        show_matrix(m)
        print()


def show_euler_angles(angles: structs.EulerAngles):
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

    sixAxesRobot = fk.Forwardkinematics("6AXIS")
    for parameter in links_parameters:
        sixAxesRobot.addLink(link.Link(*parameter))

    sixAxesRobot.setPosition()

    positions = sixAxesRobot.getPositions()
    rotation_matrices = sixAxesRobot.getRotationMatrices()
    euler_angles = sixAxesRobot.getListEulerAngles()

    print("*"*10, f' {sixAxesRobot.robot} ', "*"*10)
    sixAxesRobot.showDetails()

    # Simple Robot
    links_par = [
        [2, math.radians(90.0), 0.0, math.radians(90.0)],
        [1, math.radians(0), 0.0, math.radians(0.0)]
    ]

    simple_robot = fk.Forwardkinematics("SimpleRobot")
    for par in links_par:
        simple_robot.addLink(link.Link(*par))

    simple_robot.setPosition()
    init = simple_robot.getPositionDQ()
    init_position = simple_robot.getPositions()
    euler_angles = simple_robot.getListEulerAngles()

    print("*"*10, f' {simple_robot.robot} ', "*"*10)
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
    baseDQ = fk.Forwardkinematics.fromPointToDQ(point=[0.0, 0.0, 0.0])

    resDQ = DQ.DualQuaternion()
    lhs = DQ.DualQuaternion()
    rhs = DQ.DualQuaternion()
    diffDQ = DQ.DualQuaternion()

    init_points = [point.point for point in init_position]
    rotated_points = [point.point for point in rotated_position]

    linear_velocities = simple_robot.getLinearVelocity(
        init_point=init_points, rotated_point=rotated_points, del_t=delta_t)

    print("Velocities")
    for i, (v, w) in enumerate(
        zip(linear_velocities, angular_velocities), 1):

        print(f'\tLink {i}')
        v.showDetails()
        w.showDetails()
