"""

Dubins path planner sample code

author Atsushi Sakai(@Atsushi_twi)

"""
import math
from math import degrees

import matplotlib.patches as mpl
import matplotlib.pyplot as plt
import numpy as np
import Sumo.SumoParameters as sp


class DubinsWord:
    def __init__(self, planner):
        planners = [LSL, RSR, LSR, RSL, RLR, LRL]


def mod2pi(theta):
    return theta - 2.0 * math.pi * math.floor(theta / 2.0 / math.pi)


def pi_2_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def LSL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d + sa - sb

    mode = ["L", "S", "L"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sa - sb))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((cb - ca), tmp0)
    t = mod2pi(-alpha + tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(beta - tmp1)
    #  print(np.rad2deg(t), p, np.rad2deg(q))

    return t, p, q, mode


def RSR(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    tmp0 = d - sa + sb
    mode = ["R", "S", "R"]
    p_squared = 2 + (d * d) - (2 * c_ab) + (2 * d * (sb - sa))
    if p_squared < 0:
        return None, None, None, mode
    tmp1 = math.atan2((ca - cb), tmp0)
    t = mod2pi(alpha - tmp1)
    p = math.sqrt(p_squared)
    q = mod2pi(-beta + tmp1)

    return t, p, q, mode


def LSR(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_squared = -2 + (d * d) + (2 * c_ab) + (2 * d * (sa + sb))
    mode = ["L", "S", "R"]
    if p_squared < 0:
        return None, None, None, mode
    p = math.sqrt(p_squared)
    tmp2 = math.atan2((-ca - cb), (d + sa + sb)) - math.atan2(-2.0, p)
    t = mod2pi(-alpha + tmp2)
    q = mod2pi(-mod2pi(beta) + tmp2)

    return t, p, q, mode


def RSL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    p_squared = (d * d) - 2 + (2 * c_ab) - (2 * d * (sa + sb))
    mode = ["R", "S", "L"]
    if p_squared < 0:
        return None, None, None, mode
    p = math.sqrt(p_squared)
    tmp2 = math.atan2((ca + cb), (d - sa - sb)) - math.atan2(2.0, p)
    t = mod2pi(alpha - tmp2)
    q = mod2pi(beta - tmp2)

    return t, p, q, mode


def RLR(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    mode = ["R", "L", "R"]
    tmp_rlr = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0
    if abs(tmp_rlr) > 1.0:
        return None, None, None, mode

    p = mod2pi(2 * math.pi - math.acos(tmp_rlr))
    t = mod2pi(alpha - math.atan2(ca - cb, d - sa + sb) + mod2pi(p / 2.0))
    q = mod2pi(alpha - beta - t + mod2pi(p))
    return t, p, q, mode


def LRL(alpha, beta, d):
    sa = math.sin(alpha)
    sb = math.sin(beta)
    ca = math.cos(alpha)
    cb = math.cos(beta)
    c_ab = math.cos(alpha - beta)

    mode = ["L", "R", "L"]
    tmp_lrl = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (- sa + sb)) / 8.0
    if abs(tmp_lrl) > 1:
        return None, None, None, mode
    p = mod2pi(2 * math.pi - math.acos(tmp_lrl))
    t = mod2pi(-alpha - math.atan2(ca - cb, d + sa - sb) + p / 2.0)
    q = mod2pi(mod2pi(beta) - alpha - t + mod2pi(p))

    return t, p, q, mode


def dubins_path_planning_from_origin(ex, ey, eyaw, c, D_ANGLE):
    # normalize
    dx = ex
    dy = ey
    D = math.hypot(dx, dy)
    d = D * c
    #  print(dx, dy, D, d)

    theta = mod2pi(math.atan2(dy, dx))
    alpha = mod2pi(- theta)
    beta = mod2pi(eyaw - theta)
    #  print(theta, alpha, beta, d)

    planners = [LSL, RSR, LSR, RSL, RLR, LRL]

    bcost = float("inf")
    bt, bp, bq, bmode = None, None, None, None

    for planner in planners:
        t, p, q, mode = planner(alpha, beta, d)
        if t is None:
            continue

        cost = (abs(t) + abs(p) + abs(q))
        if bcost > cost:
            bt, bp, bq, bmode = t, p, q, mode
            bcost = cost

    #  print(bmode)
    px, py, pyaw = generate_course([bt, bp, bq], bmode, c, D_ANGLE)

    return px, py, pyaw, bmode, bcost, [bt, bp, bq]


# @HelperFcts.timing
def dubins_path_planning(start_node, end_node, curv=sp.MIN_CURVATURE, D_ANGLE=np.deg2rad(10.0)):
    """
    Dubins path plannner

    input:
        sx x position of start point [m]
        sy y position of start point [m]
        syaw yaw angle of start point [rad]
        ex x position of end point [m]
        ey y position of end point [m]
        eyaw yaw angle of end point [rad]
        c curvature [1/m]

    output:
        px
        py
        pyaw
        mode

    """

    new_end_x = end_node.x - start_node.x
    new_end_y = end_node.y - start_node.y

    lex = math.cos(start_node.direction) * new_end_x + math.sin(start_node.direction) * new_end_y
    ley = - math.sin(start_node.direction) * new_end_x + math.cos(start_node.direction) * new_end_y
    leyaw = end_node.direction - start_node.direction

    lpx, lpy, lpyaw, mode, clen, lengths = dubins_path_planning_from_origin(
        lex, ley, leyaw, curv, D_ANGLE)

    # transforms the coordinates and the jaw back to the actual position and rotation
    px = [math.cos(-start_node.direction) * x + math.sin(-start_node.direction)
          * y + start_node.x for x, y in zip(lpx, lpy)]
    py = [- math.sin(-start_node.direction) * x + math.cos(-start_node.direction)
          * y + start_node.y for x, y in zip(lpx, lpy)]
    pyaw = [pi_2_pi(iyaw + start_node.direction) for iyaw in lpyaw]

    return px, py, pyaw, mode, clen, lengths


def generate_course(length, mode, c, D_ANGLE):
    px = [0.0]
    py = [0.0]
    pyaw = [0.0]

    for m, l in zip(mode, length):
        pd = 0.0
        if m == "S":
            d = 1.0 * c
        else:  # turning course
            d = D_ANGLE


        while pd < abs(l - d):

            # print(pd, l)
            p1_x = px[-1]
            p1_y = py[-1]
            px.append(px[-1] + d / c * math.cos(pyaw[-1]))
            py.append(py[-1] + d / c * math.sin(pyaw[-1]))
            p2_x = px[-1]
            p2_y = py[-1]
            # print("DISTANCE: ", math.sqrt((p1_x - p2_x) ** 2 + (p1_y - p2_y) ** 2))

            if m == "L":  # left turn
                pyaw.append(pyaw[-1] + d)
            elif m == "S":  # Straight
                pyaw.append(pyaw[-1])
            elif m == "R":  # right turn
                pyaw.append(pyaw[-1] - d)
            pd += d

        d = l - pd
        px.append(px[-1] + d / c * math.cos(pyaw[-1]))
        py.append(py[-1] + d / c * math.sin(pyaw[-1]))
        if m == "L":  # left turn
            pyaw.append(pyaw[-1] + d)
        elif m == "S":  # Straight
            pyaw.append(pyaw[-1])
        elif m == "R":  # right turn
            pyaw.append(pyaw[-1] - d)
        pd += d

    return px, py, pyaw


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):  # pragma: no cover
    """
    Plot arrow
    """

    if not isinstance(x, float):
        for (ix, iy, iyaw) in zip(x, y, yaw):
            plot_arrow(ix, iy, iyaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width)
        plt.plot(x, y)


def main():
    print("Dubins path planner sample start!!")

    start_x = 1.0  # [m]
    start_y = 1.0  # [m]
    start_yaw = np.deg2rad(45.0)  # [rad]

    end_x = -3.0  # [m]
    end_y = -3.0  # [m]
    end_yaw = np.deg2rad(-45.0)  # [rad]

    curvature = 1.0

    px, py, pyaw, mode, clen = dubins_path_planning(start_x, start_y, start_yaw,
                                                    end_x, end_y, end_yaw, curvature)

    # if show_animation:
    plt.plot(px, py, label="final course " + "".join(mode))

    # plotting
    plot_arrow(start_x, start_y, start_yaw)
    plot_arrow(end_x, end_y, end_yaw)

        #  for (ix, iy, iyaw) in zip(px, py, pyaw):
        #  plot_arrow(ix, iy, iyaw, fc="b")

    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.show()


def test(start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature):

    # NTEST = 1
    #
    # for i in range(NTEST):
    #     start_x = (np.random.rand() - 0.5) * 10.0  # [m]
    #     start_y = (np.random.rand() - 0.5) * 10.0  # [m]
    #     start_yaw = np.deg2rad((np.random.rand() - 0.5) * 180.0)  # [rad]
    #
    #
    #     end_x = (np.random.rand() - 0.5) * 10.0  # [m]
    #     end_y = (np.random.rand() - 0.5) * 10.0  # [m]
    #     end_yaw = np.deg2rad((np.random.rand() - 0.5) * 180.0)  # [rad]
    #
    #     curvature = 1.0 / (np.random.rand() * 5.0)

    px, py, pyaw, mode, clen, lengths = dubins_path_planning(
            start_x, start_y, start_yaw, end_x, end_y, end_yaw, curvature)


    # if show_animation:
    print(mode)
    print(lengths)
    plt.cla()
    # for stopping simulation with the esc key.
    plt.gcf().canvas.mpl_connect('key_release_event',
                    lambda event: [exit(0) if event.key == 'escape' else None])
    plt.plot(px, py, label="final course " + str(mode))

    #  plotting
    plot_arrow(start_x, start_y, start_yaw)
    plot_arrow(end_x, end_y, end_yaw)

    plt.legend()
    plt.grid(True)
    plt.axis("equal")
    plt.xlim(-10, 20)
    plt.ylim(-10, 20)
    plt.pause(1.0)

    print("Test done")


def plot_dubins_curve(word, l_values, start_pos, end_pos, turning_radius):
    fig, ax = plt.subplots()
    ax.plot(start_pos[0], start_pos[1], 'bo')
    ax.plot(end_pos[0], end_pos[1], 'bo')

    cnt = 0
    for primitive in word:
        if primitive == 'S':
            dist = l_values[cnt]
            print(dist)
        elif primitive == 'L':
            gamma = l_values[cnt]
            centre = (end_pos[0], end_pos[1] + turning_radius)
            ax.plot(centre[0], centre[1], 'rx')
            arc = mpl.Arc(centre, turning_radius * 2, turning_radius * 2, 90, 180 - degrees(gamma), 180)
            ax.add_patch(arc)
        elif primitive == 'R':
            alpha = l_values[cnt]
            centre = (start_pos[0], start_pos[1] - turning_radius)
            ax.plot(centre[0], centre[1], 'rx')
            arc = mpl.Arc(centre, turning_radius * 2, turning_radius * 2, 90, 360 - degrees(alpha), 0)
            ax.add_patch(arc)
        cnt += 1

    plt.axis('equal')
    plt.show()




if __name__ == '__main__':
    test()
    main()
