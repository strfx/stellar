"""
Module contains path tracking related components.
"""
import numpy as np


def dist(a, b):
    """
    Euclidian Distance between two points.
    """
    x1, y1 = a
    x2, y2 = b

    return edist(a, b)


def edist(a, b):
    """
    Euclidian Distance between two points.
    """
    x1, y1 = a
    x2, y2 = b
    return np.sqrt(np.square(x2-x1) + np.square(y2-y1))


# HACK: Quick, last-minute hack - remove later.
FLAG = False
HAS_REACHED_TURNING_POINT = False


def cte(robot, reference, t, plt=None):
    """
    Calculate the cross track error (CTE) to reference path.
    """

    global FLAG
    global HAS_REACHED_TURNING_POINT

    # get nearest point out of trajectory reference list
    reference_point = get_nearest_point(robot, reference)

    if HAS_REACHED_TURNING_POINT:
        print("AI OFF")
        return (robot.x - reference_point[0] + reference_point[1] - robot.y)
        # return -1

        # 106 73
        # 119 90
    if float(118) < robot.x < float(120) and float(89) <= robot.y < float(92):
        print("REACHED")
        HAS_REACHED_TURNING_POINT = True

    if robot.x < 150 and not FLAG:
        return (reference_point[0] - robot.x + robot.y - reference_point[1])
    else:
        FLAG = True
    # print(reference_point, robot)
    # return (reference_point[0] - robot.x + reference_point[1] - robot.y)
    return dist((robot.x, robot.y), reference_point)


def get_nearest_point(robot, reference_trajectory):
    r = (robot.x, robot.y)
    a = [edist(k, r) for k in list(reference_trajectory)]
    i = np.argmin(a)

    p1 = reference_trajectory[i]

    return reference_trajectory[i]
