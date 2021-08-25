from math import ceil

import numpy as np


def follow_wall(front, left, right):
    """Simple follow wall controller."""
    change_direction = 0

    F = (0 < front < 4)
    L = (0 < left < 4)
    R = (0 < right < 4)

    if 0 < front < 3:
        change_direction = -10
    elif 1.0 <= left <= 2.0:
        # we're good
        change_direction = 0
    elif 0 < left < 1.0:
        change_direction = -10
    elif left > 2.0:
        change_direction = 10

    return change_direction


def move(pose, number_of_steps, direction):
    """Move the robot in the world.

    Args:
        pose:           Robots current pose (x, y, theta)
        number_of_steps:    Number of steps (i.e. cells) to move.
        direction:          Direction of movement

    Returns:
        The new pose of the robot.
    """

    x_r, y_r, theta_r = pose

    theta_r += direction

    x_new = ceil(number_of_steps * np.cos(theta_r)) + x_r
    y_new = ceil(number_of_steps * np.sin(theta_r)) + y_r

    return (x_new, y_new, theta_r)
