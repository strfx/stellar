"""
Contains implementations of the mapping algorithms.
"""
from math import floor

import numpy as np
from bresenham import bresenham

from stellar.perception.sensors import get_occupied_cell_from_distance


LOG_ODD_MAX = 5
LOG_ODD_MIN = -2.5

LOG_ODD_OCCU = 1
LOG_ODD_FREE = 0.4


def connect_pylons(positions, occupancy_grid_map):
    """
    Connects pylons with lines, i.e. sets all pixels on that line to occupied.
    """
    for index, position in enumerate(positions):
        next_index = (index + 1) % len(positions)
        next_element = positions[next_index]

        x1 = int(position[0])
        y1 = int(position[1])
        x2 = int(next_element[0])
        y2 = int(next_element[1])
        for x, y in bresenham(x1, y1, x2, y2):
            occupancy_grid_map[y, x] = LOG_ODD_MAX
            occupancy_grid_map[y+1, x] = LOG_ODD_MAX
            occupancy_grid_map[y, x+1] = LOG_ODD_MAX

    return occupancy_grid_map


def inverse_range_sensor_model(cell, pose, relative_sensor_angle, beta, z_max, z_t):
    """
    Args:
        cell: Cell m_i
        pose: A tuple of the robot pose, consisting of x, y and theta.
        relativ_sensor_angle: Angle of the sensor relative to the roboter
        beta: Opening angle of range sensor
        z_max: Maximum range of sensor
        z_t: Sensor measurement at time t

    References:
        * http://se.inf.ethz.ch/courses/2015b_fall/rpl/lectures/09_LocalizationMapping.pdf
        * Probabilistic Robotics (Thrun S. et all), Page 230

    Returns:
        An integer indicating if the cells state is unknown (0), occupied (1) or free(-1).

    """
    alpha = 2  # obstacle thickness
    xi, yi = cell  # m_i
    xr, yr, theta_r = pose

    sonar_theta = relative_sensor_angle
    k = 0  # Since we only have 1 sonar in this direction, ignoring overlaps

    # r: Distance between studied cell to the robot
    r = np.sqrt(np.square(xi-xr) + np.square(yi-yr))
    # phi: Angle between studied cell and the robot
    phi = np.arctan2(yi - yr, xi - xr) - theta_r

    if phi >= np.pi:
        phi -= 2 * np.pi

    elif phi <= -np.pi:
        phi += 2 * np.pi

    if r > min(z_max, z_t + alpha / 2) or (np.abs(phi - sonar_theta) > (beta / 2)):
        return 0
    elif (z_t < z_max) and (np.abs(r - z_t) < alpha / 2):
        return 1
    else:  # r <= z_t
        return -1


def update_occupancy_map(gridmap, pose, measurement, sonar_bearing_angle, sonar_opening_angle, z_max):
    """Update occupancy grid map with new measurement.

    Args:
        gridmap: Occupancy grid map to update (2D)
        pose: Robots current pose
        measurement: Distance measurement from sonar
        sonar_bearing_angle: Bearing angle of the sonar, relative to robot (rad).
        sonar_opening_angle: Opening angle of the sonar (rad).
        z_max: Maximum range of the sonar.


    Returns:
        An updated version of the occupancy grid map.

    """

    if measurement == -1:
        measurement = z_max

    x_r, y_r, theta_r = pose
    beam_angle = (theta_r + sonar_bearing_angle)
    beam_angle = (theta_r + sonar_bearing_angle) % (2 * np.pi)

    if beam_angle > np.pi:
        beam_angle -= 2 * np.pi

    if beam_angle < -np.pi:
        beam_angle += 2 * np.pi

    if measurement == 0:
        measurement = z_max

    B = get_occupied_cell_from_distance(gridmap,
                                        pose, measurement + 5, sonar_bearing_angle - np.deg2rad(10))
    C = get_occupied_cell_from_distance(gridmap,
                                        pose, measurement + 5, sonar_bearing_angle + np.deg2rad(10))

    max_x, min_x, max_y, min_y = fov_bounding_box(pose, B, C)

    # gridmap[min_y:max_y, min_x:max_x] = 10
    # print(max_y, min_y)
    if pose[1] <= gridmap.shape[0] and pose[0] <= gridmap.shape[1]:
        gridmap[pose[1], pose[0]] -= LOG_ODD_FREE
    for y in range(min_y, min(max_y, gridmap.shape[0])):
        for x in range(min_x, min(max_x, gridmap.shape[1])):
            p = inverse_range_sensor_model(
                (x, y),
                pose,
                sonar_bearing_angle,
                sonar_opening_angle,
                z_max,
                measurement)

            if p == -1:
                gridmap[y, x] -= LOG_ODD_FREE

            if p == 1:
                gridmap[y, x] += LOG_ODD_OCCU

            if p == 0:
                pass

    return np.clip(gridmap, a_max=LOG_ODD_MAX, a_min=LOG_ODD_MIN)


def fov_bounding_box(pose, B, C):
    """Calculate a bounding box around the sonar cone.

    Args:
        pose:   Robots current pose.
        B:
        C:

    Returns:
        A tuple containing the maximum and mininum value
        for the x- and y-axis.

    """
    A = (pose[0], pose[1])
    max_x = floor(max(A[0], max(B[0], C[0])))
    min_x = floor(min(A[0], min(B[0], C[0])))
    max_y = floor(max(A[1], max(B[1], C[1])))
    min_y = floor(min(A[1], min(B[1], C[1])))

    return (max_x, min_x, max_y, min_y)
