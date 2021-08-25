
import numpy as np


class Robot:
    """
    Simple model for our robot.
    """

    def __init__(self):
        """
        Initalize the robot with the starting values of (0, 0, 0) for 
        position (x, y) and orientation (theta).
        """
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_distance_covered = 0.0

    def in_goal(self, goal, proximity=0.0, min_distance=-1.0):
        """
        Check whether robot has reached its goal.

        Args:
            goal: x- and y coordinates of the goal position.
            proximity: Proximity to the goal position.
            min_distance: Minimum distance covered by the robot before
                          being eligible for being in goal position.
        """
        if self.total_distance_covered < min_distance:
            return False

        goal_x, goal_y = goal

        x_in_goal = goal_x - proximity <= self.x <= goal_x + proximity
        y_in_goal = goal_y - proximity <= self.y <= goal_y + proximity

        return x_in_goal and y_in_goal

    def pose_in_grid(self, scale):
        """
        Converts the pose to the current position in the gridmap.
        """
        xr = int(self.x / scale)
        yr = int(self.y / scale)
        theta = self.theta

        return (xr, yr, theta)

    def move(self, distance: int, direction: float, max_steering=np.pi / 2):
        """
        Instruct the robot to move.

        Args:
            distance: Number of steps (i.e. cells) to move.
            direction: Direction of movement (in radians)

        Returns:
            The robots new pose (x, y, theta).

        """
        if direction > max_steering:
            direction = max_steering
        if direction < -max_steering:
            direction = -max_steering

        if distance < 0.0:
            distance = 0.0

        self.total_distance_covered += distance

        self.theta = (self.theta + direction) % (2.0 * np.pi)
        self.x = self.x + (np.cos(self.theta) * distance)
        self.y = self.y + (np.sin(self.theta) * distance)

    def set(self, x: float, y: float, theta: float):
        """Change the robots current pose.

        Args:
            x: New x coordinate
            y: New y coordinate
            theta: New theta (orientation) in radians.

        Raises:
            ValueError if the new coordinates or theta is not possible.

        """
        self.x = float(x)
        self.y = float(y)
        self.theta = float(theta)

    def __str__(self):
        return f"Robot[x: {self.x}, y: {self.y}, theta: {self.theta}]"
