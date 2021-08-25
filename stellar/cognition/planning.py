"""
Contains path planning logic.
"""
import math
import numpy as np
from heapq import heappush, heappop


def heuristics(a, b):
    """Heuristics function using the Euclidian Distance."""
    weight = 1.0

    x1, y1 = a
    x2, y2 = b

    distance = np.sqrt(np.square(x2-x1) + np.square(y2-y1))
    # distance = math.hypot(x1 - x2, y1 - y2)
    return distance


def motion_model_4():
    return [
        [1, 0, 1],
        [0, 1, 1],
        [-1, 0, 1],
        [0, -1, 1],
        [-1, -1, 1],
        [-1, 1, 1],
        [1, -1, 1],
        [1, 1, 1]
    ]


class AStarPlanner:

    def __init__(self):
        pass

    def plan(self, occupancy_grid_map, start_node, goal_node):
        """Plans a path through the occupancy grid map.

        Args:
            occupancy_grid_map: The occupancy grid map.
            start_node: Coordinates of the start node.
            goal_ndoe: Coordinates of the goal node.

        Returns:
            A list of coordinates of the planned path or None, if no path
            could be constructed.

        """
        # Node; Cost to Goal; Node cost, previous node
        start_node_costs = 0
        node_to_goal = heuristics(start_node, goal_node) + start_node_costs
        frontier = [(node_to_goal, start_node_costs, start_node, None)]
        visited = []

        history = {}

        possible_movements = motion_model_4()

        # Safety guard (TODO: Remove after DEV)
        i = 0
        break_if_count_reached = 10000
        while frontier or i >= break_if_count_reached:
            i += 1

            element = heappop(frontier)
            total_cost, cost, position, previous = element

            # If we have already traversed this node (x,y), then skip it
            if position in visited:
                continue

            # Mark this position as visited
            visited.append(position)

            history[position] = previous

            # Have already reached our goal, we can abort.
            if position == goal_node:
                break

            for dx, dy, dcost in possible_movements:
                xn = position[0] + dx
                yn = position[1] + dy

                if xn < 0 or yn < 0:
                    continue

                if (xn, yn) in visited:
                    continue

                if yn >= occupancy_grid_map.shape[0] or xn >= occupancy_grid_map.shape[1]:
                    continue

                # Check if that cell is free!
                cell = occupancy_grid_map[yn][xn]
                if cell <= 0:
                    potential_cost = 0  # abs(cell)  # * 3
                    new_cost = cost + dcost + potential_cost
                    new_total_cost_to_goal = new_cost + \
                        heuristics((xn, yn), goal_node) + potential_cost

                    heappush(
                        frontier, (new_total_cost_to_goal, new_cost, (xn, yn), position))

        path = []
        while position:
            path.append(position)
            position = history[position]

        return list(reversed(path))

    def smoothen(self, occupancy_grid_map, path):
        """Smoothens the planned path.

        Utilizes gradient descent to smoothen the path.


        """
        from copy import deepcopy

        # Create a deep copy of the path
        smoothed_path = deepcopy(path)

        weight_data = 0.01
        weight_smooth = 0.8
        tolerance = 0.0000001

        smoothed_path = [list(elem) for elem in smoothed_path]

        while True:
            # Keep track of the total of changes made to check if we
            # reached convergence
            total_of_changes = 0
            for i in range(len(path)):
                # Do not smoothen start and endpoint
                if i == 0 or i == (len(path) - 1):
                    continue

                for dimension in range(len(path[i])):
                    previous = smoothed_path[i][dimension]
                    smoothed_path[i][dimension] = smoothed_path[i][dimension] + \
                        weight_data * (path[i][dimension] - smoothed_path[i][dimension]) + \
                        weight_smooth * \
                        (smoothed_path[i+1][dimension] + smoothed_path[i-1]
                         [dimension] - 2 * smoothed_path[i][dimension])

                    total_of_changes += abs(previous -
                                            smoothed_path[i][dimension])
            if total_of_changes < tolerance:
                break

        return smoothed_path


def get_nearest_point(robot, aa):
    r = (robot.x, robot.y)
    a = [edist(k, r) for k in list(reference_trajectory)]
    i = np.argmin(a)

    p1 = reference_trajectory[i]
    p2 = reference_trajectory[i+5]

    aaa = np.arctan2(p2[1] - p1[1], p2[0] - p1[0])
    print(f"l => {aaa:.4f}, {p1}, {p2}")
    return reference_trajectory[i]
