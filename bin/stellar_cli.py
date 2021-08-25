"""
Stellar CLI.

Used for demo and experimenting purposes.

TODO: The CLI contains a lot of logic, which should be moved ot the
      stellar package. However, this is a university project, and
      will probably never be refactored :)

"""
import argparse
import sys
from enum import Enum
from functools import partial
from time import sleep, time

import numpy as np
from bresenham import bresenham
from roboviz import MapVisualizer
from skimage.transform import resize
from stellar.action import motion
from stellar.cognition import mapping, planning, tracking
from stellar.models.gridmap import OccupancyGridMap
from stellar.models.robot import Robot
from stellar.perception import sensors
from stellar.perception.sensors import SensorArray
from stellar.simulation.data import load_world, png_to_ogm
from tqdm import tqdm

VERSION = "0.0.1"
HEADER = r"""
       _____ _______ ______ _      _               _____
      / ____|__   __|  ____| |    | |        /\   |  __ \
     | (___    | |  | |__  | |    | |       /  \  | |__) |
      \___ \   | |  |  __| | |    | |      / /\ \ |  _  /
      ____) |  | |  | |____| |____| |____ / ____ \| | \ \
     |_____/   |_|  |______|______|______/_/    \_\_|  \_\

     v%s
""" % VERSION
print(65*'=', end='')
print(HEADER, end='')
print(65*'=')


# World parameters
MAP_SIZE_PIXELS = 200
MAP_SIZE_METERS = 20
SPEED_MPS = 0.5


class MODE(Enum):
    WALL_FOLLOW = 1


def store_simulation_data_at_step(step, occupancy_grid_map):
    with open(f"logs/ogm_savefile_{step}.np", 'wb+') as fd:
        np.save(fd, occupancy_grid_map)


def is_in_goal(robot, goal):
    """
    Checks wether robot is in goal.
    """
    xg, yg, tg = goal

    orientation_near_goal = (np.radians(100) <= robot.theta <= np.radians(80))
    x_position_near_goal = (xg - 1 <= robot.x <= xg + 1)
    y_position_near_goal = (yg - 1 <= robot.y <= yg + 1)
    print(robot, goal)
    return (orientation_near_goal and x_position_near_goal and y_position_near_goal)


def simulate_learning_mode(robot: Robot, world: np.ndarray, sensors: SensorArray,
                           scale: float, visualization: MapVisualizer = None):
    """
    Run the learning mode simulation.

    Returns:
        The simulation returns a tuple consisting of the constructed
        occupancy grid map, the robots configuration and the history
        of its positions.

    """
    steer = 0       # Relative change in direction
    step = 0
    occupancy_grid_map = np.zeros(world.shape)
    previous_time = time()
    history = list()

    goal = (2.5, 2.5)
    robot_is_in_goal = partial(robot.in_goal,
                               min_distance=5.0,
                               proximity=0.5)

    while not robot_is_in_goal(goal):
        if visualization is not None:
            if not visualization.display(robot, occupancy_grid_map, mapping.LOG_ODD_MIN, mapping.LOG_ODD_MAX):
                store_simulation_data_at_step(step, occupancy_grid_map)
                exit(0)

        # Calculate distance based on MPS and timedelta
        current_time = time()
        distance_covered = SPEED_MPS * (current_time - previous_time)
        previous_time = current_time

        robot.move(distance_covered, np.radians(steer))

        # Get current position in grid (pixel/cell wise)
        map_scale_meters_per_pixel = scale  # TODO: move to map config?
        map_pose = robot.pose_in_grid(map_scale_meters_per_pixel)
        # Retrieve measurements from ultrasonic sensors
        distance_measurements = sensors.sense(world, map_pose)

        # Update occupancy grid map with new information
        for angle, measurement in distance_measurements:
            occupancy_grid_map = mapping.update_occupancy_map(
                occupancy_grid_map,
                map_pose,
                measurement,
                angle,
                sensors.sonar_opening_angle,
                sensors.z_max
            )

        # Convert sensor measurements back to meters
        front, left, right = [distance * map_scale_meters_per_pixel
                              for _, distance in distance_measurements]

        steer = motion.follow_wall(front, left, right)
        step += 1
        history.append((robot.x, robot.y))

    return (occupancy_grid_map, robot, history)


class Postprocessing:

    @staticmethod
    def connect_pylons(occupancy_grid_map, positions):
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
                occupancy_grid_map[y, x] = mapping.LOG_ODD_MAX
                occupancy_grid_map[y+1, x] = mapping.LOG_ODD_MAX
                occupancy_grid_map[y, x+1] = mapping.LOG_ODD_MAX

        return occupancy_grid_map


def contest(datafile):
    import matplotlib.pyplot as plt
    from matplotlib import pyplot

    # TODO: This should be part of a recorded track
    pylons = [
        (50, 75),
        (75, 150),
        (150, 150),
        (100, 75),
        (60, 50)
    ]

    occupancy_grid_map = np.load(datafile)
    robot = Robot()
    robot.set(25, 25, np.radians(90))

    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS,
                        'StellarAI Visualization', True)
    [viz.show_pylon((x, y)) for x, y in pylons]
    viz.display(robot, occupancy_grid_map,
                mapping.LOG_ODD_MIN, mapping.LOG_ODD_MAX)

    sleep(5)
    pyplot.close('all')

    occupancy_grid_map = Postprocessing.connect_pylons(occupancy_grid_map,
                                                       pylons)

    planner = planning.AStarPlanner()

    # Sample waypoints to direct A*
    robot_history = [
        (40, 100),
        (40, 150),
        (100, 170),
        (160, 175),
        (170, 170),
        (160, 130),
        (145, 100),
        (130, 70),
        (60, 30),
        (25, 25),
    ]

    robot_history = [
        (40, 100),
        (40, 150),
        (160, 165),
        (160, 164),
        (160, 163),
        (165, 140),
        (165, 130),
        (25,  25)
    ]

    should_plot_waypoints = False
    [plt.plot(x, y, marker='x')
     for x, y in robot_history if should_plot_waypoints]

    path = list()
    to_ = None
    from_ = (25, 25)
    for waypoint in robot_history:
        to_ = waypoint
        #print(from_, "=>", to_)
        planned_path = planner.plan(occupancy_grid_map, from_, to_)
        path.extend(planned_path)
        from_ = waypoint

    # path = planner.plan(occupancy_grid_map, (25, 25), robot_history[0])
    # path_second = planner.plan(occupancy_grid_map, path[-1], robot_history[1])
    # path.extend(path_second)

    # Robot
    robot = Robot()
    robot.set(25, 25, np.radians(90))

    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS,
                        'StellarAI Visualization', True)
    viz.display(robot, occupancy_grid_map,
                mapping.LOG_ODD_MIN, mapping.LOG_ODD_MAX)

    from matplotlib import pyplot
    sleep(5)
    pyplot.close('all')

    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS,
                        'StellarAI Visualization', True, reference_trajectory=np.array(path))
    viz.display(robot, occupancy_grid_map,
                mapping.LOG_ODD_MIN, mapping.LOG_ODD_MAX)

    sleep(5)
    pyplot.close('all')

    path = planner.smoothen(occupancy_grid_map, path)
    path_arr = np.array(path)

    #plt.plot(path_arr[:, 0], path_arr[:, 1], 'y')

    #plt.imshow(occupancy_grid_map, origin='lower', cmap='gray')
    #plt.plot(25, 25, marker='o')
    # plt.show(block=True)

    #params, err = twiddle(path, 200)
    #print(params, err)
    #tau_p, tau_d, tau_i = params
    tau_p = 0.005
    tau_d = 1
    tau_i = 0.00001

    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS,
                        'StellarAI Visualization', True, reference_trajectory=np.array(path))
    viz.display(robot, occupancy_grid_map,
                mapping.LOG_ODD_MIN, mapping.LOG_ODD_MAX)
    sleep(5)
    #

    xt, yt, err = run(robot, path, tau_p, tau_d, tau_i,
                      n=500, world=occupancy_grid_map, viz=viz)
    # for x, y in zip(xt, yt):
    # plt.plot(x, y, marker='o')

    # plt.imshow(occupancy_grid_map, origin='lower', cmap='gray')
    #plt.plot(25, 25, marker='o')
    plt.show(block=True)


def run(robot, reference, tau_p, tau_d, tau_i, n=100, speed=1.0, world=None, viz=None):
    """Run the robot simulation."""
    x_trajectory = []
    y_trajectory = []

    reference = np.array(reference)

    previous_crosstrack_error = tracking.cte(robot, reference, 0)
    integral_cte = 0.0
    steer = robot.theta
    err = 0
    for t in range(n):
        if t % 5 == 0:
            print("UPDATE")
            viz.display(robot, world, mapping.LOG_ODD_MIN, mapping.LOG_ODD_MAX)

        crosstrack_error = tracking.cte(robot, reference, t)

        differential_cte = (crosstrack_error - previous_crosstrack_error)
        previous_crosstrack_error = crosstrack_error
        integral_cte += crosstrack_error

        steer = (-tau_p * crosstrack_error - tau_d *
                 differential_cte - tau_i * integral_cte)

        #steer = np.radians(steer)
        print(
            f"[steer: {steer:.4f}\tcte: {crosstrack_error:.4f}, robot: {robot}]")
        # print(steer)
        robot.move(speed, steer)
        err += crosstrack_error

        x_trajectory.append(robot.x)
        y_trajectory.append(robot.y)

    return x_trajectory, y_trajectory, err / n


def twiddle(path, n, tol=0.1):
    p = [0, 0, 0.0]
    dp = [1.0, 1.0, 1.0]
    robot = Robot()
    robot.set(25, 25, np.radians(90))
    x_trajectory, y_trajectory, best_err = run(robot, path, *p, n=n)

    it = 0
    while sum(dp) > tol:
        print("Iteration {}, best error = {}, dp = {}".format(
            it, best_err, sum(dp)))
        for i in range(len(p)):
            p[i] += dp[i]
            robot = Robot()
            robot.set(25, 25, np.radians(90))
            x_trajectory, y_trajectory, err = run(robot, path, *p, n=n)

            if err < best_err:
                best_err = err
                dp[i] *= 1.1
            else:
                p[i] -= 2 * dp[i]

                x_trajectory, y_trajectory, err = run(robot, path, *p, n=n)

                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] += dp[i]
                    dp[i] *= 0.9
        it += 1
    return p, best_err


def main(parcours_filename):

    # Create a MapVisualizer to track the robots behaviour
    viz = MapVisualizer(MAP_SIZE_PIXELS, MAP_SIZE_METERS,
                        'StellarAI Visualization [LEARN]', True)

    # Load preconstructed environment / world
    mapbytes = np.array(png_to_ogm(parcours_filename, normalized=True))

    mapbytes = resize(mapbytes, (MAP_SIZE_PIXELS, MAP_SIZE_PIXELS))
    occupancy_grid_map = np.zeros(mapbytes.shape)

    # Sonar configuration: three sensors are mounted on the roboter
    # each facing a different direction: left, ahead and right. Further,
    # each sonar sensor has an opening angle of 15 degress.
    #
    # Maximum capacity is set to 8 meters.
    z_max = int(4 / viz.map_scale_meters_per_pixel)
    sonar_opening_angle = np.deg2rad(15)
    sensors = SensorArray(z_max)

    # Three sonar sensors are mounted on the robot, each facing another
    # direction, with their respective relative angles to the robots
    # driving direction (-90°, 0°, +90°)
    # sonar_bearing_angles = np.array([(1/2)*np.pi, 0, -(1/2)*np.pi])
    # TODO: sonar_bearing_angles = np.array([np.deg2rad(-90), 0, np.deg2rad(90)])

    # Initialize our robot with an initial pose
    robot = Robot()
    robot.set(2, 2, np.radians(90))

    # Start learning drive
    occupancy_grid_map, robot, history = simulate_learning_mode(
        robot, mapbytes, sensors, viz.map_scale_meters_per_pixel, visualization=viz)

    print("=> Terminated learning mode. Post-processing collected information...")

    # Post-process gathered information
    #   1. Connect pylons to form inner track boundary
    # sampled_pylon_positions = [
    #     (50, 75), (75, 150), (150, 150), (100, 75), (60, 50)]

    # occupancy_grid_map = mapping.connect_pylons(
    #     sampled_pylon_positions, occupancy_grid_map)

    # # 2. Plan path through track
    # path = list()
    # planner = planning.AStarPlanner()
    # _to, _from = None, (robot.x, robot.y)
    # for waypoint in history:
    #     _to = waypoint
    #     print(_from, "=>", _to)
    #     segment = planner.plan(occupancy_grid_map, _from, _to)
    #     path.extend(segment)
    #     _from = waypoint

    #   3. Optimize (i.e. smooth) track

    # Start contest mode


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Stellar CLI')

    subparsers = parser.add_subparsers(dest='mode')
    # Run simulation for learning mode
    learn_mode_args = subparsers.add_parser('learn')
    learn_mode_args.add_argument('--parcours', required=True,
                                 help="Path to parcours image.")

    # Run contest mode from prerecorded data
    contest_mode_args = subparsers.add_parser('contest')
    contest_mode_args.add_argument('--datafile', required=True,
                                   help='Path to the datafile.')

    args = parser.parse_args()

    sleep(5)  # TODO: remove after video

    try:
        if args.mode == 'learn':
            main(args.parcours)
        elif args.mode == 'contest':
            contest(args.datafile)
        else:
            print(f"Unknown mode: {args.mode}. Stopping.")
    except KeyboardInterrupt:
        sys.exit(1)
