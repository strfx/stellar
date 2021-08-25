import io
import queue
import struct
import sys
from math import ceil, floor
from os import path
from typing import List

import numpy as np
from bresenham import bresenham

DIRECTORY = path.dirname(path.abspath(__file__))
sys.path.append(path.dirname(DIRECTORY))

# workaround for autopep8 moving imports to the top.
if 'send_message' not in sys.modules:
    from communication.sender import send_message

picam_sim = None


class SensorArray:
    """An array of sensors."""

    sonar_sensors = [
        ('front',   np.radians(0)),
        ('outter',  np.radians(90)),
        ('inner',   np.radians(-90))
    ]

    def __init__(self, z_max):
        """
        Initialize a new sensor array.

        Args:
            z_max: Maximum distance for the sonar sensors.

        """
        self.z_max = z_max
        self.sonar_opening_angle = np.radians(15)

    def sense(self, world, map_pose):
        """
        Returns current sensor measurements about the world.
        """
        measurements = list()
        for _, sonar_angle in self.sonar_sensors:
            distance = sense_distance(
                world, map_pose, sonar_angle, z_max=self.z_max)
            measurements.append((sonar_angle, distance))

        return measurements


def get_occupied_cell_from_distance(world, pose, distance, angle):
    xr, yr, theta = pose
    angle = theta + angle

    xo = distance * np.cos(angle) + xr
    yo = distance * np.sin(angle) + yr

    return (xo, yo)


def sense_distance(world, position, direction, threshold=0.5, z_max=10):
    """
    Returns distance to nearest obstacle in given direction.

    Args:
        world:      Gridmap representing the world.
        position:   Current robots pose.
        direction:  Direction that the sensor is facing.

    Returns:
        Distance to nearest obstacle in given direction or -1 if
        there was any error.
    """
    xr, yr, theta = position
    angle = theta + direction

    x_max = floor(z_max * np.cos(angle) + xr)
    y_max = floor(z_max * np.sin(angle) + yr)

    # Span bresenham line continually to z_max with given sonar angle.
    # When the first obstacle is detected, break and return the sensed
    # (x, y) coordinates of the object, where the ping reflected.

    # TODO: Check boundaries
    ping_reflected_at_xy = (0, 0)
    for x2, y2 in bresenham(xr, yr, x_max, y_max):
        if world[y2, x2] > threshold:
            ping_reflected_at_xy = (x2, y2)
            break

    if ping_reflected_at_xy == (0, 0):
        return -1
    else:
        x2, y2 = ping_reflected_at_xy
        return np.sqrt((xr - x2)**2 + (yr - y2)**2)


def send_data_to_observatory(data: dict):
    """
    Sends data to observatory.
    """
    send_message('perception/sensors', data)


def approximate_obstacle_position(world, position, direction, z):
    """
    Given a sensor measurement `z`, returns the approximate global
    position of the sensed obstacle.

    Args:
        world:      Gridmap representing the world.
        position:   Robots current pose (x, y, theta).
        direction:  Direction that the sensor is facing.
        z:          Sensor measurement.
    """
    x, y, theta = position
    # distance = sense_distance(world, position, direction, threshold=threshold)
    distance = z
    if distance == -1:
        return (-1, -1)

    # TODO(1) swap values
    directions = {
        '>': (y, x + distance),
        'v': (distance + y, x),
        '^': (y - distance, x),
        '<': (y, x - distance)
    }

    return directions[direction]


class Sensors:
    """
    Continuously read sensor data from the tinyK22.
    """

    def read(self):
        """Reads and decodes sensor signals."""
        blob = self.device.read()
        if blob:
            values = decode_blob(blob, self.fmt)
            self.out_queue.put(values)

    def write_observatory_camera_feed(self):
        import base64
        import json

        from pylon_detection import PylonDetector

        frame = picam_sim.current_frame
        if (frame is None):
            return None

        pylons_found = PylonDetector.find_pylons(frame)
        image_out = PylonDetector.mark_pylons(frame, pylons_found)
        PylonDetector.write_image(
            image_out, "stellar/observatory/debug-ui/current-frame.jpg")

    import datetime
    time_created = datetime.datetime.now()  # temp

    def get_mock_data(self) -> dict:
        """Returns mock data to be sent to observatory."""
        import datetime
        import random

        self.write_observatory_camera_feed()
        return {
            'time': {
                'current': (datetime.datetime.now() - self.time_created).seconds * 1000,
                'best': (datetime.datetime.now() - self.time_created).seconds * 1000
            },
            'battery': 99,
            'map': None,
            'sensorMech': {
                'motor': random.randint(0, 1600),
                'steering': random.randint(-90, 90)
            },
            'sensorElec': {
                'cpu': {
                    'load': [
                        random.randint(0, 100),
                        random.randint(0, 100),
                        random.randint(0, 100),
                        random.randint(0, 100)
                    ],
                    'temp': random.randint(0, 100)
                },
                'ram': random.randint(0, 100)
            }
        }


if __name__ == '__main__':
    import time
    from threading import Thread

    from picam_simulator import PicamSimulator

    sensors = Sensors(None, None)  # temporary
    picam_sim = PicamSimulator("stellar/perception/cv_video_final.mp4")
    Thread(target=picam_sim.run).start()

    while picam_sim.is_running:
        try:
            send_data_to_observatory(sensors.get_mock_data())
        except:
            picam_sim.stop()
            raise
