"""Grid map representing the world as seen by the robot.

Credits to https://github.com/richardos/occupancy-grid-a-star
"""
import numpy as np
import matplotlib.pyplot as plt

from stellar.simulation.data import png_to_ogm


class OccupancyGridMap:

    def __init__(self, world, cell_size, occupancy_threshold=1):
        """
        Creates a new grip map.

        A cell is considered occupied if value >= occupancy_threshold, otherwise
        it is marked as free.

        Args:
            world: A 2D Array with a value of occupancy per cell (between 0 and 1)
            cell_size: Size of a cell in centimeters (cm).
            occupancy_threshold: A threshold to determine whether a cell is occpuied or free.

        """
        self.world = world
        self.cell_size = cell_size
        self.occupancy_threshold = occupancy_threshold

        self.cell_dimensions = world.shape

        # 2D array to mark visited nodes, starting with all zeroes
        # since no cells were visited at the beginning.
        self.visited = np.zeros(self.cell_dimensions, dtype=np.float32)

    def plot(self):
        """Plots the grid map."""
        plt.imshow(self.world, origin='lower', cmap='gray')
        plt.grid(b=True)
        plt.draw()

    @staticmethod
    def from_png(filename: str, cell_size: int):
        """
        Creates a OccupancyGridMap from a png image.

        Args:
            filename: Path to the image file
            cell_size: The image pixel size in centimeters (cm)

        Returns:
            An initialized OccupancyGridMap.

        """
        ogm_data = png_to_ogm(filename, normalized=True)
        ogm_data_arr = np.array(ogm_data)
        #where_0 = np.where(ogm_data_arr == 0)
        #where_1 = np.where(ogm_data_arr == 1)

        #ogm_data_arr[where_0] = 1
        #ogm_data_arr[where_1] = 0

        return OccupancyGridMap(ogm_data_arr, cell_size)
