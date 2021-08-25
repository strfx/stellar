from typing import Tuple

import numpy as np
import png
from skimage.transform import resize


def load_world(filename: str, size: Tuple[int, int], resolution: int) -> np.array:
    """Load a preconstructred track to initialize world.

        Args:
            filename:   Full path to the track file (png).
            size:       Width and height of the map
            resolution: Resolution of the grid map (i.e. into how many cells)
                        one meter is divided into.

        Returns:
            An initialized gridmap based on the preconstructed track as
            an n x m dimensional numpy array, where n is the width (num cells)
            and m the height (num cells) - (after applying resolution).

    """
    width_in_cells, height_in_cells = np.multiply(size, resolution)

    world = np.array(png_to_ogm(
        filename, normalized=True, origin='lower'))

    # If the image is already in our desired shape, no need to rescale it
    if world.shape == (height_in_cells, width_in_cells):
        return world

    # Otherwise, scale the image to our desired size.
    resized_world = resize(world, (width_in_cells, height_in_cells))

    return resized_world


def png_to_ogm(filename, normalized=False, origin='lower'):
    """Convert a png image to occupancy grid map.

    Inspired by https://github.com/richardos/occupancy-grid-a-star

    Args:
        filename:   Path to the png file.
        normalized: Whether to normalize the data, i.e. to be in value range [0, 1]
        origin:     Point of origin (0,0)

    Returns:
        2D Array

    """
    r = png.Reader(filename)
    img = r.read()
    img_data = list(img[2])

    out_img = []
    bitdepth = img[3]['bitdepth']

    for i in range(len(img_data)):
        out_img_row = []
        for j in range(len(img_data[0])):
            if j % img[3]['planes'] == 0:
                if normalized:
                    out_img_row.append(img_data[i][j]*1.0/(2**bitdepth))
                else:
                    out_img_row.append(img_data[i][j])

        out_img.append(out_img_row)

    if origin == 'lower':
        out_img.reverse()

    return out_img
