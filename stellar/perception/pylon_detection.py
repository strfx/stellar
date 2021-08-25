import math
from typing import List

import cv2
import numpy as np

DO_CREATE_DEBUG_IMAGES = True

# The maximum size of the image to be processed.
# Bigger images will be shrunk to match the size.
# Smaller images will be left as is.
MAX_IMAGE_SIZE = 300

# How small a pylon can be before detection decides to ignore it.
# Size is relative to resized image and must be scaled up when compared to original image.
MIN_PYLON_SIZE = MAX_IMAGE_SIZE / 10

# The range of width/height ratio a pylon can have.
PYLON_RATIO_RANGE = (0.5, 0.65)

# How far two pixels can be to be considered neighbors.
# Higher values increase computing time.
COLOR_SEGREGATION_SPATIAL_RADIUS = 30
# How far two colors can be to be considered similar.
COLOR_SEGREGATION_COLOR_RADIUS = 40

# The thresholds used for Canny Edge Detection.
# If the difference between two neighboring pixel colors is lower than WEAK, it is not an edge.
# If the difference is higher than STRONG, it is considered an edge.
# In between WEAK and STRONG, it is considered an edge only if it connects to a STRONG edge.
EDGE_DETECT_THRESHOLD_WEAK = 250
EDGE_DETECT_THRESHOLD_STRONG = 350

# The color in HSV we expect a pylon to be. H value is in range [0, 179] and may be negative.
PYLON_COLOR_ORANGE_THRESHOLD = np.array([[-20, 127, 127],  # MIN
                                         [15, 255, 255]])  # MAX
PYLON_COLOR_WHITE_THRESHOLD = np.array([[-20, 0, 180],     # MIN
                                        [15, 70, 255]])    # MAX

# The colors in HSV with which to mark the pylons.
MARKING_COLOR_START_PYLON = [30, 255, 255]
MARKING_COLOR_REGULAR_PYLON = [0, 255, 255]

# Used for distance estimation. Determined through measurement.
DISTANCE_ESTIMATE_MAGIC_NUMBER = 80.0 * 1105 / 3096

# Debug only: used to give debug images unique names between different test of the same test run.
run_id = 0
step_id = 0


class Pylon:
    position = (0, 0)
    width = 0
    height = 0
    estimated_distance_cm = 0
    is_start_pylon = False

    def __init__(self, x, y, width, height):
        self.position = (x, y)
        self.width = width
        self.height = height
        self.is_start_pylon = width > height


class PylonDetector:
    @staticmethod
    def load_image(path: str):
        """
        Loads image in HSV color space from specified path.
        """
        return cv2.cvtColor(cv2.imread(path), cv2.COLOR_BGR2HSV)

    @staticmethod
    def write_image(image, path: str):
        """
        Writes image to sepcified path.
        """

        image_out = image
        if len(image.shape) == 3 and image.shape[2] == 3:
            image_out = cv2.cvtColor(image_out, cv2.COLOR_HSV2BGR)

        cv2.imwrite(path, image_out)

    @staticmethod
    def write_image_debug(image, name: str):
        """
        Debug only: writes a debug image with specified name.
        """
        global step_id

        if not DO_CREATE_DEBUG_IMAGES:
            return

        PylonDetector.write_image(
            image, f"tests/pylon_test_images_output/{run_id}_{step_id}_{name}.png")
        step_id += 1

    @staticmethod
    def mark_pylons(image, pylons_found: List[Pylon]):
        """
        Marks found pylons in image by drawing rectangles around them.
        """

        image_out = np.copy(image)
        for pylon in pylons_found:
            pt1 = pylon.position
            pt2 = tuple(np.add(pt1, (pylon.width, pylon.height)))
            color = MARKING_COLOR_START_PYLON if pylon.is_start_pylon else MARKING_COLOR_REGULAR_PYLON

            cv2.rectangle(image_out, pt1, pt2, color, 2)

            text_pt = tuple(np.add(pt1, (5, 30)))
            cv2.putText(image_out, f"{pylon.estimated_distance_cm:3.1f} cm", text_pt,
                        cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

        return image_out

    @staticmethod
    def resize_image(image, max_size: int):
        """
        Resizes image while preserving aspect ratio.
        Width and height are guaranteed to be equal to
        or smaller than max_size.
        """
        x_res = image.shape[1]
        y_res = image.shape[0]

        x_factor = max_size / x_res
        y_factor = max_size / y_res
        factor = min([x_factor, y_factor])

        x_res_new = int(x_res * factor)
        y_res_new = int(y_res * factor)

        return cv2.resize(image, (x_res_new, y_res_new))

    @staticmethod
    def segregate_colors(image):
        """
        Segregates colors from image to simplify object recognition.
        """
        return cv2.pyrMeanShiftFiltering(image, COLOR_SEGREGATION_SPATIAL_RADIUS, COLOR_SEGREGATION_COLOR_RADIUS)

    @staticmethod
    def get_edges(image):
        """
        Detects edges in an image.
        """
        # Blur image first to get rid of jpg artifacts.
        image_blurred = cv2.GaussianBlur(image, (3, 3), cv2.BORDER_DEFAULT)

        edges = cv2.Canny(image_blurred, EDGE_DETECT_THRESHOLD_WEAK,
                          EDGE_DETECT_THRESHOLD_STRONG)
        return PylonDetector.resize_image(edges, MAX_IMAGE_SIZE)
        return PylonDetector.morph_image(edges, cv2.MORPH_CLOSE, 4)

    @staticmethod
    def detect_pylon_colors(image):
        t_min, t_max = PYLON_COLOR_ORANGE_THRESHOLD
        image_orange = PylonDetector.detect_color(image, t_min, t_max)
        PylonDetector.write_image_debug(image_orange, "orange")

        t_min, t_max = PYLON_COLOR_WHITE_THRESHOLD
        image_white = PylonDetector.detect_color(image, t_min, t_max)
        PylonDetector.write_image_debug(image_white, "white")

        return np.clip(image_orange + image_white, 0, 255)

    @staticmethod
    def detect_color(image, threshold_min, threshold_max):
        if threshold_min[0] >= 0:
            return cv2.inRange(image, threshold_min, threshold_max)

        # OpenCV can't handle negative H values, so we need to work in 2 steps.
        t_min = np.copy(threshold_min)
        t_max = np.copy(threshold_max)

        # Step 1: 0 to MAX
        t_min[0] = 0
        mask1 = cv2.inRange(image, t_min, t_max)

        # Step 2: MIN to 179
        t_max[0] = 179
        t_min[0] = 180 + threshold_min[0]
        mask2 = cv2.inRange(image, t_min, t_max)

        return mask1 + mask2

    @staticmethod
    def morph_image(image, operation: int, iterations: int = 1):
        s = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        morph = cv2.morphologyEx(image, operation, s, iterations=iterations)
        return cv2.threshold(morph, 60, 255, cv2.THRESH_BINARY)[1]

    @staticmethod
    def add_edges(image, edges):
        result = np.clip(image - edges, 0, 255)
        return PylonDetector.morph_image(result, cv2.MORPH_CLOSE)

    @staticmethod
    def get_pylon_map(image):
        """
        Produces a map of the same dimension and size as image,
        where 1 indicates a pixel belonging to a pylon,
        and 0 indicates a pixel not belonging to a pylon.
        """
        # Resize image to reduce computational strain.
        image_resized = PylonDetector.resize_image(image, MAX_IMAGE_SIZE)
        PylonDetector.write_image_debug(image_resized, "resized")

        # Segregate colors for easier detection.
        image_segregated = PylonDetector.segregate_colors(image_resized)
        PylonDetector.write_image_debug(image_segregated, "seg")

        # Detect edges.
        # image_edge = PylonDetector.get_edges(image_segregated)
        # PylonDetector.write_image_debug(image_edge, "edge")

        # Convert image to binary.
        image_bin = PylonDetector.detect_pylon_colors(image_segregated)
        PylonDetector.write_image_debug(image_bin, "bin")

        # Reduce noise.
        image_close = PylonDetector.morph_image(image_bin, cv2.MORPH_CLOSE)
        image_filtered = PylonDetector.morph_image(
            image_close, cv2.MORPH_OPEN, 3)
        PylonDetector.write_image_debug(image_filtered, "morphed")

        # Separate pylons.
        # image_result = PylonDetector.add_edges(image_filtered, image_edge)
        # PylonDetector.write_image_debug(image_result, "map")
        image_result = image_filtered

        return image_result

    @staticmethod
    def get_potential_pylons(image) -> List[Pylon]:
        PylonDetector.write_image_debug(image, "original")

        pylon_map = PylonDetector.get_pylon_map(image)

        _, _, stats, _ = cv2.connectedComponentsWithStats(pylon_map)

        # Ignore first element (background)
        # Keep only first 4 stats (position & dimension)
        result = stats[1:, :4]

        # Scale coordinates and dimensions back to original image size.
        factor = image.shape[0] / pylon_map.shape[0]
        # Since array is uint8 but factor is float32, we can't use the *= operator.
        result[:, :] = result[:, :] * factor

        # Ignore pylons that are too small (probably just noise)
        min_size = MIN_PYLON_SIZE * factor
        return [Pylon(p[0], p[1], p[2], p[3]) for p in result if p[2] >= min_size or p[3] >= min_size]

    @staticmethod
    def find_pylons(image) -> List[Pylon]:
        # debug only
        global run_id, step_id
        run_id += 1
        step_id = 0

        pylons = [p for p in PylonDetector.get_potential_pylons(image)
                  if PylonDetector.has_proper_size_ratio(p)]

        PylonDetector.set_distance_estimation(image.shape[0], pylons)

        return pylons

    @staticmethod
    def has_proper_size_ratio(pylon: Pylon) -> bool:
        ratio = pylon.width / pylon.height

        if pylon.is_start_pylon:
            ratio = 1 / ratio

        return ratio >= PYLON_RATIO_RANGE[0] and ratio <= PYLON_RATIO_RANGE[1]

    @staticmethod
    def set_distance_estimation(image_height: int, pylons: List[Pylon]):
        for pylon in pylons:
            pylon_height = pylon.height if not pylon.is_start_pylon else pylon.width
            pylon.estimated_distance_cm = PylonDetector.get_distance_estimation(
                image_height, pylon_height)

    @staticmethod
    def get_distance_estimation(image_height: int, pylon_height: int) -> float:
        pylon_height_rate = float(image_height) / float(pylon_height)
        return DISTANCE_ESTIMATE_MAGIC_NUMBER * pylon_height_rate
