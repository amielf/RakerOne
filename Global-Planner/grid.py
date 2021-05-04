import common
import math
import random

class Grid:
    def __init__(self, width, height, resolution):
        self.width = width; self.height = height    # mm
        self.resolution = resolution                # mm

        self.rows = math.ceil(height / resolution)
        self.cols = math.ceil(width / resolution)

        # TODO: Get these from file or something
        self.costs = [[tuple(random.randint(0, 5) for _ in range(4)) for _ in range(self.cols)] for _ in range(self.rows)]

    def get_center(self, row, col):
        return common.Location(
            (col + 0.5) * self.resolution,
            (row + 0.5) * self.resolution
        )
