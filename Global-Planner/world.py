import math
import random

class World:
    def __init__(self, width, height, resolution):
        self.width = width; self.height = height    # mm
        self.resolution = resolution                # mm

        self.rows = math.ceil(height / resolution)
        self.cols = math.ceil(width / resolution)

        self.terrain = [[random.randint(0, 100) for _ in range(self.cols)] for _ in range(self.rows)]
