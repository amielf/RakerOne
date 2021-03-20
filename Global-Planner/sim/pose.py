import math

class Pose2D:
    @property
    def position(self): return (self.x, self.y)

    @property
    def tuplify(self): return (self.x, self.y, self.theta)

    def __init__(self, x, y, theta):
        self.x = x; self.y = y
        self.theta = theta

    def close(self, target):
        return math.fabs(target[0] - self.x) < 2 and \
               math.fabs(target[1] - self.y) < 2 and \
               math.fabs(target[2] - self.theta) < 2