import math

class Pose2D:
    @property
    def position(self): return (self.x, self.y)

    def __init__(self, x, y, a):
        self.x = x; self.y = y
        self.a = a

    def distance(self, pose):
        dx = pose.x - self.x
        dy = pose.y - self.y
        return math.hypot(dx, dy)

    def __str__(self): return f"Pose2D({self.x}, {self.y}, {self.a})"
    def __repr__(self): return f"Pose2D({self.x}, {self.y}, {self.a})"
