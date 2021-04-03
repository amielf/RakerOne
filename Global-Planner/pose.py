import math

class Pose2D:
    @property
    def position(self): return (self.x, self.y)

    def __init__(self, x, y, a):
        self.x = x; self.y = y
        self.a = a

    def distance(self, position):
        dx = position[0] - self.x
        dy = position[1] - self.y
        return math.hypot(dx, dy)

    def __str__(self): return f"Pose2D({self.x}, {self.y}, {self.a})"
    def __repr__(self): return f"Pose2D({self.x}, {self.y}, {self.a})"

def relative_to(origin, pose):
    dx = pose.x - origin.x
    dy = pose.y - origin.y
    da = pose.a - origin.a

    rad = math.radians(-origin.a)
    cos = math.cos(rad); sin = math.sin(rad)

    return Pose2D(
        dx * cos - dy * sin,
        dx * sin + dy * cos,
        da
    )