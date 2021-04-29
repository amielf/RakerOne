import math

class Pose:
    @property
    def location(self): return Location(self.x, self.y)

    def __init__(self, x, y, a):
        self.x = int(x)
        self.y = int(y)
        self.a = a

    def get_absolute(self, relative):
        """
        Given the pose or location relative to this frame, what would this frame's parent call the pose or location?
        For example, (1, 1, 0).get_absolute(pose = (1, 1, 0)) => (2, 2, 0)
                     (1, 1, 0).get_absolute(location = (1, 1)) => (2, 2)
        """
        rad = math.radians(self.a)
        cos = math.cos(rad); sin = math.sin(rad)

        x = relative.x * cos - relative.y * sin + self.x
        y = relative.x * sin + relative.y * cos + self.y

        try: return Pose(x, y, relative.a - self.a)
        except AttributeError: return Location(x, y)

    def relative_to(self, frame):
        """
        Given the frame relative to the same parent as this pose, what would the frame call this pose?
        For example, (2, 2, 0).relative_to(frame = (1, 1, 0)) => (1, 1, 0)
        """
        dx = self.x - frame.x
        dy = self.y - frame.y
        da = self.a - frame.a

        rad = math.radians(-frame.a)
        cos = math.cos(rad); sin = math.sin(rad)

        return Pose(
            dx * cos - dy * sin,
            dx * sin + dy * cos,
            da
        )

    def distance(self, thing):
        dx = thing.x - self.x
        dy = thing.y - self.y
        return math.hypot(dx, dy)

    def __str__(self): return f"Pose({self.x}, {self.y}, {self.a})"
    def __repr__(self): return f"Pose({self.x}, {self.y}, {self.a})"

class Location:
    def __init__(self, x, y):
        self.x = int(x)
        self.y = int(y)

    def relative_to(self, frame):
        """
        Given the frame relative to the same parent as this location, what would the frame call this location?
        For example, (2, 2).relative_to(frame = (1, 1, 0)) => (1, 1)
        """
        dx = self.x - frame.x
        dy = self.y - frame.y

        rad = math.radians(-frame.a)
        cos = math.cos(rad); sin = math.sin(rad)

        return Location(
            dx * cos - dy * sin,
            dx * sin + dy * cos
        )

    def distance(self, thing):
        dx = thing.x - self.x
        dy = thing.y - self.y
        return math.hypot(dx, dy)

    def __str__(self): return f"Location({self.x}, {self.y})"
    def __repr__(self): return f"Location({self.x}, {self.y})"

    def __eq__(self, other): return isinstance(other, Location) and other.x == self.x and other.y == self.y
    def __hash__(self): return hash((self.x, self.y))