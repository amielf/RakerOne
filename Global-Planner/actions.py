import abc
import math

class Action(abc.ABC):
    def __init__(self):
        self.done = False
        self.failed = False
        self.payload = None

    @abc.abstractmethod
    def run(self, dt, robot): pass

class Rotate(Action):
    def __init__(self, da):
        super(Rotate, self).__init__()

        self.theta = da

        self._w = 36  # deg/s

    def run(self, dt, robot):
        da = min(self._w * dt / 1000.0, self.theta)
        da = math.copysign(da, self.theta)
        self.theta -= da
        robot.pose.a += da

        if self.theta == 0:
            self.done = True

class Move(Action):
    def __init__(self, dx, dy):
        super(Move, self).__init__()

        self._rotate = Rotate(math.degrees(math.atan2(dy, dx)))
        self.r = math.sqrt(dx * dx + dy * dy)

        self._v = 1000  # mm/s

    def run(self, dt, robot):
        rad = math.radians(robot.pose.a)
        cos = math.cos(rad); sin = math.sin(rad)

        if not self._rotate.done:
            self._rotate.run(dt, robot)

        elif self.r != 0:
            dr = min(self._v * dt / 1000.0, self.r)
            dr = math.copysign(dr, self.r)
            self.r -= dr
            robot.pose.x += dr * cos
            robot.pose.y += dr * sin

        if self._rotate.done and self.r == 0:
            self.done = True

def create(command):
    if command.name == "Move": return Move(*command.args)
    if command.name == "Rotate": return Rotate(command.args)
