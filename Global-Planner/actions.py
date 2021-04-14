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
        self._w = 36 / 1000.0   # deg/ms

    def run(self, dt, robot):
        da = min(self._w * dt, abs(self.theta))
        da = math.copysign(da, self.theta)
        self.theta -= da
        robot.pose.a += da

        if self.theta == 0:
            self.done = True

class Move(Action):
    def __init__(self, dx, dy):
        super(Move, self).__init__()

        self._face_destination = Rotate(math.degrees(math.atan2(dy, dx)))

        self.distance = math.sqrt(dx * dx + dy * dy)
        self._v = 1  # mm/ms

    def run(self, dt, robot):
        rad = math.radians(robot.pose.a)
        cos = math.cos(rad); sin = math.sin(rad)

        if not self._face_destination.done:
            self._face_destination.run(dt, robot)

        elif self.distance != 0:
            dr = min(self._v * dt, self.distance)
            dr = math.copysign(dr, self.distance)
            self.distance -= dr
            robot.pose.x += dr * cos
            robot.pose.y += dr * sin

        if self._face_destination.done and self.distance == 0:
            self.done = True

class EndEffectorAction(Action):
    def __init__(self, weight):
        super(EndEffectorAction, self).__init__()
        self.weight = weight
        self.countdown = weight * 10000

    def run(self, dt, robot):
        self.countdown -= dt
        if self.countdown <= 0:
            robot.bin += self.weight
            self.done = True

def create(command):
    if command.name == "Move": return Move(*command.args)
    if command.name == "Rotate": return Rotate(command.args)
    if command.name == "Pick": return EndEffectorAction(command.args)

    raise Exception(f"{command.name} not recognized.")
