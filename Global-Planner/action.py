import abc
import math

class Action(abc.ABC):
    def __init__(self, command):
        self.command = command

        self.done = False
        self.failed = False
        self.payload = None

    @abc.abstractmethod
    def run(self, dt, robot): pass

class Move(Action):
    def __init__(self, command):
        super(Move, self).__init__(command)

        dx, dy, da = command.args

        theta = math.degrees(math.atan2(dy, dx))
        r = math.sqrt(dx * dx + dy * dy)

        leftover = da - theta

        self.targets = [
            [0, 0, theta],
            [r, 0, 0],
            [0, 0, leftover]
        ]
        self.index = 0

        self._v = 1000  # mm/s
        self._w = 18    # deg/s

    def run(self, dt, robot):
        target = self.targets[self.index]

        rad = math.radians(robot.pose.a)
        cos = math.cos(rad); sin = math.sin(rad)

        if target[0] != 0:
            mx = min(self._v * dt / 1000, target[0])
            target[0] -= mx
            robot.pose.x += mx * cos
            robot.pose.y += mx * sin

        if target[1] != 0:
            my = min(self._v * dt / 1000,target[1])
            my = math.copysign(my, target[1])
            target[1] -= my
            robot.pose.x -= my * sin
            robot.pose.y += my * cos

        if target[2] != 0:
            ma = min(self._w * dt / 1000, target[2])
            ma = math.copysign(ma, target[2])
            target[2] -= ma
            robot.pose.a += ma

        if all(d == 0 for d in target):
            self.index += 1
            if self.index == len(self.targets):
                self.done = True

def create(command):
    if command.name == "Move": return Move(command)
