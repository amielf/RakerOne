import abc
import math

class _Action(abc.ABC):
    def __init__(self):
        self.done = False
        self.failed = False
        self.payload = None

    @abc.abstractmethod
    def run(self, dt, husky): pass

class _Rotate(_Action):
    def __init__(self, da):
        super(_Rotate, self).__init__()
        self.theta = da

    def run(self, dt, husky):
        da = husky.rotational_speed * dt / 1000
        if da > abs(self.theta):
            da = abs(self.theta)
            elapsed = da / husky.rotational_speed

        else: elapsed = dt

        da = math.copysign(da, self.theta)
        self.theta -= da
        husky.pose.a += da

        if self.theta == 0:
            self.done = True

        return elapsed

class _Move(_Action):
    def __init__(self, dx, dy):
        super(_Move, self).__init__()

        face_destination_angle = math.degrees(math.atan2(dy, dx))
        if abs(face_destination_angle) > 180: face_destination_angle -= math.copysign(180, face_destination_angle)

        self._face_destination = _Rotate(face_destination_angle)
        self.distance = math.sqrt(dx * dx + dy * dy)

    def run(self, dt, husky):
        elapsed = 0
        if not self._face_destination.done:
            elapsed += self._face_destination.run(dt, husky)
            if elapsed == dt: return dt

        # elapsed is turn time
        # available is dt - elapsed
        available = dt - elapsed

        rad = math.radians(husky.pose.a)
        cos = math.cos(rad); sin = math.sin(rad)

        dr = husky.linear_speed * available

        if dr > abs(self.distance):
            dr = abs(self.distance)
            elapsed += dr / husky.linear_speed

        else: elapsed += available

        dr = min(husky.linear_speed * dt, self.distance)
        dr = math.copysign(dr, self.distance)

        self.distance -= dr
        husky.pose.x += int(dr * cos)
        husky.pose.y += int(dr * sin)

        if self._face_destination.done and self.distance == 0:
            self.done = True

        return elapsed

class _PickAction(_Action):
    def __init__(self, trash_id, volume):
        super(_PickAction, self).__init__()
        self.trash_id = trash_id
        self.volume = volume
        self.countdown = 2000

    def run(self, dt, husky):
        elapsed = min(self.countdown, dt)

        self.countdown -= elapsed
        if self.countdown == 0:
            husky.bin += self.volume
            husky.picked_trash_id = self.trash_id
            self.done = True

        return elapsed

class _ChargeAction(_Action):
    def __init__(self):
        super(_ChargeAction, self).__init__()
        self.countdown = 0

    def run(self, dt, husky):
        self.countdown -= dt
        if self.countdown <= 0:
            husky.charge = husky.full_charge
            self.done = True

        return min(self.countdown, dt)

class _ExchangeAction(_Action):
    def __init__(self):
        super(_ExchangeAction, self).__init__()
        self.countdown = 7000

    def run(self, dt, husky):
        self.countdown -= dt
        if self.countdown <= 0:
            husky.bin = 0
            self.done = True

        return min(self.countdown, dt)

def create(command):
    if command.name == "Move": return _Move(*command.args)
    if command.name == "Rotate": return _Rotate(command.args)
    if command.name == "Pick": return _PickAction(*command.args)

    raise Exception(f"{command.name} not recognized.")
