import common

QUEUED = 0
ACTIVE = 1

class Task:
    def __init__(self, location):
        self.location = location
        self.state = QUEUED

class Retrieve(Task):
    def __init__(self, id, location, type, volume, skills):
        super(Retrieve, self).__init__(location)

        self.id = id
        self.type = type
        self.volume = volume
        self.skills = skills

    def __str__(self): return f"Retrieve({self.location}, {self.type})"
    def __repr__(self): return f"Retrieve({self.location}, {self.type})"

class Service(Task):
    def __init__(self, charge, bin):
        super(Service, self).__init__(common.Location(0, 0))

        self.charge = charge
        self.bin = bin

    def __str__(self): return f"Service({self.charge}% battery left, {self.bin}% full)"
    def __repr__(self): return f"Service({self.charge}% battery left, {self.bin}% full)"

class Explore(Task):
    def __init__(self, location):
        super(Explore, self).__init__(location)

    def __str__(self): return f"Explore({self.location})"
    def __repr__(self): return f"Explore({self.location})"