Queued = -1
Active = 0
Done = 1
Failed = 2

class Command:
    def __init__(self, name, args):
        self.name = name
        self.args = args

        self.state = Queued

    def __str__(self): return f"{self.name}({self.args})"
    def __repr__(self): return f"{self.name}({self.args})"
