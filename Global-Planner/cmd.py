class Command:
    def __init__(self, name, args):
        self.name = name
        self.args = args

    def __str__(self): return f"{self.name}({self.args})"
    def __repr__(self): return f"{self.name}({self.args})"
