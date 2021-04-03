import cmd
import math

def _transform(frame, vector):
    rad = math.radians(frame[2])
    cos = math.cos(rad); sin = math.sin(rad)

    return (
        vector[0] * cos - vector[1] * sin + frame[0],
        vector[0] * sin + vector[1] * cos + frame[1]
    )

class Map:
    def __init__(self):
        self.grid = {}

    def update(self, map):
        pass

class GlobalPlanner:
    def __init__(self):
        self.poses = {}
        self.cqs = {}

    # Sync Functions
    def sync_poses(self, poses):
        self.poses = poses

    def sync_robot(self, id, map, trash, report):
        pass

    # Run
    def get(self):
        commands = {}

        for id, pose in self.poses.items():
            if id in self.cqs: continue
            self.cqs[id] = [cmd.Command("Move", (5000, 0, 0))]

        for id, cq in self.cqs.items():
            if len(cq) == 0: continue

            if cq[0].state == cmd.Queued:
                commands[id] = cq[0]
                cq[0].state = cmd.Active

        return commands
