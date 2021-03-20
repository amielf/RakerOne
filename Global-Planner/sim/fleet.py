import math
from sim import pose

class FleetSimulator:
    def __init__(self, n):
        self.poses = {i: pose.Pose2D(99 / 2 + 150 * i, 0, 90) for i in range(n)}
        self.trajectories = {}

    def receive(self, plan):
        for id, waypoints in plan.waypoints.items():
            try: self.trajectories[id].extend(waypoints)
            except KeyError: self.trajectories[id] = waypoints

    def update(self, dt):
        for id, pose in self.poses.items():
            try:
                trajectory = self.trajectories[id]

                target = trajectory[0]
                dx = target[0] - pose.x; dy = target[1] - pose.y; dtheta = target[2] - pose.theta
                pose.x += dx / dt; pose.y += dy / dt; pose.theta += dtheta / dt

                if pose.close(target):
                    trajectory.pop(0)
                    if len(trajectory) == 0:
                        del self.trajectories[id]


            except KeyError: continue
