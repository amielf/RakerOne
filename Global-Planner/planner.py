class _Plan:
    def __init__(self):
        self.waypoints = {}

class TrafficPattern:
    def plan(self, start, goal):
        moved_y = (start[0], goal[1], start[2])
        dx = goal[0] - start[0]
        face_destination = (moved_y[0], moved_y[1], moved_y[2] - 90 if dx > 0 else 90)
        moved_x = (goal[0], goal[1], face_destination[2])

        waypoints = [moved_y, face_destination, moved_x, goal]
        return waypoints

class GlobalPlanner:
    def __init__(self):
        self._tp = TrafficPattern()

        self.poses = {}
        self.destinations = {}

    def _set_destinations(self):
        for id in self.poses:
            try: pose = self.destinations[id]
            except KeyError: pose = self.poses[id]

            self.destinations[id] = (pose[0] + 100, pose[1] + 100 * id, pose[2])

    def sync(self, poses):
        self.poses = poses

    def compute(self):
        plan = _Plan()

        self._set_destinations()

        for id, pose in self.poses.items():
            try: destination = self.destinations[id]
            except KeyError: continue

            plan.waypoints[id] = self._tp.plan(pose, destination)

        return plan