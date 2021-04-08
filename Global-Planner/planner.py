import cmd
import debug

class CarrierQueue:
    def __init__(self):
        self.retrieval_tasks = {}

    def update(self, robot_pose, litter):
        for trash in litter:
            pose, type, certainty = trash

    def assign(self, robots):
        return {}

class Map:
    def __init__(self):
        self.area = {}

    def update(self, robot_pose, grid):
        for relative_position in grid:
            row, col = relative_position

            # TODO: Get resolution from somewhere else
            absolute_position = (
                row + robot_pose.y // 1000,
                col + robot_pose.x // 1000
            )

            # TODO: Get other values from grid as well
            score = grid[relative_position]
            self.area[absolute_position] = score

class Robot:
    def __init__(self, pose):
        self.pose = pose
        self.commands = []
        self.litter = []

class GlobalPlanner:
    def __init__(self):
        self.map = Map()
        self.tasks = CarrierQueue()

        self.robots = {}

    # Sync
    def sync_poses(self, poses):
        for id, pose in poses.items():
            try: self.robots[id].pose = pose
            except KeyError: self.robots[id] = Robot(pose)

    def sync_robot(self, id, grid, litter, report):
        robot = self.robots[id]

        self.map.update(robot.pose, grid)
        self.tasks.update(robot.pose, litter)

        robot.litter.clear()
        robot.litter.extend(litter)

        if report is not None:
            done, failed, payload = report
            if failed:
                debug.log(f"{id} failed {robot.commands[0]}!")
                robot.commands[0].state = cmd.Failed

            elif done:
                debug.log(f"{id} finished {robot.commands[0]}")
                robot.commands.pop(0)

    # Run
    def get(self):
        plan = {}

        for id, robot in self.robots.items():
            if len(robot.commands) == 0: continue

            if robot.commands[0].state == cmd.Queued:
                plan[id] = robot.commands[0]
                robot.commands[0].state = cmd.Active

        return plan
