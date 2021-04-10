import cmd
import debug
import tf

class CarrierQueue:
    def __init__(self):
        self.retrieval_tasks = {}
        self.explore_tasks = set()

    def create_retrieval_tasks(self, robot_pose, litter):
        for trash in litter:
            trash_pose, type, certainty = trash
            trash_pose_absolute = tf.absolute(robot_pose, trash_pose)

            if trash_pose_absolute.position not in self.retrieval_tasks and certainty > 0.5:
                self.retrieval_tasks[trash_pose_absolute.position] = (type, certainty)

    # Do this on the fly during allocation if there are free robots
    def create_explore_tasks(self, map, lanes):
        self.explore_tasks.clear()

        half_resolution = map.resolution / 2

        # TODO: Smarter paritioning and point selection; no need for tons of explore tasks
        for row, col in map.frontier:
            position = (
                col * map.resolution + half_resolution,
                row * map.resolution + half_resolution
            )

            self.explore_tasks.add(position)

class Map:
    def __init__(self, resolution):
        self.resolution = resolution
        self.area = {}
        self.frontier = []

    def expand(self, robot_pose, robot_grid):
        pose_row = int(robot_pose.y / self.resolution)
        pose_col = int(robot_pose.x / self.resolution)

        frontier_candidates = []

        for cell in robot_grid:
            row, col = cell

            global_cell = (
                row + pose_row,
                col + pose_col
            )

            # TODO: Get other values from grid as well
            scores = robot_grid[cell]

            if global_cell not in self.area:
                frontier_candidates.append(global_cell)

            self.area[global_cell] = scores

        explored = []
        for cell in self.frontier:
            neighbors = self.get_eight_neighbors(cell)
            if len(neighbors) > 6:
                explored.append(cell)

        for cell in explored: self.frontier.remove(cell)

        for candidate in frontier_candidates:
            neighbors = self.get_eight_neighbors(candidate)
            if len(neighbors) < 8:
                self.frontier.append(candidate)

    def get_eight_neighbors(self, cell):
        row, col = cell

        neighbors = []
        for r in range(row - 1, row + 2):
            for c in range(col - 1, col + 2):
                neighbor = (r, c)

                if neighbor not in self.area: continue
                if neighbor == cell: continue

                neighbors.append(neighbor)

        return neighbors

class Robot:
    def __init__(self, pose):
        self.pose = pose
        self.to_do = []
        self.commands = [cmd.Command("Move", (20000, 0))]
        self.charge = 100

class GlobalPlanner:
    def __init__(self):
        self.map = Map(1000)
        self.tasks = CarrierQueue()

        self.robots = {}

    # Sync
    def sync_poses(self, poses):
        for id, pose in poses.items():
            try: self.robots[id].pose = pose
            except KeyError: self.robots[id] = Robot(pose)

    def sync_robot(self, id, charge, grid, litter, report):
        robot = self.robots[id]

        robot.charge = charge
        self.map.expand(robot.pose, grid)
        self.tasks.create_retrieval_tasks(robot.pose, litter)

        if report is not None:
            if len(robot.commands) == 0: return

            done, failed, payload = report
            if failed:
                debug.log(f"{id} failed {robot.commands[0]}!")
                robot.commands.clear()

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
