import cmd
import debug
import math
import pose
import tf

class Task:
    def __init__(self, location):
        self.location = location

class Pick(Task):
    def __init__(self, location, type, certainty):
        super(Pick, self).__init__(location)
        self.type = type
        self.certainty = certainty

class Charge(Task):
    def __init__(self):
        super(Charge, self).__init__((0, 0))

class EmptyBin(Task):
    def __init__(self):
        super(EmptyBin, self).__init__((0, 0))

class Scan(Task):
    def __init__(self, location):
        super(Scan, self).__init__(location)

class CarrierQueue:
    def __init__(self, map, flow):
        self.map = map
        self.flow = flow

        self.unassigned_litter = {}
        self.assignments = {}


    def create_retrieval_tasks(self, robot_pose, litter):
        for trash in litter:
            trash_pose_relative, type, certainty = trash
            trash_pose_absolute = tf.absolute(robot_pose, trash_pose_relative)

            if trash_pose_absolute.position not in self.unassigned_litter and certainty > 0.5:
                self.unassigned_litter[trash_pose_absolute] = (type, certainty, ["gripper"])

    def allocate(self, robots):
        assignments = {}

        # Check to see if robot needs to return for service
        for id, robot in robots.items():
            # create a service task
            if robot.charge <= 25: assignments[id] = Charge()
            if robot.bin >= 8: assignments[id] = EmptyBin()

        # assign retrieval tasks to Robots
        for where, data in self.unassigned_litter.items():
            type, certainty, skills = data

            # match litter skill to robot skill distance to litter used for tie breaker
            matches = [id for id, robot in robots.items() if robot.end_effector in skills and id not in assignments]

            if len(matches) == 0:
                # need to flag this item. Keep in a list?
                # print(f"Litter ({type}, {certainty}) cannot be retrieved from {where.position}; needs {skills} to pick up and no robots currently match.")
                continue

            elif len(matches) == 1:
                assignments[matches[0]] = Pick(where, type, certainty)
                continue

            else:
                min_distance = 9999
                closest_robot_id = None
                for id in matches:
                    robot = robots[id]

                    distance = math.sqrt((where[0] - robot.pose.x) ** 2 + (where.y - robot.pose.y) ** 2)
                    if distance <= min_distance:
                        min_distance = distance
                        closest_robot_id = id

                assignments[closest_robot_id] = Pick(where, type, certainty)

        for _, task in assignments.items():
            if isinstance(task, Pick): del self.unassigned_litter[task.location]

        # assign explore task to any robots not already allocated
        for id, robot in robots.items():
            if id in assignments: continue
            if len(robot.commands) > 0: continue

            max_distance = 0
            farthest_point = None

            for row, col in self.map.frontier:
                px = col * self.map.resolution + self.map.resolution / 2
                py = row * self.map.resolution + self.map.resolution / 2

                distance = math.sqrt((px - robot.pose.x) ** 2 + (py - robot.pose.y) ** 2)
                if distance >= max_distance:
                    max_distance = distance
                    farthest_point = (px, py)

            if farthest_point is not None:
                assignments[id] = Scan(pose.Pose2D(farthest_point[0], farthest_point[1], 0))

        return assignments

class Map:
    def __init__(self, resolution):
        self.resolution = resolution
        self.area = {}
        self.frontier = []

    # Update
    def expand(self, robot_pose, robot_grid):
        pose_row, pose_col = self.nearest(robot_pose.x, robot_pose.y)

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
            neighbors = self.get_all_neighbors(cell)
            if len(neighbors) > 6:
                explored.append(cell)

        for cell in explored: self.frontier.remove(cell)

        for candidate in frontier_candidates:
            neighbors = self.get_all_neighbors(candidate)
            if len(neighbors) < 8:
                self.frontier.append(candidate)

    def nearest(self, x, y):
        return (
            int(y / self.resolution),
            int(x / self.resolution)
        )

    # Neighbors
    def get_cardinal_neighbors(self, cell):
        pass

    def get_all_neighbors(self, cell):
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

        self.bin = 0
        self.commands = []
        self.charge = 100
        self.end_effector = None
        self.target = None

class LanesFlow:
    def __init__(self, map):
        self.map = map

        self.spans = [(start, start + 3000) for start in range(0, 21000, 3000)]

    def _get_lane(self, x):
        for span in self.spans:
            if span[0] <= x <= span[1]:
                return span

        return (0, 0)

    def distance(self, start, goal):
        return start.x + goal.x + start.y + goal.y

    def plan(self, start, goal):
        start_lane = self._get_lane(start.x)
        goal_lane = self._get_lane(goal.x)

        aligned_to_lane = pose.Pose2D(start.x, start.y, 90)

        if start_lane == goal_lane:
            return [start, aligned_to_lane, goal]

        partial_ys = range(int(start.y), int(goal.y), 2000)
        vertical_poses = [pose.Pose2D(start.x, y, 90) for y in partial_ys]

        angle = 0 if goal.x > start.x else 180

        ready_to_merge = [
            pose.Pose2D(start.x, goal.y, 90),
            pose.Pose2D(start.x, goal.y, angle)
        ]

        merge_xs = [x for x, _ in self.spans if start_lane[0] < x <= goal_lane[0]]
        merge_poses = [pose.Pose2D(x, goal.y, angle) for x in merge_xs]

        return [start, aligned_to_lane] + vertical_poses + ready_to_merge + merge_poses + [goal]

class RandomFlow:
    def __init__(self, map):
        self.map = map

    def distance(self, start, goal):
        return start.distance(goal)

    def plan(self, start, goal):
        start_cell = self.map.nearest(start.x, start.y)
        goal_cell = self.map.nearest(goal.x, goal.y)

        visited = set(); queue = set(start_cell)
        while len(queue) > 0:
            current = queue.pop()
            if current == goal_cell:
                return None

            queue.update(self.map.get_cardinal_neighbors(current))
            queue.difference_update(visited)

class GlobalPlanner:
    def __init__(self):
        self.map = Map(1000)
        self.flow = LanesFlow(self.map)
        self.tasks = CarrierQueue(self.map, self.flow)

        self.robots = {}

    # Sync
    def sync_poses(self, poses):
        for id, pose in poses.items():
            try: self.robots[id].pose = pose
            except KeyError: self.robots[id] = Robot(pose)

    def sync_robot(self, id, charge, end_effector, grid, litter, report):
        robot = self.robots[id]

        robot.charge = charge
        robot.end_effector = end_effector
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

        assignments = self.tasks.allocate(self.robots)
        print(assignments)

        for id, robot in self.robots.items():
            if id in assignments:
                robot.target = assignments[id].location
                waypoints = self.flow.plan(robot.pose, robot.target)
                start = waypoints.pop(0)
                for waypoint_absolute in waypoints:
                    waypoint_relative = tf.relative(start, waypoint_absolute)

                    if waypoint_relative.a != 0:
                        robot.commands.append(cmd.Command("Rotate", waypoint_relative.a))

                    if waypoint_relative.position != (0, 0):
                        robot.commands.append(cmd.Command("Move", waypoint_relative.position))

                    start = waypoint_absolute

                print(id, robot.commands)

            if len(robot.commands) > 0:
                if robot.commands[0].state == cmd.Queued:
                    plan[id] = robot.commands[0]
                    robot.commands[0].state = cmd.Active

        return plan
