import cmd
import debug
import pose
import tf
import util

class Task:
    def __init__(self, location):
        self.location = location

class Retrieve(Task):
    def __init__(self, location, type, weight, skills):
        super(Retrieve, self).__init__(location)
        self.type = type
        self.weight = weight
        self.skills = skills

    def __str__(self): return f"Retrieve({self.location}, {self.type})"
    def __repr__(self): return f"Retrieve({self.location}, {self.type})"

class Service(Task):
    def __init__(self, charge, bin):
        super(Service, self).__init__((0, 0))
        self.charge = charge
        self.bin = bin

    def __str__(self): return f"Service({self.charge}% battery left, {self.bin}% full)"
    def __repr__(self): return f"Service({self.charge}% battery left, {self.bin}% full)"

class Explore(Task):
    def __init__(self, location):
        super(Explore, self).__init__(location)

    def __str__(self): return f"Explore({self.location})"
    def __repr__(self): return f"Explore({self.location})"

class CarrierQueue:
    def __init__(self, map, flow):
        self.map = map
        self.flow = flow

        self.retrieve_tasks = []
        self.assignments = {}

    def update(self, robot_pose, litter):
        for trash in litter:
            trash_position_relative, type, weight, certainty = trash
            location = tf.absolute_position(robot_pose, trash_position_relative)

            if certainty < 0.5: continue

            already_known = False
            for existing in self.retrieve_tasks:
                if util.distance(existing.location, location) < 10:
                    already_known = True
                    break

            if not already_known:
                self.retrieve_tasks.append(Retrieve(location, type, weight, ["gripper"]))

    def allocate(self, robots):
        # Check to see if robot needs to return for service
        for id, robot in robots.items():
            # create a service task
            if robot.charge <= 25 or robot.bin >= 90:
                if len(robot.todo) > 0:
                    current_active_task = robot.todo.pop(0)
                    if isinstance(current_active_task, Retrieve):
                        self.retrieve_tasks.append(current_active_task)

                robot.todo.insert(0, Service(robot.charge, robot.bin))
                robot.commands.clear()

        # assign retrieval tasks to Robots
        assigned_litter = []

        for task in self.retrieve_tasks:
            # match litter skill to robot skill distance to litter used for tie breaker
            matches = [robot for id, robot in robots.items() if robot.end_effector in task.skills and len(robot.todo) < 3]

            if len(matches) == 0:
                # need to flag this item. Keep in a list?
                # print(f"Litter ({type}, {certainty}) cannot be retrieved from {where.position}; needs {skills} to pick up and no robots currently match.")
                continue

            elif len(matches) == 1:
                robot = matches[0]

                # If this robot is currently only exploring, don't
                if len(robot.todo) == 1 and isinstance(robot.todo[0], Explore):
                    robot.todo.pop(0)
                    robot.commands.clear()

                robot.todo.append(task)
                assigned_litter.append(task)

            else:
                min_distance = 9999
                closest_robot = None

                for match in matches:
                    distance = self.flow.distance(match.pose.position, task.location)
                    if distance <= min_distance:
                        min_distance = distance
                        closest_robot = match

                if closest_robot is not None:
                    # If the closest robot is currently only exploring, don't
                    if len(closest_robot.todo) == 1 and isinstance(closest_robot.todo[0], Explore):
                        closest_robot.todo.pop(0)
                        closest_robot.commands.clear()

                    closest_robot.todo.append(task)
                    assigned_litter.append(task)

        for task in assigned_litter:
            self.retrieve_tasks.remove(task)

        # assign explore task to any robots not already allocated
        assigned_points = []

        for id, robot in robots.items():
            if len(robot.todo) > 0: continue

            max_distance = 0
            farthest_point = None

            for row, col in self.map.frontier:
                # TODO: The map should do this for you
                px = col * self.map.resolution + self.map.resolution / 2
                py = row * self.map.resolution + self.map.resolution / 2

                if (px, py) in assigned_points: continue

                distance = self.flow.distance(robot.pose.position, (px, py))
                if distance >= max_distance:
                    max_distance = distance
                    farthest_point = (px, py)

            if farthest_point is not None:
                robot.todo.append(Explore(farthest_point))
                assigned_points.append(farthest_point)

class Map:
    def __init__(self, resolution):
        self.resolution = resolution
        self.area = {}
        self.frontier = set()

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
                self.frontier.add(candidate)

    # Util
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
        self.charge = 100
        self.todo = []
        self.commands = []
        self.end_effector = None

class LanesFlow:
    def __init__(self, map):
        self.map = map

        self.spans = [(start, start + 3000) for start in range(0, 21000, 3000)]

    def distance(self, start, goal):
        # Use L-distance for this flow, since the lanes enforce L-shaped motion
        # TODO: No they don't, change this
        return abs(goal[0] - start[0]) + abs(goal[1] - start[1])

    def plan(self, start, goal):
        # TODO: Use the map
        aligned_with_lane = pose.Pose2D(start.x, start.y, 90)

        # Doing the commands like this makes it more efficient if they get cleared out and replaced part way through
        move_up_lane = []
        for y in range(int(start.y), int(goal.y), 2000):
            move_up_lane.append(pose.Pose2D(start.x, y, 90))

        finish_vertical = pose.Pose2D(start.x, goal.y, 90)

        face_destination_orientation = 0 if goal.x > start.x else 180
        face_destination = pose.Pose2D(start.x, goal.y, face_destination_orientation)
        cross_lanes = [pose.Pose2D(x, goal.y, face_destination_orientation) for x, _ in self.spans if min(start.x, goal.x) < x < max(start.x, goal.x)]

        final_approach = pose.Pose2D(goal.x, goal.y, face_destination_orientation)

        return [start, aligned_with_lane] + move_up_lane + [finish_vertical, face_destination] + cross_lanes + [final_approach, goal]


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
        self.tasks.update(robot.pose, litter)

        if report is not None:
            if len(robot.commands) == 0: return

            done, failed, payload = report
            if failed:
                debug.log(f"{id} failed {robot.commands[0]}!")
                robot.commands.clear()

            elif done:
                robot.commands.pop(0)
                if len(robot.commands) == 0: robot.todo.pop(0)

    # Run
    def get(self):
        plan = {}

        self.tasks.allocate(self.robots)
        for id, robot in self.robots.items():
            print(id, robot.todo)

        for id, robot in self.robots.items():
            if len(robot.commands) == 0 and len(robot.todo) > 0:
                next_task = robot.todo[0]

                goal = pose.Pose2D(*next_task.location, 0)
                waypoints = self.flow.plan(robot.pose, goal)

                start = waypoints.pop(0)
                for waypoint_absolute in waypoints:
                    waypoint_relative = tf.relative_pose(start, waypoint_absolute)

                    if waypoint_relative.a != 0:
                        robot.commands.append(cmd.Command("Rotate", waypoint_relative.a))

                    if waypoint_relative.position != (0, 0):
                        robot.commands.append(cmd.Command("Move", waypoint_relative.position))

                    start = waypoint_absolute

                if isinstance(next_task, Retrieve):
                    robot.commands.append(cmd.Command("Pick", next_task.weight))

            if len(robot.commands) > 0:
                if robot.commands[0].state == cmd.Queued:
                    plan[id] = robot.commands[0]
                    robot.commands[0].state = cmd.Active

        return plan
