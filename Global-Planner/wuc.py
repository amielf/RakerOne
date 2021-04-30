import cmd
import debug
import math
import common
import tasks
import queue

class CarrierQueue:
    def __init__(self, map, flow):
        self.map = map
        self.flow = flow

        self.open_retrieval_tasks = {}
        self.active_retrieval_tasks = {}
        self.finished_retrieval_tasks = {}

        self._locations_by_bucket = {}

    def _assign_service_tasks(self, robots):
        for id, robot in robots.items():
            if robot.charge <= 25 or robot.bin >= 99:
                if len(robot.todo) > 0:
                    current_active_task = robot.todo[0]

                    # If already doing a Service task, leave it alone
                    if isinstance(current_active_task, tasks.Service): continue

                    # If taking a Retrieve task out of assignment, add it back to the open list
                    if isinstance(current_active_task, tasks.Retrieve):
                        del self.active_retrieval_tasks[current_active_task.id]
                        self.open_retrieval_tasks[current_active_task.id] = current_active_task
                        robot.todo.pop(0)

                robot.todo.insert(0, tasks.Service(robot.charge, robot.bin))

    def _assign_retrieve_task(self, task, robot):
        # If this robot is currently only exploring, stop doing that
        if len(robot.todo) == 1 and isinstance(robot.todo[0], tasks.Explore):
            robot.todo.pop(0)

        robot.todo.append(task)

    # Notify
    def notify_movement(self, dy):
        for task in self.open_retrieval_tasks.values():
            task.location.y -= dy

        for task in self.active_retrieval_tasks.values():
            task.location.y -= dy

        for task in self.finished_retrieval_tasks.values():
            task.location.y -= dy

    def notify_discoveries(self, robot_pose, discoveries):
        for discovery in discoveries:
            id, location_relative, type, volume, certainty, end_effectors = discovery
            if certainty < 0.3: continue

            location_absolute = robot_pose.get_absolute(location_relative)

            # Having the ID is cheating, since in real life there's no way to identify before discovery
            # This would normally be handled by some form of geo-caching
            if id in self.open_retrieval_tasks: continue
            if id in self.active_retrieval_tasks: continue
            if id in self.finished_retrieval_tasks: continue

            self.open_retrieval_tasks[id] = tasks.Retrieve(id, location_absolute, type, volume, end_effectors)

    @debug.profiled
    def allocate(self, robots):
        #self._assign_service_tasks(robots)

        assigned_tasks = []

        for task in self.open_retrieval_tasks.values():
            candidate_robots = [robot for id, robot in robots.items() if robot.end_effector in task.skills]
            if len(candidate_robots) == 0:
                print(f"[ERROR]: {task.type} cannot be retrieved from {task.location} by any robot; requires {task.skills}.")
                continue

            available_robots = [robot for robot in candidate_robots if len(robot.todo) < 3]

            min_score = 999999
            closest_robot = None

            for match in available_robots:
                start = match.pose.location
                if len(match.todo) > 0:
                    last_task = match.todo[-1]
                    if not isinstance(last_task, tasks.Explore): start = last_task.location

                distance = self.flow.distance(start, task.location)
                if distance is None: continue

                score = distance
                if len(match.todo) > 0 and not isinstance(match.todo[-1], tasks.Explore):
                    score += 100 * len(match.todo)

                if score < min_score:
                    min_score = score
                    closest_robot = match

            if closest_robot is not None:
                self._assign_retrieve_task(task, closest_robot)
                assigned_tasks.append(task)

        for task in assigned_tasks:
            del self.open_retrieval_tasks[task.id]
            self.active_retrieval_tasks[task.id] = task

        # assign explore task to any robots not already allocated
        assigned_points = []

        for id, robot in robots.items():
            if len(robot.todo) > 0: continue

            max_distance = 0
            farthest_point = None

            for row, col in self.map.frontier:
                center_location = self.map.get_center_location(row, col)
                if center_location in assigned_points: continue

                distance = self.flow.distance(robot.pose.location, center_location)
                if distance is None: continue

                if distance >= max_distance:
                    max_distance = distance
                    farthest_point = center_location

            if farthest_point is not None:
                robot.todo.append(tasks.Explore(farthest_point))
                assigned_points.append(farthest_point)

class Map:
    def __init__(self):
        self.resolution = 1000

        self.grid = {}
        self.frontier = set()

        self._carrier_y = 0

    # Notify
    def notify_movement(self, dy):
        self._carrier_y += dy

    def notify_grid(self, robot_pose, robot_grid):
        pose_row, pose_col = self.get_containing_cell(robot_pose.x, robot_pose.y)

        frontier_candidates = []

        for cell in robot_grid:
            row, col = cell

            global_cell = (
                row + pose_row,
                col + pose_col
            )

            scores = robot_grid[cell]

            if global_cell not in self.grid:
                frontier_candidates.append(global_cell)
                self.grid[global_cell] = scores

        if len(self.frontier) > 0:
            frontier_max_row = max(map(lambda c: c[0], self.frontier))

            explored = []
            for cell in self.frontier:
                if cell[0] < frontier_max_row:
                    explored.append(cell)

            for cell in explored: self.frontier.remove(cell)

        frontier_max_row = max(map(lambda c: c[0], self.frontier)) if len(self.frontier) > 0 else 0

        for candidate in frontier_candidates:
            if candidate[0] > frontier_max_row:
                self.frontier.add(candidate)

    # Util
    def get_containing_cell(self, x, y):
        global_y = y + self._carrier_y
        return (
            int(global_y / self.resolution),
            int(x / self.resolution)
        )

    def get_center_location(self, row, col):
        return common.Location(
            (col + 0.5) * self.resolution,
            (row + 0.5) * self.resolution - self._carrier_y
        )

    # Neighbors
    def get_cardinal_neighbors(self, cell):
        raise NotImplementedError

    def get_all_neighbors(self, cell):
        row, col = cell

        neighbors = []
        for r in range(row - 1, row + 2):
            for c in range(col - 1, col + 2):
                neighbor = (r, c)

                if neighbor not in self.grid: continue
                if neighbor == cell: continue

                neighbors.append(neighbor)

        return neighbors

    def get_all_possible_neighbors(self, cell):
        row, col = cell

        neighbors = []
        for r in range(row - 1, row + 2):
            for c in range(col - 1, col + 2):
                neighbor = (r, c)

                if neighbor == cell: continue

                neighbors.append(neighbor)

        return neighbors

    # Scan
    def all(self):
        for tup in self.grid.items():
            yield tup

class Robot:
    def __init__(self, pose):
        self.pose = pose
        self.bin = 0
        self.charge = 100
        self.todo = []
        self.end_effector = None

class PathPlanner:
    def __init__(self, map):
        self.map = map

        self.spans = [(start, start + 6000) for start in range(1000, 19000, 6000)]
        #self.spans = [(0, 19000)]

    def _get_lane(self, position):
        for span in self.spans:
            if span[0] <= position.x < span[1]: return span

    def distance(self, start, goal):
        """
        Calculate the distance between the start and goal based on the flow
        A value of None indicates that there is no path

        In this case, there is no path across lanes and within lanes it's simple Euclidean distance
        # TODO: Depending on the average terrain it may be possible to get away with simple Taxicab distance
        """
        start_lane = self._get_lane(start)
        goal_lane = self._get_lane(goal)

        if start_lane != goal_lane: return None

        penalty = 2 if goal.y < start.y else 1
        return penalty * start.distance(goal)

    def reconstruct_path(self, parents_by_child, goal_spot):
        path = [goal_spot]; current_spot = goal_spot

        while current_spot in parents_by_child:
            current_spot = parents_by_child[current_spot]
            path.append(current_spot)

        path.reverse()
        return path

    def _get_traversal_cost(self, current_spot, neighbor_spot):
        scores = self.map.grid[current_spot]

        dr = neighbor_spot[0] - current_spot[0]
        dc = neighbor_spot[1] - current_spot[1]

        total = abs(dr + dc)

        if total == 0: return math.copysign(scores[3], dr)
        if total == 2: return math.copysign(scores[1], dr)

        if dc == 0: return math.copysign(scores[2], dr)
        elif dr == 0: return math.copysign(scores[0], dc)

    @debug.profiled
    def plan(self, start_pose, goal_pose):
        start_spot = self.map.get_containing_cell(start_pose.x, start_pose.y)
        goal_spot = self.map.get_containing_cell(goal_pose.x, goal_pose.y)
        goal_center = self.map.get_center_location(*goal_spot)

        if start_spot == goal_spot:
            angle = math.degrees(
                math.atan2(
                    goal_pose.y - start_pose.y,
                    goal_pose.x - start_pose.x
                )
            )
            return [start_pose, common.Pose(goal_pose.x, goal_pose.y, angle)]

        count = 0
        open_queue = queue.PriorityQueue()

        open_queue.put((0, count, start_spot))
        parents_by_child = {}

        g_score = dict()
        g_score[start_spot] = 0

        f_score = dict()
        f_score[start_spot] = goal_center.distance(self.map.get_center_location(*start_spot))

        open_set = {start_spot}

        while not open_queue.empty():
            f, c, current_spot = open_queue.get()
            open_set.remove(current_spot)

            if current_spot == goal_spot:
                grid_path = self.reconstruct_path(parents_by_child, goal_spot)

                previous = self.map.get_center_location(*grid_path.pop(0))

                waypoint_path = []
                for r, c in grid_path:
                    current = self.map.get_center_location(r, c)
                    heading = math.degrees(
                        math.atan2(
                            current.y - previous.y,
                            current.x - previous.x
                        )
                    )
                    waypoint_path.append(common.Pose(current.x, current.y, heading))
                    previous = current

                initial_heading = math.degrees(
                    math.atan2(
                        waypoint_path[0].y - start_pose.y,
                        waypoint_path[0].x - start_pose.x
                    )
                )
                waypoint_path[0].a = initial_heading
                waypoint_path.insert(0, start_pose)

                waypoint_path.pop()
                final_heading = math.degrees(
                    math.atan2(
                        goal_pose.y - waypoint_path[-1].y,
                        goal_pose.x - waypoint_path[-1].x
                    )
                )
                waypoint_path.append(common.Pose(goal_pose.x, goal_pose.y, final_heading))

                return waypoint_path

            for neighbor_spot in self.map.get_all_neighbors(current_spot):
                traversal_cost = self._get_traversal_cost(current_spot, neighbor_spot)

                temp_g_score = g_score[current_spot] + traversal_cost

                if neighbor_spot not in g_score or temp_g_score < g_score[neighbor_spot]:
                    if current_spot in parents_by_child and parents_by_child[current_spot] == neighbor_spot: continue

                    parents_by_child[neighbor_spot] = current_spot
                    g_score[neighbor_spot] = temp_g_score

                    h_score = goal_center.distance(self.map.get_center_location(*neighbor_spot))
                    f_score[neighbor_spot] = temp_g_score + h_score

                    if neighbor_spot not in open_set:
                        count += 1
                        open_queue.put(
                            (
                                f_score[neighbor_spot],
                                count,
                                neighbor_spot
                            )
                        )
                        open_set.add(neighbor_spot)

        return None

class WorkerUnitCoordinator:
    def __init__(self):
        self.map = Map()
        self.flow = PathPlanner(self.map)
        self.tasks = CarrierQueue(self.map, self.flow)
        self.robots = {}

        self._carrier_speed = 0

    # Sync
    def sync_carrier(self, dy, vy):
        self._carrier_speed = vy

        self.map.notify_movement(dy)
        self.tasks.notify_movement(dy)

    def sync_poses(self, poses):
        for id, pose in poses.items():
            try: self.robots[id].pose = pose
            except KeyError: self.robots[id] = Robot(pose)

    def sync_robot(self, id, charge, bin, end_effector, grid, discoveries, ready):
        robot = self.robots[id]

        robot.charge = charge
        robot.bin = bin
        robot.end_effector = end_effector

        self.map.notify_grid(robot.pose, grid)
        self.tasks.notify_discoveries(robot.pose, discoveries)

        if ready and len(robot.todo) > 0:
            finished_task = robot.todo.pop(0)
            debug.log(f"{id} finished task {finished_task}")

            if isinstance(finished_task, tasks.Retrieve):
                del self.tasks.active_retrieval_tasks[finished_task.id]
                self.tasks.finished_retrieval_tasks[finished_task.id] = finished_task

    # Run
    @debug.profiled
    def get(self):
        debug.log("Planning")
        plan = {}

        self.tasks.allocate(self.robots)
        for id, robot in self.robots.items():
            debug.log(f"{id} tasks {robot.todo}")

        for id, robot in self.robots.items():
            debug.log(f"{id} pose: {robot.pose}, bin: {robot.bin}%, charge: {robot.charge}%")

            # If the first task is queued, start it
            if len(robot.todo) > 0 and robot.todo[0].state == tasks.QUEUED:
                next_task = robot.todo[0]
                debug.log(f"{id} starting task {next_task}")

                waypoints = self.flow.plan(robot.pose, next_task.location)
                debug.log(f"{id} waypoints {waypoints}")

                if waypoints is None:
                    print(f"[ERROR]: No path found for {id} from {robot.pose} to {next_task.location}!")
                    continue

                commands = []
                previous_pose = waypoints.pop(0)

                for waypoint_absolute in waypoints:
                    waypoint_relative = waypoint_absolute.relative_to(previous_pose)

                    if waypoint_relative.x != 0 or waypoint_relative.y != 0:
                        commands.append(cmd.Command("Move", (waypoint_relative.x, waypoint_relative.y)))

                    elif waypoint_relative.a != 0:
                        commands.append(cmd.Command("Rotate", waypoint_relative.a))

                    previous_pose = waypoint_absolute

                if isinstance(next_task, tasks.Retrieve):
                    commands.append(cmd.Command("Pick", (next_task.id, next_task.volume)))

                # elif isinstance(next_task, tasks.Service):
                #     # TODO: These are different than the threshold values because it's opportunistic, verify these
                #     if robot.charge <= 50:
                #         commands.append(cmd.Command("Charge", None))
                #
                #     if robot.bin >= 75:
                #         commands.append(cmd.Command("Exchange", None))

                debug.log(f"{id} commands {commands}")
                next_task.state = tasks.ACTIVE
                plan[id] = commands

        return plan
