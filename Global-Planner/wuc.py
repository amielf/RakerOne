import common
import cmd
import debug
import math
import queue
import random
import tasks

class LocationTracker:
    def __init__(self):
        self._poses = []

    def track(self, pose):
        self._poses.append(pose)

    def remove(self, pose):
        self._poses.remove(pose)

    def update(self, dy):
        for pose in self._poses:
            pose.y -= dy

class CarrierQueue:
    def __init__(self, map, flow):
        self.map = map
        self.flow = flow

        self.location_tracker = LocationTracker()
        self.retrieval_task_queue = []

        self._known_trash_ids = set()
        self._max_assignments_per_robot = 3

    def _assign_service_tasks(self, robots):
        for id, robot in robots.items():
            if robot.charge <= 25 or robot.bin >= 99:
                if len(robot.todo) > 0:
                    current_active_task = robot.todo[0]

                    # If already doing a Service task, leave it alone
                    if isinstance(current_active_task, tasks.Service): continue

                    # If taking a Retrieve task out of assignment, add it to the front of the queue
                    if isinstance(current_active_task, tasks.Retrieve):
                        self.retrieval_task_queue.insert(0, current_active_task)

                robot.todo.insert(0, tasks.Service(robot.charge, robot.bin))

    # Notify
    def notify_discoveries(self, robot_pose, discoveries):
        for discovery in discoveries:
            id, location_relative, type, volume, certainty, end_effectors = discovery

            # Having the ID is cheating, since in real life there's no way to identify before discovery
            # This would normally be handled by some form of geo-caching
            if id in self._known_trash_ids: continue
            if certainty < 0.3: continue

            location_absolute = robot_pose.get_absolute(location_relative)
            self.retrieval_task_queue.append(tasks.Retrieve(id, location_absolute, type, volume, end_effectors))
            self.location_tracker.track(location_absolute)
            self._known_trash_ids.add(id)

    @debug.profiled
    def allocate(self, robots):
        robots_list = list(robots.values())
        random.shuffle(robots_list)

        # Clear out exploration tasks; the map has likely updated anyway
        for robot in robots_list:
            if len(robot.todo) > 0 and isinstance(robot.todo[0], tasks.Explore):
                robot.todo.pop(0)

        # self._assign_service_tasks(robots)

        remove_indices = []

        self.retrieval_task_queue.sort(key =  lambda t: t.location.y)

        for i in range(len(self.retrieval_task_queue)):
            task = self.retrieval_task_queue[i]

            candidate_robots = [robot for robot in robots_list if robot.end_effector in task.skills]
            if len(candidate_robots) == 0:
                print(f"[ERROR]: {task.type} cannot be retrieved from {task.location} by any robot; requires {task.skills}.")
                continue

            available_robots = [robot for robot in candidate_robots if len(robot.todo) < self._max_assignments_per_robot]

            min_score = 999999
            closest_robot = None

            for match in available_robots:
                start = match.todo[-1].location if len(match.todo) > 0 else match.pose.location

                distance = self.flow.distance(start, task.location)
                if distance is None: continue

                score = distance + 1000 * len(match.todo)
                if score < min_score:
                    min_score = score
                    closest_robot = match

            if closest_robot is not None:
                closest_robot.todo.append(task)
                remove_indices.append(i)

        remove_indices.reverse()
        for i in remove_indices:
            self.retrieval_task_queue.pop(i)

        # assign explore task to any robots not already allocated
        assigned_points = set()

        for robot in robots_list:
            if len(robot.todo) > 0: continue

            min_distance = 999999
            closest_point = None

            for row, col in self.map.frontier:
                center_location = self.map.get_center_location(row, col)
                if center_location in assigned_points: continue

                distance = self.flow.distance(robot.pose.location, center_location)
                if distance is None: continue

                if distance < min_distance:
                    min_distance = distance
                    closest_point = center_location

            if closest_point is not None:
                robot.todo.append(tasks.Explore(closest_point))
                assigned_points.add(closest_point)

class Map:
    def __init__(self):
        self.width = 18000
        self.height = 3219000
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

        # lane_cells = 6000
        # self.lanes = [(start, start + lane_cells) for start in range(0, self.map.width, lane_cells)]
        self.lanes = [(0, self.map.width)]

    def _get_lane(self, position):
        for span in self.lanes:
            if span[0] <= position.x < span[1]: return span

        return None

    def distance(self, start, goal):
        start_lane = self._get_lane(start)
        goal_lane = self._get_lane(goal)

        # Robots may not cross lanes, indicated as a null distance
        if start_lane != goal_lane: return None

        # Double backward movement distance to cover the travel down and back
        multiplier = 2 if goal.y < start.y else 1
        return multiplier * start.distance(goal)

    def _build_path(self, parents_by_child, goal_spot):
        grid_path = [goal_spot]; current_spot = goal_spot

        while current_spot in parents_by_child:
            current_spot = parents_by_child[current_spot]
            grid_path.append(current_spot)

        grid_path.reverse()

        previous_waypoint = self.map.get_center_location(*grid_path.pop(0))

        waypoint_path = []
        for r, c in grid_path:
            current = self.map.get_center_location(r, c)
            heading = math.degrees(
                math.atan2(
                    current.y - previous_waypoint.y,
                    current.x - previous_waypoint.x
                )
            )
            waypoint_path.append(common.Pose(current.x, current.y, heading))
            previous_waypoint = current

        return waypoint_path

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
                path = self._build_path(parents_by_child, goal_spot)

                # Add the initial movement from the start pose to the SECOND grid cell center in the path
                initial_heading = math.degrees(
                    math.atan2(
                        path[0].y - start_pose.y,
                        path[0].x - start_pose.x
                    )
                )
                path[0].a = initial_heading
                path.insert(0, start_pose)

                # Remove the final grid cell center and replace it with the goal pose
                path.pop()
                final_heading = math.degrees(
                    math.atan2(
                        goal_pose.y - path[-1].y,
                        goal_pose.x - path[-1].x
                    )
                )
                path.append(common.Pose(goal_pose.x, goal_pose.y, final_heading))

                return path


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
        self.tasks.location_tracker.update(dy)

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
            self.tasks.location_tracker.remove(finished_task.location)
            debug.log(f"{id} finished task {finished_task}")

    # Run
    def get(self):
        debug.log("Planning")
        plan = {}

        self.tasks.allocate(self.robots)

        for id, robot in self.robots.items():
            debug.log(f"{id} pose: {robot.pose}, bin: {robot.bin}%, charge: {robot.charge}%")
            debug.log(f"{id} tasks: {robot.todo}")

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
