import cmd
import debug
import tf
import math

class CarrierQueue:
    def __init__(self, unassigned_litter, robots, frontier_choices):
        self.retrieval_tasks = {}
        self.explore_tasks = set()
        self.unassigned_litter = unassigned_litter
        self.robots = robots
        self.frontier_choices = frontier_choices

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

    def create_service_tasks(self):
        pass


    def allocate(self, unassigned_litter, robots, frontier_choices):
        """
        method for assigning discovered litter items to a robot to_do lists
        """
        # list to track which robots have received assignments this round
        allocated_robots=[]

        # Check to see if robot needs to return for service
        for robot in robots:
            if robot.bin >= 8:
                # create a service task
                # Service((0,0,180), robot)
                if robot not in allocated_robots:
                    allocated_robots.append(robot)
            if robot.charge <= 25:
                # create a service task
                # Service((0,0,180), robot)
                if robot not in allocated_robots:
                    allocated_robots.append(robot)

        # assign retrieval tasks to Robots
        for item in unassigned_litter:
            matches = []
            for robot in robots:
               # match litter skill to robot skill distance to litter used for tie breaker
                if robot.skill in item.skills:
                    matches.append(robot)
            if len(matches) == 0:
                print ("litter cannot be retrieved at Location" + str(item.location))
                # need to flag this item. Keep in a list?
            if len(matches) == 1:
                # Create a retrival task
                if matches[0] not in allocated_robots:
                    allocated_robots.append(matches[0])
            else:
                distance = 1000
                for robot in matches:
                    dist = math.sqrt((item.location[0] - robot.pose[0]) ** 2 + (item.location[1] - robot.pose[1]) ** 2)
                    if dist <= distance:
                        distance = dist
                        choice = robot
                # create retrieval task for choice
                choice.add_task(item.location)
                if choice not in allocated_robots:
                    allocated_robots.append(choice)

        #assign explore task to any robots not already allocated
        for robot in robots:
            if robot not in allocated_robots:
                distance = 1000
                for point in frontier_choices:
                    dist = math.sqrt((point[0] - robot.pose[0]) ** 2 + (point[1] - robot.pose[1]) ** 2)
                    if dist <= distance:
                        distance = dist
                        choice = point
                    #create explore task for robot at choice

                    
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
    def __init__(self, pose, skill):
        self.pose = pose
        self.to_do = []
        self.commands = [cmd.Command("Move", (20000, 0))]
        self.charge = 100
        self.bin = 0
        self.skill = skill #end-effector installed

class Litter:
    def __init__(self, location, skills, sort):
        self.location = location # tuple location in global coord. of litter item
        self.skills = skills #list of end effectors able to retrieve item
        self.sort = sort # which bin item sorts to

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
