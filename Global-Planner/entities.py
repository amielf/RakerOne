import abc
import actions
import debug
import math
import common
import pygame

class Entity(abc.ABC):
    def __init__(self):
        self.pose = common.Pose(0, 0, 0)
        self.dimensions = pygame.Vector2(0, 0)

class Carrier(Entity):
    def __init__(self, speed):
        super(Carrier, self).__init__()
        self.dimensions.update(2550, 11950)

        self._speed = speed
        self._dy = 0

    # Move
    def move(self, dt):
        dy = self._speed * dt
        self.pose.y += dy
        self._dy += dy

    # Monitor
    def get_monitor_data(self):
        data = (self._dy, self._speed)
        self._dy = 0
        return data

class Husky(Entity):
    def __init__(self, full_charge, bin_capacity, lidar_range, lidar_arc, yolo_range, yolo_arc, linear_speed, rotational_speed, end_effector):
        super(Husky, self).__init__()
        self.dimensions.update(990, 660)

        self.charge = float(full_charge)
        self.full_charge = full_charge

        self.bin = 0
        self.bin_capacity = float(bin_capacity)

        self.lidar_range = lidar_range
        self.lidar_arc = lidar_arc
        self.yolo_range = yolo_range
        self.yolo_arc = yolo_arc

        self.end_effector = end_effector

        self.linear_speed = linear_speed
        self.rotational_speed = rotational_speed

        self.actions = []
        self.finished_actions = []

        self._half_lidar_arc = self.lidar_arc / 2
        self._half_yolo_arc = self.yolo_arc / 2

    # Actions
    def execute(self, commands):
        self.actions.clear()
        for command in commands:
            self.actions.append(actions.create(command))

    def update(self, dt):
        if len(self.actions) == 0: return

        while dt > 0 and len(self.actions) > 0:
            elapsed = self.actions[0].run(dt, self)
            if self.actions[0].done:
                finished_action = self.actions.pop(0)
                self.finished_actions.append(finished_action)

            dt -= elapsed

    # LIDAR
    @debug.profiled
    def get_partial_grid(self, grid):
        hops = self.lidar_range // grid.resolution

        pose_row = int(self.pose.y / grid.resolution)
        pose_col = int(self.pose.x / grid.resolution)

        partial_grid = {}
        for r in range(-hops, hops + 1):
            for c in range(-hops, hops + 1):
                access_row = r + pose_row
                access_col = c + pose_col

                if not (0 <= access_row < grid.rows): continue
                if not (0 <= access_col < grid.cols): continue

                scores = grid.costs[access_row][access_col]
                partial_grid[(r, c)] = scores

        return partial_grid

    # Classifier
    @debug.profiled
    def get_visible_litter(self, litter):
        robot_bucket = (self.pose.x // 1000, self.pose.y // 1000)
        hops = self.yolo_range // 1000

        visible_litter = []
        for tid, trash in litter.items():
            if self.pose.distance(trash.pose.location) < self.yolo_range:
                pose_relative_to_robot = trash.pose.relative_to(self.pose)

                angle = math.degrees(math.atan2(pose_relative_to_robot.y, pose_relative_to_robot.x))
                if not (-self._half_yolo_arc <= angle <= self._half_yolo_arc):
                    continue

                visible_litter.append(
                    (
                        tid,
                        pose_relative_to_robot.location,
                        trash.type,
                        trash.volume,
                        trash.certainty,
                        trash.end_effectors
                    )
                )

        return visible_litter

class Trash(Entity):
    def __init__(self, id, type, volume, certainty, end_effectors):
        super(Trash, self).__init__()
        self.dimensions.update(100, 100)

        self.id = id
        self.type = type
        self.volume = volume
        self.certainty = certainty
        self.end_effectors = end_effectors
