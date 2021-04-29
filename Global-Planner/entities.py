import abc
import actions
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

        self.speed = speed

class Husky(Entity):
    def __init__(self, full_charge, bin_capacity, lidar_range, lidar_arc, yolo_range, yolo_arc, linear_speed, rotational_speed):
        super(Husky, self).__init__()
        self.dimensions.update(990, 660)

        self.action = None

        self.charge = full_charge
        self.full_charge = full_charge

        self.bin = 0
        self.bin_capacity = bin_capacity

        self.lidar_range = lidar_range
        self.lidar_arc = lidar_arc

        self.yolo_range = yolo_range
        self.yolo_arc = yolo_arc

        self._half_lidar_arc = self.lidar_arc / 2
        self._half_yolo_arc = self.yolo_arc / 2

        self.end_effector = None

        self.linear_speed = linear_speed
        self.rotational_speed = rotational_speed

    # Actions
    def execute(self, command):
        self.action = actions.create(command)

    def update(self, dt):
        if self.action is None: return
        self.action.run(dt, self)

    # LIDAR
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
    def get_visible_litter(self, all_litter):
        visible_litter = []
        for trash in all_litter:
            if self.pose.distance(trash.pose.location) < self.yolo_range:
                pose_relative_to_robot = trash.pose.relative_to(self.pose)

                angle = math.degrees(math.atan2(pose_relative_to_robot.y, pose_relative_to_robot.x))
                if not (-self._half_yolo_arc <= angle <= self._half_yolo_arc):
                    continue

                visible_litter.append(
                    (
                        trash.id,
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

        self.discovered = False
