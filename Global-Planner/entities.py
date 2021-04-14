import abc
import math

import actions
import pose
import pygame
import tf

class Entity(abc.ABC):
    def __init__(self):
        self.pose = pose.Pose2D(0, 0, 0)
        self.dimensions = pygame.Vector2(0, 0)

class Carrier(Entity):
    def __init__(self, speed):
        super(Carrier, self).__init__()
        self.dimensions.update(2550, 11950)

        self.speed = speed

class Husky(Entity):
    def __init__(self, full_charge, lidar_range, lidar_arc, yolo_range, yolo_arc):
        super(Husky, self).__init__()
        self.dimensions.update(990, 660)

        self.action = None

        self.charge = full_charge
        self.full_charge = full_charge

        self.lidar_range = lidar_range
        self.lidar_arc = lidar_arc

        self.yolo_range = yolo_range
        self.yolo_arc = yolo_arc

        self._half_yolo_arc = self.yolo_arc / 2

        self.end_effector = None

    # Actions
    def execute(self, command):
        self.action = actions.create(command)

    def update(self, dt):
        if self.action is None: return
        self.action.run(dt, self)

    # LIDAR
    def get_partial_grid(self, world):
        hops = self.lidar_range // world.resolution

        pose_row = int(self.pose.y / world.resolution)
        pose_col = int(self.pose.x / world.resolution)

        grid = {}
        for r in range(-hops, hops + 1):
            for c in range(-hops, hops + 1):
                access_row = r + pose_row
                access_col = c + pose_col

                if not (0 <= access_row < world.rows): continue
                if not (0 <= access_col < world.cols): continue

                # TODO: Simulate occupancy
                scores = world.terrain[access_row][access_col]
                grid[(r, c)] = scores

        return grid

    # Classifier
    def get_visible_litter(self, all_litter):
        visible_litter = []
        for trash in all_litter:
            if self.pose.distance(trash.pose) < self.yolo_range:
                pose_relative_to_robot = tf.relative(self.pose, trash.pose)

                angle = math.degrees(math.atan2(pose_relative_to_robot.y, pose_relative_to_robot.x))
                if not (-self._half_yolo_arc <= angle <= self._half_yolo_arc):
                    continue

                pose_relative_to_robot.x = int(pose_relative_to_robot.x)
                pose_relative_to_robot.y = int(pose_relative_to_robot.y)

                visible_litter.append(
                    (
                        pose_relative_to_robot,
                        trash.type,
                        trash.certainty
                    )
                )

        return visible_litter

class Trash(Entity):
    def __init__(self, type, certainty):
        super(Trash, self).__init__()
        self.dimensions.update(100, 100)

        self.type = type
        self.certainty = certainty
