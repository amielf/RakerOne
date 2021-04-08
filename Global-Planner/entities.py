import abc
import actions
import math
import pose
import pygame
import random
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
    def __init__(self, lidar_range, yolo_range):
        super(Husky, self).__init__()
        self.dimensions.update(990, 660)

        self.action = None

        self.lidar_range = lidar_range
        self.yolo_range = yolo_range

    # Actions
    def execute(self, command):
        self.action = actions.create(command)

    def update(self, dt):
        if self.action is None: return
        self.action.run(dt, self)

    # LIDAR
    def get_partial_grid(self, world):
        hops = self.lidar_range // world.resolution

        pose_col = int(self.pose.x // world.resolution)
        pose_row = int(self.pose.y // world.resolution)

        grid = {}
        for r in range(-hops, hops):
            for c in range(-hops, hops):
                access_row = r + pose_row
                access_col = c + pose_col

                if access_row < 0 or world.rows <= access_row: continue
                if access_col < 0 or world.cols <= access_col: continue

                score = world.terrain[access_row][access_col]

                grid[(r, c)] = score

        return grid

    # Classifier
    def get_visible_litter(self, all_litter):
        litter = []
        for trash in all_litter:
            # TODO: Implement true camera visibility shape
            if self.pose.distance(trash.pose) < self.yolo_range:
                pose_relative_to_robot = tf.relative(self.pose, trash.pose)
                litter.append(
                    (
                        pose_relative_to_robot,
                        trash.type,
                        trash.certainty
                    )
                )

        return litter

class Trash(Entity):
    def __init__(self, type, certainty):
        super(Trash, self).__init__()
        self.dimensions.update(100, 100)

        self.type = type
        self.certainty = certainty
