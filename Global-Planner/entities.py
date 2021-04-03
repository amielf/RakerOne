import abc
import pose
import pygame

class Entity(abc.ABC):
    def __init__(self):
        self.pose = pose.Pose2D(0, 0, 0)
        self.dimensions = pygame.Vector2(0, 0)

class Carrier(Entity):
    def __init__(self):
        super(Carrier, self).__init__()
        self.dimensions.update(2550, 11950)

class Robot(Entity):
    def __init__(self):
        super(Robot, self).__init__()
        self.dimensions.update(990, 660)

class Trash(Entity):
    def __init__(self, type, certainty):
        super(Trash, self).__init__()
        self.dimensions.update(100, 100)

        self.type = type
        self.certainty = certainty
