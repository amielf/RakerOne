import math
import pose
import pygame

def absolute_pose(frame, relative_pose):
    """
    Given the pose relative to the frame, what would the frame's parent call the pose?
    For example, absolute(frame = (1, 1), relative_pose = (1, 1)) => (2, 2, 0)
    """
    rad = math.radians(frame.a)
    cos = math.cos(rad); sin = math.sin(rad)

    return pose.Pose2D(
        relative_pose.x * cos - relative_pose.y * sin + frame.x,
        relative_pose.x * sin + relative_pose.y * cos + frame.y,
        relative_pose.a - frame.a
    )

def absolute_position(frame, relative_position):
    rad = math.radians(frame.a)
    cos = math.cos(rad); sin = math.sin(rad)

    return pygame.Vector2(
        relative_position[0] * cos - relative_position[1] * sin + frame.x,
        relative_position[0] * sin + relative_position[1] * cos + frame.y
    )

def relative_pose(frame, absolute_pose):
    """
    Given the frame and the pose both relative to the same parent, what would the frame call the pose?
    For example, relative(frame = (1, 1, 0), absolute_pose = (2, 2, 0)) => (1, 1, 0)
    """
    dx = absolute_pose.x - frame.x
    dy = absolute_pose.y - frame.y
    da = absolute_pose.a - frame.a

    rad = math.radians(-frame.a)
    cos = math.cos(rad); sin = math.sin(rad)

    return pose.Pose2D(
        int(dx * cos - dy * sin),
        int(dx * sin + dy * cos),
        da
    )

def relative_position(frame, absolute_position):
    """
    Given the frame and the pose both relative to the same parent, what would the frame call the pose?
    For example, relative(frame = (1, 1, 0), absolute_pose = (2, 2, 0)) => (1, 1, 0)
    """
    dx = absolute_position[0] - frame.x
    dy = absolute_position[1] - frame.y

    rad = math.radians(-frame.a)
    cos = math.cos(rad); sin = math.sin(rad)

    return pygame.Vector2(
        int(dx * cos - dy * sin),
        int(dx * sin + dy * cos)
    )