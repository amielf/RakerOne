import math
import pose

def absolute(frame, relative_pose):
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

def relative(frame, absolute_pose):
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
        dx * cos - dy * sin,
        dx * sin + dy * cos,
        da
    )