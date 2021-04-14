import math

def distance(p1, p2):
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def clip(low, value, high):
    if value < low: return low
    if value > high: return high
    return value
