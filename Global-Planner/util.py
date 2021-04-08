def clip(low, value, high):
    if value < low: return low
    if value > high: return high
    return value
