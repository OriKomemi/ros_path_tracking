import math


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(x, hi))


def hypot(dx: float, dy: float) -> float:
    return math.hypot(dx, dy)
