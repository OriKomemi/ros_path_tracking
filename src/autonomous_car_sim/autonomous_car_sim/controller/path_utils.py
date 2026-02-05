import math
from typing import Optional, Tuple

from nav_msgs.msg import Path
from .math_utils import hypot


def speed_from_vxy(vx: float, vy: float) -> float:
    return float(math.hypot(vx, vy))


def closest_index(x: float, y: float, path: Path) -> Optional[int]:
    if not path.poses:
        return None

    best_i = 0
    best_d = float('inf')
    for i, ps in enumerate(path.poses):
        dx = ps.pose.position.x - x
        dy = ps.pose.position.y - y
        d = hypot(dx, dy)
        if d < best_d:
            best_d = d
            best_i = i
    return best_i


def curvature_from_three_points(p1: Tuple[float, float], p2: Tuple[float, float], p3: Tuple[float, float]) -> float:
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3

    a = hypot(x2 - x1, y2 - y1)
    b = hypot(x3 - x2, y3 - y2)
    c = hypot(x1 - x3, y1 - y3)

    area2 = abs((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1))  # 2*Area
    if area2 < 1e-6 or a < 1e-6 or b < 1e-6 or c < 1e-6:
        return 0.0

    return (2.0 * area2) / (a * b * c)  # kappa = 1/R


def curvature_at_index(path: Path, idx: int) -> float:
    n = len(path.poses)
    if n < 3:
        return 0.0

    i1 = max(0, idx - 2)
    i2 = idx
    i3 = min(n - 1, idx + 2)

    p1 = path.poses[i1].pose.position
    p2 = path.poses[i2].pose.position
    p3 = path.poses[i3].pose.position

    return curvature_from_three_points((p1.x, p1.y), (p2.x, p2.y), (p3.x, p3.y))


def find_lookahead_point(x: float, y: float, path: Path, lookahead_distance: float) -> Tuple[Optional[object], Optional[int]]:
    if not path.poses:
        return None, None

    ci = closest_index(x, y, path)
    if ci is None:
        return None, None

    for i in range(ci, len(path.poses)):
        ps = path.poses[i]
        d = hypot(ps.pose.position.x - x, ps.pose.position.y - y)
        if d >= lookahead_distance:
            return ps, i

    return path.poses[-1], len(path.poses) - 1


def pure_pursuit_delta(
    x: float, y: float, yaw: float,
    target_pose_stamped,
    wheelbase: float
) -> float:
    dx = target_pose_stamped.pose.position.x - x
    dy = target_pose_stamped.pose.position.y - y

    tx = math.cos(yaw) * dx + math.sin(yaw) * dy
    ty = -math.sin(yaw) * dx + math.cos(yaw) * dy

    ld = math.hypot(tx, ty)
    if ld < 1e-3:
        return 0.0

    alpha = math.atan2(ty, tx)
    return math.atan2(2.0 * wheelbase * math.sin(alpha), ld)
