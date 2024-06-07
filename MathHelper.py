from math import sqrt
from typing import Iterable

from Types import Point, Line


def distance_point_to_point(p1: Point, p2: Point) -> float:
    return sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)


def distance_point_to_point2(p1: Point, p2: Point) -> float:
    return (p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2


def get_closest_point(p: Point, points: Iterable[Point]) -> Point:
    return min(points, key=lambda x: distance_point_to_point2(p, x))


def get_furthest_point(p: Point, points: Iterable[Point]) -> Point:
    return max(points, key=lambda x: distance_point_to_point2(p, x))


def distance_point_to_line(point: Point, line: Line) -> float:
    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#:~:text=horizontal%20line%20segment.-,Line%20defined%20by%20two%20points,-%5Bedit%5D
    # Positive is left of the line, negative to the right
    a2 = (line.x2 - line.x1) * (point.y - line.y1) - (point.x - line.x1) * (line.y2 - line.y1)
    b = distance_point_to_point(line.p1, line.p2)
    return a2 / b


def line_intersection_t_u(l1: Line, l2: Line) -> tuple[float, float]:
    # https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#:~:text=denominator%20is%20zero.-,Given%20two%20points%20on%20each%20line%20segment,-%5Bedit%5D
    divider = (l1.x1 - l1.x2) * (l2.y1 - l2.y2) - (l1.y1 - l1.y2) * (l2.x1 - l2.x2)
    if divider == 0.0:
        return float('inf'), float('inf')

    t = ((l1.x1 - l2.x1) * (l2.y1 - l2.y2) - (l1.y1 - l2.y1) * (l2.x1 - l2.x2)) / divider
    u = - ((l1.x1 - l1.x2) * (l1.y1 - l2.y1) - (l1.y1 - l1.y2) * (l1.x1 - l2.x1)) / divider
    return t, u


def line_intersection_t(l1: Line, l2: Line) -> float:
    try:
        return (((l1.x1 - l2.x1) * (l2.y1 - l2.y2) - (l1.y1 - l2.y1) * (l2.x1 - l2.x2)) /
                ((l1.x1 - l1.x2) * (l2.y1 - l2.y2) - (l1.y1 - l1.y2) * (l2.x1 - l2.x2)))
    except ZeroDivisionError:
        return float('inf')


def line_intersection(l1: Line, l2: Line) -> Point:
    t = line_intersection_t(l1, l2)
    return Point(l1.x1 + t * (l1.x2 - l1.x1), l1.y1 + t * (l1.y2 - l1.y1))


# def get_slope(line: Line) -> float:
#     try:
#         return (line.y2 - line.y1) / (line.x2 - line.x1)
#     except ZeroDivisionError:
#         return float('inf')


# def hit_coords_point_to_line(p: Point, line: Line) -> Point:
#     try:
#         perpendicular_slope = -1 / get_slope(line)
#     except ZeroDivisionError:
#         # Slope is 0, so same x as p and same y as line
#         return Point(p.x, line.y1)
#
#     p_line = Line(p, Point(p.x + 1, p.y + perpendicular_slope))
#     t = line_intersection_t(p_line, line)
#
#     return Point(p.x + t, p.y + t * perpendicular_slope)


# def point_to_line_t(p: Point, line: Line) -> float:
#     try:
#         perpendicular_slope = -1 / get_slope(line)
#     except ZeroDivisionError:
#         # Slope is 0, so p.x as a part (percentage) from x1 to x2
#         return (line.x2 - p.x) / (line.x2 - line.x1)
#
#     p_line = Line(p, Point(p.x + 1, p.y + perpendicular_slope))
#     return line_intersection_t(line, p_line)


def point_to_line_t_slope(p: Point, line: Line, slope_inv: float) -> float:
    if slope_inv == float('inf'):
        # Slope is 0, so p.x as a part (percentage) from x1 to x2
        return (line.x2 - p.x) / (line.x2 - line.x1)

    p_line = Line(p, Point(p.x + 1, p.y + slope_inv))
    return line_intersection_t(line, p_line)


def get_tangent_points(p: Point, c: Point, radius: float) -> tuple[Point, Point]:
    # https://stackoverflow.com/a/69641745/21797739
    # https://www.desmos.com/calculator/6u8fzo9tx9
    dx = c.x - p.x
    dy = c.y - p.y
    pc2 = dx ** 2 + dy ** 2
    cr2 = radius ** 2
    if pc2 < cr2:
        return Point(float('inf'), float('inf')), Point(float('inf'), float('inf'))

    pr2 = pc2 - cr2
    pc = sqrt(pc2)
    d = pr2 / pc  # Distance from p to x0, which is in line with tangent points x1 and x2
    h = sqrt(pr2 - d ** 2)  # Distance from x0 to x1 and x2
    x1 = Point(p.x + (dx * d - dy * h) / pc, p.y + (dy * d + dx * h) / pc)
    x2 = Point(p.x + (dx * d + dy * h) / pc, p.y + (dy * d - dx * h) / pc)
    return x1, x2
