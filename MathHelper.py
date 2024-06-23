from math import sqrt, atan2, pi
from typing import Iterable

from Types import Point, Line


def distance(p1: Point, p2: Point) -> float:
    return sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)


def distance2(p1: Point, p2: Point) -> float:
    return (p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2


def get_closest_point(p: Point, points: Iterable[Point]) -> Point:
    return min(points, key=lambda x: distance2(p, x))


def get_furthest_point(p: Point, points: Iterable[Point]) -> Point:
    return max(points, key=lambda x: distance2(p, x))


def distance_point_to_line(point: Point, line: Line) -> float:
    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#:~:text=horizontal%20line%20segment.-,Line%20defined%20by%20two%20points,-%5Bedit%5D
    # Positive is left of the line, negative to the right
    a2 = (line.x2 - line.x1) * (point.y - line.y1) - (point.x - line.x1) * (line.y2 - line.y1)
    b = distance(line.p1, line.p2)
    try:
        return a2 / b
    except ZeroDivisionError:
        return distance(point, line.p1)


# t and u indicate how far along the lines the intersection is, 0 at the start, 1 at the end
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


def hit_coords_point_to_line(p: Point, line: Line) -> Point:
    if line.slope_inv == float('inf'):
        # Slope is 0, so same x as p and same y as line
        return Point(p.x, line.y1)

    p_line = Line(p, Point(p.x + 1, p.y + line.slope_inv))
    t = line_intersection_t(p_line, line)

    return Point(p.x + t, p.y + t * line.slope_inv)


def point_to_line_t(p: Point, line: Line) -> float:
    if line.slope_inv == float('inf'):
        return (p.x - line.x1) / (line.x2 - line.x1)

    p_line = Line(p, Point(p.x + 1, p.y + line.slope_inv))
    return line_intersection_t(line, p_line)


def offset_line(line: Line, offset: float) -> Line:
    dx = line.x2 - line.x1
    dy = line.y2 - line.y1
    d = sqrt(dx ** 2 + dy ** 2)
    dx /= d
    dy /= d
    return Line(
        line.x1 + offset * dy,
        line.y1 - offset * dx,
        line.x2 + offset * dy,
        line.y2 - offset * dx
    )


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


def get_closest_to_vertex(p1: Point, p2: Point, v: Point, op: Point, clearance: float) -> Point:
    # clearance = clearance + 0.0001
    tps1 = get_tangent_points(p1, v, clearance)
    tps2 = get_tangent_points(p2, v, clearance)

    # Take the outermost tangent point
    tp1 = get_closest_point(op, tps1)
    tp2 = get_closest_point(op, tps2)

    return line_intersection(Line(p1, tp1), Line(p2, tp2))


def get_point_to_vertex_tangent(p: Point, v: Point, op: Point, clearance: float) -> Line:
    tps = get_tangent_points(p, v, clearance)
    tp = get_closest_point(op, tps)
    return Line(p, tp)


def get_vertex_to_vertex_tangent(v1: Point, v2: Point, op: Point, clearance: float) -> Line:
    line_v_to_v = Line(v1, v2)
    # todo indirect and direct common tangents
    return offset_line(line_v_to_v, clearance)


def get_points_around_robot(p1: Point, p2: Point, rp: Point, clearance: float) -> tuple[Point, Point]:
    clearance = clearance + 0.0001
    tps1 = get_tangent_points(p1, rp, clearance)
    tps2 = get_tangent_points(p2, rp, clearance)

    # Take the outermost tangent point
    paired_tps1 = tps1[0], get_closest_point(tps1[0], tps2)
    paired_tps2 = tps1[1], get_closest_point(tps1[1], tps2)

    return line_intersection(Line(p1, paired_tps1[0]), Line(p2, paired_tps1[1])), \
        line_intersection(Line(p1, paired_tps2[0]), Line(p2, paired_tps2[1]))


def dot_product(p1: Point, p2: Point) -> float:
    return p1.x * p2.x + p1.y * p2.y


def dot_product_lines(l1: Line, l2: Line) -> float:
    return dot_product(Point(l1.x2 - l1.x1, l1.y2 - l1.y1), Point(l2.x2 - l2.x1, l2.y2 - l2.y1))


def cos_angle_between_lines(l1: Line, l2: Line) -> float:
    return dot_product_lines(l1, l2) / (l1.len * l2.len)


# Gives the length of the leg of an isosceles triangle
def leg_from_base_and_cos_angle(base: float, cos_angle: float) -> float:
    return (base / 2) / sqrt((1 - cos_angle) / 2)


# Gives the length of the leg of an isosceles triangle formed by two lines and a base
# It's -cos_angle because at the outer angle the second robot is still moving towards the first
def leg_from_base_and_lines(base: float, l1: Line, l2: Line) -> float:
    return leg_from_base_and_cos_angle(base, -cos_angle_between_lines(l1, l2))


def get_angle_to_y_axis(line: Line) -> float:
    return atan2(line.x2 - line.x1, line.y2 - line.y1)


def get_heading(line: Line) -> float:
    angle = get_angle_to_y_axis(line)
    return angle if angle >= 0 else 2*pi + angle


def remove_duplicates_ordered(li: list) -> list:
    return [li[i] for i in range(len(li)) if i == 0 or li[i] != li[i - 1]]
