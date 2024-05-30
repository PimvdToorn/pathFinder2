from math import sqrt
from typing import Any

from Types import Point, Line
from objects.Move import Move
from objects.Obstacle import Obstacle


def distance_point_to_point(p1: Point, p2: Point) -> float:
    return sqrt((p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2)


def distance_point_to_line(point: Point, line: Line) -> float:
    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#:~:text=horizontal%20line%20segment.-,Line%20defined%20by%20two%20points,-%5Bedit%5D
    # Positive is left of the line, negative to the right
    a2 = (line.x2 - line.x1) * (point.y - line.y1) - (point.x - line.x1) * (line.y2 - line.y1)
    b = distance_point_to_point(line.p1, line.p2)
    return a2 / b


def get_slope(line: Line) -> float:
    try:
        return (line.y2 - line.y1) / (line.x2 - line.x1)
    except ZeroDivisionError:
        return float('inf')


def line_intersection_t_u(l1: Line, l2: Line) -> tuple[float, float]:
    # https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#:~:text=denominator%20is%20zero.-,Given%20two%20points%20on%20each%20line%20segment,-%5Bedit%5D
    divider = (l1.x1 - l1.x2) * (l2.y1 - l2.y2) - (l1.y1 - l1.y2) * (l2.x1 - l2.x2)
    if divider == 0.0:
        return float('inf'), float('inf')

    t = ((l1.x1 - l2.x1) * (l2.y1 - l2.y2) - (l1.y1 - l2.y1) * (l2.x1 - l2.x2)) / divider
    u = - ((l1.x1 - l1.x2) * (l1.y1 - l2.y1) - (l1.y1 - l1.y2) * (l1.x1 - l2.x1)) / divider
    return t, u


def line_intersection_t(line1: Line, line2: Line) -> float:
    # https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#:~:text=denominator%20is%20zero.-,Given%20two%20points%20on%20each%20line%20segment,-%5Bedit%5D
    return (((line1.x1 - line2.x1) * (line2.y1 - line2.y2) -
             (line1.y1 - line2.y1) * (line2.x1 - line2.x2)) /
            ((line1.x1 - line1.x2) * (line2.y1 - line2.y2) -
             (line1.y1 - line1.y2) * (line2.x1 - line2.x2)))


def line_intersection(line1: Line, line2: Line) -> Point:
    t = line_intersection_t(line1, line2)
    return Point(line1.x1 + t * (line1.x2 - line1.x1), line1.y1 + t * (line1.y2 - line1.y1))


def hit_coords_point_to_line(p: Point, line: Line) -> Point:
    try:
        perpendicular_slope = -1 / get_slope(line)
    except ZeroDivisionError:
        # Slope is 0, so same x as p and same y as line
        return Point(p.x, line.y1)

    p_line = Line(p, Point(p.x + 1, p.y + perpendicular_slope))
    t = line_intersection_t(p_line, line)

    return Point(p.x + t, p.y + t * perpendicular_slope)


def point_to_line_t(p: Point, line: Line) -> float:
    try:
        perpendicular_slope = -1 / get_slope(line)
    except ZeroDivisionError:
        # Slope is 0, so p.x as a part (percentage) from x1 to x2
        return (line.x2 - p.x) / (line.x2 - line.x1)

    p_line = Line(p, Point(p.x + 1, p.y + perpendicular_slope))
    return line_intersection_t(line, p_line)


def does_intersect(move: Move, obstacle: Obstacle) -> bool:
    # Check the "bounding box"
    if not (
            move.x_min < obstacle.x_max and
            move.x_max > obstacle.x_min and
            move.y_min < obstacle.y_max and
            move.y_max > obstacle.y_min):
        print("Out of bound")
        return False

    for corner in obstacle.corners:
        if (corner.x < move.x_min or corner.x > move.x_max or
                corner.y < move.y_min or corner.y > move.y_max):
            continue

        distance = distance_point_to_line(corner, move.line)
        if abs(distance) < move.clearance:
            print(f"Corner too close: {corner}, distance: {distance}")
            return True

    for vertex in obstacle.vertices:
        t, u = line_intersection_t_u(move.line, vertex)
        if 0 < t < 1 and 0 < u < 1:
            print(f"Hit, t: {t}, u: {u}, vertex: {vertex}, at: {line_intersection(move.line, vertex)}")
            return True

        # for end_point in [move.start, move.end]:
        #     if (end_point.x + move.clearance < vertex.x_min or end_point.x - move.clearance > vertex.x_max or
        #             end_point.y + move.clearance < vertex.y_min or end_point.y - move.clearance > vertex.y_max):
        #         continue
        #
        #     distance = distance_point_to_line(end_point, vertex)
        #     if abs(distance) < move.clearance:
        #         print(f"End point too close: {end_point}, distance: {distance}, at: {line_intersection(move.line, vertex)}")
        #         return True

    return False


def first_intersection(move: Move, obstacle: Obstacle) -> tuple[float, Any]:
    # Check the "bounding box"
    if not (
            move.x_min < obstacle.x_max and
            move.x_max > obstacle.x_min and
            move.y_min < obstacle.y_max and
            move.y_max > obstacle.y_min):
        return float('inf'), None

    t_offset = move.clearance / distance_point_to_point(move.start, move.end)
    t_min = 0 - t_offset
    lowest_t = 1 + t_offset
    closest_corner_or_vertex = None

    for corner in obstacle.corners:
        if (corner.x < move.x_min or corner.x > move.x_max or
                corner.y < move.y_min or corner.y > move.y_max):
            continue

        t = point_to_line_t(corner, move.line)
        if t_min < t < lowest_t and abs(distance_point_to_line(corner, move.line)) < move.clearance:
            lowest_t = t
            closest_corner_or_vertex = corner

    for vertex in obstacle.vertices:
        t, u = line_intersection_t_u(move.line, vertex)
        if 0 < t < lowest_t and 0 < u < 1:
            lowest_t = t
            closest_corner_or_vertex = vertex

        # for end_point in [move.start, move.end]:
        #     if (end_point.x + move.clearance < vertex.x_min or end_point.x - move.clearance > vertex.x_max or
        #             end_point.y + move.clearance < vertex.y_min or end_point.y - move.clearance > vertex.y_max):
        #         continue
        #
        #     t = line_intersection_t(move.line, vertex)
        #     if t_min < t < lowest_t:
        #         lowest_t = t
        #         closest_corner_or_vertex = vertex

    return lowest_t, closest_corner_or_vertex

# Check start and end of a move
