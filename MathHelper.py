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

    for vertex in obstacle.vertices:
        if (vertex.x < move.x_min or vertex.x > move.x_max or
                vertex.y < move.y_min or vertex.y > move.y_max):
            continue

        distance = distance_point_to_line(vertex, move.line)
        if abs(distance) < move.clearance:
            print(f"vertex too close: {vertex}, distance: {distance}")
            return True

    for edge in obstacle.edges:
        t, u = line_intersection_t_u(move.line, edge)
        if 0 < t < 1 and 0 < u < 1:
            print(f"Hit, t: {t}, u: {u}, edge: {edge}, at: {line_intersection(move.line, edge)}")
            return True

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
    closest_vertex_or_edge = None

    for vertex in obstacle.vertices:
        if (vertex.x < move.x_min or vertex.x > move.x_max or
                vertex.y < move.y_min or vertex.y > move.y_max):
            continue

        t = point_to_line_t(vertex, move.line)
        if t_min < t < lowest_t and abs(distance_point_to_line(vertex, move.line)) < move.clearance:
            lowest_t = t
            closest_vertex_or_edge = vertex

    for edge in obstacle.edges:
        t, u = line_intersection_t_u(move.line, edge)
        if 0 < t < lowest_t and 0 < u < 1:
            lowest_t = t
            closest_vertex_or_edge = edge

    return lowest_t, closest_vertex_or_edge


# Check start and end of a move
def is_point_in_obstacle(p: Point, obstacle: Obstacle, clearance: float) -> bool:
    if not (obstacle.x_min - clearance <= p.x <= obstacle.x_max + clearance and
            obstacle.y_min - clearance <= p.y <= obstacle.y_max + clearance):
        return False

    intersections = 0
    for vertex in obstacle.vertices:
        if distance_point_to_point(vertex, p) < clearance:
            print(f"vertex too close: {vertex}")
            return True

        if vertex.x == p.x and vertex.y > p.y:
            other_vertex1, other_vertex2 = obstacle.edges_dict[vertex]

            # Only if the two other vertices are on either side of the vertex,
            # otherwise the line_up would only touch the vertex
            if other_vertex1.x < vertex.x < other_vertex2.x or other_vertex1.x > vertex.x > other_vertex2.x:
                intersections += 1
                print(f"vertex hit: {vertex}")
            else:
                print(f"vertex grace: {vertex}")

    line_up = Line(p, Point(p.x, obstacle.y_max))
    for edge in obstacle.edges:
        t, u = line_intersection_t_u(line_up, edge)
        # Not '<=' to skip hitting vertices
        if 0 < u < 1:
            if abs(distance_point_to_line(p, edge)) < clearance:
                print(f"edge too close: {edge}, dist.: {distance_point_to_line(p, edge)}")
                return True
            elif 0 < t:
                print(f"edge hit: {edge}")
                intersections += 1

    # print(f"edges: {edges}")
    print(f"Intersections: {intersections}")
    return intersections % 2 != 0
