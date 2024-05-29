from math import sqrt

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


def line_intersection_t_u(line1: Line, line2: Line) -> tuple[float, float]:
    # https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection#:~:text=denominator%20is%20zero.-,Given%20two%20points%20on%20each%20line%20segment,-%5Bedit%5D
    x = [line1.x1, line1.x2, line2.x1, line2.x2]
    y = [line1.y1, line1.y2, line2.y1, line2.y2]

    divider = (x[0] - x[1]) * (y[2] - y[3]) - (y[0] - y[1]) * (x[2] - x[3])
    if divider == 0.0:
        return float('inf'), float('inf')

    t = ((x[0] - x[2]) * (y[2] - y[3]) - (y[0] - y[2]) * (x[2] - x[3])) / divider
    u = - ((x[0] - x[1]) * (y[0] - y[2]) - (y[0] - y[1]) * (x[0] - x[2])) / divider
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
        right_angle_slope = -1 / get_slope(line)
    except ZeroDivisionError:
        # Slope is 0, so same x as p and same y as line
        return Point(p.x, line.y1)

    p_line = Line(p, Point(p.x + 1, p.y + right_angle_slope))
    t = line_intersection_t(p_line, line)

    return Point(p.x + t, p.y + t * right_angle_slope)


def does_intersect(move: Move, obstacle: Obstacle) -> bool:
    # Check the "bounding box"
    if not (
            move.x_min < obstacle.x_max and
            move.x_max > obstacle.x_min and
            move.y_min < obstacle.y_max and
            move.y_max > obstacle.y_min):
        print("Out of bound")
        return False

    # print(f"Move bounds: x_min: {move.x_min}, x_max: {move.x_max}, y_min: {move.y_min}, y_max: {move.y_max}")
    # left, right = [], []
    for corner in obstacle.corners:
        if (corner.x < move.x_min or corner.x > move.x_max or
                corner.y < move.y_min or corner.y > move.y_max):
            # print(f"Corner out of bounds: {corner}")
            continue

        distance = distance_point_to_line(corner, move.line)
        if abs(distance) < move.clearance:
            print(f"Corner too close: {corner}, distance: {distance}")
            return True
        # if distance < 0:
        #     right.append(corner)
        # else:
        #     left.append(corner)

    # print(f"l: {left}, r: {right}")
    # if left != [] and right != []:
    #     print(f"Left and right, l: {left}, r: {right}")
    #     return True

    t_u_offset = move.clearance / distance_point_to_point(move.start, move.end)
    # print(f"Len: {distance_point_to_point(move.start, move.end)}, clr: {move.clearance}, t_offset: {t_offset}")
    for vertex in obstacle.vertices:
        t, u = line_intersection_t_u(move.line, vertex)
        if 0 - t_u_offset < t < 1 + t_u_offset and 0 - t_u_offset < u < 1 + t_u_offset:
            print(f"Hit, t: {t}, u: {u}, vertex: {vertex}, at: {line_intersection(move.line, vertex)}")
            return True
    return False
