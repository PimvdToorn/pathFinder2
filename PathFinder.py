from typing import Any

from MathHelper import distance_point_to_line, line_intersection_t_u, line_intersection, distance_point_to_point, \
    point_to_line_t_slope
from Types import Point, Line
from objects.Field import Field
from objects.Move import Move, steps_str
from objects.Obstacle import Obstacle
from objects.Robot import Robot


def does_intersect(move: Move, obstacle: Obstacle) -> bool:
    # Check the "bounding box"
    if not (
            move.x_min < obstacle.x_max and
            move.x_max > obstacle.x_min and
            move.y_min < obstacle.y_max and
            move.y_max > obstacle.y_min):
        # print("Out of bound")
        return False

    for vertex in obstacle.vertices:
        if (vertex.x < move.x_min or vertex.x > move.x_max or
                vertex.y < move.y_min or vertex.y > move.y_max):
            continue

        distance = distance_point_to_line(vertex, move.line)
        if abs(distance) < move.clearance:
            # print(f"vertex too close: {vertex}, distance: {distance}")
            return True

    for edge in obstacle.edges:
        t, u = line_intersection_t_u(move.line, edge)
        if 0 < t < 1 and 0 < u < 1:
            # print(f"Hit, t: {t}, u: {u}, edge: {edge}, at: {line_intersection(move.line, edge)}")
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

        t = point_to_line_t_slope(vertex, move.line, move.slope_inv)
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
            # print(f"vertex too close: {vertex}")
            return True

        if vertex.x == p.x and vertex.y > p.y:
            other_vertex1, other_vertex2 = obstacle.connected_vertices[vertex]

            # Only if the two other vertices are on either side of the vertex,
            # otherwise the line_up would only touch the vertex
            if other_vertex1.x < vertex.x < other_vertex2.x or other_vertex1.x > vertex.x > other_vertex2.x:
                intersections += 1
            #     print(f"vertex hit: {vertex}")
            # else:
            #     print(f"vertex grace: {vertex}")

    line_up = Line(p, Point(p.x, obstacle.y_max))
    for edge in obstacle.edges:
        t, u = line_intersection_t_u(line_up, edge)
        # Not '<=' to skip hitting vertices
        if 0 < u < 1:
            if abs(distance_point_to_line(p, edge)) < clearance:
                # print(f"edge too close: {edge}, dist.: {distance_point_to_line(p, edge)}")
                return True
            elif 0 < t:
                # print(f"edge hit: {edge}")
                intersections += 1

    # print(f"edges: {edges}")
    # print(f"Intersections: {intersections}")
    return intersections % 2 != 0


# Returns the possible paths around the first obstacle hit, or the same move if possible
# Doesn't include the last step so another function can do a breath first search on the options
def get_possible_paths(move: Move, field: Field) -> list[list[Move]]:
    # closest
    c_obstacle: Obstacle = None
    c_t: float = float('inf')
    # closest vertex or edge
    c_v_or_e: Any = None

    for obstacle in field.obstacles:
        t, vertex_or_edge = first_intersection(move, obstacle)
        if vertex_or_edge is not None and t < c_t:
            c_v_or_e = vertex_or_edge
            c_t = t
            c_obstacle = obstacle

    if not c_obstacle:
        return [[move]]

    if isinstance(c_v_or_e, Point):
        # will check around anyway
        outside_points: list[Point] = [
            c_obstacle.outside_points_dict[c_v_or_e],
            # c_obstacle.edges_dict[c_v_or_e][0],
            # c_obstacle.edges_dict[c_v_or_e][1]
        ]
    else:
        # todo Maybe only one, will check around anyway, and will check direct to other points
        outside_points = [
            c_obstacle.outside_points_dict[c_v_or_e.p1],
            c_obstacle.outside_points_dict[c_v_or_e.p2]
        ]

    paths: list[list[Move]] = []
    for point in outside_points:
        paths_around = []
        # 0 is counterclockwise
        for rotation in [0, 1]:
            new_path = [Move(Line(move.start, point), move.clearance, move.start_time)]
            move_to_dest = Move(Line(point, move.end), move.clearance, new_path[-1].end_time)

            new_point = point
            while does_intersect(move_to_dest, c_obstacle):
                new_point = c_obstacle.outside_to_outside_points[new_point][rotation]
                new_path.append(Move(
                    Line(move_to_dest.start, new_point),
                    move.clearance,
                    new_path[-1].end_time
                ))
                move_to_dest = Move(
                    Line(new_point, move.end),
                    move.clearance,
                    new_path[-1].end_time
                )

            # Don't append last step, so other function will know to make new move
            # and check with other obstacles
            # new_path.append(move_to_dest)
            paths_around.append(new_path)

            # Don't check other rotation, as the first new step works
            if new_point == point:
                break

        paths += paths_around

    return paths


# Checks if steps can be skipped and if they're possible
def reduce_path(path: list[Move], field: Field) -> list[Move]:
    new_path = path
    checking_index = 0
    while checking_index < len(new_path):
        path = new_path
        new_path = path[:checking_index]

        intersects = True
        for move in path[:checking_index:-1]:
            intersects = False
            new_move = Move(
                Line(path[checking_index].start, move.end),
                path[checking_index].clearance,
                path[checking_index].start_time
            )
            for obstacle in field.obstacles:
                if does_intersect(new_move, obstacle):
                    # New move is not possible, try the next
                    intersects = True
                    new_path.insert(checking_index, move)
                    break

            if not intersects:
                # New move is possible, add it and continue to the next
                new_path.insert(checking_index, new_move)
                break

        # If no new move was possible, check the current existing move
        if intersects:
            for obstacle in field.obstacles:
                if does_intersect(path[checking_index], obstacle):
                    print(f"Existing move intersects: {path[checking_index]}")
                    return []
            new_path.insert(checking_index, path[checking_index])

        checking_index += 1

    return new_path


# def pathfind(move: Move, field: Field) -> list[Move]:
#     destination = move.end


