from typing import Any

from MathHelper import distance_point_to_line, line_intersection_t_u, line_intersection, distance_point_to_point, \
    get_tangent_points, distance_point_to_point2, get_closest_point, get_furthest_point, \
    leg_from_base_and_lines, point_to_line_t, line_intersection_t
from Types import Point, Line
from objects.Field import Field
from objects.Move import Move, steps_str, min_distance, get_wait_move
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


def first_obstacle_intersection(move: Move, obstacle: Obstacle) -> tuple[float, Any]:
    # Check the "bounding box"
    if move.x_min > obstacle.x_max or \
            move.x_max < obstacle.x_min or \
            move.y_min > obstacle.y_max or \
            move.y_max < obstacle.y_min:
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


def first_robot_intersection(move: Move, robot: Robot) -> tuple[float, Move]:
    lowest_t = float('inf')
    lowest_t_move = None
    for r_move in robot.path:
        if move.x_min > r_move.x_max or \
                move.x_max < r_move.x_min or \
                move.y_min > r_move.y_max or \
                move.y_max < r_move.y_min:
            continue
        if r_move.start_time > move.end_time or r_move.end_time < move.start_time:
            continue

        min_dist, t = min_distance(move, r_move)
        # print(f"move: {move}, r_move: {r_move}")
        # print(f"min_dist: {min_dist}, t: {t}")
        # print(f"clearance: {move.clearance + r_move.clearance}")
        # print(f"{min_dist < move.clearance + r_move.clearance}")
        # print(f"Robot: {robot.name}, path: {steps_str(robot.path)}")
        if min_dist == float('nan'):
            continue
        if min_dist < move.clearance + r_move.clearance and t < lowest_t:
            lowest_t = t
            lowest_t_move = r_move

    return lowest_t, lowest_t_move


def path_around_obstacle(move: Move, obstacle: Obstacle, c_v_or_e: Point | Line, field: Field) -> list[list[Move]]:
    if isinstance(c_v_or_e, Point):
        point = c_v_or_e
    else:
        point = get_closest_point(move.start, [c_v_or_e.p1, c_v_or_e.p2])

    o_point = obstacle.outside_points_dict[point]

    paths: list[list[Move]] = []
    # 0 is counterclockwise
    for rotation in [0, 1]:
        new_path = [Move(Line(move.start, o_point), move.clearance, move.start_time)]
        move_to_dest = Move(Line(o_point, move.end), move.clearance, new_path[-1].end_time)

        new_o_point = o_point
        while does_intersect(move_to_dest, obstacle):
            new_o_point = obstacle.outside_to_outside_points[new_o_point][rotation]
            new_path.append(Move(
                Line(move_to_dest.start, new_o_point),
                move.clearance,
                new_path[-1].end_time
            ))
            move_to_dest = Move(
                Line(new_o_point, move.end),
                move.clearance,
                new_path[-1].end_time
            )

        # If only one corner of the obstacle is passed and the inside angle is acute,
        # it could be passed closer than the 'outside point'
        if new_o_point == o_point and obstacle.is_acute[point]:
            closest_point = get_closest_to_acute_vertex(move.start, move.end, point, obstacle.clearance)
            new_path = [Move(Line(move.start, closest_point), move.clearance, move.start_time)]
        # todo for acute angle as first or last step

        # Don't append last step, so other function will know to make new move and check with other obstacles
        # new_path.append(move_to_dest)

        new_path = reduce_path(new_path, field)

        if new_path:
            paths.append(new_path)

    return paths


def path_around_robot(move: Move, r_move: Move, robot: Robot, field: Field) -> list[list[Move]]:
    clearance = move.clearance + r_move.clearance
    t_offset = (leg_from_base_and_lines(clearance, move.line, r_move.line) * 2) / r_move.speed

    def time_at_intersection(m: Move, line: Line) -> float:
        t = line_intersection_t(m.line, line)
        return t * m.line.len / m.speed + m.start_time

    r_time_at_intersection = time_at_intersection(r_move, move.line)
    move_time_at_intersection = time_at_intersection(move, r_move.line)
    move_offset = t_offset - move_time_at_intersection + r_time_at_intersection

    wait_move = get_wait_move(move.start, clearance, move.start_time, move_offset)
    return get_possible_paths(wait_move, field)


# Returns the possible paths around the first obstacle hit, or the same move if possible
# Doesn't include the last step so another function can do a breath first search on the options
def get_possible_paths(move: Move, field: Field) -> list[list[Move]]:
    # Closest object
    c_object: Obstacle | Robot | None = None
    c_element: Point | Line | Move | None = None
    c_t: float = float('inf')

    for obstacle in field.obstacles:
        t, vertex_or_edge = first_obstacle_intersection(move, obstacle)
        if vertex_or_edge is not None and t < c_t:
            c_element = vertex_or_edge
            c_t = t
            c_object = obstacle

    for robot in field.robots:
        t, r_move = first_robot_intersection(move, robot)
        if r_move is not None and t < c_t:
            c_element = r_move
            c_t = t
            c_object = robot

    if not c_object:
        return [[move]]

    if isinstance(c_object, Obstacle):
        return path_around_obstacle(move, c_object, c_element, field)
    else:
        return path_around_robot(move, c_element, c_object, field)


def get_closest_to_acute_vertex(p1: Point, p2: Point, v: Point, clearance: float) -> Point:
    tps1 = get_tangent_points(p1, v, clearance)
    tps2 = get_tangent_points(p2, v, clearance)

    # Take the outermost tangent point
    tp1 = get_furthest_point(p2, tps1)
    tp2 = get_furthest_point(p1, tps2)

    return line_intersection(Line(p1, tp1), Line(p2, tp2))


# Checks if steps can be skipped and if they're possible
def reduce_path(path: list[Move], field: Field) -> list[Move]:
    new_path = path
    checking_index = 0
    while checking_index < len(new_path):
        path = new_path
        new_path = path[:checking_index]

        if path[checking_index].waiting:
            new_path.append(path[checking_index])
            if len(path) >= checking_index + 2:
                new_path.append(path[checking_index + 1])
            # print(f"Reduce path new path: {new_path}")
            checking_index += 1
            continue

        intersects = True
        for move in path[:checking_index:-1]:
            # if move.waiting:
            #     continue
            # print(f"Checking move: {move}")
            intersects = False
            new_move = Move(
                Line(path[checking_index].start, move.end),
                path[checking_index].clearance,
                path[checking_index].start_time
            )
            # print(f"Checking new move: {new_move}")
            for obstacle in field.obstacles:
                if does_intersect(new_move, obstacle):
                    # New move is not possible, try the next
                    intersects = True
                    new_path.insert(checking_index, move)
                    break
            # print(f"Intersects: {intersects}")
            if not intersects:
                # New move is possible, add it and continue to the next
                new_path.insert(checking_index, new_move)
                break

        # If no new move was possible, check the current existing move
        if intersects:
            for obstacle in field.obstacles:
                if does_intersect(path[checking_index], obstacle):
                    print(f"Existing move intersects: {path[checking_index]}, path: {steps_str(path)}")
                    return path[:checking_index]
            new_path.insert(checking_index, path[checking_index])

        checking_index += 1

    return new_path


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


def pathfind(move: Move, field: Field) -> list[Move]:
    destination = move.end

    # Check if the destination is in an obstacle
    for obstacle in field.obstacles:
        if is_point_in_obstacle(destination, obstacle, move.clearance):
            print("Destination in obstacle")
            return []

    paths = get_possible_paths(move, field)
    destination_reached_loops = 0
    while True:
        print(f"Destination reached loops: {destination_reached_loops}")
        if not paths:
            print("No possible paths")
            return []

        paths_next_loop: list[list[Move]] = []
        destination_reached = False
        for path in paths:
            # print("----------------------------------------------------------")
            # print(f"Path: {path}")
            if path[-1].end == destination:
                destination_reached = True
                # input("Destination reached")
                paths_next_loop.append(path)
                continue

            new_move_to_dest = Move(
                Line(path[-1].end, destination),
                move.clearance,
                path[-1].end_time
            )

            new_paths = get_possible_paths(new_move_to_dest, field)
            # print(f"New paths: {new_paths}")
            if new_paths == [[new_move_to_dest]]:
                destination_reached = True
                # input("Destination reached other")
            for new_path in new_paths:
                new_path = reduce_path(path + new_path, field)
                # print(f"new_path: {new_path}")
                # print(f"    {steps_str(new_path)}")
                paths_next_loop.append(new_path)

        if destination_reached:
            destination_reached_loops += 1
            shortest_theoretical_path = float('inf')
            shortest_path = []
            for path in paths_next_loop:
                # print(f"Path: {path}")
                if path[-1].end == destination:
                    # print(f"Destination reached in {path[-1].end_time}: {steps_str(path)}")
                    if path[-1].end_time < shortest_theoretical_path:
                        shortest_theoretical_path = path[-1].end_time
                        shortest_path = path
                elif destination_reached_loops < 4:
                    new_move_to_dest = Move(
                        Line(path[-1].end, destination),
                        move.clearance,
                        path[-1].end_time
                    )
                    if new_move_to_dest.end_time < shortest_theoretical_path:
                        shortest_theoretical_path = new_move_to_dest.end_time
                        shortest_path = []

            if shortest_path:
                return shortest_path

        paths = paths_next_loop
