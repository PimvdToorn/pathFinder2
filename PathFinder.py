from typing import Any

from MathHelper import distance_point_to_line, line_intersection_t_u, distance, get_closest_point, \
    leg_from_base_and_lines, point_to_line_t, line_intersection_t, get_points_around_robot, \
    get_point_to_vertex_tangent, get_closest_to_vertex, line_intersection, distance2
from Types import Point, Line
from objects.Field import Field
from objects.Move import Move, steps_str, min_distance2, get_wait_move, remove_duplicates, path_str, path_in_paths
from objects.Obstacle import Obstacle
from objects.Robot import Robot

ROBOT_TO_ROBOT_MARGIN_NS = 500_000
MAX_DEPTH = 5


def does_intersect(move: Move, obstacle: Obstacle) -> bool:
    # Check the "bounding box"
    if not (
            move.x_min < obstacle.x_max and
            move.x_max > obstacle.x_min and
            move.y_min < obstacle.y_max and
            move.y_max > obstacle.y_min):
        return False

    clearance2 = move.clearance ** 2

    for vertex in obstacle.vertices:
        if (vertex.x < move.x_min or vertex.x > move.x_max or
                vertex.y < move.y_min or vertex.y > move.y_max):
            continue

        t = point_to_line_t(vertex, move.line)

        if t < 0:
            if distance2(vertex, move.start) < clearance2:
                return True
            continue
        elif t > 1:
            if distance2(vertex, move.end) < clearance2:
                return True
            continue
        elif abs(distance_point_to_line(vertex, move.line)) < move.clearance:
            return True

    for edge in obstacle.edges:
        t, u = line_intersection_t_u(move.line, edge)
        if 0 < t < 1 and 0 < u < 1:
            return True

    return False


def first_obstacle_intersection(move: Move, obstacle: Obstacle) -> tuple[float, Any]:
    # Check the "bounding box"
    if move.x_min > obstacle.x_max or \
            move.x_max < obstacle.x_min or \
            move.y_min > obstacle.y_max or \
            move.y_max < obstacle.y_min:
        return float('inf'), None
    if move.waiting:
        return float('inf'), None

    clearance2 = move.clearance ** 2
    lowest_t = float('inf')
    closest_vertex_or_edge = None

    for vertex in obstacle.vertices:
        if (vertex.x < move.x_min or vertex.x > move.x_max or
                vertex.y < move.y_min or vertex.y > move.y_max):
            continue

        t = point_to_line_t(vertex, move.line)

        if t < 0:
            if distance2(vertex, move.start) < clearance2:
                return 0, vertex
            continue
        elif t > 1:
            if distance2(vertex, move.end) < clearance2:
                lowest_t = 1
                closest_vertex_or_edge = vertex
            continue
        elif t < lowest_t and abs(distance_point_to_line(vertex, move.line)) < move.clearance:
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
        if not r_move == robot.path[-1] and (r_move.start_time > move.end_time or r_move.end_time < move.start_time):
            continue

        min_dist2, t = min_distance2(move, r_move)
        if min_dist2 == float('nan'):
            continue
        if t < lowest_t and min_dist2 < (move.clearance + robot.radius) ** 2:
            lowest_t = t
            lowest_t_move = r_move

    return lowest_t, lowest_t_move


def path_around_obstacle(move: Move, obstacle: Obstacle, c_v_or_e: Point | Line, field: Field, depth: int) \
        -> list[list[Move]]:
    if is_point_in_obstacle(move.end, obstacle, move.clearance):
        return []

    # Closest vertex or edge
    if isinstance(c_v_or_e, Point):
        point = c_v_or_e
    else:
        point = get_closest_point(move.start, [c_v_or_e.p1, c_v_or_e.p2])

    # Outside point, the point at a vertex at which both neighboring outside points can be reached in a straight line
    o_point = obstacle[move.clearance].outside_points_dict[point]

    paths: list[list[Move]] = []
    # 0 is counterclockwise
    for rotation in [0, 1]:

        new_o_point = o_point

        # If the move.start is an outside point
        if obstacle[move.clearance].outside_to_outside_points.get(move.start, None):
            new_o_point = obstacle[move.clearance].outside_to_outside_points[move.start][rotation]

        new_move = Move(Line(move.start, new_o_point), move.clearance, move.start_time)

        new_paths = get_possible_paths(new_move, field, depth)
        new_paths = [(new_o_point, p) for p in new_paths]

        loops = 0
        while new_paths:
            loops += 1
            if loops > 10:
                break

            new_o_point, new_path = new_paths.pop(0)
            if new_path[-1].end == move.end:
                paths.append(new_path)
                continue

            move_to_dest = Move(Line(new_path[-1].end, move.end), move.clearance, new_path[-1].end_time)

            if does_intersect(move_to_dest, obstacle):
                new_o_point = obstacle[move.clearance].outside_to_outside_points[new_o_point][rotation]

                if new_o_point == new_path[-1].end:
                    # The path has already gone this rotation around the object
                    continue

                new_move = Move(Line(new_path[-1].end, new_o_point), move.clearance, new_path[-1].end_time)

                for path in get_possible_paths(new_move, field, depth):
                    new_paths += [(new_o_point, new_path + path)]
                continue

            paths += [new_path]

    return remove_duplicates(paths)


def path_around_robot(move: Move, r_move: Move, robot: Robot, field: Field, depth: int) -> list[list[Move]]:
    if r_move.waiting:
        wait_move = get_wait_move(move.start, move.clearance, move.start_time, int(r_move.end_time - move.start_time))
        paths = get_possible_paths(wait_move, field, depth)
        return paths

    clearance = move.clearance + r_move.clearance
    clearance2 = clearance ** 2

    # If the r_move is the last move of the robot and the end of r_move is too close to the move
    if r_move == robot.path[-1] and abs(distance_point_to_line(r_move.end, move.line)) < clearance:

        wait_move = get_wait_move(move.start, move.clearance, move.start_time, int(r_move.end_time - move.start_time))

        wait_move = get_possible_paths(wait_move, field, depth)
        if wait_move:
            wait_move = wait_move[0][0]
        else:
            return []

        if distance2(move.start, r_move.end) < clearance2:
            return []
        if distance2(move.end, r_move.end) < clearance2:
            return []

        around_points = get_points_around_robot(move.start, move.end, r_move.end, clearance)
        move1 = Move(Line(move.start, around_points[0]), move.clearance, wait_move.end_time)
        move2 = Move(Line(move.start, around_points[1]), move.clearance, wait_move.end_time)

        path1 = get_possible_paths(move1, field, depth)
        path2 = get_possible_paths(move2, field, depth)
        paths = path1 + path2

        for path in paths:
            path.insert(0, wait_move)
        return paths

    def time_at_intersection(m: Move, line: Line) -> float:
        t = line_intersection_t(m.line, line)
        return t * m.line.len / m.speed + m.start_time

    # If two robots travel at the same speed, the closest they could get is when the other robot has moved past
    # the intersection, and their distances to the intersection are equal. Their paths then form an isosceles triangle
    # with their clearances as the length of the base. This base and the angle between the lines can be used to find
    # the distance to the intersection:
    t_offset = (leg_from_base_and_lines(clearance, move.line, r_move.line) * 2) / r_move.speed

    r_time_at_intersection = time_at_intersection(r_move, move.line)
    move_time_at_intersection = time_at_intersection(move, r_move.line)
    move_offset = t_offset - move_time_at_intersection + r_time_at_intersection + ROBOT_TO_ROBOT_MARGIN_NS

    if move_offset < 1:
        return [[move]]

    wait_move = get_wait_move(move.start, move.clearance, move.start_time, int(move_offset))

    return get_possible_paths(wait_move, field, depth)


# Returns the possible paths around the first obstacle hit, or the same move if possible
# Doesn't include the last step so another function can do a breath first search on the options
def get_possible_paths(move: Move, field: Field, depth: int) -> list[list[Move]]:
    depth += 1
    if depth > MAX_DEPTH:
        print("|" * depth + f"Max depth reached: {move.__repr__()}")
        return []

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
    if move.waiting:
        return []

    if isinstance(c_object, Obstacle):
        return path_around_obstacle(move, c_object, c_element, field, depth)
    else:
        return path_around_robot(move, c_element, c_object, field, depth)


# Checks if steps can be skipped and if they're possible
def reduce_path(path: list[Move], field: Field, depth: int) -> list[list[Move]]:
    new_path = path
    checking_index = 0
    while checking_index < len(new_path):
        path = new_path
        new_path = path[:checking_index]

        if path[checking_index].waiting:
            new_path.append(path[checking_index])
            if len(path) >= checking_index + 2:
                # Combine waiting moves
                if path[checking_index + 1].waiting:
                    new_path[-1] = Move(
                        Line(path[checking_index].start, path[checking_index + 1].end),
                        path[checking_index].clearance,
                        path[checking_index].start_time,
                        path[checking_index + 1].end_time
                    )
                else:
                    new_path.append(path[checking_index + 1])

            checking_index += 1
            continue

        intersects = True
        for move in path[:checking_index:-1]:
            intersects = False
            new_move = Move(
                Line(path[checking_index].start, move.end),
                path[checking_index].clearance,
                path[checking_index].start_time
            )

            possible_paths = get_possible_paths(new_move, field, depth)
            best_time = move.end_time
            other_path = []
            while possible_paths:
                p = possible_paths.pop(0)

                if p[-1].end != move.end:
                    dest_move = Move(Line(p[-1].end, move.end), path[checking_index].clearance, p[-1].end_time)

                    for p1 in get_possible_paths(dest_move, field, depth):
                        if p1[-1].end_time < best_time:
                            possible_paths.append(p + p1)
                    continue
                if p[-1].end_time < best_time:
                    best_time = p[-1].end_time
                    other_path = p

            if other_path:
                # New path is possible and better, add it and continue to the next
                new_path = new_path[:checking_index] + other_path + new_path[checking_index:]
                break

            # New move is not possible, try the next
            intersects = True
            new_path.insert(checking_index, move)

        # If no new move was possible, check the current existing move
        if intersects:
            for obstacle in field.obstacles:
                if does_intersect(path[checking_index], obstacle):
                    return [new_path[:checking_index]]
            for robot in field.robots:
                _, lowest_t_move = first_robot_intersection(path[checking_index], robot)
                if lowest_t_move:
                    return [new_path[:checking_index]]
            new_path.insert(checking_index, path[checking_index])

        checking_index += 1

    return [new_path]


# This is to get the path as tight as possible around corners, not finished
def reduce_corners(path: list[Move], field: Field) -> list[Move]:
    mixed_path: list[Move | Line] = path

    # todo multiple vertex outside points in a row -> tangent lines between the two circles
    # List of those tangent lines, the tangent lines from other points and the waiting moves
    # Connect tangent lines into moves
    for index, move in enumerate(mixed_path):
        if move.waiting:
            continue
        for obstacle in field.obstacles:
            for v, op in obstacle[move.clearance].outside_points_dict.items():
                if op == move.end:
                    mixed_path[index] = get_point_to_vertex_tangent(move.start, v, op, move.clearance)

    for index, move in enumerate(new_path[-2::-1]):
        index = len(new_path) - index - 2
        if move.waiting:
            continue
        for obstacle in field.obstacles:
            for v, op in obstacle[move.CLEARANCE].outside_points_dict.items():
                if op == move.end:
                    if obstacle.is_acute[v] or \
                            move.start not in obstacle[move.CLEARANCE].outside_points_dict.values() or \
                            new_path[index + 1].end not in obstacle[move.CLEARANCE].outside_points_dict.values():
                        closest_point = get_closest_to_vertex(move.start, new_path[index + 1].end, v, op,
                                                              move.CLEARANCE)
                        new_path[index] = Move(
                            Line(move.start, closest_point),
                            move.CLEARANCE,
                            move.start_time
                        )
                        # print(f"New path: {new_path}")
                        # print(f"index: {index}, move: {move}")
                        # print(f"New move: {new_path[index]}")
                        new_path[index + 1] = Move(
                            Line(closest_point, new_path[index + 1].end),
                            move.CLEARANCE,
                            new_path[index].end_time
                        )
                        new_path = new_path[:index] + update_times(new_path[index:])

    updated = True
    while updated:
        updated = False
        for index, move in enumerate(new_path):
            paths = get_possible_paths(move, field)
            if paths != [[move]]:
                updated = True
                if paths[0][-1].waiting:
                    new_path.insert(index, paths[0][-1])
                    new_path = new_path[:index] + update_times(new_path[index:])
                else:
                    print(f"Collision in reduction, move: {move}, path: {steps_str(new_path)}")
                    for p in paths:
                        print(f"    {steps_str(p)}")
                    print("-------------------------------------------")
                    return new_path[:index]
    return new_path


def update_times(path: list[Move]) -> list[Move]:
    for i, rest in enumerate(path[1:]):
        if rest.waiting:
            path[i + 1] = Move(
                rest.line,
                rest.clearance,
                path[i].end_time,
                rest.end_time
            )
        path[i + 1] = Move(
            rest.line,
            rest.clearance,
            path[i].end_time
        )
    return path


def is_point_in_obstacle(p: Point, obstacle: Obstacle, clearance: float) -> bool:
    if not (obstacle.x_min - clearance <= p.x <= obstacle.x_max + clearance and
            obstacle.y_min - clearance <= p.y <= obstacle.y_max + clearance):
        return False

    intersections = 0
    for vertex in obstacle.vertices:
        if distance(vertex, p) < clearance:
            return True

        if vertex.x == p.x and vertex.y > p.y:
            other_vertex1, other_vertex2 = obstacle.connected_vertices[vertex]

            # Only if the two other vertices are on either side of the vertex,
            # otherwise the line_up would only touch the vertex
            if other_vertex1.x < vertex.x < other_vertex2.x or other_vertex1.x > vertex.x > other_vertex2.x:
                intersections += 1

    line_up = Line(p, Point(p.x, obstacle.y_max))
    for edge in obstacle.edges:
        t, u = line_intersection_t_u(line_up, edge)
        # Not '<=' to skip hitting vertices
        if 0 < u < 1:
            if abs(distance_point_to_line(p, edge)) < clearance:
                return True
            elif 0 < t:
                intersections += 1

    # If an uneven number of intersections, the point is inside the obstacle
    return intersections % 2 != 0


def pathfind(move: Move, field: Field) -> list[Move]:
    destination = move.end

    # Check if the destination is in an obstacle
    for obstacle in field.obstacles:
        if is_point_in_obstacle(destination, obstacle, move.clearance):
            print("Destination in obstacle")
            return []

    paths = remove_duplicates(get_possible_paths(move, field, 0))

    destination_reached_loops = 0
    loops = 1
    while True:
        loops += 1
        if loops > 20:
            return []

        if not paths:
            return []

        paths_next_loop: list[list[Move]] = []
        destination_reached = False
        for index, path in enumerate(paths):

            if path[-1].end == destination:
                destination_reached = True
                paths_next_loop.append(path)
                continue

            new_move_to_dest = Move(
                Line(path[-1].end, destination),
                move.clearance,
                path[-1].end_time
            )

            new_paths = get_possible_paths(new_move_to_dest, field, 0)
            new_paths = remove_duplicates(new_paths)

            if new_paths == [[new_move_to_dest]]:
                reduced_paths = reduce_path(path + new_paths[0], field, 0)
                for rp in reduced_paths:
                    if rp[-1].end == destination:
                        destination_reached = True

                    paths_next_loop.append(rp)
                continue

            for new_path in new_paths:
                reduced_paths = reduce_path(path + new_path, field, 0)
                for rp in reduced_paths:
                    paths_next_loop.append(rp)

        new_paths_next_loop = []
        if destination_reached:
            destination_reached_loops += 1
            shortest_theoretical_path_time = float('inf')
            shortest_actual_path_time = float('inf')
            shortest_path = []
            for path in paths_next_loop:
                if path[-1].end == destination:
                    if path[-1].end_time < shortest_theoretical_path_time:
                        shortest_theoretical_path_time = path[-1].end_time
                        shortest_path = path
                        shortest_actual_path_time = path[-1].end_time
                        new_paths_next_loop.append(path)

                    elif path[-1].end_time < shortest_actual_path_time:
                        shortest_actual_path_time = path[-1].end_time
                        new_paths_next_loop.append(path)

                elif destination_reached_loops < 4:
                    new_move_to_dest = Move(Line(path[-1].end, destination), move.clearance, path[-1].end_time)

                    if new_move_to_dest.end_time < shortest_theoretical_path_time:
                        shortest_theoretical_path_time = new_move_to_dest.end_time
                        shortest_path = []

                    # To remove any paths that in the best case are longer than an already found path
                    elif new_move_to_dest.end_time < shortest_actual_path_time:
                        new_paths_next_loop.append(path)

            if shortest_path:
                return shortest_path

        paths_next_loop = remove_duplicates(paths_next_loop)
        # If a path hasn't changed, remove it for the next loop
        paths = [p for p in paths_next_loop if not path_in_paths(p, paths)]


# todo check if the outside points are in an obstacle
def remove_imp_outside_points(field: Field, clearance: float) -> None:
    for obstacle in field.obstacles:
        for v, op in obstacle[clearance].outside_points_dict.items():
            if is_point_in_obstacle(op, obstacle, clearance):
                obstacle[clearance].outside_points_dict[v] = Point(float('inf'), float('inf'))
        hit_edges = []
        for edge, ol in obstacle[clearance].outside_lines_dict.items():
            hit = False
            for ob in field.obstacles:
                for e in ob.edges:
                    t, u = line_intersection_t_u(ol, e)
                    if 0 < t < 1 and 0 < u < 1:
                        print(f"Edge {edge} outside line {ol} intersects with {e}")
                        hit_edges.append(edge)
                        for v, op in obstacle[clearance].outside_points_dict.items():
                            if op == ol.p1 or op == ol.p2:
                                print(f"Outside point {op} is on the edge {ol}")
                                obstacle[clearance].outside_points_dict[v] = Point(float('inf'), float('inf'))
                        obstacle[clearance].outside_lines_dict[edge] = Line(Point(float('inf'), float('inf')),
                                                                            Point(float('inf'), float('inf')))
                        hit = True
                        break
                if hit:
                    break
        for edge in hit_edges:
            print(obstacle[clearance].outside_lines_dict[edge])
        obstacle[clearance].update_outside_to_outside(obstacle.connected_vertices)

    print("-----------------------------------")
    for obstacle in field.obstacles:
        for op in obstacle[clearance].outside_points_dict.values():
            print(f"{op.bare_str()},", end="")
        print()
