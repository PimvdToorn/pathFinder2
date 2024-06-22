from typing import Any

from MathHelper import distance_point_to_line, line_intersection_t_u, line_intersection, distance, \
    get_tangent_points, get_closest_point, leg_from_base_and_lines, point_to_line_t, line_intersection_t, offset_line, \
    get_points_around_robot
from Types import Point, Line
from objects.Field import Field
from objects.Move import Move, steps_str, min_distance, get_wait_move, remove_duplicates, path_str, path_in_paths
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
    if move.waiting:
        return float('inf'), None

    t_offset = move.clearance / distance(move.start, move.end)
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
            # print(f"vertex too close: {vertex}, distance: {distance_point_to_line(vertex, move.line)}")

    for edge in obstacle.edges:
        t, u = line_intersection_t_u(move.line, edge)
        if 0 < t < lowest_t and 0 < u < 1:
            lowest_t = t
            closest_vertex_or_edge = edge
            # print(f"Hit, t: {t}, u: {u}, edge: {edge}, at: {line_intersection(move.line, edge)}")

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

        min_dist, t = min_distance(move, r_move)
        # if min_dist < move.clearance + r_move.clearance:
        #     print("-------------------------------------------")
        #     print(f"    move: {move}, r_move: {r_move}")
        #     print(f"    min_dist: {min_dist}, t: {t}")
        #     print(f"    clearance: {move.clearance + r_move.clearance}")
        #     print(f"    Robot: {robot.name}, path: {steps_str(robot.path)}")
        #     print("-------------------------------------------")
        if min_dist == float('nan'):
            continue
        if min_dist < move.clearance + r_move.clearance and t < lowest_t:
            lowest_t = t
            lowest_t_move = r_move

    return lowest_t, lowest_t_move


def path_around_obstacle(move: Move, obstacle: Obstacle, c_v_or_e: Point | Line, field: Field, depth: int) \
        -> list[list[Move]]:
    if is_point_in_obstacle(move.end, obstacle, move.clearance):
        return []

    if isinstance(c_v_or_e, Point):
        point = c_v_or_e
    else:
        point = get_closest_point(move.start, [c_v_or_e.p1, c_v_or_e.p2])

    o_point = obstacle[move.clearance].outside_points_dict[point]
    print("|"*depth + f"Around obstacle, move: {move.__repr__()}, point: {point.__repr__()}, o_point: {o_point.__repr__()}")

    paths: list[list[Move]] = []
    # 0 is counterclockwise
    for rotation in [0, 1]:

        new_o_point = o_point

        if obstacle[move.clearance].outside_to_outside_points.get(move.start, None):
            new_o_point = obstacle[move.clearance].outside_to_outside_points[move.start][rotation]

        new_move = Move(Line(move.start, new_o_point), move.clearance, move.start_time)

        if new_move.start_time == new_move.end_time:
            input(f"--same time new move: {new_move.__repr__()}")

        new_paths = get_possible_paths(Move(Line(move.start, new_o_point), move.clearance, move.start_time), field, depth)
        new_paths = [(new_o_point, p) for p in new_paths]

        print("|"*depth + f"r{rotation}-------------------------------------------")
        loops = 0
        same_point = None
        while new_paths:
            loops += 1
            if loops > 5:
                input("|"*depth + f"Loops: {loops}")

            print("|"*depth + f"    New paths:")
            for p in new_paths:
                print("|"*depth + f"    |   {p}")
            new_o_point, new_path = new_paths.pop(0)
            if new_path[-1].end == move.end:
                paths.append(new_path)
                continue

            move_to_dest = Move(Line(new_path[-1].end, move.end), move.clearance, new_path[-1].end_time)

            if move_to_dest.start_time == move_to_dest.end_time:
                input(f"same time move to dest: {move_to_dest.__repr__()}")

            if does_intersect(move_to_dest, obstacle):
                new_o_point = obstacle[move.clearance].outside_to_outside_points[new_o_point][rotation]
                if new_o_point == new_path[-1].end:
                    # The path has already gone this rotation around the object
                    if same_point == new_o_point:
                        input("|"*depth + f"Same point, new o point: {new_o_point.__repr__()}, same: {same_point.__repr__()}")
                        continue
                    elif same_point:
                        input("|"*depth + f"Same point, diff same same point")
                    same_point = new_o_point
                    continue
                elif same_point:
                    input("|" * depth + f"Same point, diff point")

                new_move = Move(Line(new_path[-1].end, new_o_point), move.clearance, new_path[-1].end_time)
                if new_move.start_time == new_move.end_time:
                    input(f"same time new move: {new_move.__repr__()}")
                for path in get_possible_paths(new_move, field, depth):
                    for m in path:
                        if m.start_time == m.end_time:
                            input(f"---same time path: {m.__repr__()}")
                    new_paths += [(new_o_point, new_path + path)]
                continue

            print("|"*depth + f"New p ar obstacle r{rotation}:")
            for m in new_path:
                print("|"*depth + f"    {m.__repr__()}")
                # if m.start_time == m.end_time:
                #     input(f"same time new path: {m.__repr__()}")
            paths += [new_path]  # reduce_path(new_path, field, depth)

    return remove_duplicates(paths)


def path_around_robot(move: Move, r_move: Move, robot: Robot, field: Field, depth: int) -> list[list[Move]]:
    # print(f"Path around robot, move: {move.__repr__()}, r_move: {r_move.__repr__()}")
    # print(f"Robot: {robot.name}, path: {steps_str(robot.path)}")
    if r_move.waiting:
        # print("Robot waiting")
        wait_move = get_wait_move(move.start, move.clearance, move.start_time, int(r_move.end_time - move.start_time))
        paths = get_possible_paths(wait_move, field, depth)
        # print(f"Move: {move}")
        # print(f"Wait move: {wait_move}")
        # print(f"Paths: {paths}")
        # input()
        return paths

    clearance = move.clearance + r_move.clearance
    t_offset = (leg_from_base_and_lines(clearance, move.line, r_move.line) * 2) / r_move.speed

    def time_at_intersection(m: Move, line: Line) -> float:
        t = line_intersection_t(m.line, line)
        return t * m.line.len / m.speed + m.start_time

    r_time_at_intersection = time_at_intersection(r_move, move.line)
    move_time_at_intersection = time_at_intersection(move, r_move.line)
    move_offset = t_offset - move_time_at_intersection + r_time_at_intersection + ROBOT_TO_ROBOT_MARGIN_NS

    if r_move == robot.path[-1] and move_time_at_intersection + t_offset + r_time_at_intersection > r_move.end_time:
        # print(f"Hit at {line_intersection(move.line, r_move.line)}")
        # print(f"Move time at intersection: {move_time_at_intersection}")
        # print(f"Robot time at intersection: {r_time_at_intersection}")
        # print(f"t offset: {t_offset}")
        # print(f"Move offset: {move_offset}")
        # input()
        wait_move = get_wait_move(move.start, move.clearance, move.start_time, int(r_move.end_time - move.start_time))
        wait_move = get_possible_paths(wait_move, field, depth)
        if wait_move:
            wait_move = wait_move[0][0]
        else:
            return []

        # print("Move around end")
        if distance(move.start, r_move.end) < clearance:
            # input("Start too close to end")
            return []
        if distance(move.end, r_move.end) < clearance:
            # input("End too close to other end")
            return []
        around_points = get_points_around_robot(move.start, move.end, r_move.end, clearance)
        # print(f"Around points: {around_points}")
        move1 = Move(Line(move.start, around_points[0]), move.clearance, wait_move.end_time)
        move2 = Move(Line(move.start, around_points[1]), move.clearance, wait_move.end_time)
        # print(f"Move1: {move1}, Move2: {move2}")
        # print("Move 1:")
        path1 = get_possible_paths(move1, field, depth)
        # if path1:
        #     print(f"1Robot successfully around, move: {move}, robot: {robot.name}, path: {steps_str(robot.path)}")
        # print("Move 2:")
        path2 = get_possible_paths(move2, field, depth)
        # if path2:
        #     print(f"2Robot successfully around, move: {move}, robot: {robot.name}, path: {steps_str(robot.path)}")
        paths = path1 + path2
        for path in paths:
            path.insert(0, wait_move)
        return paths

    wait_move = get_wait_move(move.start, move.clearance, move.start_time, int(move_offset))
    if wait_move.end_time == wait_move.start_time:
        input("|"*depth + f"same time wait move: {wait_move.__repr__()}")
    print("|"*depth + f"Wait move: {wait_move} - move: {move}, r_move: {r_move}, robot: {robot.name}")
    paths = get_possible_paths(wait_move, field, depth)
    # print(paths)
    return paths


# Returns the possible paths around the first obstacle hit, or the same move if possible
# Doesn't include the last step so another function can do a breath first search on the options
def get_possible_paths(move: Move, field: Field, depth: int) -> list[list[Move]]:
    depth += 1
    if depth > MAX_DEPTH:
        print("|"*depth + f"Max depth reached: {move.__repr__()}")
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
        # if move.waiting:
        #     input("No object, waiting move")
        return [[move]]
    if move.waiting:
        # print(f"Move: {move}")
        # input(f"Object found, waiting move, object: {c_object.name}, t: {c_t}, ")
        return []

    if isinstance(c_object, Obstacle):
        return path_around_obstacle(move, c_object, c_element, field, depth)
    else:
        paths = path_around_robot(move, c_element, c_object, field, depth)
        # if paths:
        #     print(f"Robot successfully around, move: {move}, robot: {c_object.name}, path: {steps_str(c_object.path)}")
        return paths


# Checks if steps can be skipped and if they're possible
def reduce_path(path: list[Move], field: Field, depth: int) -> list[list[Move]]:
    new_path = path
    checking_index = 0
    print("|"*depth + "Reduce---------------------------------------------------")
    while checking_index < len(new_path):
        path = new_path
        new_path = path[:checking_index]
        print("|"*depth + f"path: {path_str(path)}")

        if path[checking_index].waiting:
            new_path.append(path[checking_index])
            if len(path) >= checking_index + 2:
                if path[checking_index + 1].waiting:
                    new_path[-1] = Move(
                        Line(path[checking_index].start, path[checking_index + 1].end),
                        path[checking_index].clearance,
                        path[checking_index].start_time,
                        path[checking_index + 1].end_time
                    )
                else:
                    new_path.append(path[checking_index + 1])
            # print(f"Reduce path new path: {new_path}")
            checking_index += 1
            continue

        intersects = True
        for move in path[:checking_index:-1]:
            # if move.waiting:
            #     continue
            print("|"*depth + f"    Checking move: {move}")
            intersects = False
            new_move = Move(
                Line(path[checking_index].start, move.end),
                path[checking_index].clearance,
                path[checking_index].start_time
            )
            print("|"*depth + f"    New move: {new_move}")

            possible_paths = get_possible_paths(new_move, field, depth)
            best_time = move.end_time
            other_path = []
            while possible_paths:
                p = possible_paths.pop(0)
                print("|"*depth + f"        Checking path: {path_str(p)}")
                # input()
                if p[-1].end != move.end:
                    dest_move = Move(
                        Line(p[-1].end, move.end),
                        path[checking_index].clearance,
                        p[-1].end_time
                    )
                    if dest_move.start_time == dest_move.end_time:
                        input("|"*depth + f"same time dest move: {dest_move.__repr__()}")
                    for p1 in get_possible_paths(dest_move, field, depth):
                        if p1[-1].end_time < best_time:
                            possible_paths.append(p + p1)
                    continue
                if p[-1].end_time < best_time:
                    best_time = p[-1].end_time
                    other_path = p

            if other_path:
                # New path is possible, add it and continue to the next
                new_path = new_path[:checking_index] + other_path + new_path[checking_index:]
                break

            # New move is not possible, try the next
            intersects = True
            new_path.insert(checking_index, move)

        # If no new move was possible, check the current existing move
        if intersects:
            for obstacle in field.obstacles:
                if does_intersect(path[checking_index], obstacle):
                    print("|"*depth + f"Existing move intersects: {path[checking_index]}, path: {steps_str(path)}")
                    return get_possible_paths(path[checking_index], field, depth)
            new_path.insert(checking_index, path[checking_index])

        checking_index += 1

    return [new_path]


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


# Check start and end of a move
def is_point_in_obstacle(p: Point, obstacle: Obstacle, clearance: float) -> bool:
    if not (obstacle.x_min - clearance <= p.x <= obstacle.x_max + clearance and
            obstacle.y_min - clearance <= p.y <= obstacle.y_max + clearance):
        return False
    # print(f"Checking point: {p.__repr__()}")
    intersections = 0
    for vertex in obstacle.vertices:
        if distance(vertex, p) < clearance:
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
                # print(f"edge hit: {edge.__repr__()}")
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

    #     clearance_points = obstacle[move.clearance]
    #     for v, op in clearance_points.outside_points_dict.items():
    #         if is_point_in_obstacle(op, obstacle, move.clearance):
    #             print("Outside point in obstacle")
    #             obstacle[move.clearance].outside_points_dict[v] = Point(float('inf'), float('inf'))
    # print(field.obstacles[0][move.clearance].outside_points_dict)

    print("Loop: 0 ==========================================")
    paths = remove_duplicates(get_possible_paths(move, field, 0))
    # for path in paths:
    #     print(f"Path: {steps_str(path)}")
    destination_reached_loops = 0
    loops = 1
    while True:
        print(f"Loop: {loops}")
        for path in paths:
            print(f"    Path: {path}")
        print("-------------------------------------------")
        loops += 1
        if loops > 20:
            # print("Too many loops")
            return []
        # print(f"Destination reached loops: {destination_reached_loops}")
        if not paths:
            # print("No possible paths")
            return []

        paths_next_loop: list[list[Move]] = []
        destination_reached = False
        for index, path in enumerate(paths):
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

            new_paths = get_possible_paths(new_move_to_dest, field, 0)
            # print(f"{index} new move: {new_move_to_dest}")
            # print(f"New paths: {new_paths}")
            if new_paths == [[new_move_to_dest]]:
                reduced_paths = reduce_path(path + new_paths[0], field, 0)
                for rp in reduced_paths:
                    if rp[-1].end == destination:
                        destination_reached = True
                    # print(f"    {steps_str(rp)}")
                    paths_next_loop.append(rp)
                    # input("Destination reached other")
                continue

            for new_path in new_paths:
                reduced_paths = reduce_path(path + new_path, field, 0)
                for rp in reduced_paths:
                    # print(f"    {steps_str(rp)}")
                    paths_next_loop.append(rp)

        # paths_next_loop = remove_duplicates(paths_next_loop)
        # for index, path in enumerate(paths_next_loop):
        #     print(f"{index}: {steps_str(path)}")
        # input()

        new_paths_next_loop = []
        if destination_reached:
            destination_reached_loops += 1
            shortest_theoretical_path_time = float('inf')
            shortest_actual_path_time = float('inf')
            shortest_path = []
            for path in paths_next_loop:
                # print(f"Path: {steps_str(path)}")
                if path[-1].end == destination:
                    # print(f"Destination reached in {path[-1].end_time}: {steps_str(path)}")
                    if path[-1].end_time < shortest_theoretical_path_time:
                        shortest_theoretical_path_time = path[-1].end_time
                        shortest_path = path
                        shortest_actual_path_time = path[-1].end_time
                        new_paths_next_loop.append(path)

                    elif path[-1].end_time < shortest_actual_path_time:
                        shortest_actual_path_time = path[-1].end_time
                        new_paths_next_loop.append(path)

                elif destination_reached_loops < 4:
                    new_move_to_dest = Move(
                        Line(path[-1].end, destination),
                        move.clearance,
                        path[-1].end_time
                    )
                    if new_move_to_dest.end_time < shortest_theoretical_path_time:
                        shortest_theoretical_path_time = new_move_to_dest.end_time
                        shortest_path = []
                    elif new_move_to_dest.end_time < shortest_actual_path_time:
                        new_paths_next_loop.append(path)

            if shortest_path:
                return shortest_path

        paths_next_loop = remove_duplicates(paths_next_loop)  # todo check why so many duplicates
        paths = paths_next_loop
        # paths = [p for p in paths_next_loop if not path_in_paths(p, paths)]


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
