from itertools import permutations

from PathFinder import pathfind
from Timer import Timer
from Types import Point
from objects.Field import Field
from objects.Move import steps_str, Move
from objects.Robot import Robot
from time import sleep


def set_path(destination: Point, robot: Robot, field: Field, time: int) -> None:
    robot.path = []
    move = robot.create_move(destination, time)
    # remove_imp_outside_points(field, move.clearance)
    robot.path = pathfind(move, field)


def set_best_paths(destinations: list[Point], field: Field, time: int, verbose_result=False, verbose_progress=False) -> None:
    unavailable_robots = []
    robots = []
    for robot in field.robots:
        if robot.destination:
            unavailable_robots.append(robot)
        elif robot.combined_robots:
            robots.insert(0, robot)
        else:
            robots.append(robot)
    field.robots = unavailable_robots + robots
    if len(robots) < len(destinations):
        print("Not enough robots")
        for robot in field.robots:
            print(f"Robot {robot.name} path: {steps_str(robot.path)}, destination: {robot.destination}")
        return

    all_dest_orders = list(permutations(destinations))
    all_robot_orders = list(
        {p[:len(destinations)] for p in permutations(range(len(unavailable_robots), len(field.robots)))}
    )
    # print(all_dest_orders)
    # print(all_robot_orders)
    total = len(all_dest_orders) * len(all_robot_orders)

    best_max_end_time = float('inf')
    best_paths: list[list[Move]] = [robot.path for robot in field.robots]
    timer = Timer()
    timer_dest_order = Timer()
    for index, dest_order in enumerate(all_dest_orders):
        if verbose_progress:
            print(f"\rProgress: {index * len(all_robot_orders)}/{total}", end="")
            # input()
            timer_dest_order.reset()

        for rindex, robot_order in enumerate(all_robot_orders):
            # for robot in field.robots:
            #     print(f"Robot {robot.name} path: {steps_str(robot.path)}")
            print("-----------------------------------------------------------------------------------------")
            print(f"Destination order: {dest_order}")
            print(f"Robot order {rindex+1}/{len(all_robot_orders)}: {robot_order}")
            # if index > 3:
            #     input()

            # todo remove known impossible orders
            max_end_time = 0.0
            for i, d in enumerate(dest_order):
                print("---------------------------------")
                robot = field.robots[robot_order[i]]
                print(f"Robot {robot.name} to {d}")
                robot.path = pathfind(robot.create_move(d, time), field)
                print(f"Robot {i + 1}: {steps_str(robot.path)}")
                if robot.path:
                    max_end_time = max(max_end_time, robot.path[-1].end_time)
                else:
                    max_end_time = float('inf')
                if max_end_time >= best_max_end_time:
                    break
            if max_end_time < best_max_end_time:
                best_max_end_time = max_end_time
                best_paths = [robot.path for robot in field.robots]

            for robot in robot_order:
                field.robots[robot].path = []
                field.robots[robot].destination = None

        if verbose_progress:
            print(f"  --  Time: {timer_dest_order.seconds():.3f}s", end="")
            print(f"  --  Time left: {(timer.seconds() / (index+1)) * (len(all_dest_orders) - index-1):.1f}s", end="")

    timer.stop()

    if verbose_result:
        print("==================================================================")
    for i, r in enumerate(field.robots):
        r.path = best_paths[i]
        if r.path:
            r.destination = r.path[-1].end

        if verbose_result:
            print(f"Robot {r.name}: {steps_str(r.path)}")
            print(r.path)
    if verbose_result:
        print(f"Total elapsed time: {timer.seconds():.3f}s")