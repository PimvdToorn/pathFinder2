from itertools import permutations

from MathHelper import remove_duplicates_ordered
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
    all_robot_orders = [p[:len(destinations)] for p in permutations(range(len(unavailable_robots), len(field.robots)))]
    all_robot_orders = remove_duplicates_ordered(all_robot_orders)
    # print(all_dest_orders)
    # print(all_robot_orders)
    total = len(all_dest_orders) * len(all_robot_orders)

    best_max_end_time = float('inf')
    best_paths: list[list[Move]] = [robot.path for robot in field.robots]
    timer = Timer()
    timer_dest_order = Timer()
    bad_dest_orders = {}
    for index, dest_order in enumerate(all_dest_orders):
        if verbose_progress:
            # print()
            print(f"\rProgress: {index}/{len(all_dest_orders)} - {index * len(all_robot_orders)}/{total}", end="")
            # input()
            timer_dest_order.reset()

        # if index < 8:
        #     continue

        # bad_orders = []
        # for b_order in bad_dest_orders:
        #     if dest_order[:len(b_order)] == b_order:
        #         bad_orders += bad_dest_orders[b_order]

        # orders_to_check = all_robot_orders todo remove known impossible orders
        for rindex, robot_order in enumerate(all_robot_orders):
            # if rindex < 37:
            #     continue

            # for robot in field.robots:
            #     print(f"Robot {robot.name} path: {steps_str(robot.path)}")
            # print("-----------------------------------------------------------------------------------------")
            # print(f"Destination order: {dest_order}")
            # print(f"Robot order {rindex+1}/{len(all_robot_orders)}: {robot_order}")

            # is_bad_order = False
            # for bad_order in bad_orders:
            #     if bad_order == robot_order[:len(bad_order)]:
            #         is_bad_order = True
            #         # if index > 7:
            #         #     input(f"Bad order: {bad_order}")
            #         break
            # if is_bad_order:
            #     continue

            # if index > 7:
            #     input()

            # todo remove known impossible orders
            max_end_time = 0.0
            for i, d in enumerate(dest_order):
                # print("---------------------------------")
                robot = field.robots[robot_order[i]]
                # print(f"Robot {robot.name} to {d}")
                robot.path = pathfind(robot.create_move(d, time), field)
                # print(f"Robot {i + 1}: {steps_str(robot.path)}")
                if robot.path:
                    max_end_time = max(max_end_time, robot.path[-1].end_time)
                else:
                    max_end_time = float('inf')
                if max_end_time >= best_max_end_time:
                    # bad_orders.append(robot_order[:i+1])
                    # bad_dest_orders[dest_order[:i+1]] = bad_dest_orders.get(dest_order[:i+1], []) + [robot_order[:i+1]]
                    break
            if max_end_time < best_max_end_time:
                print(f"New best time: {max_end_time}")
                best_max_end_time = max_end_time
                best_paths = [robot.path for robot in field.robots]

            for robot in robot_order:
                field.robots[robot].path = []
                field.robots[robot].destination = None

        if verbose_progress:
            print(f"  --  Time: {timer_dest_order.seconds():.3f}s", end="")
            print(f"  --  Time left: {(timer.seconds() / (index+1)) * (len(all_dest_orders) - index-1):.1f}s  ", end="")

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