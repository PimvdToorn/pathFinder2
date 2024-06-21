import msvcrt

from BestPaths import set_path, set_best_paths
from MathHelper import distance
from Timer import Timer
from Types import P
from objects.Field import Field
from objects.Robot import Robot


def int_input(prompt: str, minimum=None, maximum=None) -> int:
    while True:
        try:
            number = int(input(prompt))
        except ValueError:
            print("Invalid input, should be an integer")
            continue
        if minimum is not None and number < minimum:
            print(f"Invalid input, should be at least {minimum}")
            continue
        if maximum is not None and number > maximum:
            print(f"Invalid input, should be at most {maximum}")
            continue
        return number


def float_input(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Invalid input, should be a float")


def select_robots(msg: str, field: Field, amount=None) -> list[Robot]:
    available_robots = [r for r in field.robots if r.destination is None]

    if amount is None:
        amount = int_input(msg, 0, len(available_robots))
    if amount == 0:
        return []

    for i, robot in enumerate(available_robots):
        print(f"{i + 1} - {robot.name}: {robot.location.bare_str()}")

    used_robot_numbers = []
    for i in range(amount):
        while True:
            robot_number = int_input(f"Enter robot number {i + 1}: ", 1, len(available_robots))
            if robot_number in used_robot_numbers:
                print("Robot already used")
                continue
            used_robot_numbers.append(robot_number)
            break

    return [available_robots[i - 1] for i in used_robot_numbers]


def command_input(field: Field, timer: Timer) -> None:
    char = msvcrt.getch()
    match char:
        case b'q':
            exit()
        case b'r':
            if input("Reset all robots? (y/n): ").lower() == "y":
                for robot in field.robots:
                    robot.path = []
                    robot.destination = None
                print("Reset all robots========================================================================")
        case b'd':
            amount = int_input("Enter number of destinations: ", 0)
            if amount == 0:
                return

            destinations = []
            if input("Select specific robots? (y/n): ").lower() == "y":
                used_robots = select_robots("", field, amount)

                for r in used_robots:
                    x = float_input(f"Enter x position for destination {r.name}: ")
                    y = float_input(f"Enter y position for destination {r.name}: ")
                    destinations.append(P(x, y))
                for i, r in enumerate(used_robots):
                    set_path(destinations[i], r, field, timer.ns())
            else:
                for i in range(amount):
                    x = float_input(f"Enter x position for destination {i + 1}: ")
                    y = float_input(f"Enter y position for destination {i + 1}: ")
                    destinations.append(P(x, y))
                set_best_paths(destinations, field, timer.ns(), True, True)
            print("Done calculating paths====================================================================")

        case b'c':
            used_robots = select_robots("Enter number of robots in combined bot: ", field)
            amount = len(used_robots)
            if amount == 0:
                return

            x = float_input(f"Enter x position for the destination: ")
            y = float_input(f"Enter y position for the destination: ")
            destination = P(x, y)

            average_position = sum([r.location for r in used_robots], start=P(0, 0)) / amount
            distances = {distance(r.location, average_position): r for r in used_robots}
            furthest_distance = max(d for d in distances)
            combined_robot = Robot(
                "Combined",
                "",
                furthest_distance + distances[furthest_distance].radius,
                average_position
            )
            for robot in used_robots:
                combined_robot.combined_robots.append((robot, robot.location - average_position))
                field.robots.remove(robot)
            field.robots.append(combined_robot)

            set_path(destination, combined_robot, field, timer.ns())
            combined_robot.heading = -1
