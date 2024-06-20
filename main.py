import asyncio
import json
import msvcrt
import requests
import time
from math import pi, cos, sin

import websockets
from orjson import orjson

from Controller import update_speed_l_and_r, get_expected_location_and_move, update_combined_speed_l_and_r
from MathHelper import distance, get_heading
from BestPaths import set_path, set_best_paths
from Timer import Timer
from Types import L, P, Point
from objects.Move import Move, steps_str, path_str
from objects.Field import Field
from objects.Obstacle import Obstacle
from objects.Robot import Robot

CLEARANCE = 0.08
# HOSTING_IP = "192.168.137.128"
HOSTING_IP = "localhost"
HOSTING_PORT = 8765
# R1_ADDRESS = "http://192.168.137.240/"
# R2_ADDRESS = "http://192.168.137.204/"
R1_ADDRESS = ""
R2_ADDRESS = ""


field = Field((3000, 3000))
field.add_obstacles([
    # Obstacle([P(2, 1), P(2, 3), P(6, 4), P(4, 2), P(6, 0), P(4, 1)]),
    # Obstacle([P(9, 3), P(6, 6), P(7, 7)]),
    # Obstacle([P(5, 5), P(2, 5), P(1, 7), P(3, 9), P(10, 9), P(10, 8), P(3.5, 7.5), P(4, 6)])
])

field.add_robot(Robot("R1", R1_ADDRESS, CLEARANCE, P(2.450000, 2.410000)))
field.add_robot(Robot("R2", R2_ADDRESS, CLEARANCE, P(2.370000, 2.430000)))
field.add_robot(Robot("C1", "", CLEARANCE, P(-0.000000, 0.000259)))
field.add_robot(Robot("C2", "", CLEARANCE, P(0.500000, 0.000259)))
field.add_robot(Robot("C3", "", CLEARANCE, P(2.120000, 1.170259)))
field.add_robot(Robot("C4", "", CLEARANCE, P(1.600000, 1.890268)))

# set_best_paths([P(1, 0), P(1, 1), P(1, 2), P(2, 0), P(2, 1), P(2, 2)], field, 0, True, True)
# set_best_paths([P(1, 0), P(1, 1), P(1, 2), P(2, 1), P(2, 2)], field, 0, True, True)
# set_best_paths([P(1, 0), P(1, 1), P(1, 2), P(2, 0)], field, 0, True, True)

# field.add_robot(Robot("R1", R1_ADDRESS, CLEARANCE, P(10, 10)))
# field.add_robot(Robot("R2", R2_ADDRESS, CLEARANCE, P(20, 20)))
# field.add_robot(Robot("C1", "", CLEARANCE, P(30, 30)))
# field.add_robot(Robot("C2", "", CLEARANCE, P(40, 40)))
# field.add_robot(Robot("C3", "", CLEARANCE, P(50, 50)))
# field.add_robot(Robot("C4", "", CLEARANCE, P(60, 60)))
#
# set_best_paths([P(10, 20), P(20, 10), P(30, 40), P(40, 30), P(50, 60), P(60, 50)], field, 0, True, True)

# exit()
# set_path(P(0, 10), field.robots[0], field, 0)
# set_path(P(1, 10), field.robots[1], field, 0)
# set_path(P(0, 2), field.robots[2], field, 0)
# set_path(P(0.5, 2), field.robots[3], field, 0)
# set_path(P(1, 2), field.robots[4], field, 0)
# set_path(P(1.5, 2), field.robots[5], field, 0)
print("Done calculating paths")


class FrameCounter:
    frame = 0
    frame_timer = Timer()
    heading_timer = Timer()

    def __init__(self):
        self.frame_timer.reset()
        self.heading_timer.stop()

    def get_count(self):
        self.frame += 1
        return self.frame

    def get_time(self):
        elapsed_ms = self.frame_timer.ns() // 1000_000
        self.frame_timer.reset()
        return elapsed_ms


counter = FrameCounter()
timer = Timer()


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


def select_robots(msg: str, amount=None) -> list[Robot]:
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


def command_input():
    # for robot in field.robots:
    #     _ = asyncio.create_task(send_robot_update(robot, 0.0, 0.0))
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
                used_robots = select_robots("", amount)

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
            used_robots = select_robots("Enter number of robots in combined bot: ")
            amount = len(used_robots)
            if amount == 0:
                return

            x = float_input(f"Enter x position for the destination: ")
            y = float_input(f"Enter y position for the destination: ")
            destination = P(x, y)

            average_position = sum([r.location for r in used_robots], start=Point(0, 0)) / amount
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


async def handler(websocket):
    while True:
        try:
            message = await websocket.recv()

        # client disconnected?
        except websockets.ConnectionClosedOK:
            break

        dict_message = orjson.loads(message)

        # print(dict_message)
        print(f"----------------------------------------------------------------- {
            counter.get_count()} - {counter.get_time()}ms")
        for robot in field.robots:
            # todo does this work?
            if robot.combined_robots:
                locations = []
                # headings = []
                for r in robot.combined_robots:
                    r[0].location = P(*dict_message["position_" + r[0].name])

                    rotation = dict_message["rotation_" + r[0].name]
                    rotation = -rotation[3] if rotation[2] > 0 else rotation[3]
                    r[0].heading = rotation % (2 * pi)

                    locations.append(r[0].location)
                    # headings.append(r[0].heading)
                robot.location = sum(locations, start=Point(0, 0)) / len(locations)
                # robot.heading = sum(headings) / len(headings)
            else:
                robot.location = P(*dict_message["position_" + robot.name])
                rotation = dict_message["rotation_" + robot.name]
                rotation = -rotation[3] if rotation[2] > 0 else rotation[3]
                robot.heading = rotation % (2 * pi)

            # print(dict_message["rotation_" + robot.name])
            print(f"Robot {robot.name} location: {robot.location}, heading: {robot.heading}")

        if msvcrt.kbhit():
            _ = await asyncio.create_task(send(websocket, True))
            command_input()
        else:
            _ = asyncio.create_task(send(websocket))


async def send(websocket, stop=False):
    # print("Sending data")

    data: dict[str, list[float]] = {r.name: [] for r in field.robots if not r.address}
    for robot in field.robots:
        # print(f"Robot {robot.name} location: {robot.location}, heading: {robot.heading}")
        if stop:
            data[robot.name] = [0, 0]

        elif robot.combined_robots:
            current_time = timer.ns()
            expected_location, move = get_expected_location_and_move(robot, current_time, field)
            if move is None:
                for r in robot.combined_robots:
                    field.robots.append(r[0])
                    data[r[0].name] = [0, 0]
                field.robots.remove(robot)
                continue

            heading = get_heading(move.line)
            if abs(robot.heading - heading) < 0.025 * pi:
                robot.heading = heading

            only_rotate = heading != robot.heading
            # print(f"{only_rotate} - {heading} - {robot.heading}")
            if only_rotate and all(abs(r.heading-heading) < 0.025*pi for r, _ in robot.combined_robots):
                if counter.heading_timer.stopped:
                    counter.heading_timer.reset()
                elif counter.heading_timer.ns() > 100_000_000:
                    robot.heading = heading
                    # input("Heading done")
                    only_rotate = False
                    counter.heading_timer.stop()
            elif not counter.heading_timer.stopped:
                counter.heading_timer.stop()

            # only_rotate = True
            for combined_robot, offset in robot.combined_robots:
                print(f"Combined robot {combined_robot.name} heading: {combined_robot.heading} / {heading} - {abs(combined_robot.heading-heading)} < {0.025*pi}")
                # expected_location = expected_location + offset  # .rotate(heading)
                c_move = Move(move.line + offset, move.clearance, move.start_time, move.end_time)

                left, right = update_combined_speed_l_and_r(combined_robot, c_move, current_time, only_rotate)
                data[combined_robot.name] = [left, right]
                print(f"Robot {combined_robot.name} speeds: {left}, {right}")
                if robot.address:
                    _ = asyncio.create_task(send_robot_update(robot, left, right))
        else:
            left, right = update_speed_l_and_r(robot, timer.ns(), field)
            data[robot.name] = [left, right]
            print(f"Robot {robot.name} speeds: {left}, {right}")
        # print(f"Robot {robot.name} destination: {robot.destination}")

            if robot.address:
                _ = asyncio.create_task(send_robot_update(robot, left, right))

    try:
        await websocket.send(json.dumps(data))

    # client disconnected?
    except websockets.ConnectionClosedOK:
        pass


async def send_robot_update(robot: Robot, left: float, right: float):
    try:
        response = requests.get(robot.address, params={
            "left_motor_speed": f"{left:.1f}",
            "right_motor_speed": f"{right:.1f}"
        })
        # print(f"Sent data to {robot.name}: left_motor_speed: {left}, right_motor_speed: {right}")
        if response.status_code != 200:
            print(f"Error sending data to {robot.name}: {response.status_code}")
    except requests.ConnectionError as e:
        if isinstance(e.args[0], ConnectionResetError):
            print(f"Connection to {robot.name} was forcibly closed by the remote host.")
        else:
            print(f"An error occurred while trying to connect to {robot.name}: {e}")
    except requests.RequestException as e:
        print(f"An error occurred while trying to send data to {robot.name}: {e}")


async def main():
    async with websockets.serve(handler, HOSTING_IP, HOSTING_PORT):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())
