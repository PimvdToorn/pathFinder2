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

field = Field((3000, 3000))

clearance = 0.08

field.add_obstacles([
    # Obstacle([P(2, 1), P(2, 3), P(6, 4), P(4, 2), P(6, 0), P(4, 1)]),
    # Obstacle([P(9, 3), P(6, 6), P(7, 7)]),
    # Obstacle([P(5, 5), P(2, 5), P(1, 7), P(3, 9), P(10, 9), P(10, 8), P(3.5, 7.5), P(4, 6)])
])


# start_positions = [P(0, 1), P(1, 1), P(9, 7), P(12, 4), P(5, 9.5), P(5, 2)]
# destinations = [P(6, 7), P(3, 4), P(6, 2), P(2, 9)]  # , P(4.5, 7)]  # , P(7, 3)]
# start_positions = [P(0, 1)]
# destinations = [P(0, -1)]

address = "http://89.98.134.31:7807/"
# for i, sp in enumerate(start_positions):
#     if i < 2:
#         name = f"R{i+1}"
#     else:
#         name = f"C{i-1}"
#     field.add_robot(Robot(name, "", clearance, sp))

# field.add_robot(Robot("R1", "http://172.20.10.3/", clearance, P(0, 0)))
# field.add_robot(Robot("R2", "http://172.20.10.5/", clearance, P(-1, 0)))
field.add_robot(Robot("R1", "", clearance, P(0, 0)))
field.add_robot(Robot("R2", "", clearance, P(-1, 0)))
field.add_robot(Robot("C1", "", clearance, P(0, 0)))
field.add_robot(Robot("C2", "", clearance, P(0.5, 0)))
field.add_robot(Robot("C3", "", clearance, P(-0.3500000, -0.2599019)))
field.add_robot(Robot("C4", "", clearance, P(0.1700000, 0.5500981)))

# set_path(P(0, 10), field.robots[0], field, 0)
# set_path(P(1, 10), field.robots[1], field, 0)
# set_path(P(0, 2), field.robots[2], field, 0)
# set_path(P(0.5, 2), field.robots[3], field, 0)
# set_path(P(1, 2), field.robots[4], field, 0)
# set_path(P(1.5, 2), field.robots[5], field, 0)
print("Done calculating paths")


timer = Timer()


def int_input(prompt: str, minimum=None) -> int:
    while True:
        try:
            number = int(input(prompt))
        except ValueError:
            print("Invalid input, should be an integer")
            continue
        if minimum is not None and number < minimum:
            print(f"Invalid input, should be at least {minimum}")
            continue
        return number


def float_input(prompt: str) -> float:
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Invalid input, should be a float")


async def command_input():
    # for robot in field.robots:
    #     _ = asyncio.create_task(send_robot_update(robot, 0.0, 0.0))
    char = msvcrt.getch()
    match char:
        case b'q':
            exit()
        case b'd':
            amount = int_input("Enter number of destinations: ", 0)
            if amount == 0:
                return

            destinations = []
            for i in range(amount):
                x = float_input(f"Enter x position for destination {i + 1}: ")
                y = float_input(f"Enter y position for destination {i + 1}: ")
                destinations.append(P(x, y))
            set_best_paths(destinations, field, timer.ns(), True)
            print("Done calculating paths====================================================================")

        case b'c':
            available_robots = [r for r in field.robots if r.destination is None]
            while True:
                amount = int_input("Enter number of robots in combined bot: ")
                if amount == 0:
                    return
                if 0 < amount <= len(available_robots):
                    break
                print(f"Invalid number of robots, should be between 1 and {len(available_robots)}")

            for i, robot in enumerate(available_robots):
                print(f"{i+1} - {robot.name}: {robot.location.bare_str()}")

            used_robot_numbers = []
            for i in range(amount):
                while True:
                    robot_number = int_input(f"Enter robot number {i + 1}: ", 1)
                    if robot_number in used_robot_numbers:
                        print("Robot already used")
                        continue
                    used_robot_numbers.append(robot_number)
                    break

            used_robots = [available_robots[i-1] for i in used_robot_numbers]

            x = float_input(f"Enter x position for the destination: ")
            y = float_input(f"Enter y position for the destination: ")
            destination = P(x, y)

            average_position = sum([r.location for r in used_robots], start=Point(0, 0)) / amount
            distances = {distance(r.location, average_position): r for r in used_robots}
            furthest_distance = max(d for d in distances)
            combined_robot = Robot(
                "Combined",
                "",
                furthest_distance+distances[furthest_distance].radius,
                average_position
            )
            for robot in used_robots:
                combined_robot.combined_robots.append((robot, robot.location - average_position))
                field.robots.remove(robot)
            field.robots.append(combined_robot)
            set_path(destination, combined_robot, field, timer.ns())


async def handler(websocket):
    while True:
        try:
            message = await websocket.recv()

        # client disconnected?
        except websockets.ConnectionClosedOK:
            break

        dict_message = orjson.loads(message)

        # print(dict_message)
        for robot in field.robots:
            # todo does this work?
            if robot.combined_robots:
                locations = []
                headings = []
                for r in robot.combined_robots:
                    r[0].location = P(*dict_message["position_" + r[0].name])

                    rotation = dict_message["rotation_" + r[0].name]
                    rotation = -rotation[3] if rotation[2] > 0 else rotation[3]
                    r[0].heading = rotation % (2 * pi)

                    locations.append(r[0].location)
                    headings.append(r[0].heading)
                robot.location = sum(locations, start=Point(0, 0)) / len(locations)
                robot.heading = sum(headings) / len(headings)
            else:
                robot.location = P(*dict_message["position_" + robot.name])
                rotation = dict_message["rotation_" + robot.name]
                rotation = -rotation[3] if rotation[2] > 0 else rotation[3]
                robot.heading = rotation % (2*pi)

            # print(dict_message["rotation_" + robot.name])
            # print(f"Robot {robot.name} location: {robot.location}, heading: {robot.heading}")

        if msvcrt.kbhit():
            _ = await asyncio.create_task(send(websocket, True))
            _ = asyncio.create_task(command_input())
        else:
            _ = asyncio.create_task(send(websocket))


async def send(websocket, stop=False):
    # print("Sending data")

    data: dict[str, list[float]] = {r.name: [] for r in field.robots if not r.address}
    for robot in field.robots:
        # print(f"Robot {robot.name} location: {robot.location}, heading: {robot.heading}")
        if stop:
            left, right = 0, 0
        else:
            left, right = update_speed_l_and_r(robot, timer.ns(), field)
        # print(f"Robot {robot.name} speeds: {left}, {right}")

        if robot.combined_robots:
            expected_location, move = get_expected_location_and_move(robot, timer.ns(), field)
            heading = get_heading(move.line)
            for combined_robot, offset in robot.combined_robots:
                expected_location = combined_robot.location + offset.rotate(heading)
                left, right = update_combined_speed_l_and_r(combined_robot, expected_location, move, heading)
        data[robot.name] = [left, right]

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
            "left_motor_speed": left,
            "right_motor_speed": right
        })
        print(f"Sent data to {robot.name}: left_motor_speed: {left}, right_motor_speed: {right}")
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
    # async with websockets.serve(handler, "172.20.10.8", 8765):
    async with websockets.serve(handler, "localhost", 8765):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())
