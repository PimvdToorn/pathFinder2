import asyncio
import json
import msvcrt
import requests
from math import pi

import websockets
from orjson import orjson

from Controller import update_speed_l_and_r, get_expected_location_and_move, update_combined_speed_l_and_r
from MathHelper import distance, get_heading
from BestPaths import set_path, set_best_paths
from PathFinder import get_possible_paths
from Terminal import command_input
from Timer import Timer
from Types import L, P, Point
from objects.Move import Move, steps_str, path_str
from objects.Field import Field
from objects.Obstacle import Obstacle, get_box
from objects.Robot import Robot

CLEARANCE = 0.08
# HOSTING_IP = "192.168.137.128"
HOSTING_IP = "localhost"
HOSTING_PORT = 8765
# R1_ADDRESS = "http://192.168.137.240/"
# R2_ADDRESS = "http://192.168.137.204/"
R1_ADDRESS = ""
R2_ADDRESS = ""


field = Field()
field.add_obstacles([
    get_box(P(0.0, 0.6), 0.5, 0.5),
    get_box(P(1.2, 1.4), 0.5, 0.5),
    # Obstacle([P(0.4, 1), P(0.4, 1.2), P(0.9, 0.9), P(0.9, 0.75), P(1.75, 0.75), P(1.5, 0.2), P(0.8, 0.3)])
    # Obstacle([P(2, 1), P(2, 3), P(6, 4), P(4, 2), P(6, 0), P(4, 1)]),
    # Obstacle([P(9, 3), P(6, 6), P(7, 7)]),
    # Obstacle([P(5, 5), P(2, 5), P(1, 7), P(3, 9), P(10, 9), P(10, 8), P(3.5, 7.5), P(4, 6)])
])

field.add_robots([
    Robot("R1", R1_ADDRESS, CLEARANCE, P(2.450, 2.410)),
    Robot("R2", R2_ADDRESS, CLEARANCE, P(2.280, 2.430)),
    Robot("C1", "", CLEARANCE, P(-0.890, 0.490)),
    Robot("C2", "", CLEARANCE, P(0.740, 1.190)),
    Robot("C3", "", CLEARANCE, P(1.450, -0.270)),
    Robot("C4", "", CLEARANCE, P(0.460, 1.370))
])

# print(get_possible_paths(Move(L(P(0.869900, 1.069900), P(0.869900, 1.069900)), CLEARANCE, 0), field, 0))
# input()
# set_best_paths([P(1, 0), P(1, 1), P(1, 2), P(2, 0), P(2, 1), P(2, 2)], field, 0, True, True)
# for r in field.robots:
#     r.path = []
#     r.destination = None
# set_best_paths([P(0.5, 0.5), P(-0.5, 0.5), P(0.75, 1.5), P(1, 1.75), P(1.25, 1.75), P(-0.5, 0.2)], field, 0, True, True)
# for r in field.robots:
#     r.path = []
#     r.destination = None
# set_best_paths([P(0, 0.2), P(0, -0.2), P(-0.2, 0), P(0.2, 0)], field, 0, True, True)

# r = field.robots[2]
# set_path(P(0, 0.2), r, field, 0)
# print(f"Robot {r.name} path: {steps_str(r.path)}")
# print("------------------------------------------------------------------------------------------------")
# r = field.robots[4]
# set_path(P(0, -0.2), r, field, 0)
# print(f"Robot {r.name} path: {steps_str(r.path)}")
# r = field.robots[3]
# set_path(P(0.2, 0.0), r, field, 0)
# print(f"Robot {r.name} path: {steps_str(r.path)}")
# r = field.robots[5]
# set_path(P(-0.2, 0.0), r, field, 0)
# print(f"Robot {r.name} path: {steps_str(r.path)}")
# set_path(P(2, 1), field.robots[4], field, 0)
# print(f"Robot {field.robots[4].name} path: {steps_str(field.robots[4].path)}")
# set_path(P(2, 2), field.robots[5], field, 0)
# print(f"Robot {field.robots[5].name} path: {steps_str(field.robots[5].path)}")
# exit()
# print("Done calculating paths")


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


async def handler(websocket):
    while True:
        try:
            message = await websocket.recv()

        # client disconnected?
        except websockets.ConnectionClosedOK:
            break

        dict_message = orjson.loads(message)

        print(f"----------------------------------------------------------------- {
            counter.get_count()} - {counter.get_time()}ms")
        for robot in field.robots:
            if robot.combined_robots:
                locations = []
                for r in robot.combined_robots:
                    r[0].location = P(*dict_message["position_" + r[0].name])

                    # Rotation comes as an axis-angle representation
                    rotation = dict_message["rotation_" + r[0].name]
                    rotation = -rotation[3] if rotation[2] > 0 else rotation[3]
                    r[0].heading = rotation % (2 * pi)

                    locations.append(r[0].location)
                    print(f"Robot {r[0].name} location: {r[0].location}, heading: {r[0].heading}")
                robot.location = sum(locations, start=Point(0, 0)) / len(locations)
            else:
                robot.location = P(*dict_message["position_" + robot.name])
                rotation = dict_message["rotation_" + robot.name]
                rotation = -rotation[3] if rotation[2] > 0 else rotation[3]
                robot.heading = rotation % (2 * pi)

            print(f"Robot {robot.name} location: {robot.location}, heading: {robot.heading}")

        if msvcrt.kbhit():
            _ = await asyncio.create_task(send(websocket, True))
            command_input(field, timer)
        else:
            _ = asyncio.create_task(send(websocket))


async def send(websocket, stop=False):
    # print("Sending data")

    data: dict[str, list[float]] = {r.name: [] for r in field.robots if not r.address}
    for robot in field.robots:
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
            # If the new heading is only a small deviation from the current heading, first rotating is not necessary
            if abs(robot.heading - heading) < 0.025 * pi:
                robot.heading = heading

            only_rotate = heading != robot.heading
            if only_rotate and all(abs(r.heading-heading) % 2*pi < 0.025*pi for r, _ in robot.combined_robots):
                if counter.heading_timer.stopped:
                    counter.heading_timer.reset()
                # Only stop rotating if all robots have the correct heading for at least 0.1 seconds
                elif counter.heading_timer.ns() > 100_000_000:
                    robot.heading = heading
                    only_rotate = False
                    counter.heading_timer.stop()
            elif not counter.heading_timer.stopped:
                counter.heading_timer.stop()

            for combined_robot, offset in robot.combined_robots:
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
