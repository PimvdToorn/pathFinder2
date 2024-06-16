import asyncio
import json
import msvcrt
import requests
import time
from math import pi

import websockets

from Controller import update_speed_l_and_r
from PathFinder import pathfind, set_path, set_best_paths
from Timer import Timer
from Types import L, P
from objects.Move import Move, steps_str, path_str
from objects.Field import Field
from objects.Obstacle import Obstacle
from objects.Robot import Robot

field = Field((3000, 3000))

clearance = 0.2

field.add_obstacles([
    Obstacle([P(2, 1), P(2, 3), P(6, 4), P(4, 2), P(6, 0), P(4, 1)]),
    Obstacle([P(9, 3), P(6, 6), P(7, 7)]),
    Obstacle([P(5, 5), P(2, 5), P(1, 7), P(3, 9), P(10, 9), P(10, 8), P(3.5, 7.5), P(4, 6)])
])


start_positions = [P(0, 1), P(1, 1), P(9, 7), P(12, 4), P(5, 9.5)]  # , P(5, 2)]
# destinations = [P(6, 7), P(3, 4), P(6, 2), P(2, 9), P(4.5, 7)]  # , P(7, 3)]
# start_positions = [P(0, 1)]
destinations = [P(0, -1)]

address = "http://89.98.134.31:7807/"
for i, sp in enumerate(start_positions):
    field.add_robot(Robot(f"R{i+1}", address, clearance, sp))

# set_best_paths(destinations, field)
# timer = Timer()
# runs = 40
# for i in range(runs):
#     set_best_paths(destinations, field)
#     print(f"Run {i+1} done, elapsed time: {timer.seconds():.1f}s")
#
# timer.stop()
# print(f"Total time: {timer.seconds():.1f}s")
# print(f"Avg time: {timer.seconds()/runs:.3f}s")

set_path(destinations[0], field.robots[0], field, 0)
print(f"Robot 1 path: {path_str(field.robots[0].path)}")
print(update_speed_l_and_r(field.robots[0], 10_000_000, P(0.0, 0.0), 190, field))

# timer = Timer()
# while True:
#     print(f"\rTime: {timer.seconds():.1f}s", end="")
#
#     if msvcrt.kbhit():
#         char = msvcrt.getch()
#         if char == b'q':
#             break
#         # if char == b'f': todo formation
#         if b'0' < char <= b'9':
#             if int(char) > len(field.robots):
#                 print(f"\rThere is no robot {int(char)}")
#                 continue
#             robot = field.robots[int(char) - 1]
#             print(f"\rRobot {robot.name} path: {path_str(robot.path)}")
#     time.sleep(0.1)


timer = Timer()


async def handler(websocket):
    while True:
        try:
            message = await websocket.recv()

        # client disconnected?
        except websockets.ConnectionClosedOK:
            break

        field.robots[0].name = message
        _ = asyncio.create_task(send(websocket))
        print(f"Received message: {message}")


async def send(websocket):
    print("Sending data")

    if msvcrt.kbhit():
        char = msvcrt.getch()
        if char == b'q':
            print(field.robots[0].name)

    data = []
    for robot in field.robots:
        left, right = update_speed_l_and_r(robot, timer.ns(), P(0.0, 0.0), 90, field)
        print(f"Robot {robot.name} speeds: {left}, {right}")
        data.append({
            "name": robot.name,
            "left": left,
            "right": right
        })

        _ = asyncio.create_task(send_robot_update(robot, left, right))

    try:
        await websocket.send(json.dumps(data))

    # client disconnected?
    except websockets.ConnectionClosedOK:
        pass


async def send_robot_update(robot: Robot, left: float, right: float):
    try:
        print(f"Sending data to {robot.name}")
        response = requests.get(robot.address, params={
            "left_motor_speed": left,
            "right_motor_speed": right
        }, timeout=0.1)
        if response.status_code != 200:
            print(f"Error sending data to {robot.name}: {response.status_code}")
        else:
            print(f"Sent data to {robot.name}")
    except requests.exceptions.ConnectionError:
        print(f"Error connecting to {robot.name}")
    except requests.exceptions.Timeout:
        print(f"Timeout connecting to {robot.name}")


async def main():
    async with websockets.serve(handler, "localhost", 8765):
        await asyncio.Future()  # run forever


if __name__ == "__main__":
    asyncio.run(main())
