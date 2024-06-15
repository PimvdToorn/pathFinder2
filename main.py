from math import pi

from Controller import update_speed_and_heading
from MathHelper import offset_line, get_heading
from PathFinder import pathfind, set_path
from Types import L, P
from objects.Move import Move, steps_str, path_str
from objects.Field import Field
from objects.Obstacle import Obstacle
from objects.Robot import Robot

field = Field((3000, 3000))

clearance = 0.2

field.add_obstacles([
    Obstacle([P(2, 1), P(2, 3), P(6, 4), P(4, 2), P(6, 0), P(4, 1)], clearance),
    Obstacle([P(9, 3), P(6, 6), P(7, 7)], clearance),
    Obstacle([P(5, 5), P(2, 5), P(1, 7), P(3, 9), P(10, 9), P(10, 8), P(3.5, 7.5), P(4, 6)], clearance)
])


# start_positions = [P(0, 1), P(1, 1), P(9, 7), P(12, 4), P(5, 9.5), P(5, 2)]
# destinations = [P(6, 7), P(3, 4), P(6, 2), P(2, 9), P(4.5, 7), P(7, 3)]
start_positions = [P(0, 1)]
destinations = [P(0, -1)]

for i, sp in enumerate(start_positions):
    field.add_robot(Robot(f"R{i+1}", i, clearance, sp))

# get_best_paths(destinations, field)
set_path(destinations[0], field.robots[0], field, 0)
print(f"Robot 1 path: {path_str(field.robots[0].path)}")
update_speed_and_heading(field.robots[0], 10_000_000, P(0.01, 0), 0, field)


