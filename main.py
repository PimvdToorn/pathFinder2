from math import sqrt, cos, acos

from MathHelper import distance_point_to_point, line_intersection, line_intersection_t, line_intersection_t_u, \
    cos_angle_between_lines, leg_from_base_and_lines
from PathFinder import pathfind, first_robot_intersection
from Types import L, P
from objects.Move import Move, steps_str, min_distance, list_str
from objects.Field import Field
from objects.Obstacle import Obstacle
from objects.Robot import Robot

field = Field((3000, 3000))

clearance = 0.2
robot1 = Robot("R1", 0x01, clearance)
field.add_robot(robot1)

field.add_obstacles([
    Obstacle([P(2, 1), P(2, 3), P(6, 4), P(4, 2), P(6, 0), P(4, 1)], clearance),
    Obstacle([P(9, 3), P(6, 6), P(7, 7)], clearance),
    Obstacle([P(5, 5), P(2, 5), P(1, 7), P(3, 9), P(10, 9), P(10, 8), P(3.5, 7.5), P(4, 6)], clearance)
])

start = P(4, 9.5)
end = P(8, 0)

line = L(start, end)
move = Move(line, clearance)

# path = pathfind(move, field)
# print(steps_str(path))
# print(list_str(path))
# # print(path[-1].end_time)
# robot1.path = path  #+ [Move(L(path[-1].end, P(8, -4)), clearance, path[-1].end_time)]
# print(robot1.path)
# angle to side of obstacle
# Direct routes and single steps

# m1 = Move(L(12.7, 0, 7, 0.2), 0.2, 11478.501075915352)

# t, move = first_robot_intersection(m1, robot1)
# print(f"t: {t-11478.501075915352}")
# print(f"move: {move.__repr__()}")

# m1 = Move(L(2, 3, 0.5, 0), 0.2, 0, 1)
# m2 = Move(L(2, 0, 0, 3), 0.2, 0, 1)
#
# d, t = min_distance(m1, m2)
# print(f"d: {d}, t: {t}")

base = 0.4
l1 = L(-1, 1, 1, -1)
l2 = L(-1, 0, 1, 0)
a = leg_from_base_and_lines(base, l1, l2)
print(a*2)
