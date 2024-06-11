from math import sqrt

from MathHelper import distance_point_to_point, line_intersection, line_intersection_t, line_intersection_t_u
from PathFinder import pathfind
from Types import L, P
from objects.Move import Move, steps_str, min_distance
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
end = P(6, 5)

line = L(start, end)
move = Move(line, clearance)

# path = pathfind(move, field)
# print(steps_str(path))
# print(path)
# print(path[-1].end_time)


# paths = get_possible_paths(move, field)
# for path in paths:
#     print(steps_str(path))
#     print(path)
#     print("-------------------------------------------------------------")
#     path = reduce_path(path, field)
#     print(steps_str(path))
#     print(path)
#     print()
#     print("=============================================================")
#     print()

# print(reduce_path([move], field))


# angle to side of obstacle
# Direct routes and single steps

m1 = Move(L(2, 3, 1, 0), 0.2, 0, 0.001)
m2 = Move(L(2, 0, 0, 3), 0.2, 4000, 0.002)

min_dist, t = min_distance(m1, m2)
print(f"min_dist: {min_dist}, t: {t}")


# def tr(m: Move):
#     return lambda t: t/m.line.len
#
#
# def pr(m: Move):
#     return lambda t: m.start + (m.end-m.start) * (t/m.line.len)
#
#
# def dt(m1: Move, m2: Move):
#     # p1x = m1.x1 + (m1.x2-m1.x1) * (t/m1.line.len)
#     # p1y = m1.y1 + (m1.y2-m1.y1) * (t/m1.line.len)
#     # p2x = m2.x1 + (m2.x2-m2.x1) * (t/m2.line.len)
#     # p2y = m2.y1 + (m2.y2-m2.y1) * (t/m2.line.len)
#     return lambda t: distance_point_to_point(pr(m1)(t), pr(m2)(t))




