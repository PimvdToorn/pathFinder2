from PathFinder import get_possible_paths, reduce_path, pathfind, first_intersection
from Types import L, P
from objects.Move import Move, steps_str
from objects.Field import Field
from MathHelper import Line, line_intersection, distance_point_to_point
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

path = pathfind(move, field)
print(steps_str(path))
print(path)
print(path[-1].end_time)


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
