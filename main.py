from PathFinder import get_possible_paths, reduce_path
from Types import L, P
from objects.Move import Move, steps_str
from objects.Field import Field
from MathHelper import Line, line_intersection
from objects.Obstacle import Obstacle
from objects.Robot import Robot

field = Field((3000, 3000))

robot1 = Robot("R1", 0x01, 200)
field.add_robot(robot1)

obstacle = Obstacle([P(2, 1), P(2, 3), P(6, 4), P(4, 2), P(6, 0), P(4, 1)], 0.2)
field.add_obstacle(obstacle)


line = L((6, 6), (1, 0))

move = Move(line, 0.2)


paths = get_possible_paths(move, field)
for path in paths:
    print(steps_str(path))
    print(path)
    print("-------------------------------------------------------------")
    path = reduce_path(path, field)
    print(steps_str(path))
    print(path)
    print()
    print("=============================================================")
    print()

print(reduce_path([move], field))

# angle to side of obstacle
# Direct routes and single steps
