from MathHelper import offset_line
from PathFinder import pathfind
from Types import L, P
from objects.Move import Move, steps_str, list_str
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

robot1 = Robot("R1", 0x01, clearance, 1, 1)
robot2 = Robot("R2", 0x02, clearance, 9, 7)
field.add_robots([robot1, robot2])

d1 = P(6, 7)
d2 = P(3, 4)

# robot1.path = pathfind(robot1.create_move(d1, 0), field)
print("------------------------------------------------------------------")
robot2.path = pathfind(robot2.create_move(d2), field)

print(steps_str(robot1.path))
print(robot1.path)

print(steps_str(robot2.path))
print(robot2.path)
