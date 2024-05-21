from objects.Move import Move
from objects.Field import Field
from objects.Robot import Robot

field = Field((3000, 3000))

robot1 = Robot("R1", 0x01, 200)

field.add(robot1)

robot1[0] = Move(1000, (100, 150), (300, 800))
move = robot1[0]
move.end_time = 5000
robot1[0] = move
print(robot1[0])

print(robot1.path[0])
robot1.set_end_time(0, 69)
print(robot1.path[0])

# angle to side of obstacle
# Direct routes and single steps
