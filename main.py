from Types import L, P
from objects.Move import Move
from objects.Field import Field
from MathHelper import distance_point_to_line, get_slope, hit_coords_point_to_line, Line, does_intersect, \
    first_intersection, line_intersection, is_point_in_obstacle
from objects.Obstacle import Obstacle
from objects.Robot import Robot

field = Field((3000, 3000))

robot1 = Robot("R1", 0x01, 200)

field.add(robot1)
robot1.path.append(Move(1000, Line(P(0, 2), P(10, -4)), robot1.radius))
# print(robot1.get_location(500))
# print(robot1.get_location(750))

# line = L((2, 4), (6, 1))
# line = L((2, 2), (6, 2))
# line = L((0, 4.19), (10, 4.19))
# line = L((4, 0), (4, 4))
# line = L((2, 3.2), (0, 3.2))
# line = L((3, 3.3), (0, 3.2))
# line = L((3, 3.2), (0, 2))
# line = L((4, 3.7), (4, 6))
line = L((6, 6), (0.5, 0))
# line = L((5, 2), (8, 2))
# line = L((0, 2), (1.8000001, 2))
point_b = P(6, 4)

# print(hit_coord_point_to_line(point_a, line))

move = Move(1000, line, 0.2)
obstacle = Obstacle([P(2, 1), P(2, 3), P(6, 4), P(4, 2), P(6, 0), P(4, 1)])
# t, closest_vertex_or_edge = first_intersection(move, obstacle)
# print((t, closest_vertex_or_edge))
# if isinstance(closest_vertex_or_edge, Line):
#     print(f"at: {line_intersection(move.line, closest_vertex_or_edge)}")
#
# print(does_intersect(move, obstacle))

points_to_check = [
    (P(1, 2),    "outside"),
    (P(3, 2),    "simple inside"),
    (P(2, 0.5),  "outside below vertical edge"),
    (P(4, 0),    "outside below vertex with edges left and right"),
    (P(4, 1.5),  "inside below vertex with edges only right"),
    (P(5, 2),    "outside above and below polygon"),
    (P(5, 1),    "on an edge"),
    (P(4, 3.6),  "too close to an edge"),
    (P(1.81, 1), "too close to a vertex"),
]

for point, msg in points_to_check:
    print("------------------------------------------------------------------")
    print(is_point_in_obstacle(point, obstacle, 0.2))
    print(msg)

# print(is_point_in_obstacle(P(5, 2), obstacle, 0.2))

# angle to side of obstacle
# Direct routes and single steps
