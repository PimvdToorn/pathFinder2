from MathHelper import line_intersection, point_to_line_t, offset_line
from Types import Point, Line, P


class ClearancePoints:
    def __init__(self, clearance: float, connected_vertices: dict[Point, list[Point]], edges: list[Line]):
        self.clearance = clearance + 0.0001

        self.outside_lines_dict = {e: Line(0, 0, 0, 0) for e in edges}
        for edge in self.outside_lines_dict:
            self.outside_lines_dict[edge] = offset_line(edge, self.clearance)

        self.outside_points_dict = {v: Point(0, 0) for v in connected_vertices.keys()}
        for vertex in connected_vertices.keys():
            v1, v2 = connected_vertices[vertex]
            l1 = Line(vertex, v1)
            l2 = Line(v2, vertex)
            ol1 = self.outside_lines_dict[l1]
            ol2 = self.outside_lines_dict[l2]

            if l1.slope == l2.slope:
                ol2 = Line(vertex, P(vertex.x + 1, vertex.y + l1.slope_inv))

            self.outside_points_dict[vertex] = line_intersection(ol1, ol2)

        self.outside_to_outside_points = {
            self.outside_points_dict[v]: [
                self.outside_points_dict[connected_vertices[v][0]],
                self.outside_points_dict[connected_vertices[v][1]]
            ]
            for v in connected_vertices.keys()
        }

    def update_outside_to_outside(self, connected_vertices: dict[Point, list[Point]]):
        self.outside_to_outside_points = {
            self.outside_points_dict[v]: [
                self.outside_points_dict[connected_vertices[v][0]],
                self.outside_points_dict[connected_vertices[v][1]]
            ]
            for v in connected_vertices.keys()
        }


class Obstacle:
    # Give the vertices in a clockwise fashion
    def __init__(self, vertices: list[Point]):
        if len(vertices) < 3:
            raise Exception("Obstacle must have at least 3 vertices")

        self.clearance_points: dict[float, ClearancePoints] = {}

        self.vertices = vertices
        all_x = [p.x for p in vertices]
        all_y = [p.y for p in vertices]
        self.x_min = min(all_x)
        self.x_max = max(all_x)
        self.y_min = min(all_y)
        self.y_max = max(all_y)

        vertices_shifted = [vertices[-1]] + vertices[:-1:]
        self.edges: list[Line] = [Line(c1, vertices_shifted[index]) for index, c1 in enumerate(vertices)]

        self.connected_vertices = {v: [] for v in vertices}
        for edge in self.edges:
            self.connected_vertices[edge.p1] += [edge.p2]
            self.connected_vertices[edge.p2] += [edge.p1]

        # To consistently get {a: [b, c]} as Line(a, b) and Line(c, a)
        # b is counterclockwise, c clockwise
        self.connected_vertices[vertices[-1]].reverse()

        self.is_acute = {
            v: point_to_line_t(
                self.connected_vertices[v][0],
                Line(self.connected_vertices[v][1], v)
            ) < 1
            for v in self.vertices
        }

    def __getitem__(self, item: float) -> ClearancePoints:
        if item not in self.clearance_points:
            self.clearance_points[item] = ClearancePoints(item, self.connected_vertices, self.edges)

        return self.clearance_points[item]


def get_box(center: Point, x_width: float, y_width: float) -> Obstacle:
    return Obstacle([
        P(center.x - x_width / 2, center.y - y_width / 2),
        P(center.x - x_width / 2, center.y + y_width / 2),
        P(center.x + x_width / 2, center.y + y_width / 2),
        P(center.x + x_width / 2, center.y - y_width / 2),
    ])
