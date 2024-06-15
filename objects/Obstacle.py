from MathHelper import line_intersection, point_to_line_t, offset_line
from Types import Point, Line, P


class Obstacle:
    # Give the vertices in a clockwise fashion
    def __init__(self, vertices: list[Point], clearance: float):
        if len(vertices) < 3:
            raise Exception("Obstacle must have at least 3 vertices")

        # For floating point errors
        self.clearance = clearance + 0.0001

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

        outside_lines_dict = {e: Line(0, 0, 0, 0) for e in self.edges}
        for edge in outside_lines_dict:
            outside_lines_dict[edge] = offset_line(edge, self.clearance)

        self.outside_points_dict = {v: Point(0, 0) for v in vertices}
        for vertex in self.vertices:
            v1, v2 = self.connected_vertices[vertex]
            l1 = Line(vertex, v1)
            l2 = Line(v2, vertex)
            ol1 = outside_lines_dict[l1]
            ol2 = outside_lines_dict[l2]

            if l1.slope == l2.slope:
                ol2 = Line(vertex, P(vertex.x + 1, vertex.y + l1.slope_inv))

            self.outside_points_dict[vertex] = line_intersection(ol1, ol2)

        # for op in self.outside_points_dict.values():
        #     print(f"{op.bare_str()},", end="")
        # print()

        self.outside_to_outside_points = {
            self.outside_points_dict[v]: [
                self.outside_points_dict[self.connected_vertices[v][0]],
                self.outside_points_dict[self.connected_vertices[v][1]]
            ]
            for v in vertices
        }

        self.is_acute = {
            v: point_to_line_t(
                self.connected_vertices[v][0],
                Line(self.connected_vertices[v][1], v)
            ) < 1
            for v in self.vertices
        }

        # for v, a in self.is_acute.items():
        #     print("-----------------------------------")
        #     print(f"intersection: {hit_coords_point_to_line(self.connected_vertices[v][0], Line(self.connected_vertices[v][1], v))}")
        #     print(f"t: {point_to_line_t(
        #         self.connected_vertices[v][0],
        #         Line(self.connected_vertices[v][1], v)
        #     )}")
        #     print(f"Connected vertices: {self.connected_vertices[v]}")
        #     print(f"{v}: {a}")
