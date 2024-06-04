from MathHelper import distance_point_to_point, line_intersection, distance_point_to_line
from Types import Point, Line


class Obstacle:
    # Give the vertices in a clockwise fashion
    def __init__(self, vertices: list[Point], clearance: float):
        if len(vertices) < 3:
            raise Exception("Obstacle must have at least 3 vertices")

        self.vertices = vertices
        all_x = [p.x for p in vertices]
        all_y = [p.y for p in vertices]
        self.x_min = min(all_x)
        self.x_max = max(all_x)
        self.y_min = min(all_y)
        self.y_max = max(all_y)

        vertices_shifted = [vertices[-1]] + vertices[:-1:]
        self.edges: list[Line] = [Line(c1, vertices_shifted[index]) for index, c1 in enumerate(vertices)]

        self.edges_dict = {v: [] for v in vertices}
        for edge in self.edges:
            self.edges_dict[edge.p1] += [edge.p2]
            self.edges_dict[edge.p2] += [edge.p1]

        # To consistently get {a: [b, c]} as Line(a, b) and Line(c, a)
        self.edges_dict[vertices[-1]].reverse()

        self.slopes_dict = {e: float('inf') for e in self.edges}
        for edge in self.edges:
            try:
                self.slopes_dict[edge] = (
                        (edge.y2 - edge.y1) / (edge.x2 - edge.x1))
            except ZeroDivisionError:
                self.slopes_dict[edge] = float('inf')

        self.slopes_dict_inv: dict[Line, float] = \
            {e: -1/s if s != 0 else float('inf') for e, s in self.slopes_dict.items()}

        self.outside_lines_dict = {e: Line(0, 0, 0, 0) for e in self.edges}
        for edge in self.outside_lines_dict:
            if edge.x1 == edge.x2:
                if edge.y1 > edge.y2:
                    x = edge.x1 - clearance
                else:
                    x = edge.x1 + clearance
                self.outside_lines_dict[edge] = Line(x, 0, x, 1)
                continue
            if edge.y1 == edge.y2:
                if edge.x1 > edge.x2:
                    y = edge.y1 + clearance
                else:
                    y = edge.y1 - clearance
                self.outside_lines_dict[edge] = Line(0, y, 1, y)
                continue

            if edge.y1 > edge.y2:
                outside_p_far = Point(edge.x1 - 1, edge.y1 - self.slopes_dict_inv[edge])
                distance_relative = clearance / distance_point_to_point(outside_p_far, edge.p1)
                outside_p1 = Point(
                    edge.x1 - distance_relative,
                    edge.y1 - distance_relative * self.slopes_dict_inv[edge]
                )
            else:
                outside_p_far = Point(edge.x1 + 1, edge.y1 + self.slopes_dict_inv[edge])
                distance_relative = clearance / distance_point_to_point(outside_p_far, edge.p1)
                outside_p1 = Point(
                    edge.x1 + distance_relative,
                    edge.y1 + distance_relative * self.slopes_dict_inv[edge]
                )

            outside_p2 = Point(outside_p1.x + 1, outside_p1.y + self.slopes_dict[edge])
            self.outside_lines_dict[edge] = Line(outside_p1, outside_p2)

        self.outside_points_dict = {v: Point(0, 0) for v in vertices}
        for vertex in self.vertices:
            v1, v2 = self.edges_dict[vertex]
            l1 = Line(vertex, v1)
            l2 = Line(v2, vertex)
            ol1 = self.outside_lines_dict[l1]
            ol2 = self.outside_lines_dict[l2]
            self.outside_points_dict[vertex] = line_intersection(ol1, ol2)
