from Types import Point, Line


class Obstacle:
    # Give the vertices in a clockwise fashion
    def __init__(self, vertices: list[Point]):
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
        self.edges_dict = {c: [] for c in vertices}

        for edge in self.edges:
            self.edges_dict[edge.p1] += [edge.p2]
            self.edges_dict[edge.p2] += [edge.p1]

        # To consistently get {a: [b, c]} as Line(a, b) and Line(c, a)
        self.edges_dict[vertices[-1]].reverse()
