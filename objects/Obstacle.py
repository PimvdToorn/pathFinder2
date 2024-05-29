from Types import Point, Line


class Obstacle:
    def __init__(self, corners: list[Point]):
        if len(corners) < 3:
            raise Exception("Obstacle must have at least 3 corners")

        self.corners = corners
        all_x = [p.x for p in corners]
        all_y = [p.y for p in corners]
        self.x_min = min(all_x)
        self.x_max = max(all_x)
        self.y_min = min(all_y)
        self.y_max = max(all_y)

        corners_shifted = [corners[-1]] + corners[:-1:]
        self.vertices: list[Line] = [Line(c1, corners_shifted[index]) for index, c1 in enumerate(corners)]
        self.vertices_dict = {c: [] for c in corners}

        for vertex in self.vertices:
            self.vertices_dict[vertex.p1] += [vertex.p2]
            self.vertices_dict[vertex.p2] += [vertex.p1]
