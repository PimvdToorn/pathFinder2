class Point:
    def __init__(self, *args):
        if args[0] is tuple:
            self.x = args[0][0]
            self.y = args[0][1]
        else:
            self.x = args[0]
            self.y = args[1]

    def __eq__(self, other):
        if isinstance(other, Point):
            return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def __repr__(self):
        return f"P({self.x}, {self.y})"


class Line:
    def __init__(self, *args):
        if isinstance(args[0], Point):
            self.x1 = args[0].x
            self.y1 = args[0].y
            self.x2 = args[1].x
            self.y2 = args[1].y
        elif isinstance(args[0], tuple):
            self.x1 = args[0][0]
            self.y1 = args[0][1]
            self.x2 = args[1][0]
            self.y2 = args[1][1]
        else:
            self.x1 = args[0]
            self.y1 = args[1]
            self.x2 = args[2]
            self.y2 = args[3]

        self.p1 = Point(self.x1, self.y1)
        self.p2 = Point(self.x2, self.y2)

    def __eq__(self, other):
        if isinstance(other, Line):
            return self.p1 == other.p1 and self.p2 == other.p2

    def __repr__(self):
        return f"L({self.p1}, {self.p2})"


P = Point
L = Line