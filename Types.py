from math import sqrt


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

    def __str__(self):
        return f"P({self.x:.2f}, {self.y:.2f})"

    def bare_str(self):
        return f"({self.x}, {self.y})"


class Line:
    _len = None

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

        self.x_min = min(self.x1, self.x2)
        self.y_min = min(self.y1, self.y2)
        self.x_max = max(self.x1, self.x2)
        self.y_max = max(self.y1, self.y2)

    def __eq__(self, other):
        if isinstance(other, Line):
            return self.p1 == other.p1 and self.p2 == other.p2

    def __hash__(self):
        return hash((self.p1, self.p2))

    def __repr__(self):
        return f"L({self.p1.__repr__()}, {self.p2.__repr__()})"

    def __str__(self):
        return f"L({self.p1}, {self.p2})"

    @property
    def len(self) -> float:
        if self._len is not None:
            return self._len
        self._len = sqrt((self.x2 - self.x1) ** 2 + (self.y2 - self.y1) ** 2)
        return self._len

    @property
    def slope(self) -> float:
        try:
            return (self.y2 - self.y1) / (self.x2 - self.x1)
        except ZeroDivisionError:
            return float('inf')


P = Point
L = Line
