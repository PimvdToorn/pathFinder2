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

    def __add__(self, other):
        return Point(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Point(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        return Point(self.x * other, self.y * other)

    def __truediv__(self, other):
        if isinstance(other, Point):
            return Point(self.x / other.x, self.y / other.y)
        if isinstance(other, (int, float)):
            return Point(self.x / other, self.y / other)

    def __hash__(self):
        return hash((self.x, self.y))

    def __repr__(self):
        return f"P({self.x:.6f}, {self.y:.6f})"

    def __str__(self):
        return f"P({self.x:.3f}, {self.y:.3f})"

    def bare_str(self):
        return f"({self.x:.3f}, {self.y:.3f})"


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

        self.x_min = min(self.x1, self.x2)
        self.y_min = min(self.y1, self.y2)
        self.x_max = max(self.x1, self.x2)
        self.y_max = max(self.y1, self.y2)

        self._len = None
        self._slope = None
        self._slope_inv = None

    def __eq__(self, other):
        if isinstance(other, Line):
            return self.p1 == other.p1 and self.p2 == other.p2

    def __hash__(self):
        return hash((self.p1, self.p2))

    def __repr__(self):
        return f"L({self.p1.bare_str()}, {self.p2.bare_str()})"

    def __str__(self):
        return f"L({self.p1}, {self.p2})"

    def __add__(self, other: Point):
        return Line(self.p1 + other, self.p2 + other)

    @property
    def len(self) -> float:
        if self._len is None:
            self._len = sqrt((self.x2 - self.x1) ** 2 + (self.y2 - self.y1) ** 2)
        return self._len

    @property
    def slope(self) -> float:
        if self._slope is None:
            try:
                self._slope = (self.y2 - self.y1) / (self.x2 - self.x1)
            except ZeroDivisionError:
                self._slope = float('inf')
        return self._slope

    @property
    def slope_inv(self) -> float:
        if self._slope_inv is None:
            try:
                self._slope_inv = -1 / self.slope
            except ZeroDivisionError:
                self._slope_inv = float('inf')
        return self._slope_inv

    def location_at_t(self, t: float) -> Point:
        return self.p1 + (self.p2 - self.p1) * t


P = Point
L = Line
