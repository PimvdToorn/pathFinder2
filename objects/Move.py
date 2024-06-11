from MathHelper import line_intersection_t_u, distance_point_to_point
from Types import Line, Point


class Move:
    def __init__(self, line: Line, clearance: float, start_time: float = 0.0, speed: float = 0.001):
        self.x1 = line.x1
        self.y1 = line.y1
        self.x2 = line.x2
        self.y2 = line.y2
        self.line = line
        self.start_time = start_time
        self.speed = speed
        self.end_time = start_time + self.line.len/speed

        self.clearance = clearance
        self.x_min = min(self.x1, self.x2) - clearance
        self.y_min = min(self.y1, self.y2) - clearance
        self.x_max = max(self.x1, self.x2) + clearance
        self.y_max = max(self.y1, self.y2) + clearance

        try:
            self.slope = (self.y2 - self.y1) / (self.x2 - self.x1)
        except ZeroDivisionError:
            self.slope = float('inf')

        try:
            self.slope_inv = -1 / self.slope
        except ZeroDivisionError:
            self.slope_inv = float('inf')

    def __repr__(self) -> str:
        return f"Move({self.end_time}, {self.start}, {self.end})"

    @property
    def start(self):
        return self.line.p1

    @property
    def end(self):
        return self.line.p2

    def location_at_time(self, time: float) -> Point:
        if self.end_time < time:
            return Point(float('inf'), float('inf'))
        if self.start_time > time:
            return Point(float('inf'), float('inf'))

        path_completion_level = (time - self.start_time) / (self.end_time - self.start_time)
        return self.start + (self.end - self.start) * path_completion_level


# Used to copy into https://www.Desmos.com/calculator
def steps_str(moves: list[Move]) -> str:
    if not moves:
        return ''
    string = f"{moves[0].start.bare_str()}"
    for move in moves:
        string += ',' + move.end.bare_str()
    return string


def min_distance(m1: Move, m2: Move) -> tuple[float, float]:
    t, u = line_intersection_t_u(m1.line, m2.line)
    time1 = t * m1.line.len / m1.speed + m1.start_time
    time2 = u * m2.line.len / m2.speed + m2.start_time
    t_avg = (time1 * m1.speed + time2 * m2.speed) / (m1.speed + m2.speed)
    p1 = m1.location_at_time(t_avg)
    p2 = m2.location_at_time(t_avg)
    return distance_point_to_point(p1, p2), t_avg
