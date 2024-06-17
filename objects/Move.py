from math import ceil

from MathHelper import line_intersection_t_u, distance, point_to_line_t
from Types import Line, Point


class Move:
    def __init__(self, line: Line, clearance: float, start_time: int = 0, end_time: int = 0):  # , speed: float = 0.001):
        self.waiting = line.len == 0
        self.x1 = line.x1
        self.y1 = line.y1
        self.x2 = line.x2
        self.y2 = line.y2
        self.line = line
        self.start_time = start_time
        # self.speed = speed
        self.speed = 0.000_000_00025
        if line.len == 0:
            self.speed = 0.0
        if end_time == 0:
            if self.speed == 0.0:
                self.end_time = start_time
            else:
                self.end_time = start_time + ceil(self.line.len/self.speed)
        else:
            self.end_time = end_time

        self.clearance = clearance
        self.x_min = min(self.x1, self.x2) - clearance
        self.y_min = min(self.y1, self.y2) - clearance
        self.x_max = max(self.x1, self.x2) + clearance
        self.y_max = max(self.y1, self.y2) + clearance

    def __repr__(self) -> str:
        return f"Move({self.start_time}, {self.end_time}, {self.start.__repr__()}, {self.end.__repr__()})"

    def __str__(self) -> str:
        return f"Move({self.start_time/1000000000:.1f}-{self.end_time/1000000000:.1f}s, {self.start}, {self.end})"

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


def get_wait_move(at: Point, clearance: float, start_time: int, wait_time: int) -> Move:
    return Move(Line(at, at), clearance, start_time, start_time + wait_time)


def get_wait_after_move(move: Move, until_time: int) -> Move:
    return Move(Line(move.end, move.end), move.clearance, move.end_time, until_time)


# Used to copy into https://www.Desmos.com/calculator
def steps_str(moves: list[Move]) -> str:
    if not moves:
        return ''
    string = f"{moves[0].start.bare_str()}"
    for move in moves:
        string += ',' + move.end.bare_str()
    return string


def path_str(moves: list[Move]) -> str:
    string = '['
    for move in moves:
        string += move.__str__() + ', '
    return string + ']'


def path_in_paths(path: list[Move], paths: list[list[Move]]) -> bool:
    for p in paths:
        if len(p) != len(path):
            continue
        same = True
        for i, move in enumerate(path):
            if move.start != p[i].start or move.end != p[i].end:
                same = False
                break
        if same:
            return True
    return False


def remove_duplicates(paths: list[list[Move]]) -> list[list[Move]]:
    unique = []
    for path in paths:
        if not path_in_paths(path, unique):
            unique.append(path)
    return unique


def min_distance(m1: Move, m2: Move) -> tuple[float, float]:
    if m1.waiting or m2.waiting:
        if m1.waiting and m2.waiting:
            return distance(m1.start, m2.start), m1.start_time
        waiting = m1 if m1.waiting else m2
        moving = m1 if not m1.waiting else m2
        t = point_to_line_t(waiting.start, moving.line)
        t = max(0.0, min(1.0, t))
        return (distance(waiting.start, moving.line.location_at_t(t)),
                moving.start_time + t * moving.line.len / moving.speed)

    t, u = line_intersection_t_u(m1.line, m2.line)
    time1 = t * m1.line.len / m1.speed + m1.start_time
    time2 = u * m2.line.len / m2.speed + m2.start_time
    t_avg = (time1 * m1.speed + time2 * m2.speed) / (m1.speed + m2.speed)

    # If the closest point is before or after the move, check the distance at the start or end
    t_avg = max(float(m1.start_time), float(m2.start_time), t_avg)
    t_avg = min(float(m1.end_time), float(m2.end_time), t_avg)

    p1 = m1.location_at_time(t_avg)
    p2 = m2.location_at_time(t_avg)
    return distance(p1, p2), t_avg
