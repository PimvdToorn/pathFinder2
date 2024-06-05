from math import ceil

from Types import Line


class Move:
    def __init__(self, line: Line, clearance: float, start_time=0, end_time=0):
        self.x1 = line.x1
        self.y1 = line.y1
        self.x2 = line.x2
        self.y2 = line.y2
        self.line = line
        self.start_time = start_time
        if end_time == 0:
            self.end_time = start_time + ceil(self.line.len*1000)
        else:
            self.end_time = end_time

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


# Used to copy into https://www.Desmos.com/calculator
def steps_str(moves: list[Move]) -> str:
    if not moves:
        return ''
    string = f"{moves[0].start.bare_str()}"
    for move in moves:
        string += ',' + move.end.bare_str()
    return string
