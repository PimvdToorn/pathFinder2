from Types import Line


class Move:
    def __init__(self, end_time: int, line: Line, clearance: float):
        self.end_time = end_time
        self.x1 = line.x1
        self.y1 = line.y1
        self.x2 = line.x2
        self.y2 = line.y2
        self.line = line

        self.clearance = clearance
        self.x_min = min(self.x1, self.x2) - clearance
        self.y_min = min(self.y1, self.y2) - clearance
        self.x_max = max(self.x1, self.x2) + clearance
        self.y_max = max(self.y1, self.y2) + clearance

    def __repr__(self) -> str:
        return f"Move({self.end_time}, {self.start}, {self.end})"

    @property
    def start(self):
        return self.line.p1

    @property
    def end(self):
        return self.line.p2
