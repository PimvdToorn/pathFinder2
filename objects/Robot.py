from Types import Point, Line
from objects.Move import Move, get_wait_after_move


class Robot:
    heading = 0.0
    path: list[Move] = []

    def __init__(self, name: str, address: int, radius: float, x=0.0, y=0.0) -> None:
        self.name = name
        self.address = address
        self.radius = radius
        self.x = x
        self.y = y

    def get_move(self, time: int) -> Move:
        for move in self.path:
            if move.end_time > time:
                return move

    def get_location(self, time: int) -> Point:
        if not self.path or self.path[0].start_time > time:
            return Point(self.x, self.y)

        for move in self.path:
            if move.end_time > time:
                return move.location_at_time(time)

        return self.path[-1].end

    def create_move(self, line: Line, start_time: float = None) -> Move:
        if start_time is not None:
            if self.path:
                if start_time > self.path[-1].end_time:
                    self.path.append(get_wait_after_move(self.path[-1], start_time))
                start_time = max(start_time, self.path[-1].end_time)

        move = Move(line, self.radius, start_time)
        return move
