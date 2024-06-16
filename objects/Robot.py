from MathHelper import get_heading
from Types import Point, Line, P
from objects.Move import Move


class Robot:
    def __init__(self, name: str, address: str, radius: float, location: Point = P(0, 0)) -> None:
        self.name = name
        self.address = address
        self.radius = radius
        self.location = location
        self.destination = location
        self.heading = 0.0
        self.path: list[Move] = []

    def get_move(self, time: int) -> Move | None:
        if not self.path or self.path[0].start_time > time or self.path[-1].end_time < time:
            return None

        for move in self.path:
            if move.end_time > time:
                return move

    def get_next_move(self, time: int) -> Move:
        for move in self.path:
            if move.start_time > time:
                return move

    def get_expected_location(self, time: int) -> Point:
        if not self.path:
            return self.location

        if self.path[-1].end_time < time:
            return self.path[-1].end

        move = self.get_move(time)
        if move:
            return move.location_at_time(time)
        return self.location

    def get_expected_heading(self, time: int) -> float:
        if not self.path:
            return self.heading

        for i, move in enumerate(self.path):
            if move.end_time > time:
                if move.waiting:
                    return get_heading(self.path[i+1].line)
                return get_heading(move.line)

        return get_heading(self.path[-1].line)

    def create_move(self, destination: Point, start_time: int = 0) -> Move:
        start_time = max(start_time, 0)
        self.destination = destination

        return Move(Line(self.location, destination), self.radius, start_time)
