from Types import Point
from objects.Move import Move


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
        if not self.path:
            return Point(self.x, self.y)

        for move in self.path:
            if move.end_time > time:
                return move.location_at_time(time)

        return self.path[-1].end
