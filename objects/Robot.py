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

    def get_location(self, time: int) -> tuple:
        if not self.path:
            return self.x, self.y

        move_start_time = 0
        for move in self.path:
            if move.end_time > time:
                # completion of 0.6 means this move is 60% done
                path_completion_level = (time-move_start_time)/(move.end_time-move_start_time)
                self.x = move.x2 * path_completion_level + move.x1 * (1-path_completion_level)
                self.y = move.y2 * path_completion_level + move.y1 * (1-path_completion_level)
                return self.x, self.y

            move_start_time = move.end_time

        return self.path[-1].end
