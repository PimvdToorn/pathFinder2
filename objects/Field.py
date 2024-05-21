from objects.Robot import Robot


class Field:
    robots: list[Robot] = []

    def __init__(self, size: tuple[float, float]) -> None:
        self.size = size

    def add(self, robot: Robot) -> None:
        self.robots.append(robot)
