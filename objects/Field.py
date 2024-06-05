import time

from objects.Robot import Robot
from objects.Obstacle import Obstacle


class Field:
    robots: list[Robot] = []
    obstacles: list[Obstacle] = []

    def __init__(self, size: tuple[float, float]) -> None:
        self.size = size

    def add_robot(self, robot: Robot) -> None:
        self.robots.append(robot)

    def add_obstacle(self, obstacle: Obstacle) -> None:
        self.obstacles.append(obstacle)


def millis() -> int:
    return int(time.time_ns() / 1000)

