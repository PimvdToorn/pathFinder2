import time

from objects.Robot import Robot
from objects.Obstacle import Obstacle


class Field:
    robots: list[Robot] = []
    obstacles: list[Obstacle] = []

    def __init__(self, size: tuple[float, float]) -> None:
        self.size = size

    def add(self, robot: Robot) -> None:
        self.robots.append(robot)

    # def pathFind(self, robot: Robot, destination: tuple[int, int]):


def millis() -> int:
    return int(time.time_ns() / 1000)

