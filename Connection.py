from Types import Point


class Connection:
    def __init__(self, name: str) -> None:
        self.name = name
        self.address = ""


def send_message(receiver: str, message: str):
    print(f"To {receiver}: {message}")


def receive_robot_update() -> list[tuple[Point, float]]:
    location = Point(0, 0)
    heading = 0.0
    return [(location, heading)]

