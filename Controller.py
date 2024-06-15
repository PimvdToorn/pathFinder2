from MathHelper import get_heading, distance, distance_point_to_line
from PathFinder import set_path
from Types import Point, Line
from objects.Field import Field
from objects.Move import path_str
from objects.Robot import Robot

MAX_DEVIATION = 0.05
MAX_PERPENDICULAR_DEVIATION = 0.05
WAITING_DEADZONE = 0.02


def update_speed_and_heading(robot: Robot, time: int, location: Point, heading: float, field: Field) \
        -> tuple[float, float]:
    robot.location = location
    robot.heading = heading
    distance_from_expected = distance(robot.get_expected_location(time), location)
    print(f"Location: {location}, Expected location: {robot.get_expected_location(time)}, "
          f"Distance from expected: {distance_from_expected}")

    if distance_from_expected > MAX_DEVIATION:
        set_path(robot.destination, robot, field, time)
        print(f"Robot {robot.name} has deviated too far from its path. Recalculating path.")
        print(f"New path: {path_str(robot.path)}")

    move = robot.get_move(time)
    expected_heading = robot.get_expected_heading(time)
    if move is None:
        return 0.0, expected_heading

    if move.waiting:
        if distance_from_expected < WAITING_DEADZONE:
            return 0.0, expected_heading
        return 1.0, get_heading(Line(location, move.end))

    perpendicular_deviation = distance_point_to_line(location, move.line)
    heading_offset = perpendicular_deviation/MAX_PERPENDICULAR_DEVIATION * 90
    new_heading = (expected_heading + heading_offset) % 360
    print(f"Expected heading: {expected_heading}, Heading offset: {heading_offset}, New heading: {new_heading}")
    return 1.0, new_heading
