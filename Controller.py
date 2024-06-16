from MathHelper import get_heading, distance, distance_point_to_line, distance2
from PathFinder import set_path
from Types import Point, Line
from objects.Field import Field
from objects.Move import path_str
from objects.Robot import Robot

MAX_DEVIATION = 0.1
MAX_PERPENDICULAR_DEVIATION = 0.05
WAITING_DEADZONE = 0.02
WAITING_SLOWDOWN = 0.1
SPEED = 0.8
SPEED_DEVIATION = 0.2


def update_speed_l_and_r(robot: Robot, time: int, location: Point, heading: float, field: Field) \
        -> tuple[float, float]:
    robot.location = location
    robot.heading = heading
    expected_location = robot.get_expected_location(time)
    distance_from_expected = distance(expected_location, location)
    print(f"Location: {location}, Expected location: {robot.get_expected_location(time)}, "
          f"Distance from expected: {distance_from_expected}")

    if distance_from_expected > MAX_DEVIATION:
        set_path(robot.destination, robot, field, time)
        print(f"Robot {robot.name} has deviated too far from its path. Recalculating path.")
        print(f"New path: {path_str(robot.path)}")
        expected_location = robot.get_expected_location(time)
        distance_from_expected = distance(expected_location, location)

    move = robot.get_move(time)
    expected_heading = robot.get_expected_heading(time)
    if move is None:
        turning_speed = expected_heading - heading
        if turning_speed > 180:
            turning_speed -= 360
        elif turning_speed < -180:
            turning_speed += 360
        turning_speed /= 180
        return turning_speed, -turning_speed

    if move.waiting:
        if distance_from_expected < WAITING_DEADZONE:
            return 0.0, expected_heading
        return min(1.0, distance_from_expected/WAITING_SLOWDOWN), get_heading(Line(location, move.end))

    perpendicular_deviation = distance_point_to_line(location, move.line)
    heading_offset = perpendicular_deviation/MAX_PERPENDICULAR_DEVIATION * 90
    new_heading = (expected_heading + heading_offset) % 360
    print(f"Expected heading: {expected_heading}, Heading offset: {heading_offset}, New heading: {new_heading}, current heading: {heading}")
    turning_speed = new_heading - heading
    if turning_speed > 180:
        turning_speed -= 360
    elif turning_speed < -180:
        turning_speed += 360
    turning_speed /= 90
    rotation_deviation = turning_speed * SPEED

    dest_distance = distance(location, move.end)
    dest_distance_diff = dest_distance - distance(expected_location, move.end)
    speed_deviation = SPEED_DEVIATION * (dest_distance_diff / MAX_DEVIATION)
    speed = SPEED + speed_deviation

    print(f"SPEED: {speed}, ROTATION: {rotation_deviation}")
    return (min(1.0, max(0.0, speed + rotation_deviation)),
            min(1.0, max(0.0, speed - rotation_deviation)))
