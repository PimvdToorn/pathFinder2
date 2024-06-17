from math import pi

from MathHelper import get_heading, distance, distance_point_to_line, distance2
from PathFinder import set_path
from Types import Point, Line
from objects.Field import Field
from objects.Move import path_str
from objects.Robot import Robot

MAX_DEVIATION = 0.1
MAX_PERPENDICULAR_DEVIATION = 0.1
DESTINATION_DEADZONE = 0.1
WAITING_DEADZONE = 0.02
WAITING_SLOWDOWN = 0.1
SPEED = 0.7
SPEED_DEVIATION = 0.3


def update_speed_l_and_r(r: Robot, time: int, field: Field) \
        -> tuple[float, float]:
    expected_location = r.get_expected_location(time)
    distance_from_expected = distance(expected_location, r.location)
    # print(f"{r.name} location: {r.location.bare_str()}, Expected location: {expected_location.bare_str()}, "
    #       f"Distance from expected: {distance_from_expected}")

    if distance_from_expected > MAX_DEVIATION:
        # print(f"{r.name} has deviated too far from its path. Recalculating path.")
        set_path(r.destination, r, field, time)
        # print(f"New path: {path_str(r.path)}")
        expected_location = r.get_expected_location(time)
        distance_from_expected = distance(expected_location, r.location)

    # todo wen destination
    move = r.get_move(time)
    expected_heading = r.get_expected_heading(time)
    if move is None:
        if r.destination is not None:
            if distance2(r.location, r.destination) < DESTINATION_DEADZONE:
                r.destination = None
                return 0.0, 0.0
        # robot.destination = None
        turning_speed = expected_heading - r.heading
        if turning_speed > pi:
            turning_speed -= 2*pi
        elif turning_speed < -pi:
            turning_speed += 2*pi
        turning_speed /= pi
        return turning_speed, -turning_speed

    if move.waiting:
        if distance_from_expected < WAITING_DEADZONE:
            return 0.0, expected_heading
        return min(1.0, distance_from_expected/WAITING_SLOWDOWN), get_heading(Line(r.location, move.end))

    perpendicular_deviation = distance_point_to_line(r.location, move.line)
    heading_offset = perpendicular_deviation/MAX_PERPENDICULAR_DEVIATION * 0.25*pi
    new_heading = (expected_heading + heading_offset) % (2*pi)
    # print(f"Expected heading: {expected_heading/pi*180:.1f}, Heading offset: {heading_offset/pi*180:.1f}, New heading: {new_heading/pi*180:.1f}, current heading: {r.heading/pi*180:.1f}")
    turning_speed = new_heading - r.heading
    if turning_speed > pi:
        turning_speed -= 2*pi
    elif turning_speed < -pi:
        turning_speed += 2*pi
    turning_speed /= pi
    # print(f"Turning speed: {turning_speed:.2f}")

    dest_distance = distance(r.location, move.end)
    dest_distance_diff = dest_distance - distance(expected_location, move.end)

    speed_deviation = SPEED_DEVIATION * (dest_distance_diff / MAX_DEVIATION)
    speed = SPEED + speed_deviation
    rotation_deviation = turning_speed * speed

    # print(f"SPEED: {speed}, ROTATION: {rotation_deviation}")
    return (min(1.0, max(-1.0, speed + rotation_deviation)),
            min(1.0, max(-1.0, speed - rotation_deviation)))
