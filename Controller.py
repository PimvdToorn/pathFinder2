from math import pi

from MathHelper import get_heading, distance, distance_point_to_line, distance2
from BestPaths import set_path
from Types import Point, Line
from objects.Field import Field
from objects.Move import path_str, Move
from objects.Robot import Robot

MAX_DEVIATION = 0.1
MAX_PERPENDICULAR_DEVIATION = 0.1
DESTINATION_DEADZONE = 0.01
WAITING_DEADZONE = 0.02
COMBINED_DEADZONE = 0.05

SPEED = 0.7
SPEED_DEVIATION = 0.3
SPEED_LIMITER = 1.0


def update_speed_l_and_r(r: Robot, time: int, field: Field) -> tuple[float, float]:
    if r.destination is not None and not r.path:
        set_path(r.destination, r, field, time)

    expected_location = r.get_expected_location(time)
    distance_from_expected = distance(expected_location, r.location)
    # print(f"{r.name} location: {r.location.bare_str()}, Expected location: {expected_location.bare_str()}, "
    #       f"Distance from expected: {distance_from_expected}")

    if distance_from_expected > MAX_DEVIATION and r.destination is not None:
        print(f"{r.name} has deviated too far from its path. Recalculating path.")
        set_path(r.destination, r, field, time)
        print(f"New path: {path_str(r.path)}")
        expected_location = r.get_expected_location(time)
        distance_from_expected = distance(expected_location, r.location)

    move = r.get_move(time)
    expected_heading = r.get_expected_heading(time)
    turning_speed = expected_heading - r.heading
    if turning_speed > pi:
        turning_speed -= 2 * pi
    elif turning_speed < -pi:
        turning_speed += 2 * pi
    turning_speed /= pi

    if move is None:
        if r.destination is not None:
            if distance(r.location, r.destination) < DESTINATION_DEADZONE:
                r.destination = None
                r.path = []
                return 0.0, 0.0
            else:
                move = Move(Line(r.location, r.destination), r.radius, time)
        else:
            return turning_speed, -turning_speed

    if move.waiting:
        if distance_from_expected < WAITING_DEADZONE:
            return 0.0, 0.0
        new_heading = get_heading(Line(r.location, move.end))
    else:
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
    speed = (SPEED + speed_deviation) * min(1.0, max(0.1, dest_distance*10))
    rotation_deviation = turning_speed / min(1.0, max(0.5, dest_distance*10))  # * speed

    # print(f"SPEED: {speed}, ROTATION: {rotation_deviation}")
    return (min(1.0, max(-1.0, speed + rotation_deviation)) * SPEED_LIMITER,
            min(1.0, max(-1.0, speed - rotation_deviation)) * SPEED_LIMITER)


def get_expected_location_and_move(r: Robot, time: int, field: Field) -> tuple[Point, Move | None]:
    if not r.path:
        set_path(r.destination, r, field, time)

    expected_location = r.get_expected_location(time)
    distance_from_expected = distance(expected_location, r.location)
    # print(f"{r.name} distance from expected: {distance_from_expected}")

    if distance_from_expected > MAX_DEVIATION:
        print(f"{r.name} has deviated too far from its path. Recalculating path.")
        set_path(r.destination, r, field, time)
        print(f"New path: {path_str(r.path)}")
        expected_location = r.get_expected_location(time)
        distance_from_expected = distance(expected_location, r.location)

    move = r.get_move(time)

    if move is None and distance(r.location, r.destination) < DESTINATION_DEADZONE:
        return expected_location, None

    if move.waiting:
        if distance_from_expected < WAITING_DEADZONE:
            return expected_location, r.get_next_move(time)
        move = Move(Line(r.location, move.end), move.clearance, time, move.end_time)

    return expected_location, move


def update_combined_speed_l_and_r(r: Robot, move: Move, time: int, only_rotate=False) -> tuple[float, float]:
    expected_location = move.location_at_time(time)
    if time > move.end_time:
        expected_location = move.end
    distance_from_expected = distance(expected_location, r.location)

    dest_distance = distance(r.location, move.end)
    if only_rotate:
        # input("Only rotate")
        speed = 0.0
    else:
        dest_distance_diff = dest_distance - distance(expected_location, move.end)
        speed_deviation = SPEED_DEVIATION * (dest_distance_diff / MAX_DEVIATION)
        speed = (SPEED + speed_deviation) * min(1.0, max(0.1, dest_distance * 10))

    if move.waiting:
        if distance_from_expected < WAITING_DEADZONE:
            return 0.0, 0.0
        new_heading = get_heading(Line(r.location, move.end))
    elif only_rotate:
        new_heading = get_heading(move.line)
    else:
        expected_heading = get_heading(move.line)
        perpendicular_deviation = distance_point_to_line(r.location, move.line)
        heading_offset = perpendicular_deviation / MAX_PERPENDICULAR_DEVIATION * 0.25 * pi
        new_heading = (expected_heading + heading_offset) % (2 * pi)

    # print(f"Expected heading: {expected_heading/pi*180:.1f}, Heading offset: {heading_offset/pi*180:.1f}, New heading: {new_heading/pi*180:.1f}, current heading: {r.heading/pi*180:.1f}")
    turning_speed = new_heading - r.heading
    if turning_speed > pi:
        turning_speed -= 2 * pi
    elif turning_speed < -pi:
        turning_speed += 2 * pi
    turning_speed /= pi
    # print(f"Turning speed: {turning_speed:.2f}")

    rotation_deviation = turning_speed / min(1.0, max(0.5, dest_distance * 10))

    # print(f"SPEED: {speed}, ROTATION: {rotation_deviation}")
    return (min(1.0, max(-1.0, speed + rotation_deviation)) * SPEED_LIMITER,
            min(1.0, max(-1.0, speed - rotation_deviation)) * SPEED_LIMITER)
