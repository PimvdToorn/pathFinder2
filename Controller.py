from math import pi

from MathHelper import get_heading, distance, distance_point_to_line
from BestPaths import set_path
from Types import Point, Line
from objects.Field import Field
from objects.Move import path_str, Move
from objects.Robot import Robot

MAX_DEVIATION = 0.1
MAX_PERPENDICULAR_DEVIATION = 0.1
MAX_HEADING_OFFSET = 0.25 * pi
DESTINATION_DEADZONE = 0.01

SPEED = 0.7
SPEED_DEVIATION = 0.3
SPEED_LIMITER = 1.0


# Get the speed of the left and right wheel of the robot, recalculating the path if the robot has deviated too far
def update_speed_l_and_r(r: Robot, time: int, field: Field) -> tuple[float, float]:
    if r.destination is None:
        return 0.0, 0.0

    if not r.path:
        set_path(r.destination, r, field, time)
        if not r.path:
            return 0.0, 0.0

    expected_location = r.get_expected_location(time)
    distance_from_expected = distance(expected_location, r.location)
    # print(f"{r.name} location: {r.location.bare_str()}, Expected location: {expected_location.bare_str()}, "
    #       f"Distance from expected: {distance_from_expected}")

    if distance_from_expected > MAX_DEVIATION:
        print(f"{r.name} has deviated too far from its path. Recalculating path.")
        set_path(r.destination, r, field, time)
        if not r.path:
            return 0.0, 0.0
        print(f"New path: {path_str(r.path)}")
        expected_location = r.get_expected_location(time)
        distance_from_expected = distance(expected_location, r.location)

    move = r.get_move(time)
    expected_heading = r.get_expected_heading(time)

    if distance(r.location, r.destination) < DESTINATION_DEADZONE:
        r.destination = None
        r.path = []
        return 0.0, 0.0
    elif move == r.path[-1]:
        print(f"{r.name} distance: {distance(r.location, r.destination)}")

    if move is None:
        move = Move(Line(r.location, r.destination), r.radius, time)

    if move.waiting:
        if distance_from_expected < DESTINATION_DEADZONE:
            return 0.0, 0.0
        new_heading = get_heading(Line(r.location, move.end))
    else:
        perpendicular_deviation = distance_point_to_line(r.location, move.line)
        # If the robot has deviated to the side of its path, it will correct itself with an offset
        # The offset is proportional to the deviation and the maximum offset
        heading_offset = min(1.0, perpendicular_deviation/MAX_PERPENDICULAR_DEVIATION) * MAX_HEADING_OFFSET
        new_heading = (expected_heading + heading_offset) % (2*pi)

    # Turning speed from -pi to pi, then normalized to -1 for max left and 1 for max right
    turning_speed = new_heading - r.heading
    if turning_speed > pi:
        turning_speed -= 2*pi
    elif turning_speed < -pi:
        turning_speed += 2*pi
    turning_speed /= pi

    dest_distance = distance(r.location, move.end)
    dest_distance_diff = dest_distance - distance(expected_location, move.end)

    speed_deviation = SPEED_DEVIATION * (dest_distance_diff / MAX_DEVIATION)

    if move == r.path[-1]:
        # Slows down when close to the destination
        speed = (SPEED + speed_deviation) * min(1.0, max(0.1, dest_distance*5))
        # Rotation goes up when close to the destination
        rotation_deviation = turning_speed / min(1.0, max(0.25, dest_distance*10))
    else:
        speed = (SPEED + speed_deviation) * min(1.0, max(0.5, dest_distance * 20))
        rotation_deviation = turning_speed  # / min(1.0, max(0.5, dest_distance * 20))

    return (min(1.0, max(-1.0, speed + rotation_deviation)) * SPEED_LIMITER,
            min(1.0, max(-1.0, speed - rotation_deviation)) * SPEED_LIMITER)


# Get the expected location and move of a robot, used for the combined robot
def get_expected_location_and_move(r: Robot, time: int, field: Field) -> tuple[Point, Move | None]:
    if not r.path:
        set_path(r.destination, r, field, time)

    expected_location = r.get_expected_location(time)
    distance_from_expected = distance(expected_location, r.location)

    if distance_from_expected > MAX_DEVIATION:
        print(f"{r.name} has deviated too far from its path. Recalculating path.")
        set_path(r.destination, r, field, time)
        print(f"New path: {path_str(r.path)}")
        expected_location = r.get_expected_location(time)
        distance_from_expected = distance(expected_location, r.location)

    move = r.get_move(time)

    if distance(r.location, r.destination) < DESTINATION_DEADZONE:
        return expected_location, None

    if move.waiting:
        if distance_from_expected < DESTINATION_DEADZONE:
            return expected_location, r.get_next_move(time)
        move = Move(Line(r.location, move.end), move.clearance, time, move.end_time)

    return expected_location, move


# To get the l and r speeds for a robot in a combined robot,
# the only_rotate is for all the bots to first get the right heading, so they'll stay in formation
def update_combined_speed_l_and_r(r: Robot, move: Move, time: int, only_rotate=False) -> tuple[float, float]:
    expected_location = move.location_at_time(time)
    if time > move.end_time:
        expected_location = move.end
    distance_from_expected = distance(expected_location, r.location)

    dest_distance = distance(r.location, move.end)
    if only_rotate:
        speed = 0.0
    else:
        dest_distance_diff = dest_distance - distance(expected_location, move.end)
        speed_deviation = SPEED_DEVIATION * (dest_distance_diff / MAX_DEVIATION)
        speed = (SPEED + speed_deviation) * min(1.0, max(0.1, dest_distance * 10))

    if move.waiting:
        if distance_from_expected < DESTINATION_DEADZONE:
            return 0.0, 0.0
        new_heading = get_heading(Line(r.location, move.end))
    elif only_rotate:
        new_heading = get_heading(move.line)
    else:
        expected_heading = get_heading(move.line)
        perpendicular_deviation = distance_point_to_line(r.location, move.line)
        heading_offset = perpendicular_deviation / MAX_PERPENDICULAR_DEVIATION * 0.25 * pi
        new_heading = (expected_heading + heading_offset) % (2 * pi)

    turning_speed = new_heading - r.heading
    if turning_speed > pi:
        turning_speed -= 2 * pi
    elif turning_speed < -pi:
        turning_speed += 2 * pi
    turning_speed /= pi

    rotation_deviation = turning_speed / min(1.0, max(0.5, dest_distance * 10))

    return (min(1.0, max(-1.0, speed + rotation_deviation)) * SPEED_LIMITER,
            min(1.0, max(-1.0, speed - rotation_deviation)) * SPEED_LIMITER)
