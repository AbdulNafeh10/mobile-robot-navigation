from pathlib import Path
import math
import statistics

from fairis_tools.my_robot import MyRobot


robot = MyRobot()

webots_sim = Path(__file__).resolve().parents[2]
maze_file = webots_sim / "worlds" / "Spring26" / "maze0.xml"

robot.load_environment(str(maze_file))
robot.move_to_start()

print("maze0 loaded")
print("starting wall following")


TARGET_WALL_DISTANCE = 0.55
BASE_SPEED = 3.8

DISTANCE_KP = 2.8
ANGLE_KP = 1.6

FRONT_TURN_DISTANCE = 0.70
WALL_LOST_DISTANCE = 1.30

START_EXIT_DISTANCE = 0.75
FINISH_DISTANCE = 0.35


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def lidar_median(scan, start, end):
    readings = []

    for index in range(start, end + 1):
        distance = scan[index % len(scan)]

        if math.isfinite(distance):
            readings.append(distance)

    if not readings:
        return float("inf")

    return statistics.median(readings)


def distance_from_start(start_position):
    position = robot.gps.getValues()

    dx = position[0] - start_position[0]
    dy = position[1] - start_position[1]

    return math.hypot(dx, dy)


start_position = robot.gps.getValues()
left_start_area = False


while robot.experiment_supervisor.step(robot.timestep) != -1:
    scan = robot.get_lidar_range_image()

    front_distance = lidar_median(scan, 170, 190)
    right_distance = lidar_median(scan, 265, 275)
    front_right_distance = lidar_median(scan, 235, 245)
    rear_right_distance = lidar_median(scan, 295, 305)

    start_distance = distance_from_start(start_position)

    # Do not allow the starting position to count as a completed lap.
    if start_distance > START_EXIT_DISTANCE:
        left_start_area = True

    if left_start_area and start_distance < FINISH_DISTANCE:
        robot.stop()
        print("maze traversal complete")
        break

    # Turn left when a wall blocks the path ahead.
    if front_distance < FRONT_TURN_DISTANCE:
        robot.set_left_motor_velocity(0.5)
        robot.set_right_motor_velocity(3.8)
        continue

    # Follow the wall around an outside corner.
    if right_distance > WALL_LOST_DISTANCE:
        robot.set_left_motor_velocity(4.0)
        robot.set_right_motor_velocity(2.2)
        continue

    distance_error = right_distance - TARGET_WALL_DISTANCE

    if (
        math.isfinite(front_right_distance)
        and math.isfinite(rear_right_distance)
    ):
        angle_error = (
            front_right_distance - rear_right_distance
        )
    else:
        angle_error = 0.0

    correction = (
        DISTANCE_KP * distance_error
        + ANGLE_KP * angle_error
    )

    correction = clamp(correction, -1.6, 1.6)

    left_speed = BASE_SPEED + correction
    right_speed = BASE_SPEED - correction

    robot.set_left_motor_velocity(left_speed)
    robot.set_right_motor_velocity(right_speed)
