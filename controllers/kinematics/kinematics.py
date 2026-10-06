from pathlib import Path
import math

from fairis_tools.my_robot import MyRobot


robot = MyRobot()

webots_sim = Path(__file__).resolve().parents[2]
maze_file = webots_sim / "worlds" / "Spring26" / "maze0.xml"

robot.load_environment(str(maze_file))
robot.move_to_start()

print("maze0 loaded")
print("starting kinematics route")


STRAIGHT_SPEED = 6.0
TURN_SPEED = 4.0
SLOW_SPEED = 2.0
SLOW_DISTANCE = 0.15

# Small correction for the turn behavior observed in simulation.
TURN_RATIO_CALIBRATION = 0.981


def print_pose(label):
    position = robot.gps.getValues()
    heading = robot.get_compass_reading()

    print(
        f"{label} | "
        f"x={position[0]:.3f}, "
        f"y={position[1]:.3f}, "
        f"z={position[2]:.3f}, "
        f"heading={heading:.2f}°"
    )


def settle():
    robot.stop()

    for _ in range(4):
        robot.experiment_supervisor.step(robot.timestep)


def get_wheel_distances(start_left, start_right):
    current_left, current_right = robot.get_encoder_readings()

    left_distance = abs(
        (current_left - start_left) * robot.wheel_radius
    )
    right_distance = abs(
        (current_right - start_right) * robot.wheel_radius
    )

    return left_distance, right_distance


def drive_straight(distance):
    start_left, start_right = robot.get_encoder_readings()

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        left_distance, right_distance = get_wheel_distances(
            start_left,
            start_right,
        )

        distance_traveled = (
            left_distance + right_distance
        ) / 2

        remaining_distance = distance - distance_traveled

        if remaining_distance <= 0:
            settle()
            return

        speed = (
            SLOW_SPEED
            if remaining_distance < SLOW_DISTANCE
            else STRAIGHT_SPEED
        )

        robot.set_left_motor_velocity(speed)
        robot.set_right_motor_velocity(speed)


def turn_90(direction, radius):
    half_axle = robot.axel_length / 2
    turn_angle = math.pi / 2

    outer_radius = radius + half_axle
    inner_radius = radius - half_axle

    outer_target_distance = outer_radius * turn_angle

    wheel_speed_ratio = (
        inner_radius / outer_radius
    ) * TURN_RATIO_CALIBRATION

    start_left, start_right = robot.get_encoder_readings()

    while robot.experiment_supervisor.step(robot.timestep) != -1:
        left_distance, right_distance = get_wheel_distances(
            start_left,
            start_right,
        )

        if direction == "right":
            outer_distance = left_distance
        else:
            outer_distance = right_distance

        remaining_distance = (
            outer_target_distance - outer_distance
        )

        if remaining_distance <= 0:
            settle()
            return

        outer_speed = (
            SLOW_SPEED
            if remaining_distance < SLOW_DISTANCE
            else TURN_SPEED
        )

        inner_speed = outer_speed * wheel_speed_ratio

        if direction == "right":
            robot.set_left_motor_velocity(outer_speed)
            robot.set_right_motor_velocity(inner_speed)
        else:
            robot.set_left_motor_velocity(inner_speed)
            robot.set_right_motor_velocity(outer_speed)


def right_turn(radius=0.5):
    turn_90("right", radius)


def left_turn(radius=0.5):
    turn_90("left", radius)


def right_u_turn(radius=0.5):
    right_turn(radius)
    right_turn(radius)


def left_u_turn(radius=0.5):
    left_turn(radius)
    left_turn(radius)


print_pose("P0 START")

drive_straight(3.5)
print_pose("P1 after straight 3.5")

right_turn(0.5)
print_pose("P2 after right 0.5")

drive_straight(1.0)
print_pose("P3 after straight 1.0")

right_u_turn(0.5)
print_pose("P4 after right U-turn 0.5")

left_turn(0.5)
print_pose("P5 after left 0.5")

drive_straight(2.0)
print_pose("P6 after straight 2.0")

left_u_turn(0.5)
print_pose("P7 after left U-turn 0.5")

drive_straight(1.0)
print_pose("P8 after straight 1.0")

right_turn(0.5)
print_pose("P9 after right 0.5")

left_turn(0.5)
print_pose("P10 after left 0.5")

drive_straight(0.5)
print_pose("P11 after straight 0.5")

right_u_turn(0.5)
print_pose("P12 after right U-turn 0.5")

drive_straight(2.5)
print_pose("P13 after straight 2.5")

right_u_turn(0.5)
print_pose("P14 after right U-turn 0.5")

drive_straight(0.5)
print_pose("P15 FINAL")

robot.stop()

print("kinematics route complete")
