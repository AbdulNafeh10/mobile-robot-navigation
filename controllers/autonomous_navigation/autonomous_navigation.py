from pathlib import Path
import math
import statistics

from fairis_tools.my_robot import MyRobot


robot = MyRobot()

webots_sim = Path(__file__).resolve().parents[2]
maze_file = webots_sim / "worlds" / "Spring26" / "maze6.xml"

robot.load_environment(str(maze_file))
robot.move_to_start()

print("maze6 loaded")
print("starting autonomous navigation")


IMAGE_CENTER = robot.camera.getWidth() / 2

GOAL_SPEED = 5.0
GOAL_TURN_KP = 0.010
MAX_GOAL_TURN = 1.8
SEARCH_SPEED = 2.0

STOP_DISTANCE = 0.60
OBSTACLE_DISTANCE = 1.20
TARGET_LOSS_GRACE = 12

WALL_DISTANCE = 0.55
WALL_SPEED = 3.8

DISTANCE_KP = 2.8
ANGLE_KP = 1.6

FRONT_TURN_DISTANCE = 0.70
WALL_LOST_DISTANCE = 1.30

ERROR_SMOOTHING = 0.20
MOTOR_SMOOTHING = 0.18


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


def get_yellow_target():
    for detected_object in robot.camera.getRecognitionObjects():
        if detected_object.getNumberOfColors() == 0:
            continue

        colors = detected_object.getColors()

        red = colors[0]
        green = colors[1]
        blue = colors[2]

        if red > 0.7 and green > 0.7 and blue < 0.3:
            return detected_object

    return None


filtered_target_error = 0.0
left_command = 0.0
right_command = 0.0


def set_smooth_speeds(left_target, right_target):
    global left_command, right_command

    left_command += MOTOR_SMOOTHING * (
        left_target - left_command
    )
    right_command += MOTOR_SMOOTHING * (
        right_target - right_command
    )

    robot.set_left_motor_velocity(left_command)
    robot.set_right_motor_velocity(right_command)


state = "SEEK_GOAL"

last_target_error = 0.0
frames_since_target = 999
print_counter = 0


while robot.experiment_supervisor.step(robot.timestep) != -1:
    scan = robot.get_lidar_range_image()

    front_distance = lidar_median(scan, 170, 190)
    right_distance = lidar_median(scan, 265, 275)
    front_right_distance = lidar_median(scan, 235, 245)
    rear_right_distance = lidar_median(scan, 295, 305)

    target = get_yellow_target()

    print_counter += 1

    if state == "SEEK_GOAL":
        if target is not None:
            frames_since_target = 0

            image_position = target.getPositionOnImage()
            target_position = target.getPosition()

            pixel_error = image_position[0] - IMAGE_CENTER
            target_distance = target_position[0]

            filtered_target_error = (
                (1.0 - ERROR_SMOOTHING)
                * filtered_target_error
                + ERROR_SMOOTHING * pixel_error
            )

            last_target_error = filtered_target_error

            if print_counter % 10 == 0:
                print(
                    f"SEEK | "
                    f"target={target_distance:.2f} "
                    f"front={front_distance:.2f} "
                    f"error={filtered_target_error:.0f}"
                )

            if target_distance <= STOP_DISTANCE:
                robot.stop()
                print("TARGET REACHED")
                continue

            if (
                front_distance < OBSTACLE_DISTANCE
                and front_distance + 0.20 < target_distance
            ):
                print(
                    f"obstacle at "
                    f"{front_distance:.2f} -> FOLLOW_WALL"
                )

                state = "FOLLOW_WALL"
                continue

            turn = clamp(
                filtered_target_error * GOAL_TURN_KP,
                -MAX_GOAL_TURN,
                MAX_GOAL_TURN,
            )

            alignment = 1.0 - min(
                abs(filtered_target_error) / IMAGE_CENTER,
                0.50,
            )

            forward_speed = GOAL_SPEED * alignment

            set_smooth_speeds(
                forward_speed + turn,
                forward_speed - turn,
            )

            continue

        frames_since_target += 1

        if frames_since_target <= TARGET_LOSS_GRACE:
            turn = clamp(
                last_target_error * GOAL_TURN_KP,
                -MAX_GOAL_TURN,
                MAX_GOAL_TURN,
            )

            set_smooth_speeds(
                3.0 + turn,
                3.0 - turn,
            )

            continue

        if front_distance < 1.30:
            print(
                f"target occluded | "
                f"front={front_distance:.2f} -> FOLLOW_WALL"
            )

            state = "FOLLOW_WALL"
            continue

        if print_counter % 10 == 0:
            print(
                f"SEARCH | "
                f"front={front_distance:.2f} "
                f"lost={frames_since_target}"
            )

        if last_target_error < 0:
            set_smooth_speeds(
                -SEARCH_SPEED,
                SEARCH_SPEED,
            )
        else:
            set_smooth_speeds(
                SEARCH_SPEED,
                -SEARCH_SPEED,
            )

    elif state == "FOLLOW_WALL":
        if print_counter % 10 == 0:
            print(
                f"WALL | "
                f"front={front_distance:.2f} "
                f"right={right_distance:.2f}"
            )

        if target is not None:
            frames_since_target = 0

            image_position = target.getPositionOnImage()
            target_position = target.getPosition()

            pixel_error = image_position[0] - IMAGE_CENTER
            target_distance = target_position[0]

            filtered_target_error = (
                (1.0 - ERROR_SMOOTHING)
                * filtered_target_error
                + ERROR_SMOOTHING * pixel_error
            )

            last_target_error = filtered_target_error

            if target_distance <= STOP_DISTANCE:
                robot.stop()
                print("TARGET REACHED")
                continue

            if (
                abs(filtered_target_error) < 90
                and front_distance > 1.20
            ):
                print("goal visible -> SEEK_GOAL")
                state = "SEEK_GOAL"
                continue

        if front_distance < FRONT_TURN_DISTANCE:
            set_smooth_speeds(0.5, 3.8)
            continue

        if right_distance > WALL_LOST_DISTANCE:
            set_smooth_speeds(4.0, 2.2)
            continue

        distance_error = (
            right_distance - WALL_DISTANCE
        )

        if (
            math.isfinite(front_right_distance)
            and math.isfinite(rear_right_distance)
        ):
            angle_error = (
                front_right_distance
                - rear_right_distance
            )
        else:
            angle_error = 0.0

        correction = (
            DISTANCE_KP * distance_error
            + ANGLE_KP * angle_error
        )

        correction = clamp(
            correction,
            -1.6,
            1.6,
        )

        left_speed = WALL_SPEED + correction
        right_speed = WALL_SPEED - correction

        set_smooth_speeds(
            left_speed,
            right_speed,
        )
