from pathlib import Path

from fairis_tools.my_robot import MyRobot


robot = MyRobot()

webots_sim = Path(__file__).resolve().parents[2]
maze_file = webots_sim / "worlds" / "Spring26" / "maze5.xml"

robot.load_environment(str(maze_file))
robot.move_to_start()

print("maze5 loaded")
print("starting vision goal navigation")


SEARCH_SPEED = 3.0
FORWARD_SPEED = 8.0
MAX_TURN = 4.0

IMAGE_CENTER = robot.camera.getWidth() / 2
STOP_DISTANCE = 0.55


def get_yellow_target():
    for detected_object in robot.camera.getRecognitionObjects():
        color_count = detected_object.getNumberOfColors()
        color_values = detected_object.getColors()

        if color_count == 0:
            continue

        red = color_values[0]
        green = color_values[1]
        blue = color_values[2]

        if red > 0.7 and green > 0.7 and blue < 0.3:
            return detected_object

    return None


while robot.experiment_supervisor.step(robot.timestep) != -1:
    target = get_yellow_target()

    if target is None:
        print("searching...")

        robot.set_left_motor_velocity(-SEARCH_SPEED)
        robot.set_right_motor_velocity(SEARCH_SPEED)
        continue

    image_position = target.getPositionOnImage()
    target_position = target.getPosition()

    image_x = image_position[0]
    target_distance = target_position[0]
    pixel_error = image_x - IMAGE_CENTER

    print(
        f"target x={image_x:.0f}, "
        f"error={pixel_error:.0f}, "
        f"distance={target_distance:.2f}"
    )

    if target_distance <= STOP_DISTANCE:
        robot.stop()
        print("TARGET REACHED")
        continue

    turn = pixel_error * 0.02
    turn = max(-MAX_TURN, min(MAX_TURN, turn))

    left_speed = FORWARD_SPEED + turn
    right_speed = FORWARD_SPEED - turn

    if abs(pixel_error) > 80:
        if pixel_error < 0:
            robot.set_left_motor_velocity(-SEARCH_SPEED)
            robot.set_right_motor_velocity(SEARCH_SPEED)
        else:
            robot.set_left_motor_velocity(SEARCH_SPEED)
            robot.set_right_motor_velocity(-SEARCH_SPEED)
    else:
        robot.set_left_motor_velocity(left_speed)
        robot.set_right_motor_velocity(right_speed)
