# Mobile Robot Navigation

A Webots robotics project exploring mobile robot navigation through four stages: differential-drive kinematics, LiDAR wall following, camera-based target tracking, and autonomous obstacle avoidance.

## Controllers

### Kinematics
Uses wheel encoders and differential-drive geometry to follow a fixed route through maze0. Straight motion is tracked from encoder distance, while turns use different wheel speeds for a chosen turning radius.

[Watch the kinematics demo](https://github.com/AbdulNafeh10/mobile-robot-navigation/releases/download/v1.0-demo/Kinematics.mp4)

### Wall Following
Uses LiDAR feedback to maintain distance from the right wall, correct the robot's angle, and handle both inside and outside corners while traversing maze0. GPS is only used to detect when the robot has completed the loop.

[Watch the wall-following demo](https://github.com/AbdulNafeh10/mobile-robot-navigation/releases/download/v1.0-demo/Wall_Follow.mp4)

### Vision Goal
Uses camera recognition to find a yellow landmark, measure its horizontal position in the image, and steer toward it. If the target leaves the camera view, the robot rotates until it finds it again.

[Watch the vision-goal demo](https://github.com/AbdulNafeh10/mobile-robot-navigation/releases/download/v1.0-demo/Vision.Goal.mp4)

### Autonomous Navigation
Combines camera-based goal seeking with LiDAR obstacle detection and wall following. The same controller can start from different positions, move directly toward the target when the path is clear, or follow obstacles until it can return to goal seeking.

[Watch the autonomous navigation demo](https://github.com/AbdulNafeh10/mobile-robot-navigation/releases/download/v1.0-demo/Autonomous_Navigation.mp4)

## Tools

- Python
- Webots
- FAIRIS-Lite
- LiDAR
- Camera recognition
- Differential-drive kinematics
- Feedback control

## Running

This project was developed with Webots and FAIRIS-Lite.

Copy the controller folders into:

`WebotsSim/controllers/`

Then open `StartingWorld.wbt`, select the desired HamBot controller, and run the simulation.

## Demo Videos

All four demo recordings are also available in the [v1.0 demo release](https://github.com/AbdulNafeh10/mobile-robot-navigation/releases/tag/v1.0-demo).
