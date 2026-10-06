# Mobile Robot Navigation

A Webots robotics project exploring mobile robot navigation through four stages: differential-drive kinematics, LiDAR wall following, camera-based target tracking, and autonomous obstacle avoidance.

## Controllers

### Kinematics
Uses wheel encoders and differential-drive geometry to follow a fixed route through maze0. Turns are based on wheel-speed ratios and calibrated against the simulator.

### Wall Following
Uses LiDAR feedback to maintain distance from the right wall, handle corners, and traverse maze0. GPS is only used to detect when the robot has completed the loop.

### Vision Goal
Uses camera recognition to find a yellow landmark, center it in the image, and drive toward it until the robot reaches the target distance.

### Autonomous Navigation
Combines camera-based goal seeking with LiDAR obstacle detection and wall following. The controller switches between seeking the target and following obstacles until it reaches the yellow landmark.

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

## Demo Media

Demo recordings and screenshots will be added after the final runs are captured.
