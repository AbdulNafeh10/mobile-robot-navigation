<img width="1672" height="941" alt="image" src="https://github.com/user-attachments/assets/f63f9066-06ee-4208-8b65-133ed4bdff53" />

# Mobile Robot Navigation

A Webots robotics project exploring mobile robot navigation through four stages: differential-drive kinematics, LiDAR wall following, camera-based target tracking, and autonomous obstacle avoidance.

## Controllers

### Kinematics
Uses wheel encoders and differential-drive geometry to follow a fixed route through maze0. Straight motion is tracked from encoder distance, while turns use different wheel speeds for a chosen turning radius.

[![Kinematics demo](https://img.youtube.com/vi/zxgj0xpY8nE/maxresdefault.jpg)](https://youtu.be/zxgj0xpY8nE)

[Watch on YouTube](https://youtu.be/zxgj0xpY8nE)

### Wall Following
Uses LiDAR feedback to maintain distance from the right wall, correct the robot's angle, and handle both inside and outside corners while traversing maze0. GPS is only used to detect when the robot has completed the loop.

[![Wall following demo](https://img.youtube.com/vi/qgN4R79otgg/maxresdefault.jpg)](https://youtu.be/qgN4R79otgg)

[Watch on YouTube](https://youtu.be/qgN4R79otgg)

### Vision Goal
Uses camera recognition to find a yellow landmark, measure its horizontal position in the image, and steer toward it. If the target leaves the camera view, the robot rotates until it finds it again.

[![Vision goal demo](https://img.youtube.com/vi/FgE4848Mn1A/maxresdefault.jpg)](https://youtu.be/FgE4848Mn1A)

[Watch on YouTube](https://youtu.be/FgE4848Mn1A)

### Autonomous Navigation
Combines camera-based goal seeking with LiDAR obstacle detection and wall following. The same controller can start from different positions, move directly toward the target when the path is clear, or follow obstacles until it can return to goal seeking.

[![Autonomous navigation demo](https://img.youtube.com/vi/LliAZecvT_E/maxresdefault.jpg)](https://youtu.be/LliAZecvT_E)

[Watch on YouTube](https://youtu.be/LliAZecvT_E)

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

