# Package RELbot Differential Drive Controller
---
---
## Description

This package contains the node to compute the wheel velocities outputs based on the twist pair velocity output form the node ```relbot_control``` form the package ```relbot_twist_drive_controller``` which is published in the channel ```\cmd_vel```.

## Nodes

### 1. Node RELbot Control
 - **Description**: This node will subscribes to the following topic: ```\cmd_vel```. The node will compute each of the wheel speeds of the RELbot robot base on the input twist pairs to move the robot toward the target. The node will then publish the twist pair into the topics ```\right_motor\setpoint_vel``` and ```\left_motor\setpoint_vel```.
   
- **Run the node**: To run the code use the following command:

   ```bash
   # Having set the enviroment using 
   source install/local_setup.sh

   # Exec the launch file of the node
   # Remember to ensure the stream of the webcam works correctly, the launch file will run both cam2image, ball_detection, relbot_control nodes.
   ros2 launch relbot_diff_drive_control diff_drive_control.launch.py
   ```

- **Expected published topic**: If the node is running both topics should be visible

   ```bash
   # Having set the enviroment using 
   source install/local_setup.sh

   # Topic for the right wheel
   ros2 topic echo \right_motor\setpoint_vel

   # Topic for the left wheel
   ros2 topic echo \left_motor\setpoint_vel
   ```