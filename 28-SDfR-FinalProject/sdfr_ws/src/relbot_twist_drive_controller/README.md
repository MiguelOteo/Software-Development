# Package RELbot Twist Drive Controller
---
---
## Description

This package contains the the node to compute the twist pair output based on the bounding box output form the node ```ball_detection``` which is published in the channel ```\bounding_box```.

## Nodes

### 1. Node RELbot Control
 - **Description**: This node will subscribes to the following topic: ```\bounding_box```. The topic belongs to the package ```relbot_interfaces```. The node will compute the twist pair of the RELbot robot base on the input boundingbox to move the robot toward the target. The node will then publish the twist pair into the topic ```\cmd_vel```.
   
- **Run the node**: To run the code use the following command:

   ```bash
   # Having set the enviroment using 
   source install/local_setup.sh

   # Exec the launch file of the node
   # Remember to ensure the stream of the webcam works correctly, the launch file will run both cam2image and ball_detection nodes.
   ros2 launch relbot_twist_drive_controller relbot_control.launch.py
   ```

- **Parameters**: The node has a debug image funtion which can be turn on and off using:

   Change the ball size that will be use as the objective to match in the detected object
   ```bash
   # Change the size of the ball
   ros2 param set relbot_control ball_size <ball size>
   ```

   Changes int the angular gain will increase or decrease the rate of rotation of the RELbot
   ```bash
   # Change angular gain 
   ros2 param set relbot_control angular_gain <value>
   ```
   Changes in the linear gain will affect the linear speed at which the robot will move   
   ```bash
   # Change linear gain 
   ros2 param set relbot_control linear_gain <value>
   ```
   
- **Expected published topic**: If the node is running both topics should be visible

   ```bash
   # Having set the enviroment using 
   source install/local_setup.sh

   # Topic \cmd_vel
   ros2 topic echo \cmd_vel
   ```