# Package Brightness Detection
---
---
## Description

This package contains the two nodes for computing the average brightness detected through the webcam and publish it into different messages types. 

## Nodes

### 1. Node Brightness Detection 
 - **Description**: This node computes the average brightness and publish it as two separated topics: ```\light_level``` and ```\brightness_status```
   
- **Run the node**: To run the code use the following command:

   ```bash
   # Having set the enviroment using 
   source install/local_setup.sh

   # Exec the launch file of the node
   # Remember to ensure the stream of the webcam works correctly, the launch file will run the cam2image node.
   ros2 launch brightness_detection brightness_detection.launch.py
   ```

- **Expected published topic**: If the node is running both topics should be visible

   ```bash
   # Having set the enviroment using 
   source install/local_setup.sh

   # Topic \light_level
   ros2 topic echo \light_level

   # Topic \brightness_status
   ros2 topic echo \brightness_status
   ```

### 2. Node Custom Message Brightness Detection 
 - **Description**: This node computes the average brightness and publish it as two separated topics: ```\light_level``` and ```\brightness_status_custom``` where the the second one is a custom message from the package ```relbot_interfaces```

- **Run the node**: To run the code use the following command:

   ```bash
   # Having set the enviroment using 
   source install/local_setup.sh

   # Exec the launch file of the node
   # Remember to ensure the stream of the webcam works correctly, the launch file will run the cam2image node.
   ros2 launch brightness_detection custom_brightness_detection.launch.py
   ```

- **Expected published topic**: If the node is running both topics should be visible

   ```bash
   # Having set the enviroment using 
   source install/local_setup.sh

   # Topic \light_level
   ros2 topic echo \light_level

   # Topic \brightness_status_custom
   ros2 topic echo \brightness_status_custom
   ```