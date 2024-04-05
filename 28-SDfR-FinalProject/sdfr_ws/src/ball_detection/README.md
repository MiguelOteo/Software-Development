# Package Ball Detection
---
---
## Description

This package contains the one nodes for computing a bounding box around a detected object. The image segmentation is performed based on color in the HSV color space. 

## Nodes

### 1. Node Ball Detection 
 - **Description**: This node will publish the bounding box in the following topic: ```\bounding_box```. The node also has a debugging image funtionality which it is published to the topic ```\debug_image```
   
- **Run the node**: To run the code use the following command:

   ```bash
   # Having set the enviroment using 
   source install/local_setup.sh

   # Exec the launch file of the node
   # Remember to ensure the stream of the webcam works correctly, the launch file will run the cam2image node.
   ros2 launch ball_detection ball_detection.launch.py
   ```

- **Parameters**: The node has a debug image funtion which can be turn on and off using:

   ```bash
   # Set to True (default value) to enable the output on the topic \debug_image
   ros2 param set ball_detection debug_visualization <true or false>
   ```

- **Expected published topic**: If the node is running both topics should be visible

   ```bash
   # Having set the enviroment using 
   source install/local_setup.sh

   # Topic \bounding_box
   ros2 topic echo \bounding_box

   # Check the debug output on the topic \debug_image
   ros2 ros2 run rqt_image_view rqt_image_view
   ```