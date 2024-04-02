## To run the first asssignment

Run the node of the camera to image
```bash
ros2 run image_tools cam2image
```
Run the main node to detect brightness
```bash
ros2 run ros2_introduction brightness_detection
```

### Check if it is working
Run the following echo to listen to the output
```bash
ros2 topic echo /brightness_status_custom
```


## To run the second asssignment

Run the node of the camera to image
```bash
ros2 run image_tools cam2image
```

Run the main node to detect the ball
```bash
ros2 run ros2_ball_detection ball_detection
```

### Check if it is working
Run the following echo to listen to the output
```bash
ros2 topic echo /bounding_box
```

#### Debug image function
Turn on the debug image output
```bash
ros2 param set ball_detection debug_visualization true
```

Run this command to see the border around the ball on the porcessed image
```bash
ros2 run rqt_image_view rqt_image_view
```
Select the output \debug_image


## To run the third asssignment
Run the node of the camera to image
```bash
ros2 run image_tools cam2image
```

Run the secondary node to detect the ball
```bash
ros2 run ros2_ball_detection ball_detection
```

Run the main node to compute the speed etc.
```bash
ros2 run ros2_relbot_control relbot_control
```

To check the twist velocity of the motors run one of the commands
```bash
ros2 topic echo /twist_vel
ros2 topic echo /input/right_motor/setpoint_vel
ros2 topic echo /input/left_motor/setpoint_vel
```

#### Control params
Change the size of the ball
```bash
ros2 param set relbot_control ball_size <ball size>
```

Change angular gain 
```bash
ros2 param set relbot_control angular_gain <value>
```

Change linear gain 
```bash
ros2 param set relbot_control linear_gain <value>
```

#### Debug image function
Turn on the debug image output on both nodes
```bash
ros2 param set ball_detection debug_visualization true
ros2 param set relbot_control debug_visualization true
```

Run this command to see the border around the ball on the porcessed image
```bash
ros2 run rqt_image_view rqt_image_view
```
Select the output \debug_image_control

## Run the simulator
Run this command to run the simulator
```bash
ros2 launch relbot_simulator relbot_simulator.launch.py
```

## Remember 
On every console opened run the command
```bash
source install/local_setup.bash
```