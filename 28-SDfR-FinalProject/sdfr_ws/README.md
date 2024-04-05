# Using launch methods
## Run the first assignment
```bash
source install/local_setup.bash
ros2 launch ball_detection ball_detection.launch.py
```

### Parameters
Debug parameters
```bash
ros2 param set ball_detection debug_visualization <true or false>
```

## Run the second assignment
```bash
source install/local_setup.bash
ros2 launch relbot_twist_drive_controller relbot_control.launch.py
```

### Parameters
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

Debug parameters
```bash
ros2 param set ball_detection debug_visualization <true or false>
ros2 param set relbot_control debug_visualization <true or false>
```

## Run the third assignment
To run the assignment RELbot differential control execute the following command
```bash
source install/local_setup.bash
ros2 launch relbot_diff_drive_control diff_drive_control.launch.py
```

To check the diff velocities of the motors run one of the commands
```bash
ros2 topic echo /input/right_motor/setpoint_vel
ros2 topic echo /input/left_motor/setpoint_vel
```

# Check image output
```bash
ros2 run rqt_image_view rqt_image_view 
```