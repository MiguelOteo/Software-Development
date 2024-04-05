# Launch nodes
## Run the Brightness Detection nodes
### Run the brightness detection node
```bash
source install/local_setup.bash
ros2 launch brightness_detection brightness_detection.launch.py
```

#### Check results
```bash
source install/local_setup.bash
ros2 topic echo /light_level
```

```bash
source install/local_setup.bash
ros2 topic echo /brightness_status
```

### Run the custom brightness detection node
```bash
source install/local_setup.bash
ros2 launch brightness_detection custom_brightness_detection.launch.py
```

#### Check results
```bash
source install/local_setup.bash
ros2 topic echo /light_level
```

```bash
source install/local_setup.bash
ros2 topic echo /brightness_status_custom
```

## Run Ball Detection node

```bash
source install/local_setup.bash
ros2 launch ball_detection ball_detection.launch.py
```

#### Parameters
Debug parameter
```bash
ros2 param set ball_detection debug_visualization <true or false>
```

#### Check results
```bash
source install/local_setup.bash
ros2 topic echo /bounding_box
```

## Run Twist Pairs RELbot Control node
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

## Run RELbot Differential Control node

```bash
source install/local_setup.bash
ros2 launch relbot_diff_drive_control diff_drive_control.launch.py
```

#### Check results
```bash
source install/local_setup.bash
ros2 topic echo /input/right_motor/setpoint_vel
```

```bash
source install/local_setup.bash
ros2 topic echo /input/left_motor/setpoint_vel
```

## Check debug image output
```bash
ros2 run rqt_image_view rqt_image_view 
```