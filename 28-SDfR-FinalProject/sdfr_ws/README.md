# Using launch methods
## Run the first assignment
```bash
source install/local_setup.bash
ros2 launch ros2_ball_detection ball_detection.launch.py
```

### Parameters
Debug parameters
```bash
ros2 param set ball_detection debug_visualization <true or false>
```

## Run the second assignment
```bash
source install/local_setup.bash
ros2 launch ros2_relbot_control relbot_control.launch.py
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

# Check image output
```bash
ros2 run rqt_image_view rqt_image_view 
```