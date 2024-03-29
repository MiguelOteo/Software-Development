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

Run the main node to detect brightness
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

### Remember 
On every console opened run the command
```bash
source install/local_setup.bash
```