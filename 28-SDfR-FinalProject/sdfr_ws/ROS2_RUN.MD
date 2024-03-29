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
ros2 topic echo /ball_detection/bounding_box
```

Run this command to see the border around the ball on the porcessed image
```bash
ros2 topic echo /ball_detection/bounding_box
```


