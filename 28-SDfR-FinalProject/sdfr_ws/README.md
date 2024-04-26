# Compile nodes
To compile all the nodes run the following command
```bash
colcon build
```

## Compile each package

### Image Functions SDfR Package
```bash
colcon build --packages-select image_functions_sdfr
```

### RELbot Interfaces Package
```bash
colcon build --packages-select relbot_interfaces
```

### Ball Brightness Detection Package
This package depends on the packages `relbot_interfaces` and `image_functions_sdfr` 
```bash
colcon build --packages-select brightness_detection
```

### Ball Detection Package
This package depends on the packages `relbot_interfaces` and `image_functions_sdfr` 
```bash
colcon build --packages-select ball_detection
```

### RELbot Twist Pair Drive Controller Package
This package depends on the packages `relbot_interfaces` and `image_functions_sdfr` 
```bash
colcon build --packages-select relbot_twist_drive_controller
```

### RELbot Differential Drive Controller Package
```bash
colcon build --packages-select relbot_diff_drive_controller
```

### RELbot Simulator Package
```bash
colcon build --packages-select relbot_simulator
```

# How to run the Nodes and more information
This information can be found inside of each of the packages foulders
