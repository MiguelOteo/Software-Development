echo ========================================
echo == Ask root password for installation ==
echo ========================================
sudo ls > /dev/null

echo Preparing registers and permissions...

# Setting certain registers to init the SPI driver into correct state
# Website 
# https://github.com/DenizUgur/RPi4-EVL-4xSPI
sudo busybox devmem 0xfe204e00 32 0x03800000
# Give corect permissions to x-buffer
sudo chmod 666 /dev/evl/xbuf/Ros-Xeno
sudo chmod 666 /dev/evl/xbuf/Xeno-Ros

echo Starting ROS2 nodes...

# Source the install folder (assumes this script is run from the folder where the install folder is located)
source install/setup.bash

# Start all required nodes using this launch file
ros2 launch relbot_ros2_driver start_relbot_nodes.launch.py
