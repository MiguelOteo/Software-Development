from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros_xeno_bridge',
            executable='RosXenoBridge',
            name='RosXenoBridge',
            output="screen",
        ),
        Node(
            package='relbot_ros2_driver',
            executable='relbot_driver',
            name='relbot_driver',
            output="screen",
            parameters=[{
                # "max_velocity": 1.0,
                # "wheel_base_width": 0.209,
                # "wheel_radius": 0.05,
            }]
        ),
    ])