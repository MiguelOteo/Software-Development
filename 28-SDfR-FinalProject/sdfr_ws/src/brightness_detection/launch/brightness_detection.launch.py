from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    
    # -------- Launched actions --------
    return LaunchDescription([

        # Run the node of the webcam to image
        Node(
            package='image_tools',
            executable='cam2image',
            name='cam2image'
        ),

        # Run the node for the node for brightness detection
        Node(
            package='brightness_detection',
            executable='brightness_detection',
            name='brightness_detection',
        ),
    ])