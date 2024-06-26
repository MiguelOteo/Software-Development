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

        # Run the node for the ball detection
        Node(
            package='ball_detection',
            executable='ball_detection',
            name='ball_detection',
                parameters=[
                    # Set the command mode based on the launch argument
                    {"debug_visualization": True},
                ],
        ),

        # Run the node for the RELbot control to compute its inputs
        Node(
            package='relbot_twist_drive_controller',
            executable='relbot_control',
            name='relbot_control',
                parameters=[
                    # Set the command mode based on the launch argument
                    {"debug_visualization": True},
                ],
        ),

        # Run the node for the differential drive RELbot control to compute its inputs
        Node(
            package='relbot_diff_drive_control',
            executable='diff_drive_controller',
            name='diff_drive_controller',
        ),
    ])