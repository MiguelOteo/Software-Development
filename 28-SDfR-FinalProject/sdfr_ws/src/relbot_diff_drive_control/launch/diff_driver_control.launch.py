from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution, FindExecutable

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # -------- Launched actions --------
    return LaunchDescription([

        # Run the node for the differential driver RELbot control to compute its inputs
        Node(
            package='relbot_diff_drive_control',
            executable='diff_driver_controller',
            name='diff_driver_controller',
        ),
    ])