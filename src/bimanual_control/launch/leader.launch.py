from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    control_pkg = FindPackageShare('interbotix_xsarm_control').find('interbotix_xsarm_control')
    control_launch = os.path.join(control_pkg, 'launch', 'xsarm_control.launch.py')

    return LaunchDescription([
        # Launch leader robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'robot_model': 'rx200',
                'robot_name': 'rx200_1',
                'use_sim': 'true',
                'use_namespace': 'true',
                'frame_prefix': 'rx200_1/'
            }.items()
        ),

        # Launch omni_state
        Node(
            package='omni_common',
            executable='omni_state',
            name='omni_state',
            output='screen'
        ),
    ])
