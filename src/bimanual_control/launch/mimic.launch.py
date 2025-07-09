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
        # Launch mimic robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(control_launch),
            launch_arguments={
                'robot_model': 'rx200',
                'robot_name': 'mimic',
                'use_sim': 'true',
                'use_namespace': 'true',
                'frame_prefix': 'mimic/',
                'use_rviz': 'false'
            }.items()
        ),

        # Static transform between robots
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.58', '0', '0', '-3.14', '0', '0', 'rx200_1/base_link', 'mimic/base_link'],
            name='static_tf_rx2001_to_mimic',
            output='screen'
        ),
    ])

