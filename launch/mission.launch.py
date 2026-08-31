import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    target_platform_arg = DeclareLaunchArgument(
        'target_platform',
        default_value='1',
        description='Plataforma alvo para pouso autônomo (1-8 ou nome da figura)'
    )

    mission_node = Node(
        package='ufvision_controller',
        executable='autonomus_landing_mission.py',
        name='autonomus_landing_mission',
        output='screen',
        parameters=[{
            'target_platform': LaunchConfiguration('target_platform')
        }]
    )

    return LaunchDescription([
        target_platform_arg,
        mission_node
    ])
