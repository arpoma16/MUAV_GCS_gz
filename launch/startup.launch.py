import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Declare arguments
    config_arg = DeclareLaunchArgument(
        'user_config',
        default_value='',
        description='Path to user configuration file (default: config/scene.yaml)'
    )

    # Get the default config path
    default_config = os.path.join(
        get_package_share_directory('muav_gcs_gz'),
        'config',
        'scene.yaml'
    )

    # Startup spawner node with delay to ensure Gazebo is ready
    startup_spawner = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to start
        actions=[
            Node(
                package='muav_gcs_gz',
                executable='startup_spawner.py',
                name='startup_spawner',
                output='screen',
                parameters=[{
                    'config_file': LaunchConfiguration('user_config')
                }]
            )
        ]
    )

    return LaunchDescription([
        config_arg,
        startup_spawner,
    ])
