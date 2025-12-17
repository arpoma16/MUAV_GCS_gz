
import os
import tempfile
import xml.etree.ElementTree as ET

from jsonschema import ValidationError

from launch import LaunchDescription

from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
    TimerAction
)

from launch.substitutions import LaunchConfiguration

from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    gz_log_arg = LaunchConfiguration("gz_log_level").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)

    world_file = os.path.join(get_package_share_directory('muav_gcs_gz'), 'world', 'plazaAgua.sdf')

    # Build gz_args string for ros_gz_sim
    gz_log_levels = {"error": 1, "warn": 2, "msg": 3, "dbg": 4}
    gz_log_level = gz_log_levels.get(gz_log_arg, 4)

    # Build flags
    # -s = headless (server only), -r = run on start (auto-play)
    # If not using -r, simulation starts paused and must be started manually
    headless_flag = '-s' if headless.lower() == 'true' else ''
    run_flag = '-r' if autostart.lower() == 'true' else ''

    gz_args = f'{headless_flag} {run_flag} --verbose {gz_log_level} {world_file}'.strip()

    # Use ros_gz_sim to properly set up rendering environment
    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments=[
            ('gz_args', gz_args),
            ('on_exit_shutdown', 'true')
        ]
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ],
        parameters=[{'use_sim_time': True}],
        output="screen"
    )

    gzSERVICE_bridge_spawn = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_spawn_service_bridge',
        arguments=['/world/default/create@ros_gz_interfaces/srv/SpawnEntity'],
        output='screen'
    )
    gzSERVICE_bridge_delete = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_delete_service_bridge',
        arguments=['/world/default/remove@ros_gz_interfaces/srv/DeleteEntity'],
        output='screen'
    )

    return [
        gz,
        gz_sim_bridge,
        gzSERVICE_bridge_spawn,
        gzSERVICE_bridge_delete,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gz_log_level", default_value="msg",
                              description="Log level for Gazebo Options: [dbg, msg, warn, error]"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Run Gazebo in headless mode (no GUI). Options: [true, false]"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="Start Gazebo automatically. Options: [true, false]"),
        OpaqueFunction(function=launch_setup)
    ])