
import os
import tempfile
import xml.etree.ElementTree as ET
import yaml

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

    # Ensure GZ_PARTITION is set (critical for Docker/containerized environments)
    if 'GZ_PARTITION' not in os.environ:
        os.environ['GZ_PARTITION'] = 'docker_sim_harmonic'

    gz_log_arg = LaunchConfiguration("gz_log_level").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    autostart = LaunchConfiguration("autostart").perform(context)
    user_config = LaunchConfiguration("user_config").perform(context)
    world_file = os.path.join(get_package_share_directory('muav_gcs_gz'), 'world', 'plazaAgua.sdf')
    

    # Use default config if user_config is empty
    if user_config == "":
        user_config = os.path.join(get_package_share_directory('muav_gcs_gz'), 'config', 'scene.yaml')

    # Load the config file to access origin data
    with open(user_config, 'r') as file:
        config_data = yaml.safe_load(file)

    if 'origin' in config_data and config_data['origin']:
        lat = config_data['origin'][0]
        lon = config_data['origin'][1]
        alt = config_data['origin'][2]
        build_world_file_template = os.path.join(get_package_share_directory('muav_gcs_gz'), 'world', 'template.sdf')
        world_file = build_world_file(build_world_file_template, lat, lon, alt)

    
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

    # Gazebo service bridges for spawn/delete
    gzSERVICE_bridge_spawn = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_spawn_service_bridge',
        arguments=['/world/default/create@ros_gz_interfaces/srv/SpawnEntity'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    gzSERVICE_bridge_delete = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_delete_service_bridge',
        arguments=['/world/default/remove@ros_gz_interfaces/srv/DeleteEntity'],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    # spawn elements defined in user config with delay
    startup = TimerAction(
        period=3.0,  # Wait 3 seconds for Gazebo to be ready
        actions=[
            Node(
                package='muav_gcs_gz',
                executable='startup_spawner.py',
                name='startup_spawner',
                output='screen',
                parameters=[{
                    'config_file': user_config
                }]
            )
        ]
    )
    
    #  drone spawning
    drones_to_spawn = []
    for entity_name, entity_data in config_data.items():
        # Skip non-dict entries (like 'origin' which is a list)
        if not isinstance(entity_data, dict):
            continue

        if entity_data.get('type') == 'drone':
            # Get pose data
            pose = entity_data.get('pose', {})
            xyz = pose.get('xyz', [0, 0, 0])
            rpy = pose.get('rpy', [0, 0, 0])

            drone_launch = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory("muav_gcs_gz"), "launch", "spawn_px4.launch.py")]
                ),
                launch_arguments=[
                    ("ID", str(entity_data.get('id', 0))),
                    ("namespace", entity_data.get('ns', f'px4_{entity_data.get("id", 0)}')),
                    ("vehicle", entity_data.get('vehicle', 'x500_mono_cam')),
                    ("enable_camera", str(entity_data.get('enable_camera', 'True'))),
                    ("x", str(xyz[0])),
                    ("y", str(xyz[1])),
                    ("z", str(xyz[2])),
                    ("roll", str(rpy[0])),
                    ("pitch", str(rpy[1])),
                    ("yaw", str(rpy[2])),
                    ("autostart", '4001'),
                ]
            )

            # Add delay to ensure Gazebo is ready (wait 5 seconds for first drone, then stagger additional drones)
            delay_period = 25.0 + (len(drones_to_spawn) * 2.0)
            drones_to_spawn.append(
                TimerAction(
                    period=delay_period,
                    actions=[drone_launch]
                )
            )


    return [
        gz,
        gz_sim_bridge,
        gzSERVICE_bridge_spawn,
        gzSERVICE_bridge_delete,
        startup,
    ] + drones_to_spawn

def build_world_file(template_path, latitude, longitude, elevation):
    """Builds a world file from a template, substituting in the given parameters."""
    with open(template_path, 'r') as file:
        sdf_content = file.read()

    sdf_content = sdf_content.replace('$(latitude)', str(latitude))
    sdf_content = sdf_content.replace('$(longitude)', str(longitude))
    sdf_content = sdf_content.replace('$(elevation)', str(elevation))

    # Write to a temporary file
    temp_world_file = tempfile.NamedTemporaryFile(delete=False, suffix='.sdf')
    with open(temp_world_file.name, 'w') as file:
        file.write(sdf_content)

    return temp_world_file.name


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("gz_log_level", default_value="msg",
                              description="Log level for Gazebo Options: [dbg, msg, warn, error]"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="Run Gazebo in headless mode (no GUI). Options: [true, false]"),
        DeclareLaunchArgument("autostart", default_value="true",
                              description="Start Gazebo automatically. Options: [true, false]"),
        DeclareLaunchArgument("user_config", default_value="",
                              description="Path to a user config"),
        OpaqueFunction(function=launch_setup)
    ])