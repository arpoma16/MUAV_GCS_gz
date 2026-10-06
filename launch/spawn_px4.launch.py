#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import re
import sys
import tempfile

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from px4_gz_env import find_px4


# <include merge="true"><uri>model://X</uri></include> with nothing else inside
_MERGE_INCLUDE_RE = re.compile(
    r'<include\s+merge="true"\s*>\s*<uri>model://([^<\s]+)</uri>\s*</include>')


def inline_package_models(content, models_dir):
    """Inline the merge-includes of models that live in this package's models/ dir.

    A model derived from another package model (e.g. x500_gimbal_collision_lidar includes
    x500_gimbal_collision) would otherwise keep the included model's @NAMESPACE@ unfilled.
    Includes of models outside the package (PX4's x500_gimbal, ...) are left untouched.
    """
    def repl(match):
        path = os.path.join(models_dir, match.group(1), 'model.sdf')
        if not os.path.isfile(path):
            return match.group(0)
        with open(path, 'r') as f:
            inner = f.read()
        body = re.search(r'<model\b[^>]*>(.*)</model>', inner, re.S)
        if body is None:
            return match.group(0)
        return inline_package_models(body.group(1), models_dir)

    return _MERGE_INCLUDE_RE.sub(repl, content)


def fill_namespace(model_path, namespace):
    """Replace @NAMESPACE@ in a model SDF (ROS namespace of the vehicle).

    Returns the original path if nothing had to change, otherwise a temporary copy with the
    package-model includes inlined and the namespace filled in (the plugins of the model
    need it at load time).
    """
    with open(model_path, 'r') as f:
        original = f.read()
    content = inline_package_models(original, os.path.dirname(os.path.dirname(model_path)))
    if content == original and '@NAMESPACE@' not in content:
        return model_path
    out_dir = tempfile.mkdtemp(prefix='muav_model_')
    out_path = os.path.join(out_dir, 'model.sdf')
    with open(out_path, 'w') as f:
        f.write(content.replace('@NAMESPACE@', namespace))
    return out_path


def launch_setup(context, *args, **kwargs):

    # Get launch configurations (for camera bridge)
    vehicle = LaunchConfiguration('vehicle').perform(context)
    ID = LaunchConfiguration('ID').perform(context)
    world = LaunchConfiguration('world').perform(context)
    namespace_val = LaunchConfiguration('namespace').perform(context)
    enable_camera_val = LaunchConfiguration('enable_camera').perform(context)
    enable_lidar_val = LaunchConfiguration('enable_lidar').perform(context)
    enable_tf_val = LaunchConfiguration('enable_tf').perform(context)
    map_origin_val = LaunchConfiguration('map_origin').perform(context)
    gz_model_name_val = LaunchConfiguration('gz_model_name').perform(context)

    # List to hold camera bridge actions
    camera_actions = []
    
    # PX4 SITL launch
    px4_sitl_node = OpaqueFunction(function=launch_px4)

    # Only create camera bridge if enabled
    if enable_camera_val.lower() == 'true':
        # Determine namespace for remapping
        ns = f'px4_{ID}'
        if namespace_val and namespace_val != '':
            ns = namespace_val

        # Build topic names as strings (since we already performed the LaunchConfigurations)
        gz_topic = f'/world/{world}/model/{vehicle}_{ID}/link/camera_link/sensor/camera/image'
        ros_topic = f'/{ns}/camera/image'

        # Using ros_gz_image for more efficient camera bridging
        # This provides automatic compression support via image_transport
        camera_bridge = Node(
            package='ros_gz_image',
            executable='image_bridge',
            name=f'camera_bridge_{ID}',
            arguments=[gz_topic],
            remappings=[
                (gz_topic, ros_topic),
                (f'{gz_topic}/compressed', f'{ros_topic}/compressed')
            ],
            output='screen',
            parameters=[{
                'use_sim_time': True
            }]
        )

        # Delay camera bridge to ensure PX4 and Gazebo are fully started
        camera_bridge_delayed = TimerAction(
            period=6.0,
            actions=[camera_bridge]
        )
        camera_actions.append(camera_bridge_delayed)

        # NOTE: Compression node removed - ros_gz_image provides automatic compression
        # via image_transport. Compressed images are available at /{ns}/camera/image_raw/compressed
        # For H.264 compression, install: sudo apt install ros-humble-ffmpeg-image-transport
        
    # 3D lidar point cloud (x500_gimbal_collision_lidar): the only lidar data bridged to ROS.
    lidar_actions = []
    if enable_lidar_val.lower() == 'true':
        ns = namespace_val if namespace_val else f'px4_{ID}'
        model_name = gz_model_name_val if gz_model_name_val else f'{vehicle}_{ID}'
        gz_topic = f'/world/{world}/model/{model_name}/link/lidar_link/sensor/lidar/scan/points'
        lidar_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'lidar_bridge_{ID}',
            arguments=[f'{gz_topic}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'],
            remappings=[(gz_topic, f'/{ns}/lidar/points')],
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
        lidar_actions.append(TimerAction(period=6.0, actions=[lidar_bridge]))

        # Point cloud frame: the lidar SENSOR pose in base_link (see x500_gimbal_collision_lidar:
        # lidar_link (0.14, 0, 0.44) + sensor offset 0.045, minus base_link z 0.24 in the model).
        lidar_static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'lidar_static_tf_{ID}',
            arguments=['--x', '0.185', '--y', '0', '--z', '0.20',
                       '--frame-id', f'{ns}/base_link',
                       '--child-frame-id', f'{ns}/lidar_link'],
            output='screen'
        )
        lidar_actions.append(lidar_static_tf)

    # TF <ns>/odom -> <ns>/base_link from the PX4 odometry (NED/FRD -> ENU/FLU).
    tf_actions = []
    if enable_tf_val.lower() == 'true':
        ns = namespace_val if namespace_val else f'px4_{ID}'
        tf_params = {'use_sim_time': True}
        if map_origin_val:
            # Common `map` frame: ENU anchored at lat,lon,alt (the scene origin)
            lat, lon, alt = (float(v) for v in map_origin_val.split(','))
            tf_params.update({'map_lat': lat, 'map_lon': lon, 'map_alt': alt})
        tf_actions.append(Node(
            package='muav_gcs_gz',
            executable='odom_tf_broadcaster.py',
            name=f'odom_tf_broadcaster_{ID}',
            namespace=ns,
            output='screen',
            parameters=[tf_params]
        ))

    return [
        px4_sitl_node,
    ] + camera_actions + lidar_actions + tf_actions
    
def launch_px4(context):
    # Get PX4 directory path
    px4_dir = find_px4()
    # Resolve launch configurations to actual values
    x_val = context.launch_configurations['x']
    y_val = context.launch_configurations['y']
    z_val = context.launch_configurations['z']
    roll_val = context.launch_configurations['roll']
    pitch_val = context.launch_configurations['pitch']
    yaw_val = context.launch_configurations['yaw']
    vehicle_val = context.launch_configurations['vehicle']
    ID_val = context.launch_configurations['ID']
    autostart_val = context.launch_configurations['autostart']
    namespace_val = context.launch_configurations['namespace']
    gz_model_name_val = context.launch_configurations['gz_model_name']
    # Vehicles shipped in this package (models/<vehicle>) are not in PX4's model dir, so PX4
    # cannot spawn them itself: we spawn them and PX4 attaches. Default name: {vehicle}_{ID},
    # the same one PX4 would use and the camera bridge expects.
    if not gz_model_name_val:
        pkg_model = os.path.join(
            get_package_share_directory('muav_gcs_gz'), 'models', vehicle_val, 'model.sdf')
        if os.path.isfile(pkg_model):
            gz_model_name_val = f'{vehicle_val}_{ID_val}'
    # Build pose string
    pose_str = f"{x_val},{y_val},{z_val},{roll_val},{pitch_val},{yaw_val}"
    # Build environment variables dictionary
    env_vars = {
        'PX4_GZ_STANDALONE': '1',
        'PX4_SYS_AUTOSTART': autostart_val,
        'PX4_GZ_MODEL_POSE': pose_str,
        'PX4_SIM_SPEED_FACTOR': '1',  # Changed to '1' (was '1.0')
    }

    if gz_model_name_val and gz_model_name_val != '':
        # Model was already spawned externally: attach to it by exact name,
        # PX4_SIM_MODEL and PX4_GZ_MODEL_NAME are mutually exclusive.
        env_vars['PX4_GZ_MODEL_NAME'] = gz_model_name_val
        print(f"[DEBUG] Attaching to existing Gazebo model: {gz_model_name_val}")
    else:
        env_vars['PX4_SIM_MODEL'] = f"gz_{vehicle_val}"

    # Add GZ_PARTITION if set in environment (critical for Docker/containerized environments)
    if 'GZ_PARTITION' in os.environ:
        env_vars['GZ_PARTITION'] = os.environ['GZ_PARTITION']
        print(f"[DEBUG] GZ_PARTITION set to: {os.environ['GZ_PARTITION']}")
    else:
        print("[WARNING] GZ_PARTITION not found in environment!")
        
    if 'speedfactor' in os.environ:
        env_vars['PX4_SIM_SPEED_FACTOR'] = os.environ['speedfactor']
        print(f"[DEBUG] PX4_SIM_SPEED_FACTOR set to: {os.environ['speedfactor']}")

    # By default PX4 locks the GUI camera onto every model it spawns (/gui/track FOLLOW in
    # px4-rc.gzsim), which blocks navigating the world and fights between several drones.
    # Disabled by default; export PX4_GZ_NO_FOLLOW= (empty) before the launch to re-enable it.
    env_vars['PX4_GZ_NO_FOLLOW'] = os.environ.get('PX4_GZ_NO_FOLLOW', '1')

    # Add namespace if provided
    if namespace_val and namespace_val != '':
        env_vars['PX4_UXRCE_DDS_NS'] = namespace_val

    # Debug: Print all env vars being passed to PX4
    print(f"[DEBUG] Spawning PX4 with ID={ID_val}, pose={pose_str}")
    print(f"[DEBUG] Environment variables: {env_vars}")

    # Create PX4 process
    px4_process = ExecuteProcess(
        cmd=[
            px4_dir + '/build/px4_sitl_default/bin/px4',
            '-i', ID_val
        ],
        additional_env=env_vars,
        output='screen',
        shell=False,
        name=f'px4_{ID_val}'
    )

    if gz_model_name_val and gz_model_name_val != '':
        # Spawn the custom model first, and only start PX4 once the spawn
        # process has exited (it blocks until Gazebo's create service replies),
        # so PX4_GZ_MODEL_NAME always finds an existing model to attach to.
        world_val = context.launch_configurations['world']
        model_path = os.path.join(
            get_package_share_directory('muav_gcs_gz'), 'models',
            vehicle_val, 'model.sdf'
        )

        model_path = fill_namespace(
            model_path, namespace_val if namespace_val else f'px4_{ID_val}')

        spawn_process = ExecuteProcess(
            cmd=[
                'ros2', 'run', 'ros_gz_sim', 'create',
                '-name', gz_model_name_val,
                '-file', model_path,
                '-x', x_val, '-y', y_val, '-z', z_val,
                '-R', roll_val, '-P', pitch_val, '-Y', yaw_val,
                '-world', world_val,
            ],
            output='screen',
            shell=False,
            name=f'gz_spawn_{gz_model_name_val}'
        )

        px4_after_spawn = RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_process,
                on_exit=[px4_process],
            )
        )

        return [spawn_process, px4_after_spawn]

    return [px4_process]

def generate_launch_description():
    declared_arguments = []
    # Declare launch arguments
    # Vehicle pose (x,y,z,roll,pitch,yaw)
    declared_arguments.append( 
        DeclareLaunchArgument('x', default_value='0', description='X position')
        )
    declared_arguments.append(
        DeclareLaunchArgument('y', default_value='0', description='Y position')
        )
    declared_arguments.append(
        DeclareLaunchArgument('z', default_value='0', description='Z position')
        )
    declared_arguments.append(
        DeclareLaunchArgument('roll', default_value='0', description='Roll')
        )
    declared_arguments.append(
        DeclareLaunchArgument('pitch', default_value='0', description='Pitch')
        )
    declared_arguments.append(
        DeclareLaunchArgument('yaw', default_value='0', description='Yaw')
        )
    declared_arguments.append(
        DeclareLaunchArgument('vehicle',
        default_value='x500_mono_cam',
        description='Vehicle model (e.g., x500, x500_mono_cam)')
        )
    declared_arguments.append(
        DeclareLaunchArgument('ID', 
        default_value='1', 
        description='Vehicle instance ID mav_system_id = ID + 1')
        )
    declared_arguments.append(
        DeclareLaunchArgument('autostart', 
            default_value='4001', 
            description='PX4 autostart ID')
        )
    declared_arguments.append(
        DeclareLaunchArgument('namespace',
            default_value='',
            description='DDS namespace (e.g., px4_1, px4_2). If empty, no namespace is set.')
        )
    declared_arguments.append(
        DeclareLaunchArgument('gz_model_name',
            default_value='',
            description='Exact name of a model already spawned in Gazebo (PX4_GZ_MODEL_NAME). '
                         'Optional: defaults to {vehicle}_{ID} for vehicles in this package\'s models/. '
                         'If set, PX4 attaches to it instead of spawning "vehicle" itself.')
        )
    # Camera bridge option
    declared_arguments.append(
        DeclareLaunchArgument('enable_camera', default_value='false', description='Enable camera bridge (only works with camera-equipped models)')
        )
    declared_arguments.append(
        DeclareLaunchArgument('enable_lidar', default_value='false', description='Bridge the 3D lidar point cloud to /{ns}/lidar/points (only x500_gimbal_collision_lidar)')
        )
    declared_arguments.append(
        DeclareLaunchArgument('enable_tf', default_value='true', description='Publish TF <ns>/odom -> <ns>/base_link from /<ns>/fmu/out/vehicle_odometry')
        )
    declared_arguments.append(
        DeclareLaunchArgument('map_origin', default_value='', description='"lat,lon,alt" of the common `map` frame (ENU). If set, publishes map -> <ns>/odom from the PX4 EKF origin')
        )
    declared_arguments.append(
        DeclareLaunchArgument('world', default_value='default', description='Gazebo world name')
        )
    
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])